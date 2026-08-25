#include "applications/app_uartcomm.h"
#include "applications/app.h"
#include "comm/packet.h"
#include "hwconf/hw_hoverboard.h"
#include "applications/appconf_default.h"
#include <string.h>
#include <stdint.h>

_Static_assert(VESC_UART_TX_FRAME_MAX >= VESC_PACKET_BUFFER_SIZE,
               "VESC UART TX frame buffer must fit maximum packet frame");
_Static_assert(VESC_UART_RX_RING_SIZE >= VESC_PACKET_BUFFER_SIZE,
               "VESC UART RX ring must fit one maximum packet frame");

/* USART3 mapping on STM32F103:
 *   TX -> DMA1 Channel2
 *   RX -> DMA1 Channel3
 * ADC dual-current sampling already owns DMA1 Channel1 and remains priority 0.
 */
static UART_HandleTypeDef huart3_vesc;
static bool s_transport_initialized=false;

static uint8_t s_rx_dma[VESC_UART_RX_DMA_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_rx_dma_last;
static volatile uint8_t s_rx_ring[VESC_UART_RX_RING_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

typedef struct {
    volatile uint16_t len;
    uint8_t data[VESC_UART_TX_FRAME_MAX];
} tx_slot_t;

static tx_slot_t s_tx_q[VESC_UART_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile bool s_tx_active;
static osMutexId_t s_tx_mutex;
static osMutexId_t s_packet_mutex;
static uint8_t s_packet_frame[VESC_PACKET_BUFFER_SIZE];
static app_uartcomm_stats_t s_stats;

static uint8_t tx_next(uint8_t p) {
    p++;
    if (p >= VESC_UART_TX_QUEUE_DEPTH) p = 0U;
    return p;
}

static uint16_t rx_next(uint16_t p) {
    return (uint16_t)((p + 1U) & (VESC_UART_RX_RING_SIZE - 1U));
}

static void rx_ring_push(uint8_t byte) {
    uint16_t head = s_rx_head;
    uint16_t next = rx_next(head);
    if (next == s_rx_tail) {
        s_stats.rx_overruns++;
        return;
    }
    s_rx_ring[head] = byte;
    __DMB();
    s_rx_head = next;
    s_stats.rx_bytes++;
}

/* Caller either runs in IRQ context or has IRQs masked. DMA continues while
 * this executes, therefore CNDTR is sampled once and only the stable region up
 * to that position is copied. HT/TC/IDLE plus task-side polling guarantee that
 * a continuous stream does not depend on receiving an IDLE gap. */
static void rx_dma_drain_locked(void) {
    uint16_t pos = (uint16_t)(VESC_UART_RX_DMA_SIZE - DMA1_Channel3->CNDTR);
    if (pos >= VESC_UART_RX_DMA_SIZE) pos = 0U;

    uint16_t last = s_rx_dma_last;
    if (pos == last) return;

    if (pos > last) {
        for (uint16_t i = last; i < pos; i++) rx_ring_push(s_rx_dma[i]);
    } else {
        for (uint16_t i = last; i < VESC_UART_RX_DMA_SIZE; i++) rx_ring_push(s_rx_dma[i]);
        for (uint16_t i = 0U; i < pos; i++) rx_ring_push(s_rx_dma[i]);
    }
    s_rx_dma_last = pos;
}

static void tx_start_locked(void) {
    if (s_tx_active) return;

    /* Skip any corrupt slot defensively so one bad queue entry cannot wedge
     * all subsequent VESC replies. Normal operation never enters this loop. */
    while (s_tx_tail != s_tx_head) {
        tx_slot_t *candidate = &s_tx_q[s_tx_tail];
        if (candidate->len != 0U && candidate->len <= VESC_UART_TX_FRAME_MAX) break;
        candidate->len = 0U;
        s_tx_tail = tx_next(s_tx_tail);
        s_stats.uart_errors++;
    }
    if (s_tx_tail == s_tx_head) return;

    tx_slot_t *slot = &s_tx_q[s_tx_tail];
    uint16_t len = slot->len;

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    DMA1_Channel2->CMAR = (uint32_t)(uintptr_t)slot->data;
    DMA1_Channel2->CNDTR = len;
    __DMB();
    s_tx_active = true;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart == NULL || huart->Instance != USART3) return;

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

#ifdef AFIO_MAPR_USART3_REMAP
    AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
#endif

    GPIO_InitTypeDef g = {0};
    g.Pin = VESC_UART_TX_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(VESC_UART_TX_PORT, &g);

    g.Pin = VESC_UART_RX_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VESC_UART_RX_PORT, &g);
}

bool app_uartcomm_init(void) {
    if(s_transport_initialized)return true;
    memset((void *)&s_stats, 0, sizeof(s_stats));
    memset(s_rx_dma, 0, sizeof(s_rx_dma));
    memset((void *)s_rx_ring, 0, sizeof(s_rx_ring));
    memset((void *)s_tx_q, 0, sizeof(s_tx_q));
    s_rx_dma_last = 0U;
    s_rx_head = s_rx_tail = 0U;
    s_tx_head = s_tx_tail = 0U;
    s_tx_active = false;

    const osMutexAttr_t attr = {.name = "VescUartTx"};
    const osMutexAttr_t packet_attr = {.name = "VescPktTx"};
    s_tx_mutex = osMutexNew(&attr);
    if (s_tx_mutex == NULL) return false;
    s_packet_mutex = osMutexNew(&packet_attr);
    if (s_packet_mutex == NULL) return false;

    huart3_vesc.Instance = USART3;
    huart3_vesc.Init.BaudRate = VESC_UART_BAUD;
    huart3_vesc.Init.WordLength = UART_WORDLENGTH_8B;
    huart3_vesc.Init.StopBits = UART_STOPBITS_1;
    huart3_vesc.Init.Parity = UART_PARITY_NONE;
    huart3_vesc.Init.Mode = UART_MODE_TX_RX;
    huart3_vesc.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3_vesc.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3_vesc) != HAL_OK) return false;

    /* RX DMA: peripheral-to-memory, 8-bit, memory increment, circular.
     * HT/TC interrupt bounds worst-case drain latency even with no IDLE gap. */
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF3;
    DMA1_Channel3->CPAR = (uint32_t)(uintptr_t)&VESC_UART->DR;
    DMA1_Channel3->CMAR = (uint32_t)(uintptr_t)s_rx_dma;
    DMA1_Channel3->CNDTR = VESC_UART_RX_DMA_SIZE;
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_CIRC |
                         DMA_CCR_PL_1 | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE;

    /* TX DMA: memory-to-peripheral, one complete queued frame per transfer. */
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF2;
    DMA1_Channel2->CPAR = (uint32_t)(uintptr_t)&VESC_UART->DR;
    DMA1_Channel2->CMAR = 0U;
    DMA1_Channel2->CNDTR = 0U;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR |
                         DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_TEIE;

    /* Clear stale UART flags before enabling DMA requests. */
    (void)VESC_UART->SR;
    (void)VESC_UART->DR;
    VESC_UART->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE);
    VESC_UART->CR1 |= USART_CR1_IDLEIE | USART_CR1_PEIE;
    VESC_UART->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_EIE;

    /* FOC ADC DMA is priority 0. UART DMA/IDLE IRQs never call RTOS and are
     * intentionally lower priority to keep motor-control jitter bounded. */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2U, 0U);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2U, 0U);
    HAL_NVIC_SetPriority(USART3_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    DMA1_Channel3->CCR |= DMA_CCR_EN;
    s_transport_initialized=true;
    return true;
}

bool app_uartcomm_rx_get(uint8_t *byte) {
    if (byte == NULL) return false;

    /* Periodic task-side drain is deliberate. It prevents a continuous stream
     * from relying on an IDLE interrupt and keeps IRQ work bounded. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_dma_drain_locked();
    if (!primask) __enable_irq();

    uint16_t tail = s_rx_tail;
    if (tail == s_rx_head) return false;
    *byte = s_rx_ring[tail];
    __DMB();
    s_rx_tail = rx_next(tail);
    return true;
}

static uint8_t tx_queue_used_snapshot(void) {
    uint8_t head = s_tx_head;
    uint8_t tail = s_tx_tail;
    return head >= tail ? (uint8_t)(head - tail)
                        : (uint8_t)(VESC_UART_TX_QUEUE_DEPTH - tail + head);
}

static bool app_uartcomm_write_raw_class(const uint8_t *data, uint16_t len, bool low_priority) {
    if (data == NULL || len == 0U || len > VESC_UART_TX_FRAME_MAX || s_tx_mutex == NULL) return false;

    /* Queueing is deliberately non-blocking. Low-priority debug/telemetry is
     * additionally prevented from consuming the last usable ring slot. This
     * preserves one frame of response headroom for VESC command/config/fault
     * traffic without allocating a second ~520-byte queue on the 48-KiB F103. */
    if (osMutexAcquire(s_tx_mutex, 0U) != osOK) {
        if (low_priority) s_stats.tx_low_priority_drops++;
        else s_stats.tx_queue_busy_drops++;
        return false;
    }

    uint8_t used_before = tx_queue_used_snapshot();
    if (low_priority && used_before >= (uint8_t)(VESC_UART_TX_QUEUE_DEPTH - 2U)) {
        s_stats.tx_low_priority_drops++;
        (void)osMutexRelease(s_tx_mutex);
        return false;
    }

    const uint8_t next = tx_next(s_tx_head);
    if (next == s_tx_tail) {
        if (low_priority) s_stats.tx_low_priority_drops++;
        else s_stats.tx_overruns++;
        (void)osMutexRelease(s_tx_mutex);
        return false;
    }

    tx_slot_t *slot = &s_tx_q[s_tx_head];
    memcpy(slot->data, data, len);
    slot->len = len;
    __DMB();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_tx_head = next;
    uint8_t used = tx_queue_used_snapshot();
    if (used > s_stats.tx_queue_high_water) s_stats.tx_queue_high_water = used;
    tx_start_locked();
    if (!primask) __enable_irq();

    (void)osMutexRelease(s_tx_mutex);
    return true;
}

bool app_uartcomm_write_raw(const uint8_t *data, uint16_t len) {
    return app_uartcomm_write_raw_class(data, len, false);
}

bool app_uartcomm_write_raw_low_priority(const uint8_t *data, uint16_t len) {
    return app_uartcomm_write_raw_class(data, len, true);
}

void app_uartcomm_irq_handler(void) {
    uint32_t sr = VESC_UART->SR;
    s_stats.rx_irq_count++;

    uint32_t err = sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);
    bool idle = (sr & USART_SR_IDLE) != 0U;
    if (idle || err != 0U) {
        /* STM32F1 clears IDLE/ORE/FE/NE/PE with SR then DR read sequence. */
        (void)VESC_UART->DR;
        if (idle) s_stats.idle_irq_count++;
        if (err != 0U) {
            s_stats.uart_errors++;
            if ((err & USART_SR_ORE) != 0U) s_stats.rx_overruns++;
        }
        rx_dma_drain_locked();
    }
}

void app_uartcomm_dma_rx_irq_handler(void) {
    uint32_t isr = DMA1->ISR;
    s_stats.rx_dma_irq_count++;

    if ((isr & DMA_ISR_TEIF3) != 0U) {
        /* A DMA transfer error must not permanently kill the VESC UART.
         * Restart the circular receiver from a clean DMA position; any partial
         * packet is discarded naturally by packet CRC/resynchronization. */
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF3;
        s_stats.dma_errors++;
        s_stats.uart_errors++;
        s_rx_dma_last = 0U;
        DMA1_Channel3->CMAR = (uint32_t)(uintptr_t)s_rx_dma;
        DMA1_Channel3->CNDTR = VESC_UART_RX_DMA_SIZE;
        __DMB();
        DMA1_Channel3->CCR |= DMA_CCR_EN;
        return;
    }
    if ((isr & (DMA_ISR_HTIF3 | DMA_ISR_TCIF3)) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3;
        rx_dma_drain_locked();
    }
}

void app_uartcomm_dma_tx_irq_handler(void) {
    uint32_t isr = DMA1->ISR;
    s_stats.tx_dma_irq_count++;
    s_stats.tx_irq_count++;

    if ((isr & DMA_ISR_TEIF2) != 0U) {
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF2;
        s_stats.dma_errors++;
        s_stats.uart_errors++;
        if (s_tx_active && s_tx_tail != s_tx_head) {
            s_tx_q[s_tx_tail].len = 0U;
            s_tx_tail = tx_next(s_tx_tail);
        }
        s_tx_active = false;
        tx_start_locked();
        return;
    }

    if ((isr & DMA_ISR_TCIF2) != 0U) {
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;
        if (s_tx_active && s_tx_tail != s_tx_head) {
            uint16_t len = s_tx_q[s_tx_tail].len;
            if (len <= VESC_UART_TX_FRAME_MAX) s_stats.tx_bytes += len;
            s_tx_q[s_tx_tail].len = 0U;
            s_tx_tail = tx_next(s_tx_tail);
            s_stats.tx_complete_count++;
        }
        s_tx_active = false;
        tx_start_locked();
    }
}

const app_uartcomm_stats_t *app_uartcomm_get_stats(void) {
    return &s_stats;
}


/* ==================== Canonical VESC UART application API ====================
 * USART3 is also the permanent management link, so start/stop never tear down
 * the physical DMA transport. APP_NONE only means there is no autonomous app;
 * VESC Tool access must remain recoverable. */
void app_uartcomm_initialize(void){(void)app_uartcomm_init();}
void app_uartcomm_start(UART_PORT port_number){(void)port_number;(void)app_uartcomm_init();}
void app_uartcomm_stop(UART_PORT port_number){(void)port_number;/* permanent management transport */}
void app_uartcomm_configure(uint32_t baudrate,bool permanent_enabled,UART_PORT port_number){
    (void)permanent_enabled;(void)port_number;
    /* The verified hardware transport is fixed to VESC_UART_BAUD. Invalid
       rates are rejected earlier by APPCONF validation rather than silently
       reprogramming DMA/UART while packets are in flight. */
    (void)baudrate;
}
void app_uartcomm_send_packet(unsigned char *data,unsigned int len,UART_PORT port_number){
    (void)port_number;
    if(!data||len>VESC_PACKET_MAX_PAYLOAD||s_packet_mutex==NULL)return;
    /* Avoid a >500-byte transient task stack allocation on STM32F103. The
       packet scratch is serialized separately from the DMA frame queue. */
    if(osMutexAcquire(s_packet_mutex,osWaitForever)!=osOK)return;
    uint16_t fl=vesc_packet_encode(data,(uint16_t)len,s_packet_frame,sizeof(s_packet_frame));
    if(fl>0U)(void)app_uartcomm_write_raw(s_packet_frame,fl);
    (void)osMutexRelease(s_packet_mutex);
}
