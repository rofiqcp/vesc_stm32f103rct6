#include "vesc_uart.h"
#include "board_pins.h"
#include "app_config.h"
#include <string.h>

/* This is an RTOS-safe reimplementation of the transport architecture proven
 * on rofiqcp/hoverboard_vesc. It deliberately does not use RXNE/TXE for the
 * normal VESC path. DMA1 CH3 continuously receives; packet_process_thread
 * polls CNDTR and feeds bytes to the VESC packet parser. Whole encoded frames
 * are queued for DMA1 CH2 transmission. */

static UART_HandleTypeDef huart3_vesc;
static volatile uint8_t s_rx_dma[VESC_UART_RX_DMA_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_rx_old = 0U;

typedef struct {
    uint16_t len;
    uint8_t data[VESC_UART_TX_FRAME_MAX];
} tx_slot_t;

static tx_slot_t s_tx_q[VESC_UART_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head = 0U;
static volatile uint8_t s_tx_tail = 0U;
static volatile bool s_tx_busy = false;
static osMutexId_t s_tx_mutex = NULL;
static vesc_uart_stats_t s_stats;

static uint8_t tx_next(uint8_t p) {
    p++;
    if (p >= VESC_UART_TX_QUEUE_DEPTH) p = 0U;
    return p;
}

static void clear_dma2_flags(void) {
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2;
}

static void clear_dma3_flags(void) {
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;
}

static void rx_dma_start(void) {
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    clear_dma3_flags();
    DMA1_Channel3->CPAR = (uint32_t)&VESC_UART->DR;
    DMA1_Channel3->CMAR = (uint32_t)s_rx_dma;
    DMA1_Channel3->CNDTR = VESC_UART_RX_DMA_SIZE;
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_CIRC;
    s_rx_old = 0U;
    VESC_UART->CR3 |= USART_CR3_DMAR;
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

static void tx_dma_start_locked(void) {
    if (s_tx_busy || s_tx_tail == s_tx_head) return;

    tx_slot_t *slot = &s_tx_q[s_tx_tail];
    if (slot->len == 0U || slot->len > VESC_UART_TX_FRAME_MAX) {
        s_stats.tx_dma_errors++;
        s_tx_tail = tx_next(s_tx_tail);
        return;
    }

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    clear_dma2_flags();
    DMA1_Channel2->CPAR = (uint32_t)&VESC_UART->DR;
    DMA1_Channel2->CMAR = (uint32_t)slot->data;
    DMA1_Channel2->CNDTR = slot->len;
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE;

    __DMB();
    s_tx_busy = true;
    VESC_UART->CR3 |= USART_CR3_DMAT;
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

/* HAL_UART_Init is used only to reproduce the known-good 115200 8N1 USART
 * register setup. Normal RX/TX data movement is direct DMA, not HAL UART IRQ. */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart == NULL || huart->Instance != USART3) return;

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

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

bool vesc_uart_init(void) {
    memset((void *)&s_stats, 0, sizeof(s_stats));
    memset((void *)s_rx_dma, 0, sizeof(s_rx_dma));
    memset((void *)s_tx_q, 0, sizeof(s_tx_q));
    s_rx_old = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_busy = false;

    const osMutexAttr_t attr = {.name = "VescDmaTx"};
    s_tx_mutex = osMutexNew(&attr);
    if (s_tx_mutex == NULL) return false;

    __HAL_RCC_DMA1_CLK_ENABLE();

    huart3_vesc.Instance = USART3;
    huart3_vesc.Init.BaudRate = VESC_UART_BAUD;
    huart3_vesc.Init.WordLength = UART_WORDLENGTH_8B;
    huart3_vesc.Init.StopBits = UART_STOPBITS_1;
    huart3_vesc.Init.Parity = UART_PARITY_NONE;
    huart3_vesc.Init.Mode = UART_MODE_TX_RX;
    huart3_vesc.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3_vesc.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3_vesc) != HAL_OK) return false;

    /* The proven implementation leaves USART3 IRQ and RX DMA IRQ disabled.
     * DMA reads DR continuously, while framing+CRC validate received data. */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    VESC_UART->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE |
                        USART_CR1_PEIE | USART_CR1_IDLEIE);
    VESC_UART->CR3 &= ~(USART_CR3_EIE | USART_CR3_DMAT | USART_CR3_DMAR);

    (void)VESC_UART->SR;
    (void)VESC_UART->DR;

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    clear_dma2_flags();
    clear_dma3_flags();

    /* Same IRQ topology as the known-working hoverboard transport: TX DMA
     * interrupt enabled, RX DMA circular interrupt disabled. No RTOS API is
     * called by this priority-0 IRQ. */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    rx_dma_start();
    return true;
}

void vesc_uart_service(void) {
    uint32_t isr = DMA1->ISR;

    if ((isr & DMA_ISR_TEIF3) != 0U || (DMA1_Channel3->CCR & DMA_CCR_EN) == 0U) {
        s_stats.uart_errors++;
        s_stats.rx_dma_restarts++;
        rx_dma_start();
    }

    /* Poll fallback mirrors the proven main-loop service. It makes TX robust
     * even if a debugger temporarily masks the DMA2 NVIC line. */
    if ((isr & (DMA_ISR_TCIF2 | DMA_ISR_TEIF2)) != 0U) {
        vesc_uart_tx_dma_irq_handler();
    }

    if (!s_tx_busy && s_tx_tail != s_tx_head) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        tx_dma_start_locked();
        if (!primask) __enable_irq();
    }
}

bool vesc_uart_rx_get(uint8_t *byte) {
    if (byte == NULL) return false;

    uint16_t ndtr = (uint16_t)DMA1_Channel3->CNDTR;
    if (ndtr > VESC_UART_RX_DMA_SIZE) return false;
    uint16_t pos = (uint16_t)(VESC_UART_RX_DMA_SIZE - ndtr);
    if (pos >= VESC_UART_RX_DMA_SIZE) pos = 0U;

    if (s_rx_old == pos) return false;

    *byte = s_rx_dma[s_rx_old];
    s_rx_old++;
    if (s_rx_old >= VESC_UART_RX_DMA_SIZE) s_rx_old = 0U;
    s_stats.rx_bytes++;
    return true;
}

bool vesc_uart_write_raw(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U || len > VESC_UART_TX_FRAME_MAX || s_tx_mutex == NULL) return false;
    if (osMutexAcquire(s_tx_mutex, osWaitForever) != osOK) return false;

    uint32_t start = osKernelGetTickCount();
    uint8_t next;
    for (;;) {
        next = tx_next(s_tx_head);
        if (next != s_tx_tail) break;
        if ((uint32_t)(osKernelGetTickCount() - start) >= 250U) {
            s_stats.tx_overruns++;
            (void)osMutexRelease(s_tx_mutex);
            return false;
        }
        osDelay(1U);
    }

    tx_slot_t *slot = &s_tx_q[s_tx_head];
    memcpy(slot->data, data, len);
    slot->len = len;
    __DMB();
    s_tx_head = next;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tx_dma_start_locked();
    if (!primask) __enable_irq();

    (void)osMutexRelease(s_tx_mutex);
    return true;
}

void vesc_uart_tx_dma_irq_handler(void) {
    uint32_t isr = DMA1->ISR;
    bool te = (isr & DMA_ISR_TEIF2) != 0U;
    bool tc = (isr & DMA_ISR_TCIF2) != 0U;
    if (!te && !tc) return;

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    VESC_UART->CR3 &= ~USART_CR3_DMAT;
    clear_dma2_flags();

    if (te) {
        s_stats.tx_dma_errors++;
        s_stats.uart_errors++;
    } else {
        uint16_t sent = s_tx_q[s_tx_tail].len;
        s_stats.tx_bytes += sent;
        s_stats.tx_complete_count++;
    }

    s_tx_q[s_tx_tail].len = 0U;
    s_tx_tail = tx_next(s_tx_tail);
    s_tx_busy = false;
    __DMB();
    tx_dma_start_locked();
}

void vesc_uart_rx_dma_irq_handler(void) {
    /* Normal RX DMA IRQ is disabled. This defensive handler only clears flags. */
    clear_dma3_flags();
}

void vesc_uart_usart_defensive_irq_handler(void) {
    /* USART3 IRQ is disabled during normal operation. If it is accidentally
     * enabled by another component, drain status/data and disable IRQ sources
     * without feeding a second competing RX path. */
    volatile uint32_t sr = VESC_UART->SR;
    volatile uint32_t dr = VESC_UART->DR;
    (void)sr; (void)dr;
    VESC_UART->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE |
                        USART_CR1_PEIE | USART_CR1_IDLEIE);
    VESC_UART->CR3 &= ~USART_CR3_EIE;
    s_stats.uart_errors++;
}

const vesc_uart_stats_t *vesc_uart_get_stats(void) {
    return &s_stats;
}
