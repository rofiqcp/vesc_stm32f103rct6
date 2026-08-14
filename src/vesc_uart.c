#include "vesc_uart.h"
#include "board_pins.h"
#include "app_config.h"
#include <string.h>

static volatile uint8_t rx_buf[VESC_UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;

static volatile uint8_t tx_buf[VESC_UART_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0U;
static volatile uint16_t tx_tail = 0U;

static vesc_uart_stats_t uart_stats;
static osMutexId_t tx_mutex = NULL;

osThreadId_t vesc_comm_thread_id = NULL;

static uint16_t rx_next(uint16_t pos) {
    pos++;
    if (pos >= VESC_UART_RX_BUF_SIZE) pos = 0U;
    return pos;
}

static uint16_t tx_next(uint16_t pos) {
    pos++;
    if (pos >= VESC_UART_TX_BUF_SIZE) pos = 0U;
    return pos;
}

bool vesc_uart_init(void) {
    memset((void *)&uart_stats, 0, sizeof(uart_stats));
    rx_head = 0U;
    rx_tail = 0U;
    tx_head = 0U;
    tx_tail = 0U;
    vesc_comm_thread_id = NULL;

    const osMutexAttr_t attr = {.name = "VescUartTx"};
    tx_mutex = osMutexNew(&attr);
    if (tx_mutex == NULL) return false;

    /* app_uartcomm upstream ultimately reaches serial_lld.c. Configure the
     * F103 USART3 directly with the same enforced register semantics instead
     * of routing the protocol through the HAL UART generic IRQ handler. */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

#ifdef AFIO_MAPR_USART3_REMAP
    /* PB10/PB11 are the non-remapped USART3 pins required by this board. */
    AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
#endif

    GPIO_InitTypeDef g = {0};
    g.Pin = VESC_UART_TX_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(VESC_UART_TX_PORT, &g);

    /* Upstream enables a pull-up on the UART pins. On STM32F1 the RX pull-up
     * is represented as input + GPIO_PULLUP. */
    g.Pin = VESC_UART_RX_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VESC_UART_RX_PORT, &g);

    HAL_NVIC_DisableIRQ(USART3_IRQn);

    VESC_UART->CR1 = 0U;
    VESC_UART->CR2 = 0U;
    VESC_UART->CR3 = 0U;

    /* USART3 is on APB1. This mirrors serial_lld.c: BRR=PCLK/speed. */
    VESC_UART->BRR = HAL_RCC_GetPCLK1Freq() / VESC_UART_BAUD;

    /* app_uartcomm SerialConfig: cr1=0, cr2=LINEN, cr3=0. serial_lld then
     * enforces LBDIE, EIE, UE, PEIE, RXNEIE, TE and RE. No UART DMA bits. */
    VESC_UART->CR2 = USART_CR2_LINEN | USART_CR2_LBDIE;
    VESC_UART->CR3 = USART_CR3_EIE;
    VESC_UART->CR1 = USART_CR1_UE | USART_CR1_PEIE | USART_CR1_RXNEIE |
                     USART_CR1_TE | USART_CR1_RE;

    /* Same stale-status clear sequence used by upstream serial_lld.c. */
    VESC_UART->SR = 0U;
    (void)VESC_UART->SR;
    (void)VESC_UART->DR;

    /* Current upstream chooses priority 7 for serial <= 200 kbaud. Priority 7
     * is also safely below the FreeRTOS max-syscall threshold used here. */
    HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return true;
}

void vesc_uart_rx_isr_put(uint8_t byte) {
    uint16_t head = rx_head;
    uint16_t next = rx_next(head);

    if (next == rx_tail) {
        uart_stats.rx_overruns++;
        return;
    }

    bool was_empty = (head == rx_tail);
    rx_buf[head] = byte;
    __DMB();
    rx_head = next;
    uart_stats.rx_bytes++;

    /* Equivalent intent to CHN_INPUT_AVAILABLE: signal on empty->nonempty.
     * The packet thread also has the upstream-style 10 ms polling fallback. */
    if (was_empty && vesc_comm_thread_id != NULL) {
        (void)osThreadFlagsSet(vesc_comm_thread_id, VESC_RX_AVAILABLE);
    }
}

bool vesc_uart_rx_get(uint8_t *byte) {
    if (byte == NULL) return false;
    uint16_t tail = rx_tail;
    if (tail == rx_head) return false;
    *byte = rx_buf[tail];
    __DMB();
    rx_tail = rx_next(tail);
    return true;
}

bool vesc_uart_tx_isr_get(uint8_t *byte) {
    if (byte == NULL) return false;
    uint16_t tail = tx_tail;
    if (tail == tx_head) return false;
    *byte = tx_buf[tail];
    tx_tail = tx_next(tail);
    uart_stats.tx_bytes++;
    return true;
}

static uint16_t tx_free_space(void) {
    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    if (head >= tail) {
        return (uint16_t)(VESC_UART_TX_BUF_SIZE - (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

bool vesc_uart_write_raw(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U || tx_mutex == NULL) return false;
    if (osMutexAcquire(tx_mutex, osWaitForever) != osOK) return false;

    uint32_t start = osKernelGetTickCount();
    while (tx_free_space() < len) {
        if ((uint32_t)(osKernelGetTickCount() - start) > 250U) {
            uart_stats.tx_overruns++;
            (void)osMutexRelease(tx_mutex);
            return false;
        }
        osDelay(1U);
    }

    uint16_t head = tx_head;
    for (uint16_t i = 0U; i < len; i++) {
        tx_buf[head] = data[i];
        head = tx_next(head);
    }
    __DMB();
    tx_head = head;

    /* Upstream SerialDriver output-queue notify does exactly this: TXEIE
     * starts draining the queued bytes. */
    VESC_UART->CR1 |= USART_CR1_TXEIE;

    (void)osMutexRelease(tx_mutex);
    return true;
}

void vesc_uart_error_isr(void) {
    uart_stats.uart_errors++;
}

void vesc_uart_tx_complete_isr(void) {
    uart_stats.tx_complete_count++;
    if (vesc_comm_thread_id != NULL) {
        (void)osThreadFlagsSet(vesc_comm_thread_id, VESC_TX_COMPLETE);
    }
}

const vesc_uart_stats_t *vesc_uart_get_stats(void) {
    return &uart_stats;
}
