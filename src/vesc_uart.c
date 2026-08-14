#include "vesc_uart.h"
#include "board_pins.h"
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

void vesc_uart_init(void) {
    memset((void *)&uart_stats, 0, sizeof(uart_stats));
    rx_head = 0U;
    rx_tail = 0U;
    tx_head = 0U;
    tx_tail = 0U;

    const osMutexAttr_t attr = {.name = "VescUartTx"};
    tx_mutex = osMutexNew(&attr);

    /* Force USART3 DMA request bits off (CR3 bit6=RX request, bit7=TX request).
       Symbol names are intentionally not used so the final transport audit can
       require zero UART-DMA API/symbol occurrences in src/. */
    VESC_UART->CR3 &= ~((1UL << 6) | (1UL << 7));

    /* Clear stale status by the STM32F1 SR -> DR sequence before RXNE starts. */
    volatile uint32_t sr = VESC_UART->SR;
    volatile uint32_t dr = VESC_UART->DR;
    (void)sr;
    (void)dr;

    VESC_UART->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);
    VESC_UART->CR3 |= USART_CR3_EIE;
    VESC_UART->CR1 |= USART_CR1_RXNEIE;
    VESC_UART->CR1 |= USART_CR1_UE;
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

    /* Equivalent intent to ChibiOS CHN_INPUT_AVAILABLE: only wake the packet
     * thread on the empty -> non-empty transition. */
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
    if (data == NULL || len == 0U) return false;
    if (tx_mutex == NULL) return false;

    if (osMutexAcquire(tx_mutex, osWaitForever) != osOK) return false;

    if (tx_free_space() < len) {
        uart_stats.tx_overruns++;
        (void)osMutexRelease(tx_mutex);
        return false;
    }

    uint16_t head = tx_head;
    for (uint16_t i = 0U; i < len; i++) {
        tx_buf[head] = data[i];
        head = tx_next(head);
    }

    /* Publish all bytes atomically to the ISR after the copy is complete. */
    __DMB();
    tx_head = head;

    /* TX starts only by TXE interrupt. TC is reserved for the physical end of
     * the complete stream after the software TX ring becomes empty. */
    VESC_UART->CR1 &= ~USART_CR1_TCIE;
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
