#pragma once

#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

#define VESC_UART_RX_BUF_SIZE 256U
#define VESC_UART_TX_BUF_SIZE 2048U

#define VESC_RX_AVAILABLE     (1UL << 0)
#define VESC_TX_COMPLETE      (1UL << 1)

typedef struct {
    volatile uint32_t rx_bytes;
    volatile uint32_t tx_bytes;
    volatile uint32_t rx_overruns;
    volatile uint32_t tx_overruns;
    volatile uint32_t uart_errors;
    volatile uint32_t tx_complete_count;
} vesc_uart_stats_t;

/* Set by packet_process_thread. The USART ISR only signals this thread; it
 * never parses VESC packets in interrupt context. */
extern osThreadId_t vesc_comm_thread_id;

void vesc_uart_init(void);
void vesc_uart_rx_isr_put(uint8_t byte);
bool vesc_uart_rx_get(uint8_t *byte);
bool vesc_uart_tx_isr_get(uint8_t *byte);
bool vesc_uart_write_raw(const uint8_t *data, uint16_t len);
void vesc_uart_error_isr(void);
void vesc_uart_tx_complete_isr(void);
const vesc_uart_stats_t *vesc_uart_get_stats(void);
