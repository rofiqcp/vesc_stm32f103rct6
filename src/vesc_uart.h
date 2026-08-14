#pragma once

#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

/* Transport intentionally follows the proven hoverboard_vesc USART3 path:
 * RX = DMA1 Channel3 circular, TX = DMA1 Channel2 queued, 115200 8N1.
 * Parsing still runs in packet_process_thread, never in an interrupt. */
#define VESC_UART_RX_DMA_SIZE       1024U
#define VESC_UART_TX_QUEUE_DEPTH    6U
#define VESC_UART_TX_FRAME_MAX      520U

typedef struct {
    volatile uint32_t rx_bytes;
    volatile uint32_t tx_bytes;
    volatile uint32_t rx_overruns;
    volatile uint32_t tx_overruns;
    volatile uint32_t uart_errors;
    volatile uint32_t tx_complete_count;
    volatile uint32_t rx_dma_restarts;
    volatile uint32_t tx_dma_errors;
} vesc_uart_stats_t;

bool vesc_uart_init(void);
void vesc_uart_service(void);
bool vesc_uart_rx_get(uint8_t *byte);
bool vesc_uart_write_raw(const uint8_t *data, uint16_t len);
void vesc_uart_tx_dma_irq_handler(void);
void vesc_uart_rx_dma_irq_handler(void);
void vesc_uart_usart_defensive_irq_handler(void);
const vesc_uart_stats_t *vesc_uart_get_stats(void);
