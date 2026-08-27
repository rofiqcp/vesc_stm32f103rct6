#pragma once

#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

/* Permanent VESC Tool UART transport for STM32F103 hoverboard hardware.
 *
 * RX: USART3 -> DMA1 Channel3 circular buffer. IDLE and DMA HT/TC are only
 *     wake/drain opportunities; packet framing remains in task context.
 * TX: queued complete VESC frames -> DMA1 Channel2, one frame at a time.
 *
 * No RTOS API, packet parsing, printf, malloc or blocking HAL is used from
 * USART/DMA IRQ context. ADC/FOC DMA1 Channel1 remains the highest-priority
 * interrupt and is completely independent from this transport. */
#define VESC_UART_RX_RING_SIZE       1024U
#define VESC_UART_RX_DMA_SIZE         256U
#define VESC_UART_TX_QUEUE_DEPTH        8U
#define VESC_UART_TX_FRAME_MAX        520U

#if (VESC_UART_RX_RING_SIZE & (VESC_UART_RX_RING_SIZE - 1U)) != 0
#error "VESC_UART_RX_RING_SIZE must be power-of-two"
#endif
#if (VESC_UART_RX_DMA_SIZE & (VESC_UART_RX_DMA_SIZE - 1U)) != 0
#error "VESC_UART_RX_DMA_SIZE must be power-of-two"
#endif

typedef struct {
    volatile uint32_t rx_bytes;
    volatile uint32_t tx_bytes;
    volatile uint32_t rx_overruns;
    volatile uint32_t tx_overruns;
    volatile uint32_t uart_errors;
    volatile uint32_t tx_complete_count;
    volatile uint32_t rx_irq_count;      /* compatibility: USART3 IRQ count */
    volatile uint32_t tx_irq_count;      /* compatibility: TX DMA IRQ count */
    volatile uint32_t rx_dma_irq_count;
    volatile uint32_t tx_dma_irq_count;
    volatile uint32_t idle_irq_count;
    volatile uint32_t dma_errors;
    volatile uint32_t tx_queue_high_water;
    volatile uint32_t tx_queue_busy_drops;
    volatile uint32_t tx_low_priority_drops;
} app_uartcomm_stats_t;

bool app_uartcomm_init(void);
bool app_uartcomm_rx_get(uint8_t *byte);
bool app_uartcomm_write_raw(const uint8_t *data, uint16_t len);
/* Low-priority telemetry/debug transport. It reserves one software TX slot for
 * command/config/fault responses so bulk samples cannot starve control replies. */
bool app_uartcomm_write_raw_low_priority(const uint8_t *data, uint16_t len);

/* IRQ entry points called only from stm32f1xx_it.c. */
void app_uartcomm_irq_handler(void);
void app_uartcomm_dma_rx_irq_handler(void);
void app_uartcomm_dma_tx_irq_handler(void);

const app_uartcomm_stats_t *app_uartcomm_get_stats(void);

/* Canonical VESC app UART API lives in applications/app.h. Low-level B4 DMA
 * functions above remain available to commands.c and IRQ glue. */
