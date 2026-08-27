#pragma once

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>

/* Permanent VESC Tool UART transport.
 *
 * SmartESC-compatible mapping/architecture:
 *   USART3 TX PB10 -> DMA1 Channel2 (normal)
 *   USART3 RX PB11 -> DMA1 Channel3 (circular)
 *   RX packet task polls DMA CNDTR; no duplicate software RX ring.
 *   DMA IRQ handlers delegate to STM32 HAL.
 */
#define VESC_UART_RX_DMA_SIZE          512U
#define VESC_UART_TX_FRAME_MAX         520U

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
    volatile uint32_t rx_irq_count;
    volatile uint32_t tx_irq_count;
    volatile uint32_t rx_dma_irq_count;
    volatile uint32_t tx_dma_irq_count;
    volatile uint32_t idle_irq_count;       /* retained for debug ABI; unused */
    volatile uint32_t dma_errors;
    volatile uint32_t tx_queue_high_water;  /* retained for debug ABI; no SW TX queue */
    volatile uint32_t tx_queue_busy_drops;
    volatile uint32_t tx_low_priority_drops;
    volatile uint32_t rx_restarts;
} app_uartcomm_stats_t;

extern UART_HandleTypeDef huart3_vesc;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern app_uartcomm_stats_t g_vesc_uart_stats;

bool app_uartcomm_init(void);
bool app_uartcomm_rx_get(uint8_t *byte);
bool app_uartcomm_write_raw(const uint8_t *data, uint16_t len);
bool app_uartcomm_write_raw_low_priority(const uint8_t *data, uint16_t len);

void app_uartcomm_dma_rx_irq_handler(void);
void app_uartcomm_dma_tx_irq_handler(void);

const app_uartcomm_stats_t *app_uartcomm_get_stats(void);
