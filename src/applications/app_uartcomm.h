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
#define VESC_UART_RX_DMA_SIZE          1024U
#define VESC_UART_TX_FRAME_MAX         520U

#if (VESC_UART_RX_DMA_SIZE & (VESC_UART_RX_DMA_SIZE - 1U)) != 0
#error "VESC_UART_RX_DMA_SIZE must be power-of-two"
#endif

typedef struct {
    // Variabel rx_bytes: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t rx_bytes;
    // Variabel tx_bytes: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t tx_bytes;
    // Variabel rx_overruns: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t rx_overruns;
    // Variabel tx_overruns: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t tx_overruns;
    // Variabel uart_errors: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t uart_errors;
    // Variabel tx_complete_count: pencacah kejadian atau sampel.
    volatile uint32_t tx_complete_count;
    // Variabel rx_irq_count: pencacah kejadian atau sampel.
    volatile uint32_t rx_irq_count;
    // Variabel tx_irq_count: pencacah kejadian atau sampel.
    volatile uint32_t tx_irq_count;
    // Variabel rx_dma_irq_count: pencacah kejadian atau sampel.
    volatile uint32_t rx_dma_irq_count;
    // Variabel tx_dma_irq_count: pencacah kejadian atau sampel.
    volatile uint32_t tx_dma_irq_count;
    // Variabel idle_irq_count: pencacah kejadian atau sampel.
    volatile uint32_t idle_irq_count; /* retained for debug ABI; unused */
    // Variabel dma_errors: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t dma_errors;
    // Variabel tx_queue_high_water: handle atau state antrean FreeRTOS untuk pertukaran data antartask.
    volatile uint32_t tx_queue_high_water; /* retained for debug ABI; no SW TX queue */
    // Variabel tx_queue_busy_drops: handle atau state antrean FreeRTOS untuk pertukaran data antartask.
    volatile uint32_t tx_queue_busy_drops;
    // Variabel tx_low_priority_drops: tingkat prioritas task atau interrupt.
    volatile uint32_t tx_low_priority_drops;
    // Variabel rx_restarts: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t rx_restarts;
} app_uartcomm_stats_t;

// Variabel huart3_vesc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern UART_HandleTypeDef huart3_vesc;
// Variabel hdma_usart3_rx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern DMA_HandleTypeDef hdma_usart3_rx;
// Variabel hdma_usart3_tx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern DMA_HandleTypeDef hdma_usart3_tx;
// Variabel g_vesc_uart_stats: state global firmware yang dibagikan antarbagian modul.
extern app_uartcomm_stats_t g_vesc_uart_stats;

// Fungsi app_uartcomm_init: menginisialisasi app uartcomm init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
bool app_uartcomm_init(void);
// Parameter byte: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi app_uartcomm_rx_get: membaca app uartcomm rx get tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
bool app_uartcomm_rx_get(uint8_t *byte);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi app_uartcomm_write_raw: menyusun atau mengirim app uartcomm write raw dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
bool app_uartcomm_write_raw(const uint8_t *data, uint16_t len);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi app_uartcomm_write_raw_low_priority: menyusun atau mengirim app uartcomm write raw low priority dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
bool app_uartcomm_write_raw_low_priority(const uint8_t *data, uint16_t len);

// Fungsi app_uartcomm_dma_rx_irq_handler: menangani app uartcomm dma rx irq handler pada konteks interrupt
// dengan pekerjaan minimum agar timing FOC tetap deterministik.
void app_uartcomm_dma_rx_irq_handler(void);
// Fungsi app_uartcomm_dma_tx_irq_handler: menangani app uartcomm dma tx irq handler pada konteks interrupt
// dengan pekerjaan minimum agar timing FOC tetap deterministik.
void app_uartcomm_dma_tx_irq_handler(void);

// Fungsi app_uartcomm_get_stats: membaca app uartcomm get stats tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
const app_uartcomm_stats_t *app_uartcomm_get_stats(void);
