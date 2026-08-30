#include "applications/app_uartcomm.h"
#include "applications/app.h"
#include "comm/packet.h"
#include "hwconf/hw_hoverboard.h"
#include "applications/appconf_default.h"
#include <string.h>
#include <stdint.h>

/*
 * VESC management transport for STM32F103RCT6 hoverboard hardware.
 *
 * This implementation deliberately follows the proven SmartESC pattern:
 *   USART3 PB10/PB11
 *   RX  -> DMA1 Channel3, CIRCULAR, task polls CNDTR write position
 *   TX  -> DMA1 Channel2, NORMAL, HAL_UART_Transmit_DMA()
 *   DMA IRQ handlers -> HAL_DMA_IRQHandler()
 *   packet parsing remains entirely in the packet_process FreeRTOS task
 *
 * Like SmartESC, USART3 itself has no required NVIC handler in the packet path.
 * DMA1 Channel2/3 IRQs feed the HAL DMA state machine; after TX DMA completion
 * the task polls the hardware TC flag and releases HAL gState. Packet parsing
 * never runs in interrupt context. This removes the previous custom DMA/ring race.
 */

UART_HandleTypeDef huart3_vesc;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

static bool s_transport_initialized = false;
static uint8_t s_rx_dma[VESC_UART_RX_DMA_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_rx_read_pos = 0U;
static volatile bool s_rx_restart_needed = false;

static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_packet_mutex;
static uint8_t s_packet_frame[VESC_UART_TX_FRAME_MAX];
app_uartcomm_stats_t g_vesc_uart_stats;
#define s_stats g_vesc_uart_stats

_Static_assert((VESC_UART_RX_DMA_SIZE & (VESC_UART_RX_DMA_SIZE - 1U)) == 0U,
               "VESC_UART_RX_DMA_SIZE must be power-of-two");
_Static_assert(VESC_UART_TX_FRAME_MAX >= VESC_PACKET_BUFFER_SIZE,
               "VESC UART TX frame buffer must fit maximum packet frame");

static uint16_t rx_dma_write_pos(void) {
    DMA_Channel_TypeDef *ch = hdma_usart3_rx.Instance;
    if (ch == NULL) return s_rx_read_pos;
    return (uint16_t)((VESC_UART_RX_DMA_SIZE - ch->CNDTR) &
                      (VESC_UART_RX_DMA_SIZE - 1U));
}

static bool restart_rx_dma_if_needed(void) {
    if (!s_rx_restart_needed) return true;

    /* Recovery is task-context only. Do not restart a HAL DMA stream from the
     * interrupt context. Discard any incomplete VESC frame and resume at index 0;
     * the canonical packet decoder will re-synchronise on the next start byte. */
    if (HAL_UART_DMAStop(&huart3_vesc) != HAL_OK) {
        s_stats.dma_errors++;
    }

    memset(s_rx_dma, 0, sizeof(s_rx_dma));
    s_rx_read_pos = 0U;
    __HAL_UART_CLEAR_OREFLAG(&huart3_vesc);

    if (HAL_UART_Receive_DMA(&huart3_vesc, s_rx_dma, VESC_UART_RX_DMA_SIZE) != HAL_OK) {
        s_stats.dma_errors++;
        s_stats.uart_errors++;
        return false;
    }

    /* SmartESC clears EIE after starting the circular RX DMA. Keep packet RX
     * DMA-driven and avoid a noisy USART error interrupt storm. TX TCIE is
     * still enabled transiently by HAL_UART_Transmit_DMA as required. */
    CLEAR_BIT(VESC_UART->CR3, USART_CR3_EIE);
    s_rx_restart_needed = false;
    s_stats.rx_restarts++;
    return true;
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart == NULL || huart->Instance != USART3) return;

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* USART3 uses the STM32F103 reset/default mapping PB10/PB11. Do not touch
     * AFIO->MAPR here: STM32F1 SWJ_CFG has unsafe readback semantics for RMW.
     * motor_hw_init() applies the complete known-safe MAPR image once while
     * explicitly preserving SWD. */

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = VESC_UART_TX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(VESC_UART_TX_PORT, &gpio);

    gpio.Pin = VESC_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VESC_UART_RX_PORT, &gpio);

    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) return;
    __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) return;
    __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

    /* Same DMA channels as SmartESC. Priority 5 is below hard-real-time FOC
     * IRQs and at the FreeRTOS syscall boundary. These handlers themselves do
     * not call FreeRTOS, but using 5 keeps the interrupt map easy to reason about. */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    /* Match SmartESC: USART3 transport is serviced by DMA1 Ch2/Ch3 only.
     * TX final-TC is polled in task context after DMA completion, therefore no
     * USART3 NVIC dependency exists and UART RX can never be disturbed by an
     * error/IDLE ISR path. */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart == NULL || huart->Instance != USART3) return;

    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    HAL_DMA_DeInit(&hdma_usart3_rx);
    HAL_DMA_DeInit(&hdma_usart3_tx);
    HAL_GPIO_DeInit(GPIOB, VESC_UART_TX_PIN | VESC_UART_RX_PIN);
    __HAL_RCC_USART3_CLK_DISABLE();
}

bool app_uartcomm_init(void) {
    if (s_transport_initialized) return true;

    memset((void *)&s_stats, 0, sizeof(s_stats));
    memset(s_rx_dma, 0, sizeof(s_rx_dma));
    memset(s_packet_frame, 0, sizeof(s_packet_frame));
    s_rx_read_pos = 0U;
    s_rx_restart_needed = false;

    s_tx_mutex = xSemaphoreCreateMutex();
    s_packet_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL || s_packet_mutex == NULL) return false;

    memset(&huart3_vesc, 0, sizeof(huart3_vesc));
    huart3_vesc.Instance = USART3;
    huart3_vesc.Init.BaudRate = VESC_UART_BAUD;
    huart3_vesc.Init.WordLength = UART_WORDLENGTH_8B;
    huart3_vesc.Init.StopBits = UART_STOPBITS_1;
    huart3_vesc.Init.Parity = UART_PARITY_NONE;
    huart3_vesc.Init.Mode = UART_MODE_TX_RX;
    huart3_vesc.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3_vesc.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3_vesc) != HAL_OK) return false;

    if (huart3_vesc.hdmarx != &hdma_usart3_rx ||
        huart3_vesc.hdmatx != &hdma_usart3_tx) {
        return false;
    }

    /* Proven SmartESC receive architecture: one circular DMA buffer and CNDTR
     * polling from the VESC packet task. No second software RX ring. */
    if (HAL_UART_Receive_DMA(&huart3_vesc, s_rx_dma, VESC_UART_RX_DMA_SIZE) != HAL_OK) {
        return false;
    }
    CLEAR_BIT(VESC_UART->CR3, USART_CR3_EIE);

    s_transport_initialized = true;
    return true;
}

bool app_uartcomm_rx_get(uint8_t *byte) {
    if (byte == NULL || !s_transport_initialized) return false;
    if (!restart_rx_dma_if_needed()) return false;

    const uint16_t write_pos = rx_dma_write_pos();
    uint16_t read_pos = s_rx_read_pos;
    if (read_pos == write_pos) return false;

    *byte = s_rx_dma[read_pos];
    read_pos = (uint16_t)((read_pos + 1U) & (VESC_UART_RX_DMA_SIZE - 1U));
    s_rx_read_pos = read_pos;
    s_stats.rx_bytes++;
    return true;
}

static bool uart_tx_wait_ready(TickType_t timeout_ticks) {
    const bool scheduler_running =
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
    const TickType_t start_tick = scheduler_running ? xTaskGetTickCount() : 0U;
    const uint32_t start_ms = scheduler_running ? 0U : HAL_GetTick();
    uint32_t timeout_ms = ((uint32_t)timeout_ticks * 1000U) /
                          (uint32_t)configTICK_RATE_HZ;
    if (timeout_ms == 0U) timeout_ms = 1U;

    for (;;) {
        /* RX DMA is permanently BUSY_RX. Never use HAL_UART_GetState() here,
         * because it combines gState and RxState. SmartESC only needs the TX
         * side to be idle before starting the next DMA frame. */
        if (huart3_vesc.gState == HAL_UART_STATE_READY &&
            HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY) {
            return true;
        }

        if (scheduler_running) {
            if ((xTaskGetTickCount() - start_tick) >= timeout_ticks) return false;
            vTaskDelay(pdMS_TO_TICKS(1U));
        } else {
            if ((HAL_GetTick() - start_ms) >= timeout_ms) return false;
            HAL_Delay(1U);
        }
    }
}

static bool uart_tx_wait_complete(TickType_t timeout_ticks) {
    const bool scheduler_running =
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
    const TickType_t start_tick = scheduler_running ? xTaskGetTickCount() : 0U;
    const uint32_t start_ms = scheduler_running ? 0U : HAL_GetTick();
    uint32_t timeout_ms = ((uint32_t)timeout_ticks * 1000U) /
                          (uint32_t)configTICK_RATE_HZ;
    if (timeout_ms == 0U) timeout_ms = 1U;

    for (;;) {
        /* HAL's DMA IRQ puts the DMA handle back to READY. In non-circular TX
         * HAL then enables UART TCIE and leaves gState BUSY_TX until USART TC.
         * SmartESC manually releases gState; here we additionally wait for the
         * real hardware TC flag so the last stop bit has left PB10. */
        if (HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY &&
            (__HAL_UART_GET_FLAG(&huart3_vesc, UART_FLAG_TC) != RESET)) {
            __HAL_UART_DISABLE_IT(&huart3_vesc, UART_IT_TC);
            huart3_vesc.gState = HAL_UART_STATE_READY;
            s_stats.tx_complete_count++;
            return true;
        }

        if (scheduler_running) {
            if ((xTaskGetTickCount() - start_tick) >= timeout_ticks) return false;
            vTaskDelay(pdMS_TO_TICKS(1U));
        } else {
            if ((HAL_GetTick() - start_ms) >= timeout_ms) return false;
            HAL_Delay(1U);
        }
    }
}

static bool app_uartcomm_write_raw_class(const uint8_t *data, uint16_t len,
                                         bool low_priority) {
    if (!s_transport_initialized || data == NULL || len == 0U ||
        len > VESC_UART_TX_FRAME_MAX || s_tx_mutex == NULL) {
        return false;
    }

    const bool scheduler_running =
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
    bool mutex_taken = false;
    if (scheduler_running) {
        TickType_t lock_wait = low_priority ? 0U : pdMS_TO_TICKS(100U);
        if (xSemaphoreTake(s_tx_mutex, lock_wait) != pdTRUE) {
            if (low_priority) s_stats.tx_low_priority_drops++;
            else s_stats.tx_queue_busy_drops++;
            return false;
        }
        mutex_taken = true;
    }
    /* Before vTaskStartScheduler() there is only one execution context using
     * this path (early-fatal recovery polling), so do not touch mutex ownership
     * at all. This keeps COMM_FW_VERSION usable even if motor init fails. */

    bool ok = false;
    if (!uart_tx_wait_ready(pdMS_TO_TICKS(100U))) {
        s_stats.uart_errors++;
        (void)HAL_UART_AbortTransmit(&huart3_vesc);
        huart3_vesc.gState = HAL_UART_STATE_READY;
    }

    if (HAL_UART_Transmit_DMA(&huart3_vesc, (uint8_t *)(uintptr_t)data, len) == HAL_OK) {
        if (uart_tx_wait_complete(pdMS_TO_TICKS(100U))) {
            s_stats.tx_bytes += len;
            ok = true;
        } else {
            s_stats.uart_errors++;
            s_stats.tx_overruns++;
            (void)HAL_UART_AbortTransmit(&huart3_vesc);
            __HAL_UART_DISABLE_IT(&huart3_vesc, UART_IT_TC);
            huart3_vesc.gState = HAL_UART_STATE_READY;
        }
    } else {
        s_stats.uart_errors++;
        s_stats.tx_overruns++;
    }

    if (mutex_taken) (void)xSemaphoreGive(s_tx_mutex);
    return ok;
}

bool app_uartcomm_write_raw(const uint8_t *data, uint16_t len) {
    return app_uartcomm_write_raw_class(data, len, false);
}

bool app_uartcomm_write_raw_low_priority(const uint8_t *data, uint16_t len) {
    return app_uartcomm_write_raw_class(data, len, true);
}

void app_uartcomm_dma_rx_irq_handler(void) {
    s_stats.rx_dma_irq_count++;
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void app_uartcomm_dma_tx_irq_handler(void) {
    s_stats.tx_dma_irq_count++;
    s_stats.tx_irq_count++;
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL || huart->Instance != USART3) return;
    s_stats.uart_errors++;
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) s_stats.rx_overruns++;
    s_rx_restart_needed = true;
}

const app_uartcomm_stats_t *app_uartcomm_get_stats(void) {
    return &s_stats;
}

/* ==================== Canonical VESC UART application API ==================== */
void app_uartcomm_initialize(void) {
    (void)app_uartcomm_init();
}

void app_uartcomm_start(UART_PORT port_number) {
    (void)port_number;
    (void)app_uartcomm_init();
}

void app_uartcomm_stop(UART_PORT port_number) {
    (void)port_number;
    /* USART3 is the permanent VESC Tool management transport. */
}

void app_uartcomm_configure(uint32_t baudrate, bool permanent_enabled,
                            UART_PORT port_number) {
    (void)baudrate;
    (void)permanent_enabled;
    (void)port_number;
    /* This board intentionally locks management UART to VESC_UART_BAUD. */
}

void app_uartcomm_send_packet(unsigned char *data, unsigned int len,
                              UART_PORT port_number) {
    (void)port_number;
    if (data == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD ||
        s_packet_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(s_packet_mutex, portMAX_DELAY) != pdTRUE) return;
    uint16_t frame_len = vesc_packet_encode(data, (uint16_t)len,
                                             s_packet_frame,
                                             (uint16_t)sizeof(s_packet_frame));
    if (frame_len != 0U) {
        (void)app_uartcomm_write_raw(s_packet_frame, frame_len);
    }
    (void)xSemaphoreGive(s_packet_mutex);
}
