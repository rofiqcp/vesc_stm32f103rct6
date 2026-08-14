#include "status_io.h"
#include "board_pins.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"

static volatile bool s_tone_level = false;
static volatile bool s_tone_running = false;
static volatile uint32_t s_last_vesc_packet_tick = 0U;
static volatile bool s_seen_vesc_packet = false;

void status_io_early_gpio_init(void) {
    /* This executes immediately after HAL_Init, while the MCU is still on the
     * reset HSI clock. It therefore proves execution even if the later 64 MHz
     * PLL or motor subsystem fails. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &g);

    g.Pin = BUZZER_PIN | OFF_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    /* Proven hoverboard board behavior: PA5 HIGH holds the controller powered.
     * It is a power-latch/OFF control, not the TIM1/TIM8 MOE gate-enable. */
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void status_io_tone_timer_init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1 = 0U;
    TIM3->DIER = 0U;
    TIM3->SR = 0U;
    /* At the required 64 MHz hoverboard clock, APB1=/2 but timer clock is
     * doubled back to 64 MHz. PSC=63 produces a 1 MHz timer counter. */
    TIM3->PSC = 63U;
    TIM3->ARR = 499U;
    TIM3->CNT = 0U;
    TIM3->EGR = TIM_EGR_UG;
    HAL_NVIC_SetPriority(TIM3_IRQn, 8U, 0U);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void status_io_led(bool on) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void status_io_tone_start(uint16_t hz) {
    if (hz < 100U) hz = 100U;
    if (hz > 5000U) hz = 5000U;

    uint32_t arr = 500000UL / (uint32_t)hz;
    if (arr == 0U) arr = 1U;
    arr -= 1U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->ARR = arr;
    TIM3->CNT = 0U;
    TIM3->SR = 0U;
    TIM3->DIER = TIM_DIER_UIE;
    s_tone_level = false;
    s_tone_running = true;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    TIM3->CR1 |= TIM_CR1_CEN;
    if (!primask) __enable_irq();
}

void status_io_tone_stop(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM3->DIER = 0U;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0U;
    s_tone_running = false;
    s_tone_level = false;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    if (!primask) __enable_irq();
}

void status_io_tim3_irq_handler(void) {
    if ((TIM3->SR & TIM_SR_UIF) == 0U) return;
    TIM3->SR &= ~TIM_SR_UIF;
    if (!s_tone_running) return;
    s_tone_level = !s_tone_level;
    if (s_tone_level) {
        BUZZER_PORT->BSRR = BUZZER_PIN;
    } else {
        BUZZER_PORT->BRR = BUZZER_PIN;
    }
}

void status_io_note_vesc_packet(void) {
    s_last_vesc_packet_tick = osKernelGetTickCount();
    s_seen_vesc_packet = true;
}

bool status_io_vesc_link_recent(uint32_t age_ms) {
    if (!s_seen_vesc_packet) return false;
    return (uint32_t)(osKernelGetTickCount() - s_last_vesc_packet_tick) <= age_ms;
}

void status_io_early_fatal_loop(void) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    for (;;) {
        /* 4 rapid LED flashes + a fixed buzzer level indicate failure before
         * the RTOS status threads could start. */
        for (uint8_t i = 0U; i < 4U; i++) {
            status_io_led(true);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            HAL_Delay(80U);
            status_io_led(false);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            HAL_Delay(80U);
        }
        HAL_Delay(800U);
    }
}
