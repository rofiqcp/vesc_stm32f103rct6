#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile bool s_tone_level = false;
static volatile bool s_tone_running = false;
static volatile bool s_power_held = true;
static osThreadId_t s_status_thread;
static bool s_status_started = false;

typedef struct {
    uint16_t hz;
    uint16_t duration_ms;
} startup_note_t;

static const startup_note_t s_startup_notes[] = {
    {900U, 90U},
    {0U, 50U},
    {1350U, 90U},
    {0U, 50U},
    {1900U, 140U}
};

/* Highest-priority fault present on either bridge owns both indicators. */
static const motor_fault_t s_fault_priority[] = {
    MOTOR_FAULT_FLASH_CONFIG,
    MOTOR_FAULT_BREAK,
    MOTOR_FAULT_MCU_UNDER_VOLTAGE,
    MOTOR_FAULT_ABS_OVER_CURRENT,
    MOTOR_FAULT_ADC_DMA,
    MOTOR_FAULT_FOC_ISR_OVERRUN,
    MOTOR_FAULT_OVER_VOLTAGE,
    MOTOR_FAULT_UNDER_VOLTAGE,
    MOTOR_FAULT_CURRENT_OFFSET,
    MOTOR_FAULT_OVER_TEMP_BOARD,
    MOTOR_FAULT_OVER_TEMP_MOTOR,
    MOTOR_FAULT_OVERSPEED,
    MOTOR_FAULT_UNDERSPEED,
    MOTOR_FAULT_ABS_OVERSPEED,
    MOTOR_FAULT_ENCODER_SLIP,
    MOTOR_FAULT_HALL_INVALID,
    MOTOR_FAULT_SENSOR_DETECT,
    MOTOR_FAULT_SENSORLESS_OBSERVER
};

static void status_thread(void *argument);

static motor_fault_t highest_priority_fault(void) {
    const motor_fault_t left = g_motor_left.fault;
    const motor_fault_t right = g_motor_right.fault;

    for (uint32_t i = 0U;
            i < (uint32_t)(sizeof(s_fault_priority) / sizeof(s_fault_priority[0]));
            i++) {
        if (left == s_fault_priority[i] || right == s_fault_priority[i]) {
            return s_fault_priority[i];
        }
    }
    return MOTOR_FAULT_NONE;
}

static uint8_t fault_digit_pulses(uint8_t digit) {
    return digit == 0U ? 10U : digit;
}

void hw_status_early_init(void) {
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
    s_power_held = true;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

void hw_status_timer_init(void) {
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

void hw_status_tone_start(uint16_t hz) {
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

void hw_status_power_hold(bool on) {
    s_power_held = on;
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool hw_status_power_is_held(void) {
    return s_power_held;
}

void hw_status_tone_stop(void) {
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

void hw_status_tim3_irq_handler(void) {
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

static void status_thread(void *argument) {
    (void)argument;

    uint32_t next = osKernelGetTickCount();
    uint32_t startup_deadline = next;
    uint32_t fault_deadline = next;
    uint8_t led_phase = 0U;
    uint8_t led_divider = 0U;
    uint8_t startup_index = 0U;
    bool startup_done = false;

    motor_fault_t announced = MOTOR_FAULT_NONE;
    uint8_t fault_tens = 0U;
    uint8_t fault_ones = 0U;
    uint8_t fault_group = 0U;
    uint8_t fault_pulses_left = 0U;
    uint8_t fault_stage = 0U;
    bool fault_output_on = false;

    for (;;) {
        next += 10U;
        const uint32_t now = osKernelGetTickCount();
        const motor_fault_t fault = highest_priority_fault();

        if (fault != MOTOR_FAULT_NONE && !startup_done) {
            hw_status_tone_stop();
            startup_done = true;
        }

        if (fault != MOTOR_FAULT_NONE) {
            if (fault != announced) {
                announced = fault;
                const uint8_t code = (uint8_t)motor_fault_to_vesc(fault);
                fault_tens = (uint8_t)(code / 10U);
                fault_ones = (uint8_t)(code % 10U);
                fault_group = fault_tens != 0U ? 0U : 1U;
                const uint8_t digit = fault_group == 0U ? fault_tens : fault_ones;
                fault_pulses_left = fault_digit_pulses(digit);
                fault_stage = 1U;
                fault_output_on = true;
                motor_hw_led(true);
                hw_status_tone_start(fault_group == 0U ? 1500U : 2400U);
                fault_deadline = now + 100U;
            } else if ((int32_t)(now - fault_deadline) >= 0) {
                if (fault_stage == 1U) {
                    if (fault_output_on) {
                        motor_hw_led(false);
                        hw_status_tone_stop();
                        fault_output_on = false;
                        if (fault_pulses_left > 0U) {
                            fault_pulses_left--;
                        }
                        if (fault_pulses_left == 0U) {
                            if (fault_group == 0U) {
                                fault_stage = 2U;
                                fault_deadline = now + 350U;
                            } else {
                                fault_stage = 3U;
                                fault_deadline = now + 1000U;
                            }
                        } else {
                            fault_deadline = now + 100U;
                        }
                    } else {
                        fault_output_on = true;
                        motor_hw_led(true);
                        hw_status_tone_start(fault_group == 0U ? 1500U : 2400U);
                        fault_deadline = now + 100U;
                    }
                } else if (fault_stage == 2U) {
                    fault_group = 1U;
                    fault_pulses_left = fault_digit_pulses(fault_ones);
                    fault_stage = 1U;
                    fault_output_on = true;
                    motor_hw_led(true);
                    hw_status_tone_start(2400U);
                    fault_deadline = now + 100U;
                } else {
                    fault_group = fault_tens != 0U ? 0U : 1U;
                    const uint8_t digit = fault_group == 0U ? fault_tens : fault_ones;
                    fault_pulses_left = fault_digit_pulses(digit);
                    fault_stage = 1U;
                    fault_output_on = true;
                    motor_hw_led(true);
                    hw_status_tone_start(fault_group == 0U ? 1500U : 2400U);
                    fault_deadline = now + 100U;
                }
            }
        } else {
            if (announced != MOTOR_FAULT_NONE || fault_output_on) {
                hw_status_tone_stop();
                fault_output_on = false;
            }
            announced = MOTOR_FAULT_NONE;
            fault_stage = 0U;

            led_divider++;
            if (led_divider >= 10U) {
                led_divider = 0U;
                const bool calibrating = !foc_calibration_done();
                const bool detecting = g_motor_left.detect.busy ||
                        g_motor_right.detect.busy;
                const bool running = g_motor_left.pwm_enabled ||
                        g_motor_right.pwm_enabled;
                bool led_on;

                if (calibrating) {
                    led_on = (led_phase & 1U) == 0U;
                } else if (detecting) {
                    led_on = led_phase == 0U || led_phase == 2U;
                } else if (running) {
                    led_on = led_phase < 2U ||
                            (led_phase >= 5U && led_phase < 7U);
                } else {
                    led_on = led_phase < 5U;
                }
                motor_hw_led(led_on);
                led_phase = (uint8_t)((led_phase + 1U) % 10U);
            }

            if (!startup_done && (int32_t)(now - startup_deadline) >= 0) {
                if (startup_index >=
                        (uint8_t)(sizeof(s_startup_notes) / sizeof(s_startup_notes[0]))) {
                    hw_status_tone_stop();
                    startup_done = true;
                } else {
                    const startup_note_t *note = &s_startup_notes[startup_index++];
                    if (note->hz == 0U) {
                        hw_status_tone_stop();
                    } else {
                        hw_status_tone_start(note->hz);
                    }
                    startup_deadline = now + note->duration_ms;
                }
            }
        }

        osDelayUntil(next);
    }
}

bool hw_status_init(void) {
    if (s_status_started) {
        return s_status_thread != NULL;
    }

    s_status_started = true;
    const osThreadAttr_t attributes = {
        .name = "StatusIO",
        .priority = osPriorityNormal,
        .stack_size = 512U
    };
    s_status_thread = osThreadNew(status_thread, NULL, &attributes);
    return s_status_thread != NULL;
}

uint32_t hw_status_stack_free_bytes(void) {
    if (s_status_thread == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)s_status_thread) *
            (uint32_t)sizeof(StackType_t);
}

void hw_status_early_fatal_loop(void) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    for (;;) {
        /* 4 rapid LED flashes + a fixed buzzer level indicate failure before
         * the RTOS status threads could start. */
        for (uint8_t i = 0U; i < 4U; i++) {
            motor_hw_led(true);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            HAL_Delay(80U);
            motor_hw_led(false);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            HAL_Delay(80U);
        }
        HAL_Delay(800U);
    }
}
