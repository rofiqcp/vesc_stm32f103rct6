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
/* VESC-correct finite-beep support. A tone may be infinite (stopped only by an
 * explicit hw_status_tone_stop) or self-terminate after a fixed number of
 * half-cycles. The self-terminating path is what the vesc_stm32f103rct6 status-thread refactor
 * accidentally dropped when it removed the old s_tone_toggle_remaining guard:
 * without it a tone that the status task fails to stop (delayed/blocked/stopped
 * thread) keeps TIM3 toggling the pin forever, i.e. a stuck-ON buzzer. */
static volatile bool s_tone_infinite = false;
static volatile uint32_t s_tone_toggle_remaining = 0U;
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

void status_thread(void *argument);

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
    /* Infinite tone: only an explicit hw_status_tone_stop ends it. Retained for
     * VESC API parity; the fault/startup sequences below prefer the bounded
     * hw_status_tone_start_for() so the buzzer can never be left stuck ON. */
    hw_status_tone_start_for(hz, 0U);
}

void hw_status_tone_start_for(uint16_t hz, uint32_t duration_ms) {
    if (hz < 100U) hz = 100U;
    if (hz > 5000U) hz = 5000U;

    /* hw_status_timer_init() sets PSC=63 on a 64 MHz APB1 (timer clock is
     * doubled back to 64 MHz), so the counter runs at 1 MHz. ARR is therefore
     * 1,000,000/hz counts per full period; the ISR toggles the pin each
     * update event, giving the requested audio frequency. */
    uint32_t arr = 1000000UL / (uint32_t)hz;
    if (arr == 0U) arr = 1U;
    arr -= 1U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_tone_infinite = false;
    s_tone_toggle_remaining = 0U;
    if (duration_ms == 0U) {
        s_tone_infinite = true;          /* unbounded until tone_stop() */
    } else {
        /* Count half-cycles so the tone runs for exactly duration_ms. */
        uint64_t toggles = ((uint64_t)hz * 2ULL * (uint64_t)duration_ms +
                            999ULL) / 1000ULL;
        if (toggles > 0xFFFFFFFEULL) toggles = 0xFFFFFFFEULL;
        s_tone_toggle_remaining = (uint32_t)toggles;
        if (s_tone_toggle_remaining == 0U) {
            /* Duration too short to fit one half-cycle: stay silent. */
            __enable_irq();
            hw_status_tone_stop();
            return;
        }
    }

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
    s_tone_infinite = false;
    s_tone_toggle_remaining = 0U;
    s_tone_level = false;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    if (!primask) __enable_irq();
}

bool hw_status_tone_is_running(void) {
    return s_tone_running;
}

bool hw_status_tone_level(void) {
    return s_tone_level;
}

void hw_status_tim3_irq_handler(void) {
    if ((TIM3->SR & TIM_SR_UIF) == 0U) return;
    TIM3->SR &= ~TIM_SR_UIF;
    if (!s_tone_running) return;

    if (!s_tone_infinite) {
        if (s_tone_toggle_remaining == 0U) {
            /* Bounded tone finished: silence the pin and halt the timer so the
             * buzzer can never be left stuck ON even if the status task that
             * scheduled it is delayed or stops. */
            TIM3->DIER = 0U;
            TIM3->CR1 &= ~TIM_CR1_CEN;
            s_tone_running = false;
            s_tone_level = false;
            BUZZER_PORT->BRR = BUZZER_PIN;
            return;
        }
        s_tone_toggle_remaining--;
    }

    s_tone_level = !s_tone_level;
    if (s_tone_level) {
        BUZZER_PORT->BSRR = BUZZER_PIN;
    } else {
        BUZZER_PORT->BRR = BUZZER_PIN;
    }
}

void status_thread(void *argument) {
    (void)argument;

    uint32_t next = osKernelGetTickCount();
    uint32_t startup_deadline = next;
    uint32_t fault_deadline = next;
    uint8_t led_state = 0U;       /* 0=burst / 1=gap */
    uint8_t led_pulses_left = 0U;
    bool led_output_on = false;
    uint32_t led_deadline = 0U;
    bool hb_led_on = true;
    uint8_t hb_divider = 0U;
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
                hw_status_tone_start_for(fault_group == 0U ? 1500U : 2400U, 100U);
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
                    hw_status_tone_start_for(2400U, 100U);
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
                /* Reset mesin mode-cue (burst/gap) biar LED gak nyangkut di
                 * tengah pulsa saat keluar dari fault. */
                led_state = 0U;
                led_pulses_left = 0U;
                led_output_on = false;
                hb_divider = 0U;
            }
            announced = MOTOR_FAULT_NONE;
            fault_stage = 0U;

            /* LED mode cue (plain GPIO flash, bukan PWM). Setiap 1 detik
             * keluarkan N pulsa: nyala 200 ms / mati 200 ms, lalu mati 1 detik.
             *   calibrating -> 1 pulsa
             *   detecting   -> 2 pulsa
             *   running     -> 3 pulsa
             * Di luar mode itu -> heartbeat 1 Hz (1 s nyala / 1 s mati). */
            const bool calibrating = !foc_calibration_done();
            const bool detecting = g_motor_left.detect.busy ||
                    g_motor_right.detect.busy;
            const bool running = g_motor_left.pwm_enabled ||
                    g_motor_right.pwm_enabled;
            const uint8_t pulses = calibrating ? 1U :
                                   (detecting ? 2U :
                                   (running ? 3U : 0U));

            if (pulses != 0U) {
                if (led_state == 0U) {          /* burst */
                    if (led_pulses_left == 0U) {
                        led_pulses_left = pulses;
                        led_output_on = true;
                        motor_hw_led(true);
                        led_deadline = now + 200U;
                    } else if ((int32_t)(now - led_deadline) >= 0) {
                        led_output_on = !led_output_on;
                        motor_hw_led(led_output_on);
                        if (!led_output_on) {   /* barusan mati */
                            led_pulses_left--;
                            if (led_pulses_left == 0U) {
                                led_state = 1U; /* masuk gap 1 detik */
                                led_deadline = now + 1000U;
                            } else {
                                led_deadline = now + 200U;
                            }
                        } else {
                            led_deadline = now + 200U;
                        }
                    }
                } else {                        /* gap */
                    motor_hw_led(false);
                    if ((int32_t)(now - led_deadline) >= 0) {
                        led_state = 0U;
                        led_pulses_left = 0U;
                        led_output_on = false;
                    }
                }
            } else {
                /* Heartbeat 1 Hz (1 detik nyala / 1 detik mati). */
                hb_divider++;
                if (hb_divider >= 100U) {       /* 100 * 10 ms = 1 s */
                    hb_divider = 0U;
                    hb_led_on = !hb_led_on;
                    motor_hw_led(hb_led_on);
                }
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
                        /* Each note self-terminates after its duration, so the
                         * melody cannot leave the buzzer stuck ON if the thread
                         * is late reaching the next scheduler tick. */
                        hw_status_tone_start_for(note->hz, note->duration_ms);
                    }
                    startup_deadline = now + note->duration_ms;
                }
            }
        }

        osDelayUntil(next);
    }
}

void hw_status_set_thread_id(osThreadId_t id) {
    s_status_thread = id;
}

bool hw_status_init(void) {
    /* The StatusIO thread is spawned centrally in main.c via osThreadNew. This
     * function only validates that the handle was registered and guards against
     * a repeated call. */
    if (s_status_started) {
        return s_status_thread != NULL;
    }

    s_status_started = true;
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
