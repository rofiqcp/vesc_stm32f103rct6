#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "comm/commands.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile bool s_tone_level = false;
volatile uint16_t g_vesc_buzzer_hz = 0U;
volatile uint32_t g_vesc_buzzer_remaining = 0U;
volatile uint8_t g_vesc_startup_melody_active = 0U;
volatile uint8_t g_vesc_startup_melody_index = 0U;
static volatile bool s_tone_running = false;
/* VESC-correct finite-beep support. A tone may be infinite (stopped only by an
 * explicit hw_status_tone_stop) or self-terminate after a fixed number of
 * half-cycles. The self-terminating path is what the vesc_stm32f103rct6 status-thread refactor
 * accidentally dropped when it removed the old s_tone_toggle_remaining guard:
 * without it a tone that the status task fails to stop (delayed/blocked/stopped
 * thread) keeps TIM3 toggling the pin forever, i.e. a stuck-ON buzzer. */
static volatile bool s_tone_infinite = false;
static volatile uint32_t s_tone_toggle_remaining = 0U;
static volatile bool s_melody_sequencer = false;
static volatile bool s_melody_gap = false;
static volatile uint8_t s_melody_next_index = 0U;
volatile uint8_t g_vesc_buzzer_running = 0U;
static volatile bool s_power_held = true;
static bool s_status_started = false;

typedef struct {
    uint16_t hz;
    uint16_t duration_ms;
} startup_note_t;

/* ~3 s VESC-inspired power-up chime. This is intentionally a buzzer melody,
 * not a blocking motor-beep sequence: the real VESC startup sound is normally
 * produced through the motor phases. The ascending E-major arpeggio gives the
 * same short, technical VESC-like character while keeping both bridges OFF.
 * Every tone/gap is advanced autonomously by TIM3, so it starts before the
 * FreeRTOS scheduler and never blocks UART or FOC startup. */
static const startup_note_t s_startup_notes[] = {
    {659U,  180U}, /* E5  */
    {0U,     55U},
    {831U,  180U}, /* G#5 */
    {0U,     55U},
    {988U,  210U}, /* B5  */
    {0U,     70U},
    {1319U, 260U}, /* E6  */
    {0U,    120U},
    {988U,  150U},
    {1175U, 150U}, /* D6  */
    {1319U, 180U},
    {0U,     80U},
    {1661U, 220U}, /* G#6 */
    {0U,     70U},
    {1976U, 260U}, /* B6  */
    {0U,    100U},
    {2637U, 520U}, /* E7 resolve */
    {0U,    350U}
};

/* TIM3 owns the complete startup melody so it starts before FreeRTOS and is
 * independent of timer_thread latency. Tone segments count half-cycles; silent
 * gaps run TIM3 at 1 kHz and count milliseconds. All helpers below are called
 * with interrupts already masked or from TIM3 IRQ itself. */
static void status_timer_stop_locked(void) {
    TIM3->DIER = 0U;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0U;
    s_tone_running = false;
    s_tone_infinite = false;
    s_tone_toggle_remaining = 0U;
    s_melody_gap = false;
    g_vesc_buzzer_running = 0U;
    g_vesc_buzzer_hz = 0U;
    g_vesc_buzzer_remaining = 0U;
    s_tone_level = false;
    BUZZER_PORT->BRR = BUZZER_PIN;
}

static void status_program_segment_locked(uint16_t hz, uint32_t duration_ms, bool gap) {
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->DIER = 0U;
    TIM3->SR = 0U;
    TIM3->CNT = 0U;
    s_tone_level = false;
    BUZZER_PORT->BRR = BUZZER_PIN;
    s_tone_infinite = false;
    s_melody_gap = gap;

    if (gap) {
        TIM3->ARR = 999U; /* 1 MHz / 1000 = 1 ms update */
        s_tone_toggle_remaining = duration_ms ? duration_ms : 1U;
        g_vesc_buzzer_hz = 0U;
    } else {
        if (hz < 100U) hz = 100U;
        if (hz > 5000U) hz = 5000U;
        uint32_t arr = 500000UL / (uint32_t)hz;
        if (arr == 0U) arr = 1U;
        TIM3->ARR = arr - 1U;
        uint64_t toggles = ((uint64_t)hz * 2ULL * (uint64_t)duration_ms + 999ULL) / 1000ULL;
        if (toggles == 0ULL) toggles = 1ULL;
        if (toggles > 0xFFFFFFFEULL) toggles = 0xFFFFFFFEULL;
        s_tone_toggle_remaining = (uint32_t)toggles;
        g_vesc_buzzer_hz = hz;
    }
    g_vesc_buzzer_remaining = s_tone_toggle_remaining;
    s_tone_running = true;
    g_vesc_buzzer_running = 1U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static void status_melody_load_next_locked(void) {
    if (s_melody_next_index >= (uint8_t)(sizeof(s_startup_notes) / sizeof(s_startup_notes[0]))) {
        s_melody_sequencer = false;
        g_vesc_startup_melody_active = 0U;
        status_timer_stop_locked();
        return;
    }
    const startup_note_t note = s_startup_notes[s_melody_next_index++];
    g_vesc_startup_melody_index = s_melody_next_index;
    status_program_segment_locked(note.hz, note.duration_ms, note.hz == 0U);
}

static void hw_status_startup_melody_begin(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_melody_sequencer = true;
    s_melody_next_index = 0U;
    g_vesc_startup_melody_active = 1U;
    g_vesc_startup_melody_index = 0U;
    status_melody_load_next_locked();
    if (!primask) __enable_irq();
}

void hw_status_startup_melody_replay(void) {
    hw_status_startup_melody_begin();
}

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

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* A user/fault tone pre-empts the startup melody deterministically. */
    s_melody_sequencer = false;
    g_vesc_startup_melody_active = 0U;
    s_melody_gap = false;

    if (duration_ms == 0U) {
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->DIER = 0U;
        uint32_t arr = 500000UL / (uint32_t)hz;
        if (arr == 0U) arr = 1U;
        TIM3->ARR = arr - 1U; TIM3->CNT = 0U; TIM3->EGR = TIM_EGR_UG; TIM3->SR = 0U;
        s_tone_infinite = true;
        s_tone_toggle_remaining = 0U;
        s_tone_level = false;
        s_tone_running = true;
        g_vesc_buzzer_running = 1U;
        g_vesc_buzzer_hz = hz;
        g_vesc_buzzer_remaining = 0U;
        BUZZER_PORT->BRR = BUZZER_PIN;
        TIM3->DIER = TIM_DIER_UIE; TIM3->CR1 |= TIM_CR1_CEN;
    } else {
        status_program_segment_locked(hz, duration_ms, false);
    }
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
    s_melody_sequencer = false;
    g_vesc_startup_melody_active = 0U;
    status_timer_stop_locked();
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

    if (s_tone_infinite) {
        s_tone_level = !s_tone_level;
        if (s_tone_level) BUZZER_PORT->BSRR = BUZZER_PIN; else BUZZER_PORT->BRR = BUZZER_PIN;
        return;
    }

    if (s_tone_toggle_remaining > 0U) s_tone_toggle_remaining--;
    g_vesc_buzzer_remaining = s_tone_toggle_remaining;
    if (!s_melody_gap) {
        s_tone_level = !s_tone_level;
        if (s_tone_level) BUZZER_PORT->BSRR = BUZZER_PIN; else BUZZER_PORT->BRR = BUZZER_PIN;
    } else {
        s_tone_level = false;
        BUZZER_PORT->BRR = BUZZER_PIN;
    }

    if (s_tone_toggle_remaining == 0U) {
        if (s_melody_sequencer) {
            status_melody_load_next_locked();
        } else {
            status_timer_stop_locked();
        }
    }
}

void hw_status_service_10ms(uint32_t now) {
    /* Status/LED/buzzer is intentionally NOT a sixth task. Its non-blocking
     * state machine is serviced by VESC timer_thread every 10 ms. */
    static bool initialized = false;
    static uint32_t fault_deadline;
    static uint8_t led_state;       /* 0=burst / 1=gap */
    static uint8_t led_pulses_left;
    static bool led_output_on;
    static uint32_t led_deadline;
    static bool hb_led_on;
    static uint32_t hb_deadline;
    static uint8_t led_mode;        /* 0=heartbeat, 1=cal, 2=detect, 3=run */
    static motor_fault_t announced;
    static uint8_t fault_tens;
    static uint8_t fault_ones;
    static uint8_t fault_group;
    static uint8_t fault_pulses_left;
    static uint8_t fault_stage;
    static bool fault_output_on;

    if (!initialized) {
        fault_deadline = now;
        led_state = 0U;
        led_pulses_left = 0U;
        led_output_on = false;
        led_deadline = now;
        hb_led_on = false;
        hb_deadline = now + 500U;
        led_mode = 0xFFU;          /* force first-mode initialization */
        motor_hw_led(false);
        announced = MOTOR_FAULT_NONE;
        fault_tens = fault_ones = fault_group = 0U;
        fault_pulses_left = fault_stage = 0U;
        fault_output_on = false;
        initialized = true;
    }

    const motor_fault_t fault = highest_priority_fault();

    if (fault != MOTOR_FAULT_NONE && g_vesc_startup_melody_active) {
        hw_status_tone_stop();
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
            hb_led_on = false;
            hb_deadline = now + 500U;
            led_mode = 0xFFU;
            motor_hw_led(false);
        }
        announced = MOTOR_FAULT_NONE;
        fault_stage = 0U;

        /* LED status follows the proven SmartESC idea: a healthy controller
         * must always have an obvious periodic heartbeat. Additional burst modes
         * are retained for calibration/detection/running, but every transition
         * is explicitly re-initialized so the LED cannot inherit a stale pulse
         * counter/deadline from the previous mode.
         *
         *   calibrating -> 1 pulse burst (200 ms on/off), 1 s gap
         *   detecting   -> 2 pulse burst, 1 s gap
         *   running     -> 3 pulse burst, 1 s gap
         *   idle/ready  -> 500 ms toggle heartbeat (SmartESC-like normal blink)
         */
        const bool calibrating = !foc_calibration_done();
        const bool detecting = g_motor_left.detect.busy ||
                g_motor_right.detect.busy;
        const bool running = g_motor_left.pwm_enabled ||
                g_motor_right.pwm_enabled;
        const uint8_t mode = calibrating ? 1U :
                             (detecting ? 2U :
                             (running ? 3U : 0U));
        const uint8_t pulses = mode;

        if (mode != led_mode) {
            led_mode = mode;
            led_state = 0U;
            led_pulses_left = 0U;
            led_output_on = false;
            led_deadline = now;
            hb_led_on = false;
            hb_deadline = now + 500U;
            motor_hw_led(false);
        }

        if (pulses != 0U) {
            if (led_state == 0U) {
                if (led_pulses_left == 0U) {
                    led_pulses_left = pulses;
                    led_output_on = true;
                    motor_hw_led(true);
                    led_deadline = now + 200U;
                } else if ((int32_t)(now - led_deadline) >= 0) {
                    led_output_on = !led_output_on;
                    motor_hw_led(led_output_on);
                    if (!led_output_on) {
                        led_pulses_left--;
                        if (led_pulses_left == 0U) {
                            led_state = 1U;
                            led_deadline = now + 1000U;
                        } else {
                            led_deadline = now + 200U;
                        }
                    } else {
                        led_deadline = now + 200U;
                    }
                }
            } else {
                motor_hw_led(false);
                if ((int32_t)(now - led_deadline) >= 0) {
                    led_state = 0U;
                    led_pulses_left = 0U;
                    led_output_on = false;
                }
            }
        } else if ((int32_t)(now - hb_deadline) >= 0) {
            /* SmartESC normal path toggles its LED roughly every 0.5 s. Use an
             * absolute deadline rather than an 8-bit divider so delayed timer
             * service cannot permanently distort the blink cadence. */
            hb_led_on = !hb_led_on;
            motor_hw_led(hb_led_on);
            hb_deadline = now + 500U;
        }

    }

}

bool hw_status_init(void) {
    s_status_started = true;
    /* Start the complete 3.21 s power-on melody before FreeRTOS. TIM3 advances
     * notes/gaps autonomously, so UART/FOC startup proceeds concurrently and the
     * melody cannot disappear merely because timer_thread has not run yet. */
    hw_status_startup_melody_begin();
    return true;
}

static void early_fatal_delay_with_comm(uint32_t delay_ms) {
    const uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < delay_ms) {
        /* If USART3/commands were already brought up, preserve VESC Tool
         * recovery access even though the motor subsystem is in early-fatal. */
        if (commands_is_initialized()) {
            (void)vesc_comm_poll_once();
        }
        HAL_Delay(1U);
    }
}

void hw_status_early_fatal_loop(void) {
    hw_status_tone_stop();
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    for (;;) {
        /* 4 rapid LED flashes + buzzer indicate a pre-scheduler failure while
         * the management UART remains serviceable when it was initialized. */
        for (uint8_t i = 0U; i < 4U; i++) {
            motor_hw_led(true);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            early_fatal_delay_with_comm(80U);
            motor_hw_led(false);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            early_fatal_delay_with_comm(80U);
        }
        early_fatal_delay_with_comm(800U);
    }
}
