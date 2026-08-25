#include "motor_tasks.h"
#include "motor/mc_interface.h"
#include "hwconf/hw.h"
#include "motor/mcpwm_foc.h"
#include "debug_sample.h"
#include "telemetry.h"
#include "comm/commands.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "applications/appconf_default.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "timeout.h"
#include "status_io.h"
#include <stddef.h>

/* CMSIS-RTOS2 equivalents of the requested VESC-style threads.
 * The names deliberately match the upstream semantic thread names.
 * Fast ADC->FOC->SVPWM stays in DMA1_Channel1_IRQHandler(), not here. */
static osThreadId_t timer_thread_tp;
static osThreadId_t sample_send_tp;
static osThreadId_t fault_stop_tp;
static osThreadId_t status_thread_tp;
static bool status_threads_started = false;

/* Refuse to advertise a motor-ready controller if task creation consumes
 * nearly all heap. This reserve is runtime headroom for RTOS queue/mutex
 * operations and future transient control-path allocations. */
#define RTOS_READY_HEAP_RESERVE_BYTES 2048U

uint32_t motor_threads_free_heap_bytes(void) {
    return (uint32_t)xPortGetFreeHeapSize();
}

uint32_t motor_threads_min_ever_free_heap_bytes(void) {
    return (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

static uint32_t thread_stack_free_bytes(osThreadId_t id) {
    if (id == NULL) return 0U;
    return (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)id) * (uint32_t)sizeof(StackType_t);
}

void motor_threads_get_resource_stats(motor_runtime_resource_stats_t *out) {
    if (out == NULL) return;
    out->heap_free_bytes = motor_threads_free_heap_bytes();
    out->heap_min_ever_bytes = motor_threads_min_ever_free_heap_bytes();
    out->motor_service_stack_free_bytes = thread_stack_free_bytes(timer_thread_tp);
    out->sample_sender_stack_free_bytes = thread_stack_free_bytes(sample_send_tp);
    out->fault_stack_free_bytes = thread_stack_free_bytes(fault_stop_tp);
    out->status_stack_free_bytes = thread_stack_free_bytes(status_thread_tp);
}

static void timer_thread(void *arg);
static void sample_send_thread(void *arg);
static void fault_stop_thread(void *arg);
static void status_thread(void *arg);

void motor_threads_fault_signal(motor_id_t id) {
    if (fault_stop_tp != NULL) {
        (void)osThreadFlagsSet(fault_stop_tp, 1UL << (uint32_t)id);
    }
}

void motor_threads_sample_signal(void) {
    if (sample_send_tp != NULL) {
        (void)osThreadFlagsSet(sample_send_tp, 1UL);
    }
}

/* VESC-style 1 kHz housekeeping. */
static void timer_thread(void *arg) {
    (void)arg;
    bool current_offset_fault_reported = false;
    uint8_t cal_div = 0U;
    uint8_t ten_ms_div = 0U;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 1U;
        uint32_t now = osKernelGetTickCount();
        timeout_heartbeat(TIMEOUT_HEARTBEAT_MOTOR_SERVICE);

        motor_slow_update_1khz(&g_motor_left, now);
        motor_slow_update_1khz(&g_motor_right, now);
        motor_rpm_update_1khz(&g_motor_left);
        motor_rpm_update_1khz(&g_motor_right);

        /* Command source ownership and PA2/PA3 application processing live in
         * this existing 1-kHz service. No APP ADC RTOS thread/stack is added. */
        app_command_service_1khz(now);
        app_adc_service_1khz(now);

        /* Outer position/speed/duty/current-target control follows the fresh
           slow snapshot in the same deterministic 1-kHz service context. */
        motor_pid_update_1khz(&g_motor_left);
        motor_pid_update_1khz(&g_motor_right);

        ten_ms_div++;
        if (ten_ms_div >= 10U) {
            ten_ms_div = 0U;
            timeout_update_10ms(now);
            timeout_watchdog_update_10ms(now);
            telemetry_stats_update_100hz();
            telemetry_snapshot_100hz();
            vesc_comm_periodic_100hz();
        }

        /* Kalibrasi offset tidak membutuhkan stack/thread sendiri. ISR ADC hanya
         * mengakumulasi sampel fixed-point; perubahan state/MOE tetap dilakukan
         * di task context setiap 5 ms. */
        cal_div++;
        if (cal_div >= 5U) {
            cal_div = 0U;
            foc_calibration_service_task();
            if (foc_calibration_done()) {
                if (!foc_calibration_valid() && !current_offset_fault_reported) {
                    current_offset_fault_reported = true;
                    motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_CURRENT_OFFSET);
                    motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_CURRENT_OFFSET);
                    motor_threads_fault_signal(MOTOR_LEFT);
                    motor_threads_fault_signal(MOTOR_RIGHT);
                } else if (foc_calibration_valid()) {
                    current_offset_fault_reported = false;
                    if (g_motor_left.fault == MOTOR_FAULT_CURRENT_OFFSET) motor_clear_fault(&g_motor_left);
                    if (g_motor_right.fault == MOTOR_FAULT_CURRENT_OFFSET) motor_clear_fault(&g_motor_right);
                }
            }
        }

        uint32_t pending = motor_take_pending_fault_mask();
        if ((pending & (1UL << MOTOR_LEFT)) != 0U) motor_threads_fault_signal(MOTOR_LEFT);
        if ((pending & (1UL << MOTOR_RIGHT)) != 0U) motor_threads_fault_signal(MOTOR_RIGHT);

        if (debug_sample_ready()) motor_threads_sample_signal();
        if (!g_motor_left.pwm_enabled && !g_motor_right.pwm_enabled) {
            motor_hw_gate_global(false);
        }
        osDelayUntil(next);
    }
}

/* Oscilloscope/sample sender. The ISR only writes RAM and raises an event. */
static void sample_send_thread(void *arg) {
    (void)arg;
    for (;;) {
        uint32_t f = osThreadFlagsWait(1UL, osFlagsWaitAny, osWaitForever);
        if ((f & osFlagsError) != 0U) continue;
        if (debug_sample_ready()) {
            vesc_comm_send_sample_buffer(debug_sample_data(), debug_sample_count());
            debug_sample_mark_sent();
        }
    }
}

/* Software side of fault handling. Hardware MOE is already dropped from the
 * fault/FOC path before this thread runs. Audible reporting is owned by the
 * non-blocking status_thread so fault handling itself never blocks on beeps. */
static void fault_stop_thread(void *arg) {
    (void)arg;
    const uint32_t mask = (1UL << MOTOR_LEFT) | (1UL << MOTOR_RIGHT);
    for (;;) {
        /* A finite wait makes the dedicated fault manager observable by the
           hardware watchdog even when no fault event is pending. */
        uint32_t f = osThreadFlagsWait(mask, osFlagsWaitAny, 50U);
        timeout_heartbeat(TIMEOUT_HEARTBEAT_FAULT);
        if ((f & osFlagsError) != 0U) continue;
        if ((f & (1UL << MOTOR_LEFT)) != 0U) motor_hw_set_pwm_enabled(&g_motor_left, false);
        if ((f & (1UL << MOTOR_RIGHT)) != 0U) motor_hw_set_pwm_enabled(&g_motor_right, false);
    }
}

/* PB2 + buzzer memakai satu status thread non-blocking. Tone TIM3 tetap
 * dibangkitkan oleh ISR timer; status thread hanya mengganti frekuensi/state.
 * Tidak ada osDelay panjang di fault path, sehingga LED tetap responsif. */
static motor_fault_t audible_fault(void) {
    motor_fault_t a = g_motor_left.fault;
    motor_fault_t b = g_motor_right.fault;
    const motor_fault_t order[] = {
        MOTOR_FAULT_FLASH_CONFIG, MOTOR_FAULT_BREAK, MOTOR_FAULT_MCU_UNDER_VOLTAGE,
        MOTOR_FAULT_ABS_OVER_CURRENT, MOTOR_FAULT_ADC_DMA,
        MOTOR_FAULT_FOC_ISR_OVERRUN, MOTOR_FAULT_OVER_VOLTAGE,
        MOTOR_FAULT_UNDER_VOLTAGE, MOTOR_FAULT_CURRENT_OFFSET,
        MOTOR_FAULT_OVER_TEMP_BOARD, MOTOR_FAULT_OVER_TEMP_MOTOR,
        MOTOR_FAULT_OVERSPEED, MOTOR_FAULT_UNDERSPEED, MOTOR_FAULT_ABS_OVERSPEED,
        MOTOR_FAULT_ENCODER_SLIP, MOTOR_FAULT_HALL_INVALID,
        MOTOR_FAULT_SENSOR_DETECT, MOTOR_FAULT_SENSORLESS_OBSERVER
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(order) / sizeof(order[0])); i++) {
        if (a == order[i] || b == order[i]) return order[i];
    }
    return MOTOR_FAULT_NONE;
}

/* LED and buzzer report the actual decimal VESC fault number with the same
 * pulse groups, so there is one canonical code instead of two independent
 * switch tables that can drift apart. Example FAULT 24 = 2 pulses, group gap,
 * 4 pulses, long repeat gap. Digit zero is represented by ten pulses. */
static uint8_t fault_digit_pulses(uint8_t digit) { return digit == 0U ? 10U : digit; }

typedef struct {
    uint16_t hz;
    uint16_t duration_ms;
} startup_note_t;

static const startup_note_t s_startup_notes[] = {
    {900U, 90U}, {0U, 50U}, {1350U, 90U}, {0U, 50U}, {1900U, 140U}
};

static void status_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    uint32_t startup_deadline = next;
    uint32_t fault_deadline = next;
    uint8_t led_phase = 0U;
    uint8_t led_div = 0U;
    uint8_t startup_index = 0U;
    bool startup_done = false;

    motor_fault_t announced = MOTOR_FAULT_NONE;
    uint8_t fault_tens = 0U, fault_ones = 0U;
    uint8_t fault_group = 0U;      /* 0=tens, 1=ones */
    uint8_t fault_pulses_left = 0U;
    uint8_t fault_stage = 0U;      /* 0=idle, 1=pulsing, 2=group gap, 3=cycle gap */
    bool fault_output_on = false;

    for (;;) {
        next += 10U;
        uint32_t now = osKernelGetTickCount();
        motor_fault_t f = audible_fault();

        /* A real fault preempts the cosmetic boot melody immediately. */
        if (f != MOTOR_FAULT_NONE && !startup_done) {
            status_io_tone_stop();
            startup_done = true;
        }

        if (f != MOTOR_FAULT_NONE) {
            if (f != announced) {
                announced = f;
                uint8_t code = (uint8_t)motor_fault_to_vesc(f);
                fault_tens = (uint8_t)(code / 10U);
                fault_ones = (uint8_t)(code % 10U);
                fault_group = fault_tens != 0U ? 0U : 1U;
                uint8_t digit = fault_group == 0U ? fault_tens : fault_ones;
                fault_pulses_left = fault_digit_pulses(digit);
                fault_stage = 1U;
                fault_output_on = true;
                status_io_led(true);
                status_io_tone_start(fault_group == 0U ? 1500U : 2400U);
                fault_deadline = now + 100U;
            } else if ((int32_t)(now - fault_deadline) >= 0) {
                if (fault_stage == 1U) {
                    if (fault_output_on) {
                        status_io_led(false);
                        status_io_tone_stop();
                        fault_output_on = false;
                        if (fault_pulses_left > 0U) fault_pulses_left--;
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
                        status_io_led(true);
                        status_io_tone_start(fault_group == 0U ? 1500U : 2400U);
                        fault_deadline = now + 100U;
                    }
                } else if (fault_stage == 2U) {
                    fault_group = 1U;
                    fault_pulses_left = fault_digit_pulses(fault_ones);
                    fault_stage = 1U;
                    fault_output_on = true;
                    status_io_led(true);
                    status_io_tone_start(2400U);
                    fault_deadline = now + 100U;
                } else {
                    fault_group = fault_tens != 0U ? 0U : 1U;
                    uint8_t digit = fault_group == 0U ? fault_tens : fault_ones;
                    fault_pulses_left = fault_digit_pulses(digit);
                    fault_stage = 1U;
                    fault_output_on = true;
                    status_io_led(true);
                    status_io_tone_start(fault_group == 0U ? 1500U : 2400U);
                    fault_deadline = now + 100U;
                }
            }
        } else {
            if (announced != MOTOR_FAULT_NONE || fault_output_on) {
                status_io_tone_stop();
                fault_output_on = false;
            }
            announced = MOTOR_FAULT_NONE;
            fault_stage = 0U;

            /* Normal LED state remains non-blocking and independent of UART/FOC. */
            led_div++;
            if (led_div >= 10U) {
                led_div = 0U;
                bool detecting = g_motor_left.detect.busy || g_motor_right.detect.busy;
                bool running = g_motor_left.pwm_enabled || g_motor_right.pwm_enabled;
                bool on;
                if (detecting) on = (led_phase == 0U || led_phase == 2U);
                else if (running) on = (led_phase < 2U) || (led_phase >= 5U && led_phase < 7U);
                else on = led_phase < 5U;
                status_io_led(on);
                led_phase = (uint8_t)((led_phase + 1U) % 10U);
            }

            /* Power-on melody: non-blocking, and only when no fault owns buzzer. */
            if (!startup_done && (int32_t)(now - startup_deadline) >= 0) {
                if (startup_index >= (uint8_t)(sizeof(s_startup_notes) / sizeof(s_startup_notes[0]))) {
                    status_io_tone_stop();
                    startup_done = true;
                } else {
                    const startup_note_t *n = &s_startup_notes[startup_index++];
                    if (n->hz == 0U) status_io_tone_stop(); else status_io_tone_start(n->hz);
                    startup_deadline = now + n->duration_ms;
                }
            }
        }

        osDelayUntil(next);
    }
}

bool status_threads_init(void) {
    if (status_threads_started) return status_thread_tp != NULL;
    status_threads_started = true;
    const osThreadAttr_t status_attr = {
        .name="StatusIO", .priority=osPriorityNormal, .stack_size=512U
    };
    status_thread_tp = osThreadNew(status_thread, NULL, &status_attr);
    return status_thread_tp != NULL;
}

bool motor_threads_init(void) {
    const osThreadAttr_t timer_attr  = {.name="motor service", .priority=osPriorityHigh,        .stack_size=896U};
    const osThreadAttr_t sample_attr = {.name="SampleSender",  .priority=osPriorityBelowNormal, .stack_size=640U};
    const osThreadAttr_t fault_attr  = {.name="Fault Stop",    .priority=osPriorityHigh,        .stack_size=512U};

    /* Initialize all command/application state before the service thread can
     * observe it. Both objects are static BSS; this adds no RTOS heap/stack. */
    app_command_init();
    app_adc_init();

    /* Initialize timeout state before the service thread starts evaluating it. */
    const bool timeout_ok = timeout_init();
    timer_thread_tp = osThreadNew(timer_thread, NULL, &timer_attr);
    sample_send_tp  = osThreadNew(sample_send_thread, NULL, &sample_attr);
    fault_stop_tp   = osThreadNew(fault_stop_thread, NULL, &fault_attr);

    const bool threads_ok = timer_thread_tp != NULL && sample_send_tp != NULL &&
                            fault_stop_tp != NULL && timeout_ok;
    const bool heap_ok = motor_threads_free_heap_bytes() >= RTOS_READY_HEAP_RESERVE_BYTES;
    const bool ok = threads_ok && heap_ok;
    if (ok) timeout_watchdog_start();
    return ok;
}
