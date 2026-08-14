#include "motor_tasks.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_control.h"
#include "sensor_detect.h"
#include "debug_sample.h"
#include "telemetry.h"
#include "vesc_comm.h"
#include "cmsis_os2.h"
#include "app_config.h"
#include "app_adc_port.h"
#include "vesc_timeout.h"
#include "status_io.h"
#include <stddef.h>

/* CMSIS-RTOS2 equivalents of the requested VESC-style threads.
 * The names deliberately match the upstream semantic thread names.
 * Fast ADC->FOC->SVPWM stays in DMA1_Channel1_IRQHandler(), not here. */
static osThreadId_t current_cal_thread_tp;
static osThreadId_t timer_thread_tp;
static osThreadId_t pid_thread_tp;
static osThreadId_t rpm_thread_tp;
static osThreadId_t sample_send_tp;
static osThreadId_t fault_stop_tp;
static osThreadId_t stat_thread_tp;
static osThreadId_t periodic_thread_tp;
static osThreadId_t led_thread_tp;
static osThreadId_t buzzer_thread_tp;
static bool status_threads_started = false;

static void current_cal_thread(void *arg);
static void timer_thread(void *arg);
static void pid_thread(void *arg);
static void rpm_thread(void *arg);
static void sample_send_thread(void *arg);
static void fault_stop_thread(void *arg);
static void stat_thread(void *arg);
static void periodic_thread(void *arg);
static void led_thread(void *arg);
static void buzzer_thread(void *arg);

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

/* Port-specific startup current-calibration supervisor. This is intentionally
 * NOT named adc_thread: upstream adc_thread belongs to applications/app_adc.c. */
static void current_cal_thread(void *arg) {
    (void)arg;
    bool fault_reported = false;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 5U; /* 200 Hz supervision */
        if (foc_calibration_done()) {
            if (!foc_calibration_valid() && !fault_reported) {
                fault_reported = true;
                motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_CURRENT_OFFSET);
                motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_CURRENT_OFFSET);
                motor_threads_fault_signal(MOTOR_LEFT);
                motor_threads_fault_signal(MOTOR_RIGHT);
            }
            if (foc_calibration_valid()) {
                fault_reported = false;
            }
        }
        osDelayUntil(next);
    }
}

/* VESC-style 1 kHz housekeeping. */
static void timer_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 1U;
        uint32_t now = osKernelGetTickCount();

        sensor_detect_update_1khz(&g_motor_left, now);
        sensor_detect_update_1khz(&g_motor_right, now);
        motor_slow_update_1khz(&g_motor_left, now);
        motor_slow_update_1khz(&g_motor_right, now);

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

/* Position/speed controller. Current Id/Iq PI remains in FOC ISR. */
static void pid_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 1U;
        motor_pid_update_1khz(&g_motor_left);
        motor_pid_update_1khz(&g_motor_right);
        osDelayUntil(next);
    }
}

/* F103 port sensor-estimator service. Upstream FOC does not require the BLDC
 * mcpwm.c rpm_thread; this task exists only to update Hall/ABI speed mirrors
 * outside the current ISR. */
static void rpm_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 1U;
        motor_rpm_update_1khz(&g_motor_left);
        motor_rpm_update_1khz(&g_motor_right);
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
 * independent buzzer_thread so fault handling itself never blocks on beeps. */
static void fault_stop_thread(void *arg) {
    (void)arg;
    const uint32_t mask = (1UL << MOTOR_LEFT) | (1UL << MOTOR_RIGHT);
    for (;;) {
        uint32_t f = osThreadFlagsWait(mask, osFlagsWaitAny, osWaitForever);
        if ((f & osFlagsError) != 0U) continue;
        if ((f & (1UL << MOTOR_LEFT)) != 0U) motor_hw_set_pwm_enabled(&g_motor_left, false);
        if ((f & (1UL << MOTOR_RIGHT)) != 0U) motor_hw_set_pwm_enabled(&g_motor_right, false);
    }
}

/* VESC-like statistics accumulation. */
static void stat_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += STAT_PERIOD_MS;
        telemetry_stats_update_100hz();
        osDelayUntil(next);
    }
}

/* 10 ms periodic thread: snapshot telemetry, rotor-position stream, pending
 * replies and communication housekeeping. */
static void periodic_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += ROTOR_STREAM_PERIOD_MS;
        telemetry_snapshot_100hz();
        vesc_comm_periodic_100hz();
        osDelayUntil(next);
    }
}

/* PB2 status is motor/system state only. VESC traffic no longer changes the
 * steady LED pattern; that V8/V10 diagnostic double-flash confused a healthy
 * communication request with a fault/status condition.
 *   boot/calibration or stopped-ready : 1 Hz heartbeat
 *   motor running                     : 2 Hz heartbeat
 *   detect/calibrate                  : double flash
 *   any motor fault                   : fast 5 Hz blink
 */
static void led_thread(void *arg) {
    (void)arg;
    uint8_t phase = 0U;
    for (;;) {
        bool fault = (g_motor_left.fault != MOTOR_FAULT_NONE) ||
                     (g_motor_right.fault != MOTOR_FAULT_NONE);
        bool detecting = g_motor_left.detect.busy || g_motor_right.detect.busy;
        bool running = g_motor_left.pwm_enabled || g_motor_right.pwm_enabled;
        bool on;

        if (fault) {
            on = (phase & 1U) == 0U;                  /* 5 Hz */
        } else if (detecting) {
            on = (phase == 0U || phase == 2U);       /* double flash */
        } else if (running) {
            on = (phase < 2U) || (phase >= 5U && phase < 7U); /* 2 Hz-ish */
        } else {
            on = phase < 5U;                         /* 1 Hz, 50% */
        }

        status_io_led(on);
        phase++;
        if (phase >= 10U) phase = 0U;
        osDelay(100U);
    }
}

static motor_fault_t audible_fault(void) {
    motor_fault_t a = g_motor_left.fault;
    motor_fault_t b = g_motor_right.fault;
    /* Prefer safety-critical faults when both sides report something. */
    motor_fault_t order[] = {
        MOTOR_FAULT_ABS_OVER_CURRENT,
        MOTOR_FAULT_ADC_DMA,
        MOTOR_FAULT_FOC_ISR_OVERRUN,
        MOTOR_FAULT_OVER_VOLTAGE,
        MOTOR_FAULT_UNDER_VOLTAGE,
        MOTOR_FAULT_CURRENT_OFFSET,
        MOTOR_FAULT_HALL_INVALID,
        MOTOR_FAULT_SENSOR_DETECT,
        MOTOR_FAULT_COMMAND_TIMEOUT
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(order) / sizeof(order[0])); i++) {
        if (a == order[i] || b == order[i]) return order[i];
    }
    return MOTOR_FAULT_NONE;
}

static void beep_once(uint16_t hz, uint32_t on_ms, uint32_t off_ms) {
    status_io_tone_start(hz);
    osDelay(on_ms);
    status_io_tone_stop();
    if (off_ms != 0U) osDelay(off_ms);
}

static void startup_melody(void) {
    /* Three ascending notes: unmistakable indication that the scheduler and
     * dedicated buzzer thread are running. */
    beep_once(900U, 90U, 45U);
    beep_once(1350U, 90U, 45U);
    beep_once(1900U, 140U, 0U);
}

static void fault_pattern(motor_fault_t f) {
    uint8_t count = 1U;
    uint16_t hz = 1200U;
    switch (f) {
        case MOTOR_FAULT_UNDER_VOLTAGE:    count = 2U; hz = 900U;  break;
        case MOTOR_FAULT_OVER_VOLTAGE:     count = 2U; hz = 2600U; break;
        case MOTOR_FAULT_ABS_OVER_CURRENT: count = 3U; hz = 3000U; break;
        case MOTOR_FAULT_HALL_INVALID:
        case MOTOR_FAULT_SENSOR_DETECT:    count = 4U; hz = 1700U; break;
        case MOTOR_FAULT_CURRENT_OFFSET:
        case MOTOR_FAULT_ADC_DMA:          count = 5U; hz = 1200U; break;
        case MOTOR_FAULT_FOC_ISR_OVERRUN:  count = 6U; hz = 2200U; break;
        case MOTOR_FAULT_COMMAND_TIMEOUT:  count = 1U; hz = 1100U; break;
        default:                           count = 1U; hz = 1500U; break;
    }
    for (uint8_t i = 0U; i < count; i++) {
        beep_once(hz, 120U, 120U);
    }
}

static void buzzer_thread(void *arg) {
    (void)arg;
    startup_melody();
    for (;;) {
        motor_fault_t f = audible_fault();
        if (f == MOTOR_FAULT_NONE) {
            status_io_tone_stop();
            osDelay(100U);
            continue;
        }
        fault_pattern(f);
        osDelay(1800U);
    }
}

bool status_threads_init(void) {
    if (status_threads_started) return led_thread_tp != NULL && buzzer_thread_tp != NULL;
    status_threads_started = true;

    const osThreadAttr_t led_attr = {
        .name="led_thread", .priority=osPriorityNormal, .stack_size=384U
    };
    const osThreadAttr_t buzzer_attr = {
        .name="buzzer_thread", .priority=osPriorityNormal, .stack_size=448U
    };
    led_thread_tp = osThreadNew(led_thread, NULL, &led_attr);
    buzzer_thread_tp = osThreadNew(buzzer_thread, NULL, &buzzer_attr);
    (void)led_thread_tp;
    (void)buzzer_thread_tp;
    return led_thread_tp != NULL && buzzer_thread_tp != NULL;
}

void motor_threads_init(void) {
    const osThreadAttr_t cal_attr      = {.name="current_cal_thread", .priority=osPriorityAboveNormal, .stack_size=512U};
    const osThreadAttr_t timer_attr    = {.name="timer_thread",       .priority=osPriorityAboveNormal, .stack_size=640U};
    const osThreadAttr_t pid_attr      = {.name="pid_thread",         .priority=osPriorityHigh,        .stack_size=512U};
    const osThreadAttr_t rpm_attr      = {.name="rpm_thread",         .priority=osPriorityAboveNormal, .stack_size=512U};
    const osThreadAttr_t sample_attr   = {.name="sample_send_thread", .priority=osPriorityBelowNormal, .stack_size=640U};
    const osThreadAttr_t fault_attr    = {.name="fault_stop_thread",  .priority=osPriorityHigh,        .stack_size=512U};
    const osThreadAttr_t stat_attr     = {.name="stat_thread",        .priority=osPriorityNormal,      .stack_size=512U};
    const osThreadAttr_t periodic_attr = {.name="periodic_thread",    .priority=osPriorityNormal,      .stack_size=640U};

    current_cal_thread_tp = osThreadNew(current_cal_thread, NULL, &cal_attr);
    timer_thread_tp    = osThreadNew(timer_thread, NULL, &timer_attr);
    pid_thread_tp      = osThreadNew(pid_thread, NULL, &pid_attr);
    rpm_thread_tp      = osThreadNew(rpm_thread, NULL, &rpm_attr);
    sample_send_tp     = osThreadNew(sample_send_thread, NULL, &sample_attr);
    fault_stop_tp      = osThreadNew(fault_stop_thread, NULL, &fault_attr);
    stat_thread_tp     = osThreadNew(stat_thread, NULL, &stat_attr);
    periodic_thread_tp = osThreadNew(periodic_thread, NULL, &periodic_attr);

    app_adc_port_init();
    vesc_timeout_init();

    (void)current_cal_thread_tp; (void)timer_thread_tp; (void)pid_thread_tp;
    (void)rpm_thread_tp; (void)sample_send_tp; (void)fault_stop_tp;
    (void)stat_thread_tp; (void)periodic_thread_tp;
}
