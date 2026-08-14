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

static void current_cal_thread(void *arg);
static void timer_thread(void *arg);
static void pid_thread(void *arg);
static void rpm_thread(void *arg);
static void sample_send_thread(void *arg);
static void fault_stop_thread(void *arg);
static void stat_thread(void *arg);
static void periodic_thread(void *arg);
static void led_thread(void *arg);

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
 * fault/FOC path before this thread runs. */
static void fault_stop_thread(void *arg) {
    (void)arg;
    const uint32_t mask = (1UL << MOTOR_LEFT) | (1UL << MOTOR_RIGHT);
    for (;;) {
        uint32_t f = osThreadFlagsWait(mask, osFlagsWaitAny, osWaitForever);
        if ((f & osFlagsError) != 0U) continue;
        if ((f & (1UL << MOTOR_LEFT)) != 0U) motor_hw_set_pwm_enabled(&g_motor_left, false);
        if ((f & (1UL << MOTOR_RIGHT)) != 0U) motor_hw_set_pwm_enabled(&g_motor_right, false);
        motor_hw_buzzer(true);
        osDelay(40U);
        motor_hw_buzzer(false);
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

/* Dedicated status LED thread, kept separate as requested. */
static void led_thread(void *arg) {
    (void)arg;
    bool state = false;
    for (;;) {
        bool fault = (g_motor_left.fault != MOTOR_FAULT_NONE) ||
                     (g_motor_right.fault != MOTOR_FAULT_NONE);
        state = !state;
        motor_hw_led(state);
        osDelay(fault ? 100U : 500U);
    }
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
    const osThreadAttr_t led_attr      = {.name="led_thread",         .priority=osPriorityLow,         .stack_size=384U};

    current_cal_thread_tp = osThreadNew(current_cal_thread, NULL, &cal_attr);
    timer_thread_tp    = osThreadNew(timer_thread, NULL, &timer_attr);
    pid_thread_tp      = osThreadNew(pid_thread, NULL, &pid_attr);
    rpm_thread_tp      = osThreadNew(rpm_thread, NULL, &rpm_attr);
    sample_send_tp     = osThreadNew(sample_send_thread, NULL, &sample_attr);
    fault_stop_tp      = osThreadNew(fault_stop_thread, NULL, &fault_attr);
    stat_thread_tp     = osThreadNew(stat_thread, NULL, &stat_attr);
    periodic_thread_tp = osThreadNew(periodic_thread, NULL, &periodic_attr);
    led_thread_tp      = osThreadNew(led_thread, NULL, &led_attr);

    app_adc_port_init();
    vesc_timeout_init();

    (void)current_cal_thread_tp; (void)timer_thread_tp; (void)pid_thread_tp;
    (void)rpm_thread_tp; (void)sample_send_tp; (void)fault_stop_tp;
    (void)stat_thread_tp; (void)periodic_thread_tp; (void)led_thread_tp;
}
