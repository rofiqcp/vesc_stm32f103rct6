#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "datatypes.h"
#include "applications/app.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "hwconf/hw.h"
#include "timeout.h"

static app_configuration g_conf;
static MotorRuntime g_motors[2];
static uint16_t g_raw1, g_raw2;
static bool g_adc_ready = true;
static bool g_cal_done = true, g_cal_valid = true;
static bool g_disabled = false;
static int g_stop_calls[2];
static int g_current_rel_calls[2];
static int g_brake_calls[2];
static int g_duty_calls[2];
static int g_speed_calls[2];
static float g_last_value[2];

static uint16_t raw_from_v(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 3.3f) v = 3.3f;
    return (uint16_t)lroundf(v * 4095.0f / 3.3f);
}

static void set_adc_v(float v1, float v2) {
    g_raw1 = raw_from_v(v1);
    g_raw2 = raw_from_v(v2);
}

static void clear_calls(void) {
    memset(g_stop_calls, 0, sizeof(g_stop_calls));
    memset(g_current_rel_calls, 0, sizeof(g_current_rel_calls));
    memset(g_brake_calls, 0, sizeof(g_brake_calls));
    memset(g_duty_calls, 0, sizeof(g_duty_calls));
    memset(g_speed_calls, 0, sizeof(g_speed_calls));
    memset(g_last_value, 0, sizeof(g_last_value));
}

static void config_defaults(void) {
    memset(&g_conf, 0, sizeof(g_conf));
    g_conf.controller_id = 1U;
    g_conf.timeout_msec = 500U;
    g_conf.permanent_uart_enabled = true;
    g_conf.app_to_use = APP_ADC;
    g_conf.app_uart_baudrate = 115200U;
    adc_config *a = &g_conf.app_adc_conf;
    a->ctrl_type = ADC_CTRL_TYPE_CURRENT;
    a->hyst = 0.0f;
    a->voltage_start = 0.9f;
    a->voltage_end = 3.0f;
    a->voltage_min = 0.0f;
    a->voltage_max = 3.3f;
    a->voltage_center = 1.65f;
    a->voltage2_start = 0.9f;
    a->voltage2_end = 3.0f;
    a->use_filter = false;
    a->safe_start = SAFE_START_REGULAR;
    a->throttle_exp_mode = THR_EXP_POLY;
    a->ramp_time_pos = 0.0f;
    a->ramp_time_neg = 0.0f;
    a->update_rate_hz = 1000U;

    memset(g_motors, 0, sizeof(g_motors));
    for (int i = 0; i < 2; i++) {
        g_motors[i].id = (motor_id_t)i;
        g_motors[i].fault = MOTOR_FAULT_NONE;
        g_motors[i].current_min_a = -15.0f;
        g_motors[i].current_max_a = 15.0f;
        g_motors[i].max_duty = 0.95f;
        g_motors[i].max_erpm = 30000.0f;
    }
}

/* ---- Stubs used by the real app_adc.c + app_command.c ---- */
const app_configuration *app_get_configuration(void) { return &g_conf; }
bool app_is_output_disabled(void) { return g_disabled; }
MotorRuntime *motor_get(motor_id_t id) { return &g_motors[id == MOTOR_RIGHT ? 1 : 0]; }
int mc_interface_try_input_motor(motor_id_t id) { (void)id; return 1; }
bool foc_calibration_done(void) { return g_cal_done; }
bool foc_calibration_valid(void) { return g_cal_valid; }
bool motor_hw_get_app_adc_raw(uint16_t *a, uint16_t *b) {
    if (!g_adc_ready) return false;
    *a = g_raw1; *b = g_raw2; return true;
}
void timeout_reset(void) {}
void motor_stop(MotorRuntime *m) { g_stop_calls[m->id]++; g_last_value[m->id] = 0.0f; }
void motor_set_current_rel(MotorRuntime *m, float rel) { g_current_rel_calls[m->id]++; g_last_value[m->id] = rel; }
void motor_set_brake_current(MotorRuntime *m, float amp) { g_brake_calls[m->id]++; g_last_value[m->id] = amp; }
void motor_set_duty(MotorRuntime *m, float duty) { g_duty_calls[m->id]++; g_last_value[m->id] = duty; }
void motor_set_speed(MotorRuntime *m, float erpm) { g_speed_calls[m->id]++; g_last_value[m->id] = erpm; }

static app_adc_status_t status(void) {
    app_adc_status_t s;
    memset(&s, 0, sizeof(s));
    app_adc_get_status(&s);
    return s;
}

int main(void) {
    config_defaults();
    app_command_init();
    app_adc_init();

    /* 1) Before first real PA2/PA3 sample, no ownership or command. */
    g_adc_ready = false;
    app_adc_service_1khz(0U);
    app_adc_status_t s = status();
    assert((s.fault_flags & APP_ADC_FAULT_NOT_READY) != 0U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_NONE);
    g_adc_ready = true;

    /* 2) Active throttle at startup must not arm. */
    set_adc_v(2.0f, 0.9f);
    clear_calls();
    app_adc_service_1khz(1U);
    s = status();
    assert(!s.armed_left);
    assert(g_current_rel_calls[0] == 0);
    assert((s.fault_flags & APP_ADC_FAULT_START_ACTIVE) != 0U);

    /* 3) Neutral dwell is wall-time based: 499 ms is not enough. */
    set_adc_v(0.9f, 0.9f);
    app_adc_service_1khz(10U);
    app_adc_service_1khz(508U);
    s = status();
    assert(!s.armed_left);
    app_adc_service_1khz(510U);
    s = status();
    assert(s.armed_left);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_ADC);

    /* 4) Full throttle maps through the motor API to relative current. */
    clear_calls();
    set_adc_v(3.0f, 0.9f);
    app_adc_service_1khz(511U);
    assert(g_current_rel_calls[0] == 1);
    assert(g_last_value[0] > 0.99f && g_last_value[0] <= 1.0f);

    /* 5) Dual ADC brake wins over simultaneous throttle. */
    g_conf.app_adc_conf.ctrl_type = ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC;
    clear_calls();
    set_adc_v(3.0f, 3.0f);
    app_adc_service_1khz(512U);
    assert(g_current_rel_calls[0] == 0);
    assert(g_brake_calls[0] == 1);
    assert(g_last_value[0] > 14.9f && g_last_value[0] <= 15.0f);

    /* 6) Out-of-range/rail input revokes ADC ownership and stops. */
    g_conf.app_adc_conf.voltage_max = 3.10f;
    clear_calls();
    set_adc_v(3.3f, 0.9f);
    app_adc_service_1khz(513U);
    s = status();
    assert((s.fault_flags & APP_ADC_FAULT_THROTTLE_RANGE) != 0U);
    assert(!s.armed_left);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_NONE);
    assert(g_stop_calls[0] >= 1);

    /* 7) Restoring voltage while still active cannot bypass re-arm. */
    g_conf.app_adc_conf.voltage_max = 3.3f;
    clear_calls();
    set_adc_v(2.0f, 0.9f);
    app_adc_service_1khz(514U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_NONE);
    assert(g_current_rel_calls[0] == 0);

    /* 8) Neutral again for 500 ms re-arms. */
    set_adc_v(0.9f, 0.9f);
    app_adc_service_1khz(600U);
    app_adc_service_1khz(1100U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_ADC);

    /* 9) UART lease preempts ADC and forces another neutral re-arm. */
    assert(app_command_uart_claim(MOTOR_LEFT));
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_UART);
    clear_calls();
    set_adc_v(3.0f, 0.9f);
    app_adc_service_1khz(1101U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_UART);
    assert(g_current_rel_calls[0] == 0);
    app_command_release(MOTOR_LEFT, true);
    clear_calls();
    app_adc_service_1khz(1102U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_NONE);
    assert(g_current_rel_calls[0] == 0);

    /* 10) VESC SAFE_START_NO_FAULT: an ADC-owned motor fault revokes torque
       but does not require a new neutral dwell after the fault is cleared. */
    g_conf.app_adc_conf.safe_start = SAFE_START_NO_FAULT;
    set_adc_v(0.9f, 0.9f);
    app_adc_service_1khz(1200U);
    app_adc_service_1khz(1700U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_ADC);
    set_adc_v(2.0f, 0.9f);
    app_adc_service_1khz(1701U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_ADC);
    g_motors[0].fault = MOTOR_FAULT_HALL_INVALID;
    app_command_service_1khz(1702U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_NONE);
    g_motors[0].fault = MOTOR_FAULT_NONE;
    clear_calls();
    app_adc_service_1khz(1703U);
    assert(app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_ADC);
    assert(g_current_rel_calls[0] == 1); /* active throttle resumes immediately in NO_FAULT mode */

    /* 11) APP_ADC multi_esc uses the same validated command on both local motors. */
    g_conf.app_adc_conf.safe_start = SAFE_START_DISABLED;
    g_conf.app_adc_conf.ctrl_type = ADC_CTRL_TYPE_DUTY;
    g_conf.app_adc_conf.multi_esc = true;
    app_command_release(MOTOR_LEFT, false);
    app_command_release(MOTOR_RIGHT, false);
    /* release forces re-arm even with SAFE_START_DISABLED: neutral must be seen once */
    set_adc_v(0.9f, 0.9f);
    app_adc_service_1khz(1800U);
    app_adc_service_1khz(2300U);
    clear_calls();
    set_adc_v(3.0f, 0.9f);
    app_adc_service_1khz(2301U);
    assert(g_duty_calls[0] == 1 && g_duty_calls[1] == 1);
    assert(g_last_value[0] > 0.94f && g_last_value[1] > 0.94f);

    printf("ALL STAGE-1 APP ADC RUNTIME TESTS: PASS\n");
    return 0;
}
