#include "applications/app_command.h"
#include "applications/app.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "cmsis_os2.h"
#include <stddef.h>

static volatile app_command_source_t s_source[2];
static volatile uint32_t s_uart_last_ms[2];
static volatile bool s_adc_rearm[2];


void app_command_init(void) {
    for (unsigned i = 0U; i < 2U; i++) {
        s_source[i] = APP_CMD_SRC_NONE;
        s_uart_last_ms[i] = 0U;
        s_adc_rearm[i] = true;
    }
}

void app_command_configuration_changed(void) {
    /* APPCONF writes are accepted only while both bridges are off. Drop any
     * stale application lease and require PA2/PA3 to pass neutral safe-start
     * under the new calibration before control can resume. Safe during boot:
     * this function only touches the small arbitration state. */
    for (unsigned i = 0U; i < 2U; i++) {
        s_source[i] = APP_CMD_SRC_NONE;
        s_uart_last_ms[i] = 0U;
        s_adc_rearm[i] = true;
    }
}

void app_command_service_1khz(uint32_t now_ms) {
    const app_configuration *conf = app_get_configuration();
    if (conf == NULL) return;

    for (unsigned i = 0U; i < 2U; i++) {
        MotorRuntime *m = motor_get((motor_id_t)i);

        /* Any application-disable request or motor fault immediately revokes
         * a stale application owner. Hardware fault handling already drops
         * MOE; this also prevents a cleared fault from resuming old throttle. */
        if (app_is_output_disabled() || m->fault != MOTOR_FAULT_NONE) {
            const app_command_source_t old_source = s_source[i];
            if (old_source == APP_CMD_SRC_ADC || old_source == APP_CMD_SRC_UART) motor_stop(m);
            s_source[i] = APP_CMD_SRC_NONE;
            /* Match VESC SAFE_START_NO_FAULT semantics: a motor fault revokes
             * torque ownership, but does not force a new neutral dwell when
             * the ADC application explicitly selects NO_FAULT. Output-disable
             * requests and UART ownership still force re-arm. */
            if (app_is_output_disabled() || old_source != APP_CMD_SRC_ADC ||
                conf->app_adc_conf.safe_start != SAFE_START_NO_FAULT) {
                s_adc_rearm[i] = true;
            }
            continue;
        }

        /* Detection/calibration own the power stage exclusively. These source
         * states are observational guards only; the existing detection and
         * calibration code keeps its proven direct motor-control path. */
        if (m->detect.busy) {
            s_source[i] = APP_CMD_SRC_DETECTION;
            s_adc_rearm[i] = true;
            continue;
        }
        if (s_source[i] == APP_CMD_SRC_DETECTION) {
            s_source[i] = APP_CMD_SRC_NONE;
            s_adc_rearm[i] = true;
        }

        if (!foc_calibration_done() || !foc_calibration_valid()) {
            s_source[i] = APP_CMD_SRC_CALIBRATION;
            s_adc_rearm[i] = true;
            continue;
        }
        if (s_source[i] == APP_CMD_SRC_CALIBRATION) {
            s_source[i] = APP_CMD_SRC_NONE;
            s_adc_rearm[i] = true;
        }

        /* Per-motor serial lease is necessary even when APP_ADC_UART keeps the
         * legacy global timeout alive. It also makes LEFT/RIGHT command expiry
         * independent rather than stopping both because one host went stale. */
        if (s_source[i] == APP_CMD_SRC_UART && conf->timeout_msec != 0U &&
            (uint32_t)(now_ms - s_uart_last_ms[i]) > conf->timeout_msec) {
            motor_stop(m);
            s_source[i] = APP_CMD_SRC_NONE;
            s_adc_rearm[i] = true;
        }
    }
}

bool app_command_uart_claim(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return false;
    MotorRuntime *m = motor_get(id);
    if (m == NULL || m->detect.busy || app_is_output_disabled() ||
        m->fault != MOTOR_FAULT_NONE || !foc_calibration_done() || !foc_calibration_valid()) return false;
    s_source[id] = APP_CMD_SRC_UART;
    s_uart_last_ms[id] = osKernelGetTickCount();
    s_adc_rearm[id] = true;
    return true;
}

void app_command_uart_keepalive(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return;
    if (s_source[id] == APP_CMD_SRC_UART) s_uart_last_ms[id] = osKernelGetTickCount();
}

bool app_command_adc_claim(motor_id_t id, bool neutral_stable) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return false;
    MotorRuntime *m = motor_get(id);
    if (m == NULL || m->detect.busy || app_is_output_disabled() || m->fault != MOTOR_FAULT_NONE) return false;
    if (!mc_interface_try_input_motor(id)) {
        if (s_source[id] == APP_CMD_SRC_ADC) motor_stop(m);
        s_source[id] = APP_CMD_SRC_NONE;
        s_adc_rearm[id] = true;
        return false;
    }
    if (s_source[id] == APP_CMD_SRC_UART) return false;
    if (s_adc_rearm[id]) {
        if (!neutral_stable) return false;
        s_adc_rearm[id] = false;
    }
    s_source[id] = APP_CMD_SRC_ADC;
    return true;
}

void app_command_adc_block(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return;
    if (s_source[id] == APP_CMD_SRC_ADC) s_source[id] = APP_CMD_SRC_NONE;
    s_adc_rearm[id] = true;
}

void app_command_adc_release(motor_id_t id, bool stop_motor) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return;
    if (s_source[id] == APP_CMD_SRC_ADC) {
        if (stop_motor) motor_stop(motor_get(id));
        s_source[id] = APP_CMD_SRC_NONE;
    }
    s_adc_rearm[id] = true;
}

void app_command_force_adc_rearm(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return;
    s_adc_rearm[id] = true;
}

void app_command_release(motor_id_t id, bool stop_motor) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return;
    if (stop_motor) motor_stop(motor_get(id));
    s_source[id] = APP_CMD_SRC_NONE;
    s_adc_rearm[id] = true;
}

app_command_source_t app_command_get_source(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? s_source[MOTOR_RIGHT] : s_source[MOTOR_LEFT];
}

bool app_command_adc_rearm_required(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? s_adc_rearm[MOTOR_RIGHT] : s_adc_rearm[MOTOR_LEFT];
}
