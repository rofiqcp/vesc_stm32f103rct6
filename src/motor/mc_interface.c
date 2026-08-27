#include "motor/mc_interface.h"
#include "hwconf/hw.h"
#include "motor/foc_math.h"
#include "motor/mc_math.h"
#include "motor/mcpwm_foc.h"
#include "comm/packet.h"
#include "telemetry.h"
#include "encoder/encoder.h"
#include "motor/mcconf_default.h"
#include "hwconf/hw_hoverboard.h"
#include "conf_general.h"
#include "confgenerator.h"
#include <string.h>
#include <math.h>
#include <limits.h>
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "comm/commands.h"
#include "hwconf/hw.h"
#include "motor/mcpwm_foc.h"
#include "telemetry.h"
#include "timeout.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

MotorRuntime g_motor_left;
MotorRuntime g_motor_right;
static volatile uint32_t s_pending_fault_mask = 0U;

/* VESC setup statistics are task-side diagnostics, never part of the hard FOC ISR.
 * One accumulator is kept per local bridge and sampled at 100 Hz. */
static setup_stats s_setup_stats[2];
static uint8_t s_setup_stats_div[2];
static float s_temp_motor_override=NAN;

static int32_t amp_to_current_q15(float a) {
    float q = (a / FOC_CURRENT_Q_BASE_A) * 32768.0f;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int32_t)q;
}

static int32_t volt_to_q15(float v) {
    float q = (v / FOC_VOLTAGE_Q_BASE_V) * 32768.0f;
    if (q > 32767.0f) q = 32767.0f;
    if (q < 0.0f) q = 0.0f;
    return (int32_t)q;
}

void motor_set_foc_targets(MotorRuntime *m, float id_a, float iq_a) {
    if (m == NULL) return;
    float hi = (m->current_max_a > 0.0f) ? m->current_max_a : FOC_MAX_CURRENT_A;
    float lo = (m->current_min_a < 0.0f) ? m->current_min_a : -hi;
    id_a = foc_clampf(id_a, lo, hi);
    iq_a = foc_clampf(iq_a, lo, hi);
    m->id_target = id_a;
    m->iq_target = iq_a;
    const int32_t id_q15 = amp_to_current_q15(id_a);
    const int32_t iq_q15 = amp_to_current_q15(iq_a);
    m->id_target_base_q15 = id_q15;
    m->iq_target_base_q15 = iq_q15;
    m->id_target_q15 = id_q15;
    m->iq_target_q15 = iq_q15;
}

void motor_set_current_pi_gains(MotorRuntime *m, float kp, float ki) {
    if (m == NULL) return;
    if (!isfinite(kp) || kp < 0.000001f) kp = 0.000001f;
    if (!isfinite(ki) || ki < 0.0f) ki = 0.0f;
    m->current_kp = kp;
    m->current_ki = ki;
    m->current_kp_q16 = (int32_t)((kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->current_ki_dt_q16 = (int32_t)((ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    /* A gain change is a control discontinuity. Reset only the fast current
       integrators; outer-loop state is left to the caller. */
    m->vd_int_q31 = 0; m->vq_int_q31 = 0;
    m->vd_int_q15 = 0; m->vq_int_q15 = 0;
}

static void init_hall_defaults(MotorRuntime *m) {
    const int8_t table[8] = HALL_TABLE_DEFAULT;
    memcpy(m->hall_table, table, sizeof(table));
    for (unsigned i = 0; i < 8U; i++) {
        if (table[i] >= 0) {
            m->foc_hall_table[i] = (uint8_t)(((unsigned)table[i] * 200U) / 6U);
            m->hall_angle_u16[i] = (uint16_t)((unsigned)table[i] * (65536U / 6U));
        } else {
            m->foc_hall_table[i] = 255U;
            m->hall_angle_u16[i] = 0U;
        }
    }
    m->hall.sector = -1;
    m->hall.direction = 1;
}

static void motor_defaults(MotorRuntime *m, motor_id_t id) {
    memset(m, 0, sizeof(*m));
    m->id = id;
    m->pwm_tim = (id == MOTOR_LEFT) ? LEFT_TIM : RIGHT_TIM;
    m->pole_pairs = (id == MOTOR_LEFT) ? LEFT_POLE_PAIRS : RIGHT_POLE_PAIRS;
    m->motor_type = MCCONF_MOTOR_TYPE_DEFAULT;
    m->pwm_mode = MCCONF_PWM_MODE_DEFAULT;
    m->comm_mode = MCCONF_COMM_MODE_DEFAULT;
    m->foc_sensor_mode = (id == MOTOR_LEFT) ? MCCONF_FOC_SENSOR_LEFT_DEFAULT : MCCONF_FOC_SENSOR_RIGHT_DEFAULT;
    m->state = MC_STATE_OFF;
    m->sensor_mode = (id == MOTOR_LEFT && LEFT_SENSOR_BOOT_MODE == SENSOR_MODE_ENCODER) ?
                     SENSOR_MODE_ENCODER : SENSOR_MODE_HALL;
    m->sensor_request_mode = (id == MOTOR_LEFT) ? LEFT_SENSOR_BOOT_MODE : RIGHT_SENSOR_BOOT_MODE;
    m->control_mode = MOTOR_CTRL_OFF;
    m->fault = MOTOR_FAULT_NONE;
    m->current_kp = (id == MOTOR_LEFT) ? LEFT_FOC_KP : RIGHT_FOC_KP;
    m->current_ki = (id == MOTOR_LEFT) ? LEFT_FOC_KI : RIGHT_FOC_KI;
    m->current_scale = (id == MOTOR_LEFT) ? LEFT_CURRENT_A_PER_COUNT : RIGHT_CURRENT_A_PER_COUNT;
    m->dc_current_scale = (id == MOTOR_LEFT) ? LEFT_DC_CURRENT_A_PER_COUNT : RIGHT_DC_CURRENT_A_PER_COUNT;

    m->current_scale_q16 = (int32_t)((m->current_scale / FOC_CURRENT_Q_BASE_A) * 32768.0f * 65536.0f);
    m->dc_current_scale_q16 = (int32_t)((m->dc_current_scale / FOC_CURRENT_Q_BASE_A) * 32768.0f * 65536.0f);
    m->current_kp_q16 = (int32_t)((m->current_kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->current_ki_dt_q16 = (int32_t)((m->current_ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->duty_u_q15 = m->duty_v_q15 = m->duty_w_q15 = FOC_Q15_HALF;
    m->sampling_window_clamp_count = 0U;
    m->sampling_margin_min_q15 = (uint16_t)(FOC_Q15_HALF - PWM_MIN_DUTY_Q15);
    m->current_max_a = FOC_MAX_CURRENT_A; m->current_min_a = -FOC_MAX_CURRENT_A;
    m->input_current_max_a = FOC_MAX_CURRENT_A; m->input_current_min_a = -FOC_MAX_CURRENT_A;
    m->abs_current_max_a = FOC_ABS_CURRENT_TRIP_A;
    m->abs_current_trip_q15 = amp_to_current_q15(FOC_ABS_CURRENT_TRIP_A);
    m->slow_abs_current = false;
    m->temp_fet_start = MCCONF_L_TEMP_FET_START_DEFAULT;
    m->temp_fet_end = MCCONF_L_TEMP_FET_END_DEFAULT;
    m->temp_motor_start = MCCONF_L_TEMP_MOTOR_START_DEFAULT;
    m->temp_motor_end = MCCONF_L_TEMP_MOTOR_END_DEFAULT;
    m->temp_accel_dec = MCCONF_L_TEMP_ACCEL_DEC_DEFAULT;
    m->additional_faults = MCCONF_L_ADDITIONAL_FAULTS_DEFAULT;
    m->board_temp_c = 25.0f; m->board_temp_filter_c = 25.0f; m->board_temp_valid = false;
    m->abs_phase_current_filter_q15 = 0;
    m->abs_current_peak_q15 = 0;
    m->abs_current_fault_count = 0U;
    m->min_vin = VBUS_MIN_RUN_V; m->max_vin = VBUS_MAX_RUN_V;
    m->battery_cut_start = 36.0f; m->battery_cut_end = 32.0f;
    m->battery_regen_cut_start = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    m->battery_regen_cut_end = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    m->min_vin_q15 = volt_to_q15(m->min_vin); m->max_vin_q15 = volt_to_q15(m->max_vin);
    m->hard_max_vin_q15 = volt_to_q15(fminf(m->max_vin + FOC_VBUS_HARD_OV_MARGIN_V, FOC_VBUS_HARD_MAX_V));
    m->hard_min_vin_q15 = volt_to_q15(fmaxf(m->min_vin - FOC_VBUS_HARD_UV_MARGIN_V, FOC_VBUS_HARD_MIN_V));
    m->over_voltage_fault_count = 0U; m->under_voltage_fault_count = 0U;
    m->max_erpm = MOTOR_DEFAULT_MAX_ERPM; m->min_erpm = MOTOR_DEFAULT_MIN_ERPM; m->erpm_start = MCCONF_L_ERPM_START_DEFAULT;
    m->erpm_fault_filter = 0.0f;
    m->foc_start_curr_dec = MCCONF_FOC_START_CURR_DEC_DEFAULT;
    m->foc_start_curr_dec_rpm = MCCONF_FOC_START_CURR_DEC_RPM_DEFAULT;
    m->foc_short_ls_on_zero_duty = MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT;
    m->full_brake_active = false;
    m->max_duty = MOTOR_DEFAULT_MAX_DUTY; m->min_duty = MOTOR_DEFAULT_MIN_DUTY;
    m->duty_limit_now = m->max_duty;
    m->duty_was_pi = false; m->duty_pi_duty_last = 0.0f;
    m->force_zero_modulation = false;
    m->brake_speed_before_q16 = 0; m->brake_vq_before_q15 = 0;
    m->brake_zero_hold_ticks = 1U; m->brake_zero_active = false;
    m->current_max_scale = MCCONF_L_CURRENT_MAX_SCALE_DEFAULT;
    m->current_min_scale = MCCONF_L_CURRENT_MIN_SCALE_DEFAULT;
    m->watt_max = MCCONF_L_WATT_MAX_DEFAULT; m->watt_min = MCCONF_L_WATT_MIN_DEFAULT;
    m->duty_start = MCCONF_L_DUTY_START_DEFAULT;
    m->lo_current_max_a = m->current_max_a; m->lo_current_min_a = m->current_min_a;
    m->lo_input_current_max_a = m->input_current_max_a; m->lo_input_current_min_a = m->input_current_min_a;
    m->input_current_map_start = MCCONF_L_IN_CURRENT_MAP_START_DEFAULT;
    m->input_current_map_filter = MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT;
    m->input_current_map_filtered_a = 0.0f;
    m->input_current_map_limit_a = m->current_max_a;
    m->speed_pid.kp = SPEED_PID_KP; m->speed_pid.ki = SPEED_PID_KI; m->speed_pid.kd = SPEED_PID_KD;
    m->speed_kd_filter = SPEED_PID_D_FILTER;
    m->speed_derivative_filtered = 0.0f;
    m->speed_pid_min_erpm = SPEED_PID_MIN_ERPM;
    m->speed_pid_ramp_erpms_s = SPEED_PID_RAMP_ERPMS_S;
    m->speed_pid_allow_braking = SPEED_PID_ALLOW_BRAKING;
    m->speed_pid_source = SPEED_PID_SOURCE_DEFAULT;
    m->speed_pid_set_erpm = 0.0f;
    m->position_pid.kp = POSITION_PID_KP_CURRENT_PER_DEG;
    m->position_pid.ki = POSITION_PID_KI_CURRENT_PER_DEG_S;
    m->position_pid.kd = POSITION_PID_KD_CURRENT_PER_DEGPS;
    m->position_kd_filter = POSITION_PID_D_FILTER;
    m->position_kd_proc = POSITION_PID_KD_PROC;
    m->position_ang_div = POSITION_PID_ANG_DIV;
    m->position_gain_dec_angle = POSITION_PID_GAIN_DEC_ANGLE;
    m->position_offset_deg = POSITION_PID_OFFSET_DEG;
    m->position_derivative_filtered = 0.0f;
    m->position_derivative_proc_filtered = 0.0f;
    m->position_prev_process_deg = 0.0f;
    m->position_dt_integrator = 0.0f;
    m->position_dt_process_integrator = 0.0f;
    m->sensorless_start_failures = 0U;
    m->cc_min_current = CURRENT_CTRL_MIN_CURRENT_A;
    m->duty_kp = DUTY_PID_KP_CURRENT_PER_DUTY;
    m->duty_ki = DUTY_PID_KI_CURRENT_PER_DUTY_S;
    m->si_gear_ratio = 1.0f;
    m->si_wheel_diameter = 0.1f;
    m->si_battery_type = 0U;
    m->si_battery_cells = 10U;
    m->si_battery_ah = 10.0f;
    m->si_motor_nl_current = 1.0f;

    m->foc_motor_r = MCCONF_FOC_MOTOR_R_DEFAULT;
    m->foc_motor_l = MCCONF_FOC_MOTOR_L_DEFAULT;
    m->foc_motor_ld_lq_diff = 0.0f;
    m->foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE_DEFAULT;
    m->res_est_ohm = m->foc_motor_r;
    m->res_est_state_ohm = m->foc_motor_r;
    m->res_est_valid = false;
    m->foc_speed_source = MCCONF_FOC_SPEED_SOURCE_DEFAULT;
    m->foc_dt_us = FOC_DEADTIME_COMP_US;
    m->deadtime_comp_q15 = 0;
    m->foc_observer_gain = MCCONF_FOC_OBSERVER_GAIN_DEFAULT;
    m->foc_observer_gain_slow = MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT;
    m->foc_observer_offset = MCCONF_FOC_OBSERVER_OFFSET_DEFAULT;
    m->foc_sat_comp_mode = MCCONF_FOC_SAT_COMP_MODE_DEFAULT;
    m->foc_sat_comp = MCCONF_FOC_SAT_COMP_DEFAULT;
    m->foc_observer_type = MCCONF_FOC_OBSERVER_TYPE_DEFAULT;
    m->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP_DEFAULT;
    m->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI_DEFAULT;
    m->foc_current_filter_const = MCCONF_FOC_CURRENT_FILTER_CONST_DEFAULT;
    m->foc_cc_decoupling = MCCONF_FOC_CC_DECOUPLING_DEFAULT;
    m->foc_mtpa_mode = MCCONF_FOC_MTPA_MODE_DEFAULT;
    m->foc_fw_current_max = MCCONF_FOC_FW_CURRENT_MAX_DEFAULT;
    m->foc_fw_duty_start = MCCONF_FOC_FW_DUTY_START_DEFAULT;
    m->foc_fw_ramp_time = MCCONF_FOC_FW_RAMP_TIME_DEFAULT;
    m->foc_fw_q_current_factor = MCCONF_FOC_FW_Q_CURRENT_FACTOR_DEFAULT;
    m->foc_fw_backoff = MCCONF_FOC_FW_BACKOFF_DEFAULT;
    m->foc_mag_vd_max = MCCONF_FOC_MAG_VD_MAX_DEFAULT;
    m->foc_overmod_factor = MCCONF_FOC_OVERMOD_FACTOR_DEFAULT;
    m->foc_temp_comp = MCCONF_FOC_TEMP_COMP_DEFAULT;
    m->foc_temp_comp_base_temp = MCCONF_FOC_TEMP_COMP_BASE_TEMP_DEFAULT;
    m->foc_offsets_cal_mode = MCCONF_FOC_OFFSETS_CAL_MODE_DEFAULT;
    m->foc_calibrate_on_boot = MCCONF_FOC_CALIBRATE_ON_BOOT_DEFAULT;
    m->foc_fw_current_now = 0.0f; m->mtpa_id_target = 0.0f;
    m->foc_fw_current_acc_q31 = 0; m->foc_fw_current_q15 = 0; m->foc_fw_duty_filter_q15 = 0;
    m->fw_override_current_q15 = 0; m->foc_fw_fast_active = false;
    m->foc_fw_hold_request = false; m->foc_current_limit_q15 = amp_to_current_q15(FOC_MAX_CURRENT_A);
    m->encoder_slip_bad_ticks = 0U; m->encoder_slip_error_phase = 0;
    m->encoder_slip_check_active = false;
    m->foc_pll_kp = MCCONF_FOC_PLL_KP_DEFAULT;
    m->foc_pll_ki = MCCONF_FOC_PLL_KI_DEFAULT;
    m->foc_hall_interp_erpm = 500.0f;
    m->foc_hall_interp_erpm_u32 = 500U;
    m->foc_sl_erpm_start = MCCONF_FOC_SL_ERPM_START_DEFAULT;
    m->foc_sl_erpm = MCCONF_FOC_SL_ERPM_DEFAULT;
    m->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM_DEFAULT;
    m->foc_openloop_rpm_low = MCCONF_FOC_OPENLOOP_RPM_LOW_DEFAULT;
    m->foc_sl_openloop_time_lock = MCCONF_FOC_SL_OPENLOOP_T_LOCK_DEFAULT;
    m->foc_sl_openloop_time_ramp = MCCONF_FOC_SL_OPENLOOP_T_RAMP_DEFAULT;
    m->foc_sl_openloop_time = MCCONF_FOC_SL_OPENLOOP_TIME_DEFAULT;
    m->foc_sl_openloop_hyst = MCCONF_FOC_SL_OPENLOOP_HYST_DEFAULT;
    m->foc_sl_openloop_boost_q = MCCONF_FOC_SL_OPENLOOP_BOOST_Q_DEFAULT;
    m->foc_sl_openloop_max_q = MCCONF_FOC_SL_OPENLOOP_MAX_Q_DEFAULT;
    init_hall_defaults(m);

    if (id == MOTOR_LEFT) {
        m->encoder.cpr = LEFT_ENCODER_CPR;
        m->encoder.inverted = LEFT_ENCODER_INVERTED_DEFAULT ? true : false;
        m->encoder.elec_offset_u16 = foc_deg_to_u16(LEFT_ENCODER_ELEC_OFFSET_DEG_DEFAULT);
        m->encoder.electrical_ratio = (float)m->pole_pairs;
        m->encoder.electrical_ratio_q16 = (uint32_t)lrintf(m->encoder.electrical_ratio * 65536.0f);
        m->encoder.phase_per_count_q16 = (uint32_t)(((uint64_t)m->encoder.electrical_ratio_q16 << 16) / m->encoder.cpr);
        m->encoder.synced = false;
        m->encoder.motion_proved = false;
        m->hall_offset_u16 = foc_deg_to_u16(LEFT_HALL_ELEC_OFFSET_DEG_DEFAULT);
    } else {
        m->hall_offset_u16 = foc_deg_to_u16(RIGHT_HALL_ELEC_OFFSET_DEG_DEFAULT);
    }
}

void motor_control_init(void) {
    motor_defaults(&g_motor_left, MOTOR_LEFT);
    motor_defaults(&g_motor_right, MOTOR_RIGHT);
    memset(s_setup_stats, 0, sizeof(s_setup_stats));
    memset(s_setup_stats_div, 0, sizeof(s_setup_stats_div));
    s_setup_stats[MOTOR_LEFT].time_start = osKernelGetTickCount();
    s_setup_stats[MOTOR_RIGHT].time_start = s_setup_stats[MOTOR_LEFT].time_start;
    foc_math_init();
    /* Precompute every float-derived coefficient before ADC/DMA can enter the
       hard FOC ISR. Configuration changes repeat this atomically task-side. */
    foc_precalc_values(&g_motor_left);
    foc_precalc_values(&g_motor_right);
    mcpwm_foc_init_hw();
    motor_hw_configure_sensor(&g_motor_left, g_motor_left.sensor_mode);
    motor_hw_configure_sensor(&g_motor_right, g_motor_right.sensor_mode);
    if (g_motor_left.sensor_mode == SENSOR_MODE_HALL) motor_hall_edge_isr(&g_motor_left);
    motor_hall_edge_isr(&g_motor_right);
}

MotorRuntime *motor_get(motor_id_t id) { return (id == MOTOR_RIGHT) ? &g_motor_right : &g_motor_left; }

void motor_touch_command(MotorRuntime *m) {
    m->last_command_tick = osKernelGetTickCount();
    m->command_active = true;
    m->timeout_active = false;
}

void motor_keepalive(MotorRuntime *m) {
    if (m == NULL) return;
    m->last_command_tick = osKernelGetTickCount();
    m->timeout_active = false;
}
void motor_set_current(MotorRuntime *m, float amp) {
    if (m == NULL) return;
    mcpwm_foc_set_current_motor(m, amp);
    motor_touch_command(m);
}
void motor_set_brake_current(MotorRuntime *m, float amp) {
    if (m == NULL) return;
    mcpwm_foc_set_brake_current_motor(m, amp);
    motor_touch_command(m);
}
void motor_set_current_rel(MotorRuntime *m, float rel) {
    if (m == NULL) {
        return;
    }

    rel = foc_clampf(rel, -1.0f, 1.0f);
    motor_set_current(m, rel >= 0.0f ? rel * m->current_max_a : (-rel) * m->current_min_a);
}
void motor_set_handbrake(MotorRuntime *m, float amp) {
    if (m == NULL) {
        return;
    }

    mcpwm_foc_set_handbrake_motor(m, amp);
    motor_touch_command(m);
}
void motor_set_speed(MotorRuntime *m, float erpm) {
    if (m == NULL) return;
    mcpwm_foc_set_pid_speed_motor(m, erpm);
    motor_touch_command(m);
}
void motor_set_position(MotorRuntime *m, float deg) {
    if (m == NULL) return;
    mcpwm_foc_set_pid_pos_motor(m, deg);
    motor_touch_command(m);
}
void motor_set_duty(MotorRuntime *m, float duty) {
    if (m == NULL) return;
    mcpwm_foc_set_duty_motor(m, duty);
    motor_touch_command(m);
}

void motor_stop(MotorRuntime *m) {
    if (m == NULL) return;
    mcpwm_foc_release_motor_motor(m);
    m->command_active=false;
}

void motor_clear_fault(MotorRuntime *m) {
    if (m == NULL) return;
    /* PVD/BKIN faults are reset-latched power-stage faults. Refuse to hide
       them from VESC Tool while hardware re-enable is still blocked. */
    if (motor_hw_powerstage_fault_latched() ||
        (m->fault == MOTOR_FAULT_FLASH_CONFIG && !conf_general_integrity_ok()) ||
        m->fault == MOTOR_FAULT_MCU_UNDER_VOLTAGE || m->fault == MOTOR_FAULT_BREAK) {
        motor_stop(m);
        return;
    }
    motor_stop(m);
    m->fault=MOTOR_FAULT_NONE;
    /* A stale observer-speed filter can immediately recreate an
     * ABS_OVERSPEED fault on the next 1-kHz service tick, preventing a safe
     * stopped current recalibration from ever arming MOE. */
    m->erpm_fault_filter = 0.0f;
    m->erpm = 0.0f;
    m->pll_erpm_q16 = 0;
    m->speed_est_fast_erpm_q16 = 0;
    m->vd_int=m->vq_int=0.0f;
    m->vd_int_q31=m->vq_int_q31=0; m->vd_int_q15=m->vq_int_q15=0;
    m->hall.invalid_count=0U; m->hall.sequence_error_count=0U; m->hall.recovery_valid_ticks=0U;
}

void motor_clear_fault_for_cal(MotorRuntime *m) {
    if (m == NULL) return;
    /* PVD/BKIN faults are hardware-latched and must never be silently cleared.
     * A config-flash fault is cleared here so a stopped zero-vector calibration
     * can always re-arm the bridge MOE for driven offset measurement. */
    if (motor_hw_powerstage_fault_latched() ||
        m->fault == MOTOR_FAULT_MCU_UNDER_VOLTAGE ||
        m->fault == MOTOR_FAULT_BREAK) {
        motor_stop(m);
        return;
    }
    motor_stop(m);
    m->fault = MOTOR_FAULT_NONE;
    m->erpm_fault_filter = 0.0f;
    m->erpm = 0.0f;
    m->pll_erpm_q16 = 0;
    m->speed_est_fast_erpm_q16 = 0;
    m->vd_int = m->vq_int = 0.0f;
    m->vd_int_q31 = m->vq_int_q31 = 0;
    m->vd_int_q15 = m->vq_int_q15 = 0;
    m->hall.invalid_count = 0U;
    m->hall.sequence_error_count = 0U;
    m->hall.recovery_valid_ticks = 0U;
}

void motor_raise_fault_from_task(MotorRuntime *m, motor_fault_t fault) {
    if (!m) return;
    if (m->fault==MOTOR_FAULT_NONE) m->fault=fault;
    motor_stop(m);
    uint32_t primask=__get_PRIMASK();
    __disable_irq();
    s_pending_fault_mask |= (1UL << (uint32_t)m->id);
    if(!primask)__enable_irq();
}
void motor_request_fault_from_isr(MotorRuntime *m, motor_fault_t fault) {
    if (m->fault==MOTOR_FAULT_NONE) m->fault=fault;
    m->pwm_tim->BDTR &= ~TIM_BDTR_MOE; m->pwm_enabled=false; s_pending_fault_mask |= (1UL << (uint32_t)m->id);
}
uint32_t motor_take_pending_fault_mask(void) {
    uint32_t primask=__get_PRIMASK(); __disable_irq(); uint32_t mask=s_pending_fault_mask; s_pending_fault_mask=0U; if(!primask)__enable_irq(); return mask;
}

bool motor_select_sensor_mode(MotorRuntime *m, uint8_t mode) {
    if (m == NULL || mode > SENSOR_MODE_ENCODER) return false;
    motor_stop(m);
    if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER) return false;
    /* Upstream VESC keeps detection as explicit blocking COMM_DETECT_* work.
       AUTO is therefore not a background sensor state-machine here; use
       COMM_DETECT_APPLY_ALL_FOC or the standard Hall/encoder detect commands. */
    if (mode == SENSOR_MODE_AUTO) return false;
    m->sensor_request_mode = mode;
    m->foc_sensor_mode = (mode == SENSOR_MODE_ENCODER) ? FOC_SENSOR_MODE_ENCODER_AB : FOC_SENSOR_MODE_HALL;
    m->stats.tachometer_source_valid = false;
    if (m->id == MOTOR_LEFT && mode == SENSOR_MODE_ENCODER) {
        m->encoder.synced=false; m->encoder.motion_proved=false;
        m->encoder.sync_active=false; m->encoder.speed_sample_valid=false;
    }
    motor_hw_configure_sensor(m, mode);
    if (mode == SENSOR_MODE_HALL) motor_hall_edge_isr(m);
    return true;
}

int32_t motor_encoder_extended_count(MotorRuntime *m) {
    if (m == NULL || m->id != MOTOR_LEFT) return 0;
    uint32_t primask=__get_PRIMASK(); __disable_irq(); int32_t turns=m->encoder.turns; uint16_t cnt=motor_hw_encoder_cnt(); if(!primask)__enable_irq();
    return turns*(int32_t)m->encoder.cpr + (int32_t)cnt;
}

static void update_encoder_speed_position(MotorRuntime *m) {
    if (m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER) return;
    int32_t ext=motor_encoder_extended_count(m); m->encoder.extended_count=ext;
    if (!m->encoder.speed_sample_valid) {
        m->encoder.prev_extended_count=ext;
        m->encoder.speed_sample_valid=true;
        m->mech_rpm=0.0f; m->erpm=0.0f;
    }
    int32_t d=ext-m->encoder.prev_extended_count;
    m->encoder.prev_extended_count=ext;
    m->mech_rpm=((float)d*60.0f*1000.0f)/(float)m->encoder.cpr;
    m->erpm=m->mech_rpm*(float)m->pole_pairs;
    m->position_deg=((float)(ext-m->encoder.mechanical_zero_count)*360.0f)/(float)m->encoder.cpr;
    /* m_invert_direction changes the external VESC coordinate system as well
       as torque polarity. This keeps positive RPM/POS feedback consistent with
       a positive command after the physical motor direction is inverted. */
    if (m->invert_direction) { m->mech_rpm=-m->mech_rpm; m->erpm=-m->erpm; m->position_deg=-m->position_deg; }
}

static void update_hall_speed_position(MotorRuntime *m) {
    if (m->sensor_mode != SENSOR_MODE_HALL || !m->hall.valid || m->hall.period_cycles==0U) return;
    uint32_t age=DWT->CYCCNT-m->hall.edge_cycle;
    if (age > (CPU_CLOCK_HZ/5U)) { m->erpm=0.0f; m->mech_rpm=0.0f; return; }
    float edge_hz=(float)CPU_CLOCK_HZ/(float)m->hall.period_cycles;
    m->erpm=(float)m->hall.direction*edge_hz*10.0f;
    m->mech_rpm=m->erpm/(float)m->pole_pairs;
    m->position_deg=((float)m->hall.edge_count*60.0f)/(float)m->pole_pairs;
    if (m->invert_direction) { m->mech_rpm=-m->mech_rpm; m->erpm=-m->erpm; m->position_deg=-m->position_deg; }
}

void motor_rpm_update_1khz(MotorRuntime *m) {
    if (m == NULL) return;

    /* Speed feedback follows the configured FOC phase strategy, not merely
       whichever GPIO peripheral happens to be initialized. SENSORLESS must
       never accidentally report Hall RPM just because the Hall pins are left
       configured as harmless inputs on this board. */
    if (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
        m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) {
        update_encoder_speed_position(m);
    } else if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL) {
        update_hall_speed_position(m);
    } else if (m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
        m->erpm = (float)m->pll_erpm_q16 / 65536.0f;
        m->mech_rpm = (m->pole_pairs > 0U) ? (m->erpm / (float)m->pole_pairs) : 0.0f;
        if (m->invert_direction) {
            m->erpm = -m->erpm;
            m->mech_rpm = -m->mech_rpm;
        }
        /* SENSORLESS has no absolute mechanical counter. Integrate the
           observer-derived mechanical speed at this fixed 1 kHz service rate
           so VESC tachometer/distance/odometer telemetry remains cumulative
           instead of freezing while RPM is valid.  rpm * 360/60 * 1 ms =
           rpm * 0.006 degree per service tick. */
        m->position_deg += m->mech_rpm * 0.006f;
    } else {
        m->erpm = 0.0f;
        m->mech_rpm = 0.0f;
    }
    m->erpm_int = (int32_t)m->erpm;
}

static float lp(float oldv, float newv, float a) { return oldv + a*(newv-oldv); }

/* Current VESC online motor-resistance estimator, intentionally kept at 1 kHz
 * on Cortex-M3. It is an estimate/diagnostic state and does not silently alter
 * the configured observer resistance. */
static void update_res_estimator_1khz(MotorRuntime *m) {
    if (!m) return;
    const float r_nom = fmaxf(m->foc_motor_r, 1.0e-5f);
    if (!isfinite(m->res_est_state_ohm) || m->res_est_state_ohm < 0.25f*r_nom ||
        m->res_est_state_ohm > 3.0f*r_nom) {
        m->res_est_state_ohm = r_nom;
        m->res_est_ohm = r_nom;
        m->res_est_valid = false;
    }
    if (!m->pwm_enabled || !m->observer_valid || m->detect.busy) return;

    const float ia = m->ia;
    const float ib = m->ib;
    const float i_alpha = ia;
    const float i_beta = (ia + 2.0f*ib) * 0.57735026919f;
    const float i2 = i_alpha*i_alpha + i_beta*i_beta;
    if (i2 < 0.25f) return; /* avoid adapting on ADC noise near zero current */

    const float gain = 0.00002f;
    const float l = fmaxf(m->foc_motor_l, 1.0e-8f);
    const float r_est = m->res_est_state_ohm - 0.5f*gain*l*i2;
    const float v_alpha = (float)m->observer_v_alpha_q15_prev * (FOC_VOLTAGE_Q_BASE_V / 32768.0f);
    const float v_beta = (float)m->observer_v_beta_q15_prev * (FOC_VOLTAGE_Q_BASE_V / 32768.0f);
    const float omega = m->observer_speed_rad_s;
    const float x1 = m->observer_flux_alpha;
    const float x2 = m->observer_flux_beta;
    const float res_dot = -gain * (r_est*i2 + omega*(i_beta*x1 - i_alpha*x2) -
                                    (i_alpha*v_alpha + i_beta*v_beta));
    m->res_est_state_ohm += res_dot * 0.001f;
    m->res_est_state_ohm = foc_clampf(m->res_est_state_ohm, 0.25f*r_nom, 3.0f*r_nom);
    m->res_est_ohm = foc_clampf(m->res_est_state_ohm - 0.5f*gain*l*i2,
                                0.25f*r_nom, 3.0f*r_nom);
    m->res_est_valid = isfinite(m->res_est_ohm);
}

void motor_slow_update_1khz(MotorRuntime *m, uint32_t now_ms) {
    if (m->control_mode==MOTOR_CTRL_OPENLOOP || m->control_mode==MOTOR_CTRL_OPENLOOP_DUTY) {
        int32_t st=(int32_t)lrintf(m->openloop_command_erpm*(65536.0f/60.0f)*0.001f);
        m->openloop_command_phase_u16=(uint16_t)(m->openloop_command_phase_u16+st);
    }
    m->vbus=((float)m->vbus_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->ia=((float)m->ia_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->ib=((float)m->ib_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->ic=((float)m->ic_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->id_meas=((float)m->id_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->iq_meas=((float)m->iq_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->foc_fw_current_now=((float)m->foc_fw_current_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->id_filter=((float)m->id_filter_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->iq_filter=((float)m->iq_filter_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->vd=((float)m->vd_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f; m->vq=((float)m->vq_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->vd_filter=lp(m->vd_filter,m->vd,FOC_CURRENT_FILTER_CONST); m->vq_filter=lp(m->vq_filter,m->vq,FOC_CURRENT_FILTER_CONST);
    m->duty_u=(float)m->duty_u_q15/32768.0f; m->duty_v=(float)m->duty_v_q15/32768.0f; m->duty_w=(float)m->duty_w_q15/32768.0f;
    float duty_dev_u = fabsf(m->duty_u - 0.5f);
    float duty_dev_v = fabsf(m->duty_v - 0.5f);
    float duty_dev_w = fabsf(m->duty_w - 0.5f);
    m->duty_now = 2.0f * fmaxf(duty_dev_u, fmaxf(duty_dev_v, duty_dev_w));
    /* VESC FOC duty sign follows the applied q-axis voltage, not torque
       current. During regenerative braking Iq can reverse while Vq (and shaft
       electrical direction) keeps its sign. */
    if (m->vq_filter < 0.0f) {
        m->duty_now = -m->duty_now;
    }
    float dc=((float)(m->dc_current_offset_counts-(int32_t)m->dc_current_raw))*m->dc_current_scale;
    m->dc_current_a=dc; m->dc_current_filter=lp(m->dc_current_filter,dc,DC_CURRENT_FILTER_CONST); m->input_current=m->dc_current_filter;
    {
        const float a = foc_clampf(m->input_current_map_filter, 0.0f, 1.0f);
        if (a > 0.0f) m->input_current_map_filtered_a = lp(m->input_current_map_filtered_a, m->input_current, a);
        else m->input_current_map_filtered_a = m->input_current;
    }
    m->vbus_filter=lp(m->vbus_filter,m->vbus,VBUS_FILTER_CONST);

    /* Board thermal proxy from the STM32F103 internal sensor. This runs in the
       1-kHz task only; the fast current ISR never performs temperature math. */
    {
        float t_board = 0.0f;
        if (motor_hw_board_temperature_c(&t_board)) {
            m->board_temp_c = t_board;
            if (!m->board_temp_valid) {
                m->board_temp_filter_c = t_board;
                m->board_temp_valid = true;
            } else {
                m->board_temp_filter_c = lp(m->board_temp_filter_c, t_board, 0.02f);
            }
        }
    }
    if (m->motor_type == MOTOR_TYPE_FOC) {
        float imag=sqrtf(m->id_filter*m->id_filter + m->iq_filter*m->iq_filter);
        /* Standard VESC FOC motor current is signed by Iq (torque direction).
           Battery power direction belongs to Iinput, not to this field. */
        m->motor_current=(m->iq_filter < 0.0f) ? -imag : imag;

        /* VESC-style motor-temperature (resistance) compensation. The F103
           port has no motor NTC, so the STM32 board-temperature proxy is the
           thermal input. comp_factor = 1 + 0.00386*(T - base_temp); R and Ki
           are scaled by it so the current loop and observer track copper drift. */
        {
            const volatile mc_configuration *cfg = mc_interface_get_configuration();
            if (cfg && cfg->foc_temp_comp && m->board_temp_valid) {
                const float comp = 1.0f + 0.00386f * (m->board_temp_filter_c - cfg->foc_temp_comp_base_temp);
                m->res_temp_comp_ohm = m->foc_motor_r * comp;
                m->current_ki_temp_comp = m->current_ki * comp;
            } else {
                m->res_temp_comp_ohm = m->foc_motor_r;
                m->current_ki_temp_comp = m->current_ki;
            }
        }
    }

    /* setup_stats is sampled at 100 Hz. This is intentionally task-side: the
       Cortex-M3 hard FOC path stays fixed-point and telemetry/statistics never
       lengthen the 16-kHz ADC ISR. */
    {
        unsigned si=(unsigned)m->id;
        if (++s_setup_stats_div[si] >= 10U) {
            s_setup_stats_div[si]=0U;
            setup_stats *st=&s_setup_stats[si];
            double speed=(double)fabsf(m->mech_rpm);
            double power=(double)fabsf(m->vbus_filter*m->input_current);
            double current=(double)fabsf(m->motor_current);
            st->samples += 1.0;
            st->speed_sum += speed;
            st->power_sum += power;
            st->current_sum += current;
            if (speed > (double)st->max_speed) st->max_speed=(float)speed;
            if (power > (double)st->max_power) st->max_power=(float)power;
            if (current > (double)st->max_current) st->max_current=(float)current;
            if (m->board_temp_valid) {
                st->temp_mos_sum += (double)m->board_temp_filter_c;
                if (m->board_temp_filter_c > st->max_temp_mos) st->max_temp_mos=m->board_temp_filter_c;
            }
            if (isfinite(s_temp_motor_override)) {
                st->temp_motor_sum += (double)s_temp_motor_override;
                if (s_temp_motor_override > st->max_temp_motor) st->max_temp_motor=s_temp_motor_override;
            }
        }
    }

    /* Observer is a FOC backend function. The fixed-point observer keeps the same ADC/current
       telemetry but does not run the flux observer. */
    if (m->motor_type == MOTOR_TYPE_FOC) foc_observer_update_1khz(m);
    if (m->motor_type == MOTOR_TYPE_FOC) update_res_estimator_1khz(m);
    if (m->id == MOTOR_LEFT &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB || m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) &&
        m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.synced && m->observer_valid) {
        /* Source switching is owned by the 16-kHz phase selector using the
           fast corrected speed estimate. Task-side code only performs the ABI
           counter rebase while observer mode is active. */
        if (!m->using_encoder) foc_encoder_ab_sync_from_observer(m);
    }

    m->rotor_elec_deg=((float)motor_sensor_electrical_phase_u16(m)*360.0f)/65536.0f;

    (void)now_ms; /* command timeout is global and handled by motor service. */
    if (m->fault==MOTOR_FAULT_HALL_INVALID && m->foc_sensor_mode==FOC_SENSOR_MODE_HALL &&
        m->sensor_mode==SENSOR_MODE_HALL && !m->pwm_enabled) {
        /* Recover a latched Hall glitch only after 50 consecutive 1-kHz valid
           observations. This keeps real 000/111/unmapped faults latched while
           allowing a valid stationary Hall state to restart without reboot. */
        motor_hall_edge_isr(m);
        if (m->hall.valid && m->hall.invalid_count==0U && m->hall.sequence_error_count==0U) {
            if (m->hall.recovery_valid_ticks < 50U) m->hall.recovery_valid_ticks++;
            if (m->hall.recovery_valid_ticks >= 50U) motor_clear_fault(m);
        } else {
            m->hall.recovery_valid_ticks=0U;
        }
    }
    if (m->fault != MOTOR_FAULT_NONE && !foc_calibration_in_progress()) { motor_hw_set_pwm_enabled(m,false); return; }
    /* VESC offset-calibration mode bit2: when the motor is stopped (state OFF)
       and no calibration is in progress, periodically re-measure the current
       offsets so drift from temperature/aging does not accumulate. This mirrors
       upstream's motor-stopped DC-offset recalibration. It is gated behind
       foc_offsets_cal_mode bit2 and never preempts a running motor. */
    {
        const volatile mc_configuration *cfg = mc_interface_get_configuration();
        if (cfg && cfg->foc_calibrate_on_boot &&
            (cfg->foc_offsets_cal_mode & (1u << 2)) &&
            m->state == MC_STATE_OFF && !foc_calibration_in_progress()) {
            foc_request_recalibration();
        }
    }
    /* Sebelum driven-offset calibration selesai, service kalibrasi di timer_thread
       mengendalikan MOE. Jangan biarkan policy stopped-state mematikan zero-vector
       50% pada tick 1-kHz berikutnya. */
    if (!foc_calibration_done()) return;

    bool encoder_foc = m->id==MOTOR_LEFT &&
                       (m->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER_AB ||
                        m->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER);
    bool encoder_ready = !encoder_foc || m->encoder.synced || m->encoder.sync_active ||
                         m->openloop_started || m->phase_observer_override || m->detect.busy;
    const float min_hold_current = fmaxf(m->cc_min_current, 0.001f);
    const bool current_request_active =
        fabsf(m->id_target) >= min_hold_current ||
        fabsf(m->iq_target) >= min_hold_current ||
        m->current_off_delay_s > 0.0f;
    const bool modulation_mode =
        m->control_mode==MOTOR_CTRL_DUTY ||
        m->control_mode==MOTOR_CTRL_OPENLOOP ||
        m->control_mode==MOTOR_CTRL_OPENLOOP_PHASE ||
        m->control_mode==MOTOR_CTRL_OPENLOOP_DUTY ||
        m->control_mode==MOTOR_CTRL_OPENLOOP_DUTY_PHASE ||
        m->control_mode==MOTOR_CTRL_HANDBRAKE;
    bool wants_pwm = m->detect.busy || m->encoder.sync_active || m->openloop_started ||
                     m->phase_observer_override ||
                     (m->command_active && m->control_mode!=MOTOR_CTRL_OFF &&
                      (current_request_active || modulation_mode));
    if (foc_calibration_valid() && encoder_ready && wants_pwm && m->vbus_filter>=VBUS_MIN_RUN_V && m->vbus_filter<=VBUS_MAX_RUN_V) motor_hw_set_pwm_enabled(m,true);
    else if (!m->detect.busy && !m->command_active) motor_hw_set_pwm_enabled(m,false);
}

static float current_limit_pos(const MotorRuntime *m) {
    float lim = (m->lo_current_max_a > 0.0f) ? m->lo_current_max_a :
                (m->current_max_a * fmaxf(m->current_max_scale, 0.0f));
    return fmaxf(lim, 0.0f);
}

static float current_limit_neg(const MotorRuntime *m) {
    float lim = (m->lo_current_min_a < 0.0f) ? m->lo_current_min_a :
                (m->current_min_a * fmaxf(m->current_min_scale, 0.0f));
    return fminf(lim, 0.0f);
}

static float configured_iq_limit(const MotorRuntime *m) {
    return fmaxf(current_limit_pos(m), fabsf(current_limit_neg(m)));
}

static float map_clamped(float x, float in0, float in1, float out0, float out1) {
    if (fabsf(in1 - in0) < 1.0e-9f) return out1;
    float t = (x - in0) / (in1 - in0);
    t = foc_clampf(t, 0.0f, 1.0f);
    return out0 + (out1 - out0) * t;
}

#define ENCODER_SLIP_LIMIT_PHASE_U16 ((uint16_t)2731U) /* 15 electrical degrees */
#define ENCODER_SLIP_TIME_TICKS      ((uint16_t)500U)  /* 500 ms at 1 kHz */

static bool read_encoder_slip_snapshot(const MotorRuntime *m, uint16_t *obs_raw,
                                       uint16_t *enc, int32_t *erpm_q16) {
    if (!m || !obs_raw || !enc || !erpm_q16) return false;
    for (unsigned attempt = 0; attempt < 4U; attempt++) {
        uint32_t s1 = m->rt_snapshot_seq;
        if (s1 & 1U) continue;
        __DMB();
        uint16_t o = m->rt_snapshot.phase_observer_u16;
        uint16_t e = m->rt_snapshot.phase_encoder_u16;
        int32_t r = m->rt_snapshot.erpm_fast_q16;
        __DMB();
        uint32_t s2 = m->rt_snapshot_seq;
        if (s1 == s2 && !(s2 & 1U)) {
            *obs_raw = o; *enc = e; *erpm_q16 = r;
            return true;
        }
    }
    return false;
}

static void update_encoder_slip_fault_1khz(MotorRuntime *m) {
    if (!m) return;
    const bool enabled = (m->additional_faults & MCCONF_L_ADDITIONAL_FAULT_ENCODER_SLIP) != 0U;
    const bool encoder_mode = m->id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
         m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER);

    if (!enabled || !encoder_mode || !m->encoder.synced || !m->observer_valid ||
        !m->pwm_enabled || !m->command_active) {
        m->encoder_slip_bad_ticks = 0U;
        m->encoder_slip_error_phase = 0;
        m->encoder_slip_check_active = false;
        return;
    }

    uint16_t obs_raw = 0U, enc = 0U;
    int32_t erpm_q16 = 0;
    if (!read_encoder_slip_snapshot(m, &obs_raw, &enc, &erpm_q16)) return;

    const int32_t openloop_q16 = (int32_t)lrintf(fabsf(m->foc_openloop_rpm) * 1.10f * 65536.0f);
    int32_t erpm_abs = erpm_q16 >= 0 ? erpm_q16 : (erpm_q16 == INT32_MIN ? INT32_MAX : -erpm_q16);
    if (erpm_abs <= openloop_q16) {
        m->encoder_slip_bad_ticks = 0U;
        m->encoder_slip_error_phase = 0;
        m->encoder_slip_check_active = false;
        return;
    }

    /* Compare against the same PWM/sample-delay compensated observer phase
       used by high-speed control. This avoids false slip at high ERPM caused
       only by the observer offset/half-cycle delay compensation. */
    const int64_t num = (int64_t)erpm_q16 * (int64_t)m->observer_offset_factor_q15;
    const int64_t den = 60LL * (int64_t)FOC_ISR_EVENT_HZ * 32768LL;
    int64_t adv = den != 0 ? num / den : 0;
    if (adv > 32767) adv = 32767;
    if (adv < -32768) adv = -32768;
    uint16_t obs = (uint16_t)(obs_raw + (int16_t)adv);
    int16_t diff = (int16_t)(enc - obs);
    m->encoder_slip_error_phase = diff;
    m->encoder_slip_check_active = true;

    uint16_t err = (uint16_t)(diff < 0 ? -(int32_t)diff : diff);
    if (err > ENCODER_SLIP_LIMIT_PHASE_U16) {
        if (m->encoder_slip_bad_ticks < UINT16_MAX) m->encoder_slip_bad_ticks++;
        if (m->encoder_slip_bad_ticks >= ENCODER_SLIP_TIME_TICKS) {
            motor_raise_fault_from_task(m, MOTOR_FAULT_ENCODER_SLIP);
        }
    } else {
        m->encoder_slip_bad_ticks = 0U;
    }
}

/* Reduced VESC-style override-limit update. Board temperature, RPM, duty,
 * battery/input and watt limits are evaluated at 1 kHz. BMS/CAN limits remain
 * absent because this PCB has no such backend. The hard ISR only consumes the
 * resulting lo_* values and therefore stays deterministic/fixed-point. */
static void update_runtime_limits_1khz(MotorRuntime *m) {
    if (!m) return;

    /* Do not latch optional speed faults while current-offset calibration has not
     * completed successfully. The observer and Hall/encoder state are untrusted
     * before driven offsets exist, and a stale ABS_OVERSPEED fault permanently
     * blocks the safe 50% zero-vector MOE handshake needed to calibrate.
     * Checking s_cal_done (not s_cal_valid) keeps the guard active both during
     * the in-progress boot calibration and after it fails, so the 1-kHz task
     * cannot re-raise speed faults while recalibration is attempted. */
    if (!foc_calibration_done()) {
        m->erpm_fault_filter = 0.0f;
        if (m->fault == MOTOR_FAULT_ABS_OVERSPEED ||
            m->fault == MOTOR_FAULT_OVERSPEED ||
            m->fault == MOTOR_FAULT_UNDERSPEED) {
            m->fault = MOTOR_FAULT_NONE;
        }
        return;
    }

    const float base_max = fmaxf(0.0f, m->current_max_a * fmaxf(m->current_max_scale, 0.0f));
    const float base_min = fminf(0.0f, m->current_min_a * fmaxf(m->current_min_scale, 0.0f));
    float lo_max = base_max;
    float lo_min = base_min;

    /* Current VESC uses a deliberately slower speed for optional hard RPM
       faults. Keep the limiter on the fresh 1-kHz RPM, but filter fault RPM so
       one estimator spike cannot latch an additional-fault condition. */
    m->erpm_fault_filter = lp(m->erpm_fault_filter, m->erpm, 0.02f);
    update_encoder_slip_fault_1khz(m);
    if (m->fault != MOTOR_FAULT_NONE) return;
    if ((m->additional_faults & MCCONF_L_ADDITIONAL_FAULT_OVERSPEED) != 0U &&
        m->erpm_fault_filter > m->max_erpm) {
        motor_raise_fault_from_task(m, MOTOR_FAULT_OVERSPEED);
        return;
    }
    if ((m->additional_faults & MCCONF_L_ADDITIONAL_FAULT_UNDERSPEED) != 0U &&
        m->erpm_fault_filter < m->min_erpm) {
        motor_raise_fault_from_task(m, MOTOR_FAULT_UNDERSPEED);
        return;
    }
    if ((m->additional_faults & MCCONF_L_ADDITIONAL_FAULT_ABS_SPEED) != 0U) {
        const float abs_lim = fmaxf(fabsf(m->min_erpm), fabsf(m->max_erpm));
        if (fabsf(m->erpm_fault_filter) > abs_lim) {
            motor_raise_fault_from_task(m, MOTOR_FAULT_ABS_OVERSPEED);
            return;
        }
    }

    /* VESC-style thermal current limiting. The board has no MOSFET NTC, so
       the FET channel is intentionally a board/MCU temperature proxy. A real
       motor-temperature sensor can still drive the existing override API. */
    if (m->board_temp_valid) {
        const float t = m->board_temp_filter_c;
        if (t > (m->temp_fet_end - 0.1f)) {
            m->lo_current_max_a = 0.0f;
            m->lo_current_min_a = 0.0f;
            motor_raise_fault_from_task(m, MOTOR_FAULT_OVER_TEMP_BOARD);
            return;
        } else if (t >= (m->temp_fet_start + 0.1f)) {
            const float maxc0 = fmaxf(fabsf(base_max), fabsf(base_min));
            const float maxc = mc_math_thermal_current_limit(maxc0, t, m->temp_fet_start, m->temp_fet_end);
            if (lo_max > maxc) lo_max = maxc;
            if (fabsf(lo_min) > maxc) lo_min = -maxc;
        }

        /* l_temp_accel_dec moves acceleration-only thresholds toward 25 C,
           preserving more braking authority close to a thermal limit. */
        const float accel_lim = mc_math_thermal_accel_limit(base_max, t,
                                                               m->temp_fet_start, m->temp_fet_end,
                                                               m->temp_accel_dec);
        lo_max = fminf(lo_max, accel_lim);
    }

    if (isfinite(s_temp_motor_override)) {
        const float t = s_temp_motor_override;
        if (t > (m->temp_motor_end - 0.1f)) {
            m->lo_current_max_a = 0.0f;
            m->lo_current_min_a = 0.0f;
            motor_raise_fault_from_task(m, MOTOR_FAULT_OVER_TEMP_MOTOR);
            return;
        } else if (t >= (m->temp_motor_start + 0.1f)) {
            const float maxc0 = fmaxf(fabsf(base_max), fabsf(base_min));
            const float maxc = mc_math_thermal_current_limit(maxc0, t, m->temp_motor_start, m->temp_motor_end);
            if (lo_max > maxc) lo_max = maxc;
            if (fabsf(lo_min) > maxc) lo_min = -maxc;
        }
        const float accel_lim = mc_math_thermal_accel_limit(base_max, t,
                                                               m->temp_motor_start, m->temp_motor_end,
                                                               m->temp_accel_dec);
        lo_max = fminf(lo_max, accel_lim);
    }

    const float erpm_start = foc_clampf(m->erpm_start, 0.0f, 1.0f);
    if (m->max_erpm > 1.0f && m->erpm > m->max_erpm * erpm_start) {
        lo_max = fminf(lo_max, map_clamped(m->erpm, m->max_erpm * erpm_start,
                                           m->max_erpm, lo_max, 0.0f));
    }
    if (m->min_erpm < -1.0f && m->erpm < m->min_erpm * erpm_start) {
        lo_min = fmaxf(lo_min, map_clamped(m->erpm, m->min_erpm * erpm_start,
                                           m->min_erpm, lo_min, 0.0f));
    }

    /* VESC foc_start_curr_dec limits acceleration current around standstill.
       The later sign-aware clamp swaps lo_max/lo_min with shaft direction, so
       braking authority is not reduced by this startup-current feature. */
    const float rpm_abs = fabsf(m->erpm);
    const float start_lim = mc_math_start_current_limit(base_max, rpm_abs,
                                                         m->foc_start_curr_dec,
                                                         m->foc_start_curr_dec_rpm);
    lo_max = fminf(lo_max, start_lim);

    /* VESC l_duty_start begins tapering torque-producing current before the
       configured modulation ceiling. Preserve opposite-sign braking current. */
    const float dstart = foc_clampf(m->duty_start, 0.0f, 1.0f) * fmaxf(m->max_duty, 0.001f);
    const float dabs = fabsf(m->duty_now);
    if (dabs > dstart && m->max_duty > dstart + 1.0e-4f) {
        const float scale = map_clamped(dabs, dstart, m->max_duty, 1.0f, 0.0f);
        if (m->duty_now >= 0.0f) lo_max *= scale;
        else lo_min *= scale;
    }

    m->lo_current_max_a = fmaxf(lo_max, 0.0f);
    m->lo_current_min_a = fminf(lo_min, 0.0f);

    const float v = (m->vbus_filter > 0.1f) ? m->vbus_filter : m->vbus;
    float in_max = mc_math_battery_cut_input_max(m->input_current_max_a, v,
                                                  m->battery_cut_start, m->battery_cut_end);
    float in_min = mc_math_battery_regen_cut_input_min(m->input_current_min_a, v,
                                                        m->battery_regen_cut_start,
                                                        m->battery_regen_cut_end);
    if (v > 0.5f) {
        if (isfinite(m->watt_max) && m->watt_max >= 0.0f) in_max = fminf(in_max, m->watt_max / v);
        if (isfinite(m->watt_min) && m->watt_min <= 0.0f) in_min = fmaxf(in_min, m->watt_min / v);
    }

    /* VESC l_in_current_map_start semantics, but driven by the hoverboard's
       physical DC-current measurement. Reduce positive torque capability
       smoothly before the input limit instead of waiting for clipping. */
    m->input_current_map_limit_a = m->lo_current_max_a;
    const float map_start = foc_clampf(m->input_current_map_start, 0.0f, 1.0f);
    const float map_in_max = fmaxf(0.0f, in_max);
    if (map_start < 0.98f && map_in_max > 0.05f && m->input_current_map_filtered_a > 0.0f) {
        const float frac = m->input_current_map_filtered_a / map_in_max;
        if (frac > map_start) {
            const float scale = foc_clampf((1.0f - frac) / fmaxf(1.0f - map_start, 0.001f), 0.0f, 1.0f);
            const float cap = base_max * scale;
            m->lo_current_max_a = fminf(m->lo_current_max_a, cap);
            m->input_current_map_limit_a = m->lo_current_max_a;
        }
    }
    m->lo_input_current_max_a = fmaxf(0.0f, in_max);
    m->lo_input_current_min_a = fminf(0.0f, in_min);
}

/* Battery/input-current limiter. The numeric core lives in mc_math.c so it
 * can be unit-tested independently from RTOS/hardware state. */
static float limit_iq_by_input_current(MotorRuntime *m, float iq) {
    if (m == NULL || iq == 0.0f) return iq;
    return mc_math_limit_input_current(iq, m->erpm, m->duty_now, m->input_current,
                                       m->lo_input_current_max_a, m->lo_input_current_min_a);
}

static float speed_feedback_erpm(const MotorRuntime *m) {
    if (m == NULL) return 0.0f;
    float rpm;
    switch (m->speed_pid_source) {
    case S_PID_SPEED_SRC_FAST:
        rpm = (float)m->speed_est_fast_erpm_q16 / 65536.0f;
        break;
    case S_PID_SPEED_SRC_FASTER:
        rpm = (float)m->speed_est_faster_erpm_q16 / 65536.0f;
        break;
    case S_PID_SPEED_SRC_PLL:
    default:
        rpm = (float)m->pll_erpm_q16 / 65536.0f;
        break;
    }
    /* The fast/PLL estimators are kept in the physical electrical direction,
       while VESC-facing RPM/position feedback is expressed in the external
       (m_invert_direction-aware) coordinate system.  Compute speed-loop error
       in that same external coordinate; the final Iq is direction-multiplied
       once later in motor_pid_update_1khz(). */
    return m->invert_direction ? -rpm : rpm;
}

static float step_towards_f(float value, float target, float step) {
    if (step <= 0.0f) return target;
    if (value < target) return fminf(value + step, target);
    if (value > target) return fmaxf(value - step, target);
    return target;
}

static float normalized_output_to_current(const MotorRuntime *m, float output) {
    output = foc_clampf(output, -1.0f, 1.0f);
    if (output >= 0.0f) return output * current_limit_pos(m);
    return (-output) * current_limit_neg(m);
}

/* VESC speed PID semantics:
 * - optional ERPM/s input ramp
 * - selectable PLL/fast/faster speed source
 * - release below s_pid_min_erpm
 * - normalized PID output with the historical 1/20 gain scaling
 * - optional prevention of active braking.
 */
static float speed_pid_step(MotorRuntime *m,float target_erpm) {
    const float dt = 0.001f;
    if (m == NULL) return 0.0f;

    target_erpm = foc_clampf(target_erpm, m->min_erpm, m->max_erpm);
    if (m->speed_pid_ramp_erpms_s > 0.0f) {
        m->speed_pid_set_erpm = step_towards_f(m->speed_pid_set_erpm, target_erpm,
                                               m->speed_pid_ramp_erpms_s * dt);
    } else {
        m->speed_pid_set_erpm = target_erpm;
    }
    m->speed_pid_set_erpm = foc_clampf(m->speed_pid_set_erpm, m->min_erpm, m->max_erpm);

    const float rpm = speed_feedback_erpm(m);
    const float err = m->speed_pid_set_erpm - rpm;

    if (fabsf(m->speed_pid_set_erpm) < m->speed_pid_min_erpm) {
        m->speed_pid.integrator = 0.0f;
        m->speed_pid.prev_error = err;
        m->speed_derivative_filtered = 0.0f;
        return 0.0f;
    }

    const float p = err * m->speed_pid.kp * (1.0f / 20.0f);
    const float d_raw = (err - m->speed_pid.prev_error) *
                        (m->speed_pid.kd / dt) * (1.0f / 20.0f);
    m->speed_pid.prev_error = err;
    const float df = foc_clampf(m->speed_kd_filter, 0.0f, 1.0f);
    m->speed_derivative_filtered += df * (d_raw - m->speed_derivative_filtered);

    float out = p + m->speed_pid.integrator + m->speed_derivative_filtered;
    out = foc_clampf(out, -1.0f, 1.0f);

    m->speed_pid.integrator += err * m->speed_pid.ki * dt * (1.0f / 20.0f);
    m->speed_pid.integrator = foc_clampf(m->speed_pid.integrator, -1.0f, 1.0f);
    if (m->speed_pid.ki < 1.0e-9f) m->speed_pid.integrator = 0.0f;

    if (!m->speed_pid_allow_braking) {
        if (rpm > 20.0f && out < 0.0f) out = 0.0f;
        if (rpm < -20.0f && out > 0.0f) out = 0.0f;
    }

    return normalized_output_to_current(m, out);
}

static float position_pid_step(MotorRuntime *m) {
    const float dt = 0.001f;
    if (m == NULL) return 0.0f;

    /* LEFT incremental A/B is the steering actuator on this target. Preserve
       its extended mechanical coordinate across 0/360 instead of applying the
       generic rotary shortest-path wrap, which can command a steering turn in
       the wrong direction near the wrap boundary. Hall/sensorless retain the
       normal circular VESC semantics. */
    const bool linear_position = m->id == MOTOR_LEFT &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
         m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER);
    const float now = linear_position ?
        (m->position_deg + m->position_offset_deg) :
        foc_wrap_deg(m->position_deg + m->position_offset_deg);
    float err = m->position_target_deg - now;
    if (!linear_position) {
        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
    }

    /* Match VESC position sign semantics for encoder configurations. */
    float err_sign = 1.0f;
    if (m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.inverted) err_sign = -1.0f;
    err *= err_sign;

    float kp = m->position_pid.kp;
    float ki = m->position_pid.ki;
    float kd = m->position_pid.kd;
    float kd_proc = m->position_kd_proc;
    if (m->position_gain_dec_angle > 0.1f && m->position_ang_div > 0.001f) {
        const float min_err = m->position_gain_dec_angle / m->position_ang_div;
        const float ae = fabsf(err);
        if (min_err > 1.0e-6f && ae < min_err) {
            const float scale = ae / min_err;
            kp *= scale; ki *= scale; kd *= scale; kd_proc *= scale;
        }
    }

    const float p_term = err * kp;
    m->position_pid.integrator += err * ki * dt;

    m->position_dt_integrator += dt;
    float d_term = 0.0f;
    if (err != m->position_pid.prev_error && m->position_dt_integrator > 0.0f) {
        d_term = (err - m->position_pid.prev_error) *
                 (kd / m->position_dt_integrator);
        m->position_dt_integrator = 0.0f;
    }
    const float df = foc_clampf(m->position_kd_filter, 0.0f, 1.0f);
    m->position_derivative_filtered += df * (d_term - m->position_derivative_filtered);

    m->position_dt_process_integrator += dt;
    float d_proc = 0.0f;
    if (now != m->position_prev_process_deg && m->position_dt_process_integrator > 0.0f) {
        float proc_diff = now - m->position_prev_process_deg;
        if (!linear_position) {
            while (proc_diff > 180.0f) proc_diff -= 360.0f;
            while (proc_diff < -180.0f) proc_diff += 360.0f;
        }
        d_proc = -proc_diff * err_sign *
                 (kd_proc / m->position_dt_process_integrator);
        m->position_dt_process_integrator = 0.0f;
    }
    m->position_derivative_proc_filtered +=
        df * (d_proc - m->position_derivative_proc_filtered);

    const float p_clip = foc_clampf(p_term, -1.0f, 1.0f);
    const float i_lim = fmaxf(1.0f - fabsf(p_clip), 0.0f);
    m->position_pid.integrator =
        foc_clampf(m->position_pid.integrator, -i_lim, i_lim);

    m->position_pid.prev_error = err;
    m->position_prev_process_deg = now;

    float out = p_term + m->position_pid.integrator +
                m->position_derivative_filtered +
                m->position_derivative_proc_filtered;
    out = foc_clampf(out, -1.0f, 1.0f);
    return normalized_output_to_current(m, out);
}

static inline int sign_i32_mc(int32_t v) {
    return (v > 0) - (v < 0);
}

/* VESC duty controller semantics. The dedicated PI is used only when an
   already-generated duty must be reduced safely. During normal/ramp-up duty
   control the current request goes to the allowed motor-current limit while
   the FOC voltage circle is lowered to the requested modulation. */
static float duty_control_step_1khz(MotorRuntime *m, float duty_set,
                                    float current_max_for_duty,
                                    bool brake_zero_guard) {
    if (!m) return 0.0f;
    current_max_for_duty = fabsf(current_max_for_duty);
    if (current_max_for_duty < 1.0e-3f) {
        m->duty_limit_now = m->max_duty;
        m->force_zero_modulation = false;
        m->duty_was_pi = false;
        return 0.0f;
    }

    duty_set = foc_clampf(duty_set, -m->max_duty, m->max_duty);
    const float duty_now = m->duty_now;
    const float duty_abs = fabsf(duty_now);
    const int sign_now = (duty_now > 0.0f) - (duty_now < 0.0f);
    const int sign_last = (m->duty_pi_duty_last > 0.0f) - (m->duty_pi_duty_last < 0.0f);

    const bool downramp = fabsf(duty_set) < (duty_abs - 0.01f) &&
                          (!m->duty_was_pi || sign_last == sign_now);
    if (downramp) {
        /* Do not hard-reduce the voltage circle while duty is above target.
           The PI produces the current command that brings duty down smoothly. */
        m->duty_limit_now = m->max_duty;
        m->force_zero_modulation = false;
        m->duty_pi_duty_last = duty_now;
        m->duty_was_pi = true;

        /* Upstream resets the integrator sign only in ordinary duty mode. In
           brake zero-cross mode continuity is retained to avoid a current jump. */
        if (!brake_zero_guard) {
            if (duty_now > 0.0f && m->duty_pid.integrator > 0.0f) {
                m->duty_pid.integrator = 0.0f;
            } else if (duty_now < 0.0f && m->duty_pid.integrator < 0.0f) {
                m->duty_pid.integrator = 0.0f;
            }
        }

        const float err = duty_set - duty_now;
        const float scale = 1.0f / fmaxf(m->vbus_filter, 1.0f);
        m->duty_pid.integrator += err * (m->foc_duty_dowmramp_ki * 0.001f) * scale;
        m->duty_pid.integrator = foc_clampf(m->duty_pid.integrator, -1.0f, 1.0f);
        float out = err * m->foc_duty_dowmramp_kp * scale + m->duty_pid.integrator;
        out = foc_clampf(out, -1.0f, 1.0f);
        return out * current_max_for_duty;
    }

    /* Match upstream hand-off: initialize the normalized duty I-term from the
       actual q current before leaving PI duty reduction. */
    m->duty_pid.integrator = foc_clampf(m->iq_filter / current_max_for_duty, -1.0f, 1.0f);
    m->duty_was_pi = false;

    if (brake_zero_guard && fabsf(duty_set) < 0.001f) {
        /* At the zero-duty target the ISR emits a centered zero vector. Keep a
           braking-current sign request for diagnostics/current limiting even
           though the voltage vector itself is forced to zero. */
        m->duty_limit_now = 0.0f;
        m->force_zero_modulation = true;
        int dir = sign_i32_mc(m->speed_est_fast_erpm_q16);
        if (dir == 0) dir = sign_i32_mc(m->brake_vq_before_q15);
        return -(float)dir * current_max_for_duty;
    }

    m->force_zero_modulation = false;
    if (fabsf(duty_set) < m->min_duty) {
        m->duty_limit_now = m->max_duty;
        return 0.0f;
    }

    m->duty_limit_now = fabsf(duty_set);
    return (duty_set > 0.0f ? 1.0f : -1.0f) * current_max_for_duty;
}

/* VESC current-brake zero-cross guard adapted to the 1-kHz service loop. A
   single service tick spans 16 hard FOC samples, so BR_ZERO_MIN_HOLD_TICKS=1
   exceeds upstream's minimum ten current-control cycles. The guard is kept
   active while short-circuit/zero-modulation braking current has not reached
   the requested brake current. */
static bool brake_zero_guard_1khz(MotorRuntime *m, float brake_target_a) {
    if (!m) return false;
    const uint8_t BR_ZERO_MIN_HOLD_TICKS = 1U;
    const int32_t speed_now = m->speed_est_fast_erpm_q16;
    const int32_t vq_now = m->vq_q15;
    const int speed_sign_now = sign_i32_mc(speed_now);
    const int speed_sign_prev = sign_i32_mc(m->brake_speed_before_q16);
    const int vq_sign_now = sign_i32_mc(vq_now);
    const int vq_sign_prev = sign_i32_mc(m->brake_vq_before_q15);
    const float current_abs = sqrtf(m->id_filter * m->id_filter + m->iq_filter * m->iq_filter);
    const bool need_more_brake_current = current_abs < fabsf(brake_target_a);

    const bool transition = speed_sign_now != speed_sign_prev ||
                            vq_sign_now != vq_sign_prev ||
                            fabsf(m->duty_now) < 0.001f ||
                            m->brake_zero_hold_ticks < BR_ZERO_MIN_HOLD_TICKS;

    if (transition && need_more_brake_current) {
        m->brake_zero_active = true;
        m->brake_zero_hold_ticks = 0U;
    } else if (m->brake_zero_hold_ticks < BR_ZERO_MIN_HOLD_TICKS) {
        m->brake_zero_active = true;
        m->brake_zero_hold_ticks++;
    } else {
        m->brake_zero_active = false;
    }

    m->brake_speed_before_q16 = speed_now;
    m->brake_vq_before_q15 = vq_now;
    return m->brake_zero_active;
}

static void apply_mtpa_1khz(MotorRuntime *m, float *id, float *iq) {
    if (!m || !id || !iq || m->foc_mtpa_mode == MTPA_MODE_OFF ||
        fabsf(m->foc_motor_ld_lq_diff) < 1.0e-12f) {
        if (m) m->mtpa_id_target = 0.0f;
        return;
    }

    const float diff = m->foc_motor_ld_lq_diff;
    const float lambda = m->foc_motor_flux_linkage;

    /* Current VESC MTPA semantics: calculate the reluctance-producing Id from
       target/measured Iq, then reduce the commanded Iq so MTPA rotates the
       requested current vector instead of silently increasing its magnitude.
       Field weakening is composed later in the 16-kHz ISR by selecting the
       larger-magnitude d-axis request rather than summing both negative Ids. */
    const float iq_cmd = *iq;
    const float iq_ref = (m->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED)
                       ? fminf(fabsf(iq_cmd), fabsf(m->iq_filter)) * ((iq_cmd < 0.0f) ? -1.0f : 1.0f)
                       : iq_cmd;
    const float term = lambda * lambda + 8.0f * (diff * iq_ref) * (diff * iq_ref);
    const float id_mtpa = (lambda - sqrtf(fmaxf(term, 0.0f))) / (4.0f * diff);
    const float iq_sq = iq_cmd * iq_cmd - id_mtpa * id_mtpa;
    *id = id_mtpa;
    *iq = ((iq_cmd < 0.0f) ? -1.0f : 1.0f) * sqrtf(fmaxf(iq_sq, 0.0f));
    m->mtpa_id_target = id_mtpa;
}

void motor_pid_update_1khz(MotorRuntime *m) {
    if (m==NULL || m->detect.busy) return;

    if (m->current_off_delay_s > 0.0f) {
        m->current_off_delay_s=fmaxf(0.0f,m->current_off_delay_s-0.001f);
    }
    update_runtime_limits_1khz(m);
    /* FW ramp/target generation is owned by the fixed-point 16-kHz FOC path.
       The ISR only raises this one-way request; float current-off timing stays
       in task context. */
    if (m->foc_fw_hold_request) {
        m->current_off_delay_s = fmaxf(m->current_off_delay_s, 1.0f);
        m->foc_fw_hold_request = false;
    }
    /* Ordinary control modes start each service tick with the configured
       voltage circle. Duty/brake control may lower this ceiling below. */
    m->duty_limit_now = m->max_duty;
    m->force_zero_modulation = false;
    foc_update_modulation_limit(m);

    if (m->fault!=MOTOR_FAULT_NONE || !m->command_active) {
        m->foc_fw_current_now = 0.0f;
        m->foc_fw_current_acc_q31 = 0;
        m->foc_fw_current_q15 = 0;
        m->foc_fw_duty_filter_q15 = 0;
        m->foc_fw_fast_active = false;
        m->mtpa_id_target = 0.0f;
        m->brake_zero_active = false;
        m->brake_zero_hold_ticks = 1U;
        motor_set_foc_targets(m,0.0f,0.0f);
        return;
    }
    /* Incremental AB is a valid FOC phase source only after this boot's
       runtime reference has been established. Hall remains usable immediately
       when its state is valid. */
    bool explicit_openloop=(m->control_mode==MOTOR_CTRL_OPENLOOP ||
                            m->control_mode==MOTOR_CTRL_OPENLOOP_PHASE ||
                            m->control_mode==MOTOR_CTRL_OPENLOOP_DUTY ||
                            m->control_mode==MOTOR_CTRL_OPENLOOP_DUTY_PHASE);
    const bool sensorless_foc = m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
    if (!explicit_openloop && sensorless_foc) {
        float direction_hint = 0.0f;
        float iq_hint = m->iq_target;
        switch (m->control_mode) {
        case MOTOR_CTRL_CURRENT:
            direction_hint = m->current_command_a;
            iq_hint = m->current_command_a;
            break;
        case MOTOR_CTRL_SPEED:
            direction_hint = m->speed_target_erpm;
            break;
        case MOTOR_CTRL_DUTY:
            direction_hint = m->duty_command;
            break;
        case MOTOR_CTRL_BRAKE_CURRENT:
        case MOTOR_CTRL_HANDBRAKE:
        case MOTOR_CTRL_POSITION:
            /* Pure sensorless has no absolute/zero-speed rotor reference and
               must never invent one. Brake/hold/position can proceed only
               when the observer is already valid from driven operation. */
            direction_hint = 0.0f;
            break;
        default:
            direction_hint = 0.0f;
            break;
        }

        const int32_t sl_speed_abs = m->speed_est_fast_erpm_q16 >= 0 ?
                                     m->speed_est_fast_erpm_q16 :
                                     (m->speed_est_fast_erpm_q16 == INT32_MIN ? INT32_MAX :
                                      -m->speed_est_fast_erpm_q16);
        const bool observer_min_ready = m->observer_valid &&
                                        sl_speed_abs >= m->foc_sl_erpm_start_q16;
        const bool observer_control_ready = m->observer_valid &&
                                            sl_speed_abs >= m->foc_sl_erpm_q16;
        if ((m->control_mode == MOTOR_CTRL_BRAKE_CURRENT ||
             m->control_mode == MOTOR_CTRL_HANDBRAKE ||
             m->control_mode == MOTOR_CTRL_POSITION) && !observer_min_ready) {
            foc_sensorless_startup_abort(m);
            m->mtpa_id_target = 0.0f;
            motor_set_foc_targets(m, 0.0f, 0.0f);
            return;
        }
        /* Once forced startup has begun it must keep owning the phase until
           foc_sensorless_startup_1khz() finishes its bounded observer blend.
           Do not bypass the blend merely because observer_control_ready became
           true during the forced sequence. */
        if (m->openloop_started || !observer_control_ready) {
            if (!foc_sensorless_startup_1khz(m, osKernelGetTickCount(),
                                             direction_hint, iq_hint)) {
                if (m->sensorless_start_failures >= 3U) {
                    motor_raise_fault_from_task(m, MOTOR_FAULT_SENSORLESS_OBSERVER);
                }
                return;
            }
        }
        if (observer_control_ready && !m->openloop_started) {
            m->sensorless_start_failures = 0U;
        }
    }
    if (!explicit_openloop && m->id==MOTOR_LEFT &&
        (m->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER_AB || m->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER) &&
        m->sensor_mode==SENSOR_MODE_ENCODER && !m->encoder.synced) {
        /* Incremental AB requires a runtime electrical reference. Use the
           proven observer/open-loop alignment path before switching to AB. */
        if (!foc_encoder_ab_startup_1khz(m, osKernelGetTickCount())) return;
        m->using_encoder = true;
    }
    float iq=0.0f;
    switch(m->control_mode) {
        case MOTOR_CTRL_CURRENT: iq=m->current_command_a; break;
        case MOTOR_CTRL_BRAKE_CURRENT: {
            const float brake_lim = fminf(fabsf(current_limit_neg(m)), configured_iq_limit(m));
            const float brake_target = fminf(fabsf(m->brake_current_a), brake_lim);
            if (brake_zero_guard_1khz(m, brake_target)) {
                iq = duty_control_step_1khz(m, 0.0f, brake_lim, true);
            } else {
                int dir = sign_i32_mc(m->speed_est_fast_erpm_q16);
                iq = -(float)dir * brake_target;
                m->duty_limit_now = m->max_duty;
                m->force_zero_modulation = false;
            }
            break;
        }
        case MOTOR_CTRL_SPEED: iq=speed_pid_step(m,m->speed_target_erpm); break;
        case MOTOR_CTRL_POSITION: iq=position_pid_step(m); break;
        case MOTOR_CTRL_DUTY:
            iq=duty_control_step_1khz(m, m->duty_command, configured_iq_limit(m), false);
            break;
        case MOTOR_CTRL_OPENLOOP:
            iq=m->current_command_a; break;
        case MOTOR_CTRL_OPENLOOP_PHASE:
            foc_update_modulation_limit(m);
            motor_set_foc_targets(m,m->current_command_a,0.0f); return;
        case MOTOR_CTRL_OPENLOOP_DUTY:
        case MOTOR_CTRL_OPENLOOP_DUTY_PHASE:
            /* Direct modulation is handled in the fixed-point ISR. */
            foc_update_modulation_limit(m);
            motor_set_foc_targets(m,0.0f,0.0f); return;
        case MOTOR_CTRL_HANDBRAKE:
            foc_update_modulation_limit(m);
            motor_set_foc_targets(m, m->handbrake_current_a, 0.0f); return;
        default: iq=0.0f; break;
    }
    /* duty_limit_now may have changed in duty/brake mode. Publish the new
       fixed-point voltage-circle coefficient before the next FOC ISR sample. */
    foc_update_modulation_limit(m);
    if (m->invert_direction) iq=-iq;

    float id = 0.0f;
    apply_mtpa_1khz(m, &id, &iq);

    /* VESC 7.x composition is completed in the hard loop. Keep this task-side
       request as MTPA-only Id + torque Iq; fast FW will select max-absolute Id,
       apply q-axis FW compensation, then re-apply the current circle. */
    iq = limit_iq_by_input_current(m, iq);

    /* Sign-aware motor-current limits preserve braking authority while
       preventing acceleration torque from crossing the computed lo_* bounds. */
    if (m->duty_now >= 0.0f) iq = foc_clampf(iq, current_limit_neg(m), current_limit_pos(m));
    else iq = foc_clampf(iq, -current_limit_pos(m), -current_limit_neg(m));

    const float current_abs = configured_iq_limit(m);
    id = foc_clampf(id, -current_abs, current_abs);
    const float iq_abs = sqrtf(fmaxf(current_abs * current_abs - id * id, 0.0f));
    iq = foc_clampf(iq, -iq_abs, iq_abs);

    /* VESC cc_min_current semantics: below the minimum useful current the
       bridge may release, but the outer controller command remains alive so
       speed/position control can re-engage automatically when error grows.
       current_off_delay keeps modulation alive temporarily after field
       weakening or an explicit compatibility API request. */
    const float release_threshold=fmaxf(m->cc_min_current,0.001f);
    if (m->current_off_delay_s<=0.0f &&
        sqrtf(id*id+iq*iq)<release_threshold) {
        id=0.0f; iq=0.0f;
    }
    m->foc_current_limit_q15 = amp_to_current_q15(current_abs);
    motor_set_foc_targets(m,id,iq);
}

/* ========================================================================
 * VESC master-compatible mc_interface wrappers
 * ======================================================================== */
typedef struct { osThreadId_t tid; uint8_t motor; } motor_thread_sel_t;
static motor_thread_sel_t s_motor_sel[16];
static bool s_mc_interface_inited=false;
static volatile bool s_mc_locked=false;
static volatile bool s_mc_lock_override_once=false;
static volatile uint32_t s_ignore_until[2]={0,0};
static void (*s_pwm_callback)(void)=NULL;
static void (* volatile s_sample_reply_func)(unsigned char *data, unsigned int len)=NULL;
static mc_configuration s_mcconf_mirror[2];
static volatile gnss_data s_gnss={0};
static uint64_t s_odometer[2]={0U,0U};
static float s_odometer_fraction_m[2]={0.0f,0.0f};
static bool s_wheel_speed_override=false;
static float s_wheel_speed_override_value=0.0f;

static int sel_index(osThreadId_t tid,bool alloc){
    if(!tid)return -1;
    for(int i=0;i<16;i++)if(s_motor_sel[i].tid==tid)return i;
    if(alloc){for(int i=0;i<16;i++)if(!s_motor_sel[i].tid){s_motor_sel[i].tid=tid;s_motor_sel[i].motor=1;return i;}}
    return -1;
}
void mc_interface_select_motor_thread(int motor) {
    if (motor < 1) {
        motor = 1;
    }
    if (motor > 2) {
        motor = 2;
    }

    osThreadId_t tid = osThreadGetId();
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    int i = sel_index(tid, true);
    if (i >= 0) {
        s_motor_sel[i].motor = (uint8_t)motor;
    }
    if (pm == 0U) {
        __enable_irq();
    }
}
int mc_interface_get_motor_thread(void){osThreadId_t tid=osThreadGetId();int i=sel_index(tid,false);return i>=0?s_motor_sel[i].motor:1;}
int mc_interface_motor_now(void){
    /* VESC: the motor active in the FOC ISR wins over the thread-selected one. */
    int isr_motor = mcpwm_foc_isr_motor();
    if (isr_motor == 1 || isr_motor == 2) {
        return isr_motor;
    }
    int selected = mc_interface_get_motor_thread();
    return selected == 2 ? 2 : 1;
}
MotorRuntime *mc_interface_motor_runtime_now(void){
    int m = mc_interface_motor_now();
    return motor_get(m == 2 ? MOTOR_RIGHT : MOTOR_LEFT);
}

static void mirror_from_runtime(const MotorRuntime*m,mc_configuration*c){
    memset(c,0,sizeof(*c));
    c->l_current_max=m->current_max_a;c->l_current_min=m->current_min_a;
    c->l_in_current_max=m->input_current_max_a;c->l_in_current_min=m->input_current_min_a;
    c->l_in_current_map_start=m->input_current_map_start;c->l_in_current_map_filter=m->input_current_map_filter;
    c->l_abs_current_max=m->abs_current_max_a;c->l_min_erpm=m->min_erpm;c->l_max_erpm=m->max_erpm;c->l_erpm_start=m->erpm_start;
    c->l_temp_fet_start=m->temp_fet_start;c->l_temp_fet_end=m->temp_fet_end;
    c->l_temp_motor_start=m->temp_motor_start;c->l_temp_motor_end=m->temp_motor_end;c->l_temp_accel_dec=m->temp_accel_dec;
    c->l_additional_faults=m->additional_faults;
    c->l_min_vin=m->min_vin;c->l_max_vin=m->max_vin;c->l_battery_cut_start=m->battery_cut_start;c->l_battery_cut_end=m->battery_cut_end;c->l_slow_abs_current=m->slow_abs_current;
    c->l_battery_regen_cut_start=m->battery_regen_cut_start;c->l_battery_regen_cut_end=m->battery_regen_cut_end;
    c->l_min_duty=m->min_duty;c->l_max_duty=m->max_duty;
    c->l_watt_max=m->watt_max;c->l_watt_min=m->watt_min;
    c->l_current_max_scale=m->current_max_scale;c->l_current_min_scale=m->current_min_scale;c->l_duty_start=m->duty_start;
    c->lo_current_max=m->lo_current_max_a;c->lo_current_min=m->lo_current_min_a;
    c->lo_in_current_max=m->lo_input_current_max_a;c->lo_in_current_min=m->lo_input_current_min_a;
    c->pwm_mode=m->pwm_mode;c->comm_mode=m->comm_mode;c->motor_type=MOTOR_TYPE_FOC;c->sensor_mode=SENSOR_MODE_SENSORED;
    {
        const int8_t legacy_hall[8] = { -1, 1, 3, 2, 5, 6, 4, -1 };
        memcpy(c->hall_table, legacy_hall, sizeof(c->hall_table));
    }
    c->sensor_mode=SENSOR_MODE_SENSORED;
    c->hall_sl_erpm=2000.0f;
    c->foc_current_kp=m->current_kp;c->foc_current_ki=m->current_ki;c->foc_f_zv=(float)VESC_FOC_F_ZV_HZ;c->foc_dt_us=m->foc_dt_us;
    c->foc_encoder_offset=(float)m->encoder.elec_offset_u16*360.0f/65536.0f;c->foc_encoder_inverted=m->encoder.inverted;c->foc_encoder_ratio=m->encoder.electrical_ratio;
    c->foc_motor_l=m->foc_motor_l;c->foc_motor_ld_lq_diff=m->foc_motor_ld_lq_diff;c->foc_motor_r=m->foc_motor_r;c->foc_motor_flux_linkage=m->foc_motor_flux_linkage;
    c->foc_observer_gain=m->foc_observer_gain;c->foc_observer_gain_slow=m->foc_observer_gain_slow;c->foc_observer_offset=m->foc_observer_offset;
    c->foc_sat_comp_mode=m->foc_sat_comp_mode;c->foc_sat_comp=m->foc_sat_comp;c->foc_observer_type=m->foc_observer_type;
    c->foc_duty_dowmramp_kp=m->foc_duty_dowmramp_kp;c->foc_duty_dowmramp_ki=m->foc_duty_dowmramp_ki;
    c->foc_start_curr_dec=m->foc_start_curr_dec;c->foc_start_curr_dec_rpm=m->foc_start_curr_dec_rpm;
    c->foc_short_ls_on_zero_duty=m->foc_short_ls_on_zero_duty;
    c->foc_current_filter_const=m->foc_current_filter_const;c->foc_cc_decoupling=m->foc_cc_decoupling;c->foc_mtpa_mode=m->foc_mtpa_mode;
    c->foc_fw_current_max=m->foc_fw_current_max;c->foc_fw_duty_start=m->foc_fw_duty_start;c->foc_fw_ramp_time=m->foc_fw_ramp_time;
    c->foc_fw_q_current_factor=m->foc_fw_q_current_factor;c->foc_fw_backoff=m->foc_fw_backoff;
    c->foc_mag_vd_max=m->foc_mag_vd_max;c->foc_overmod_factor=m->foc_overmod_factor;
    c->foc_temp_comp=m->foc_temp_comp;c->foc_temp_comp_base_temp=m->foc_temp_comp_base_temp;c->foc_offsets_cal_mode=m->foc_offsets_cal_mode;c->foc_calibrate_on_boot=m->foc_calibrate_on_boot;
    c->foc_pll_kp=m->foc_pll_kp;c->foc_pll_ki=m->foc_pll_ki;
    c->foc_openloop_rpm=m->foc_openloop_rpm;c->foc_openloop_rpm_low=m->foc_openloop_rpm_low;c->foc_sl_openloop_hyst=m->foc_sl_openloop_hyst;
    c->foc_sl_openloop_time=m->foc_sl_openloop_time;c->foc_sl_openloop_time_lock=m->foc_sl_openloop_time_lock;c->foc_sl_openloop_time_ramp=m->foc_sl_openloop_time_ramp;
    c->foc_sl_openloop_boost_q=m->foc_sl_openloop_boost_q;c->foc_sl_openloop_max_q=m->foc_sl_openloop_max_q;
    c->foc_sensor_mode=m->foc_sensor_mode;
    memcpy(c->foc_hall_table,m->foc_hall_table,8);c->foc_hall_interp_erpm=m->foc_hall_interp_erpm;c->foc_sl_erpm_start=m->foc_sl_erpm_start;c->foc_sl_erpm=m->foc_sl_erpm;
    c->foc_speed_source=m->foc_speed_source;
    c->s_pid_kp=m->speed_pid.kp;c->s_pid_ki=m->speed_pid.ki;c->s_pid_kd=m->speed_pid.kd;c->s_pid_kd_filter=m->speed_kd_filter;
    c->s_pid_min_erpm=m->speed_pid_min_erpm;c->s_pid_allow_braking=m->speed_pid_allow_braking;
    c->s_pid_ramp_erpms_s=m->speed_pid_ramp_erpms_s;c->s_pid_speed_source=m->speed_pid_source;
    c->p_pid_kp=m->position_pid.kp;c->p_pid_ki=m->position_pid.ki;c->p_pid_kd=m->position_pid.kd;
    c->p_pid_kd_proc=m->position_kd_proc;c->p_pid_kd_filter=m->position_kd_filter;
    c->p_pid_ang_div=m->position_ang_div;c->p_pid_gain_dec_angle=m->position_gain_dec_angle;
    c->p_pid_offset=m->position_offset_deg;
    c->cc_startup_boost_duty=0.0f;
    c->cc_min_current=m->cc_min_current;
    c->cc_gain=1.0f;
    c->cc_ramp_step_max=0.01f;
    c->m_encoder_counts=m->encoder.cpr;c->m_sensor_port_mode=m->sensor_mode==SENSOR_MODE_ENCODER?SENSOR_PORT_MODE_ABI:SENSOR_PORT_MODE_HALL;c->m_invert_direction=m->invert_direction;c->si_motor_poles=(uint8_t)(m->pole_pairs*2U);
    c->si_gear_ratio=m->si_gear_ratio;c->si_wheel_diameter=m->si_wheel_diameter;c->si_battery_type=m->si_battery_type;c->si_battery_cells=m->si_battery_cells;
    c->si_battery_ah=m->si_battery_ah;c->si_motor_nl_current=m->si_motor_nl_current;
}

void mc_interface_init(bool reset_conf){
	if(!s_mc_interface_inited){
		motor_control_init();
		s_mc_interface_inited=true;
	}
	if(reset_conf){
		/* Factory reset the live VESC-6.00 wire configs, then persist them so
		 * a later boot does not reload the old transactional record. VESC Tool
		 * issues this path via the standard GET/SET MCCONF reset command. */
		vesc_config_init_defaults();
		if(!vesc_config_apply_defaults()){
			motor_raise_fault_from_task(&g_motor_left,MOTOR_FAULT_FLASH_CONFIG);
			motor_raise_fault_from_task(&g_motor_right,MOTOR_FAULT_FLASH_CONFIG);
		}
		if(!conf_general_store_all()){
			motor_raise_fault_from_task(&g_motor_left,MOTOR_FAULT_FLASH_CONFIG);
			motor_raise_fault_from_task(&g_motor_right,MOTOR_FAULT_FLASH_CONFIG);
		}
	}
	mirror_from_runtime(&g_motor_left,&s_mcconf_mirror[0]);
	mirror_from_runtime(&g_motor_right,&s_mcconf_mirror[1]);
}
const volatile mc_configuration* mc_interface_get_configuration(void){MotorRuntime*m=mc_interface_motor_runtime_now();mirror_from_runtime(m,&s_mcconf_mirror[m->id]);return &s_mcconf_mirror[m->id];}

static bool config_float_same(float a, float b) {
    if (!isfinite(a) || !isfinite(b)) return false;
    const float scale = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));
    return fabsf(a - b) <= (1.0e-6f * scale);
}

static bool mc_interface_unsupported_configuration_unchanged(const mc_configuration *c,
                                                              const MotorRuntime *m) {
    /* Keep the public/internal configuration API under the same ownership
       rules as SET_MCCONF. A caller may change only fields with a real backend
       in this F103 port. Unsupported VESC fields are still mirrored so callers
       can round-trip the complete schema, but changing one must never look as
       though it was applied. */
    mc_configuration expected;
    mirror_from_runtime(m, &expected);

#define SAME_F(field) config_float_same(c->field, expected.field)
    if (c->pwm_mode != expected.pwm_mode ||
        c->comm_mode != expected.comm_mode ||
        c->sensor_mode != expected.sensor_mode ||
        memcmp(c->hall_table, expected.hall_table, sizeof(c->hall_table)) != 0 ||
        !SAME_F(sl_min_erpm) ||
        !SAME_F(sl_min_erpm_cycle_int_limit) ||
        !SAME_F(sl_max_fullbreak_current_dir_change) ||
        !SAME_F(sl_cycle_int_limit) ||
        !SAME_F(sl_phase_advance_at_br) ||
        !SAME_F(sl_cycle_int_rpm_br) ||
        !SAME_F(sl_bemf_coupling_k) ||
        !SAME_F(hall_sl_erpm) ||
        !SAME_F(lo_current_max) ||
        !SAME_F(lo_current_min) ||
        !SAME_F(lo_in_current_max) ||
        !SAME_F(lo_in_current_min) ||
        !SAME_F(foc_f_zv) ||
        !SAME_F(foc_sl_openloop_hyst) ||
        !SAME_F(foc_sl_erpm_start) ||
        !SAME_F(l_battery_regen_cut_start) ||
        !SAME_F(l_battery_regen_cut_end) ||
        c->foc_sample_v0_v7 != expected.foc_sample_v0_v7 ||
        c->foc_sample_high_current != expected.foc_sample_high_current ||
        !SAME_F(foc_fw_backoff) ||
        !SAME_F(foc_mag_vd_max) ||
        !SAME_F(foc_overmod_factor) ||
        !SAME_F(cc_startup_boost_duty) ||
        !SAME_F(cc_gain) ||
        !SAME_F(cc_ramp_step_max) ||
        !SAME_F(si_motor_nl_current)) {
        return false;
    }
    if (m->id == MOTOR_RIGHT &&
        (!SAME_F(foc_encoder_offset) || c->foc_encoder_inverted != expected.foc_encoder_inverted ||
         !SAME_F(foc_encoder_ratio) || c->m_encoder_counts != expected.m_encoder_counts)) {
        return false;
    }
#undef SAME_F
    return true;
}

static bool mc_interface_configuration_runtime_valid(const mc_configuration *c,
                                                     const MotorRuntime *m) {
    if (c == NULL || m == NULL || c->motor_type != MOTOR_TYPE_FOC) return false;
    if (!mc_interface_unsupported_configuration_unchanged(c, m)) return false;
    const bool encoder_ab = (c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ||
                             c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB);
    const bool sensorless = c->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
    if (m->id == MOTOR_LEFT) {
        if (!sensorless && !encoder_ab && c->foc_sensor_mode != FOC_SENSOR_MODE_HALL) return false;
        if (encoder_ab) {
            if (c->m_sensor_port_mode != SENSOR_PORT_MODE_ABI ||
                c->m_encoder_counts < 4U || c->m_encoder_counts > 65535U ||
                c->foc_encoder_offset < 0.0f || c->foc_encoder_offset >= 360.0f ||
                c->foc_encoder_ratio <= 0.0f || c->foc_encoder_ratio > 1000.0f) return false;
            const uint64_t rq = (uint64_t)llrintf(c->foc_encoder_ratio * 65536.0f);
            if (rq == 0U || rq > UINT32_MAX ||
                ((rq << 16) / c->m_encoder_counts) > UINT32_MAX) return false;
        } else if (c->m_sensor_port_mode != SENSOR_PORT_MODE_HALL) {
            /* Sensorless still leaves the physical sensor pins in the benign
               Hall-input configuration; no fictitious encoder peripheral is
               enabled while the observer is the phase source. */
            return false;
        }
    } else {
        if ((!sensorless && c->foc_sensor_mode != FOC_SENSOR_MODE_HALL) || encoder_ab ||
            c->m_sensor_port_mode != SENSOR_PORT_MODE_HALL) return false;
    }

    const float f[] = {
        c->l_current_max, c->l_current_min, c->l_in_current_max, c->l_in_current_min, c->l_in_current_map_start, c->l_in_current_map_filter,
        c->l_abs_current_max, c->l_min_erpm, c->l_max_erpm, c->l_erpm_start, c->l_min_vin, c->l_max_vin,
        c->l_temp_fet_start, c->l_temp_fet_end, c->l_temp_motor_start, c->l_temp_motor_end, c->l_temp_accel_dec,
        c->foc_dt_us,
        c->l_battery_cut_start, c->l_battery_cut_end, c->l_min_duty, c->l_max_duty,
        c->l_watt_max, c->l_watt_min, c->l_current_max_scale, c->l_current_min_scale, c->l_duty_start,
        c->foc_current_kp, c->foc_current_ki, c->foc_encoder_offset, c->foc_encoder_ratio,
        c->foc_motor_l, c->foc_motor_ld_lq_diff,
        c->foc_motor_r, c->foc_motor_flux_linkage, c->foc_observer_gain,
        c->foc_observer_gain_slow, c->foc_observer_offset, c->foc_sat_comp,
        c->foc_duty_dowmramp_kp, c->foc_duty_dowmramp_ki, c->foc_start_curr_dec, c->foc_start_curr_dec_rpm,
        c->foc_current_filter_const, c->foc_fw_current_max, c->foc_fw_duty_start, c->foc_fw_ramp_time, c->foc_fw_q_current_factor,
        c->foc_pll_kp, c->foc_pll_ki, c->foc_openloop_rpm, c->foc_openloop_rpm_low, c->foc_sl_openloop_hyst,
        c->foc_sl_openloop_time, c->foc_sl_openloop_time_lock,
        c->foc_sl_openloop_time_ramp, c->foc_sl_openloop_boost_q,
        c->foc_sl_openloop_max_q, c->foc_hall_interp_erpm, c->foc_sl_erpm,
        c->s_pid_kp, c->s_pid_ki, c->s_pid_kd, c->s_pid_kd_filter,
        c->s_pid_min_erpm, c->s_pid_ramp_erpms_s,
        c->p_pid_kp, c->p_pid_ki, c->p_pid_kd, c->p_pid_kd_proc, c->p_pid_kd_filter,
        c->p_pid_ang_div, c->p_pid_gain_dec_angle, c->p_pid_offset, c->cc_min_current,
        c->si_gear_ratio, c->si_wheel_diameter, c->si_battery_ah,
        c->si_motor_nl_current
    };
    for (unsigned k = 0U; k < sizeof(f) / sizeof(f[0]); k++) {
        if (!isfinite(f[k])) return false;
    }
    if (c->l_current_max < 0.1f || c->l_current_max > FOC_MAX_CURRENT_A ||
        c->l_current_min < -FOC_MAX_CURRENT_A || c->l_current_min > 0.0f ||
        c->l_in_current_max < 0.0f || c->l_in_current_max > FOC_MAX_CURRENT_A ||
        c->l_in_current_min < -FOC_MAX_CURRENT_A || c->l_in_current_min > 0.0f ||
        c->l_in_current_map_start < 0.0f || c->l_in_current_map_start > 1.0f ||
        c->l_in_current_map_filter < 0.0f || c->l_in_current_map_filter > 1.0f ||
        c->l_abs_current_max < fmaxf(c->l_current_max, fabsf(c->l_current_min)) ||
        c->l_abs_current_max > FOC_ABS_CURRENT_TRIP_A ||
        c->l_min_erpm < -MOTOR_DEFAULT_MAX_ERPM || c->l_min_erpm > -1.0f ||
        c->l_max_erpm < 1.0f || c->l_max_erpm > MOTOR_DEFAULT_MAX_ERPM ||
        c->l_erpm_start < 0.0f || c->l_erpm_start > 1.0f ||
        c->l_min_vin < VBUS_MIN_RUN_V || c->l_min_vin > (VBUS_MAX_RUN_V - 0.5f) ||
        c->l_max_vin > VBUS_MAX_RUN_V || c->l_max_vin < (c->l_min_vin + 0.5f) ||
        c->l_battery_cut_end < c->l_min_vin || c->l_battery_cut_start > c->l_max_vin ||
        c->l_battery_cut_start <= c->l_battery_cut_end ||
        c->l_min_duty < 0.0f || c->l_max_duty < 0.01f || c->l_max_duty > 0.98f ||
        c->l_min_duty > c->l_max_duty || c->l_watt_max < 0.0f || c->l_watt_min > 0.0f ||
        c->l_watt_min >= c->l_watt_max || c->l_current_max_scale < 0.0f || c->l_current_max_scale > 1.0f ||
        c->l_current_min_scale < 0.0f || c->l_current_min_scale > 1.0f || c->l_duty_start < 0.0f || c->l_duty_start > 1.0f ||
        c->l_temp_fet_start < HOVERBOARD_MCU_TEMP_MIN_VALID_C || c->l_temp_fet_end > HOVERBOARD_MCU_TEMP_MAX_VALID_C ||
        c->l_temp_fet_end <= c->l_temp_fet_start + 0.5f || c->l_temp_motor_start < -100.0f || c->l_temp_motor_end > 250.0f ||
        c->l_temp_motor_end <= c->l_temp_motor_start + 0.5f || c->l_temp_accel_dec < 0.0f || c->l_temp_accel_dec > 1.0f ||
        (c->l_additional_faults & ~(MCCONF_L_ADDITIONAL_FAULT_ENCODER_SLIP | MCCONF_L_ADDITIONAL_FAULT_OVERSPEED | MCCONF_L_ADDITIONAL_FAULT_UNDERSPEED | MCCONF_L_ADDITIONAL_FAULT_ABS_SPEED)) != 0U) return false;
    if (c->foc_current_kp < 0.00001f || c->foc_current_kp > 10.0f ||
        c->foc_current_ki < 0.0f || c->foc_current_ki > 200000.0f ||
        c->foc_dt_us < 0.0f || c->foc_dt_us > 5.0f ||
        c->foc_motor_l < 1.0e-7f || c->foc_motor_l > 0.1f ||
        c->foc_motor_ld_lq_diff < -0.1f || c->foc_motor_ld_lq_diff > 0.1f ||
        c->foc_motor_r < 1.0e-5f || c->foc_motor_r > 100.0f ||
        c->foc_motor_flux_linkage < 1.0e-6f ||
        c->foc_motor_flux_linkage > (FOC_FLUX_Q_BASE_WB * 1.90f) ||
        c->foc_observer_gain < 0.0f || c->foc_observer_gain > 1000000.0f ||
        c->foc_observer_gain_slow < 0.0f || c->foc_observer_gain_slow > 1.0f ||
        c->foc_pll_kp < 0.0f || c->foc_pll_kp > 100000.0f ||
        c->foc_pll_ki < 0.0f || c->foc_pll_ki > 1000000.0f ||
        c->foc_sat_comp_mode > SAT_COMP_LAMBDA_AND_FACTOR ||
        c->foc_sat_comp < 0.0f || c->foc_sat_comp > 1.0f ||
        c->foc_openloop_rpm < 10.0f || c->foc_openloop_rpm > MOTOR_DEFAULT_MAX_ERPM ||
        c->foc_openloop_rpm_low < 0.0f || c->foc_openloop_rpm_low > MOTOR_DEFAULT_MAX_ERPM ||
        c->foc_sl_openloop_hyst < 0.0f || c->foc_sl_openloop_hyst > 100.0f ||
        c->foc_sl_openloop_time < 0.01f || c->foc_sl_openloop_time > 20.0f ||
        c->foc_sl_openloop_time_lock < 0.0f || c->foc_sl_openloop_time_lock > 20.0f ||
        c->foc_sl_openloop_time_ramp < 0.01f || c->foc_sl_openloop_time_ramp > 20.0f ||
        c->foc_sl_openloop_boost_q < 0.0f || c->foc_sl_openloop_boost_q > FOC_MAX_CURRENT_A ||
        c->foc_sl_openloop_max_q < 0.1f || c->foc_sl_openloop_max_q > FOC_MAX_CURRENT_A ||
        c->foc_hall_interp_erpm < 0.0f || c->foc_hall_interp_erpm > MOTOR_DEFAULT_MAX_ERPM ||
        c->foc_sl_erpm < 10.0f || c->foc_sl_erpm > MOTOR_DEFAULT_MAX_ERPM ||
        c->foc_observer_offset < -10.0f || c->foc_observer_offset > 10.0f ||
        c->foc_duty_dowmramp_kp < 0.0f || c->foc_duty_dowmramp_kp > 100000.0f ||
        c->foc_duty_dowmramp_ki < 0.0f || c->foc_duty_dowmramp_ki > 1000000.0f ||
        c->foc_start_curr_dec < 0.0f || c->foc_start_curr_dec > 1.0f ||
        c->foc_start_curr_dec_rpm < 0.0f || c->foc_start_curr_dec_rpm > MOTOR_DEFAULT_MAX_ERPM ||
        c->foc_current_filter_const < 0.0f || c->foc_current_filter_const > 1.0f ||
        c->foc_speed_source > FOC_SPEED_SRC_OBSERVER ||
        c->foc_observer_type > FOC_OBSERVER_MXV_LAMBDA_COMP_LIN ||
        c->foc_cc_decoupling > FOC_CC_DECOUPLING_CROSS_BEMF || c->foc_mtpa_mode > MTPA_MODE_IQ_MEASURED ||
        c->foc_fw_current_max < 0.0f || c->foc_fw_current_max > FOC_MAX_CURRENT_A ||
        c->foc_fw_duty_start < 0.0f || c->foc_fw_duty_start > 1.0f ||
        c->foc_fw_ramp_time < 0.01f || c->foc_fw_ramp_time > 30.0f ||
        c->foc_fw_q_current_factor < 0.0f || c->foc_fw_q_current_factor > 1.0f) return false;
    if (c->s_pid_kp < 0.0f || c->s_pid_kp > 1000.0f ||
        c->s_pid_ki < 0.0f || c->s_pid_ki > 1000.0f ||
        c->s_pid_kd < 0.0f || c->s_pid_kd > 1000.0f ||
        c->s_pid_kd_filter < 0.0f || c->s_pid_kd_filter > 1.0f ||
        c->s_pid_min_erpm < 0.0f || c->s_pid_min_erpm > MOTOR_DEFAULT_MAX_ERPM ||
        c->s_pid_ramp_erpms_s < 0.0f || c->s_pid_ramp_erpms_s > 1000000.0f ||
        c->s_pid_speed_source > S_PID_SPEED_SRC_FASTER ||
        c->p_pid_kp < 0.0f || c->p_pid_kp > 1000.0f ||
        c->p_pid_ki < 0.0f || c->p_pid_ki > 1000.0f ||
        c->p_pid_kd < 0.0f || c->p_pid_kd > 1000.0f ||
        c->p_pid_kd_proc < 0.0f || c->p_pid_kd_proc > 1000.0f ||
        c->p_pid_kd_filter < 0.0f || c->p_pid_kd_filter > 1.0f ||
        c->p_pid_ang_div < 0.01f || c->p_pid_ang_div > 1000.0f ||
        c->p_pid_gain_dec_angle < 0.0f || c->p_pid_gain_dec_angle > 3600.0f ||
        c->p_pid_offset < -36000.0f || c->p_pid_offset > 36000.0f ||
        c->cc_min_current < 0.0f || c->cc_min_current > FOC_MAX_CURRENT_A ||
        c->si_gear_ratio < 0.01f || c->si_gear_ratio > 1000.0f ||
        c->si_wheel_diameter < 0.001f || c->si_wheel_diameter > 10.0f ||
        c->si_battery_ah < 0.0f || c->si_battery_ah > 10000.0f ||
        c->si_motor_nl_current < 0.0f || c->si_motor_nl_current > FOC_MAX_CURRENT_A) return false;
    if (c->si_motor_poles < 2U || (c->si_motor_poles & 1U) != 0U || c->si_motor_poles > 120U) return false;
    return true;
}

void mc_interface_set_configuration(mc_configuration *c) {
    MotorRuntime *m = mc_interface_motor_runtime_now();
    if (m == NULL || c == NULL) return;

    /* Validate the complete set of runtime-consumed fields before touching
       MotorRuntime. The wire setter already does the same preflight; this API
       path must not become a second, weaker configuration semantics. */
    if (!mc_interface_configuration_runtime_valid(c, m)) return;

    const bool encoder_ab = (c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
                             c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER);
    const bool encoder_was_active = (m->id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER);
    const uint32_t encoder_old_cpr = m->encoder.cpr;
    const bool encoder_old_inverted = m->encoder.inverted;
    const uint16_t encoder_old_offset_u16 = m->encoder.elec_offset_u16;
    const float encoder_old_ratio = m->encoder.electrical_ratio;
    motor_stop(m);
    m->pwm_mode = c->pwm_mode;
    m->comm_mode = c->comm_mode;
    m->motor_type = MOTOR_TYPE_FOC;
    m->current_max_a = c->l_current_max;
    m->current_min_a = c->l_current_min;
    m->input_current_max_a = c->l_in_current_max;
    m->input_current_min_a = c->l_in_current_min;
    m->input_current_map_start = c->l_in_current_map_start;
    m->input_current_map_filter = c->l_in_current_map_filter;
    m->input_current_map_filtered_a = m->input_current;
    m->input_current_map_limit_a = m->current_max_a;
    m->abs_current_max_a = c->l_abs_current_max;
    m->abs_current_trip_q15 = amp_to_current_q15(m->abs_current_max_a);
    m->slow_abs_current = c->l_slow_abs_current;
    m->temp_fet_start=c->l_temp_fet_start; m->temp_fet_end=c->l_temp_fet_end;
    m->temp_motor_start=c->l_temp_motor_start; m->temp_motor_end=c->l_temp_motor_end;
    m->temp_accel_dec=c->l_temp_accel_dec; m->additional_faults=c->l_additional_faults;
    m->abs_current_fault_count = 0U;
    m->min_vin = c->l_min_vin; m->max_vin = c->l_max_vin;
    m->battery_cut_start = c->l_battery_cut_start; m->battery_cut_end = c->l_battery_cut_end;
    m->battery_regen_cut_start = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    m->battery_regen_cut_end = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    m->min_vin_q15 = volt_to_q15(m->min_vin); m->max_vin_q15 = volt_to_q15(m->max_vin);
    m->hard_max_vin_q15 = volt_to_q15(fminf(m->max_vin + FOC_VBUS_HARD_OV_MARGIN_V, FOC_VBUS_HARD_MAX_V));
    m->hard_min_vin_q15 = volt_to_q15(fmaxf(m->min_vin - FOC_VBUS_HARD_UV_MARGIN_V, FOC_VBUS_HARD_MIN_V));
    m->over_voltage_fault_count = 0U; m->under_voltage_fault_count = 0U;
    m->min_erpm = c->l_min_erpm;
    m->max_erpm = c->l_max_erpm; m->erpm_start = c->l_erpm_start;
    m->foc_start_curr_dec=c->foc_start_curr_dec; m->foc_start_curr_dec_rpm=c->foc_start_curr_dec_rpm;
    m->foc_short_ls_on_zero_duty=c->foc_short_ls_on_zero_duty;
    m->max_duty = c->l_max_duty;
    m->min_duty = c->l_min_duty;
    m->duty_limit_now = m->max_duty;
    m->watt_max=c->l_watt_max; m->watt_min=c->l_watt_min;
    m->current_max_scale=c->l_current_max_scale; m->current_min_scale=c->l_current_min_scale; m->duty_start=c->l_duty_start;
    m->invert_direction = c->m_invert_direction;

    motor_set_current_pi_gains(m, c->foc_current_kp, c->foc_current_ki);
    m->foc_temp_comp = c->foc_temp_comp;
    m->foc_temp_comp_base_temp = c->foc_temp_comp_base_temp;
    m->foc_offsets_cal_mode = c->foc_offsets_cal_mode;
    m->foc_calibrate_on_boot = c->foc_calibrate_on_boot;
    m->foc_motor_l = c->foc_motor_l;
    m->foc_motor_ld_lq_diff = c->foc_motor_ld_lq_diff;
    m->foc_motor_r = c->foc_motor_r;
    m->res_est_ohm = m->foc_motor_r; m->res_est_state_ohm = m->foc_motor_r; m->res_est_valid = false;
    m->foc_speed_source = c->foc_speed_source;
    m->foc_motor_flux_linkage = c->foc_motor_flux_linkage;
    m->foc_dt_us = c->foc_dt_us;
    m->foc_observer_gain = c->foc_observer_gain;
    m->foc_observer_gain_slow = c->foc_observer_gain_slow;
    m->foc_observer_offset = c->foc_observer_offset;
    m->foc_pll_kp = c->foc_pll_kp;
    m->foc_pll_ki = c->foc_pll_ki;
    m->foc_sat_comp_mode = c->foc_sat_comp_mode;
    m->foc_sat_comp = c->foc_sat_comp;
    m->foc_observer_type = c->foc_observer_type;
    m->foc_duty_dowmramp_kp = c->foc_duty_dowmramp_kp; m->foc_duty_dowmramp_ki = c->foc_duty_dowmramp_ki;
    m->foc_current_filter_const = c->foc_current_filter_const; m->foc_cc_decoupling = c->foc_cc_decoupling;
    m->foc_mtpa_mode = c->foc_mtpa_mode; m->foc_fw_current_max = c->foc_fw_current_max;
    m->foc_fw_duty_start = c->foc_fw_duty_start; m->foc_fw_ramp_time = c->foc_fw_ramp_time;
    m->foc_fw_q_current_factor = c->foc_fw_q_current_factor;
    m->foc_openloop_rpm = c->foc_openloop_rpm;
    m->foc_openloop_rpm_low = c->foc_openloop_rpm_low;
    m->foc_sl_openloop_time = c->foc_sl_openloop_time;
    m->foc_sl_openloop_time_lock = c->foc_sl_openloop_time_lock;
    m->foc_sl_openloop_time_ramp = c->foc_sl_openloop_time_ramp;
    m->foc_sl_openloop_boost_q = c->foc_sl_openloop_boost_q;
    m->foc_sl_openloop_max_q = c->foc_sl_openloop_max_q;
    m->foc_hall_interp_erpm = c->foc_hall_interp_erpm;
    m->foc_hall_interp_erpm_u32 = (uint32_t)lrintf(c->foc_hall_interp_erpm);
    m->foc_sl_erpm = c->foc_sl_erpm;
    memcpy(m->foc_hall_table, c->foc_hall_table, sizeof(m->foc_hall_table));
    m->speed_pid.kp = c->s_pid_kp;
    m->speed_pid.ki = c->s_pid_ki;
    m->speed_pid.kd = c->s_pid_kd;
    m->speed_kd_filter = c->s_pid_kd_filter;
    m->speed_pid_min_erpm = c->s_pid_min_erpm;
    m->speed_pid_allow_braking = c->s_pid_allow_braking;
    m->speed_pid_ramp_erpms_s = c->s_pid_ramp_erpms_s;
    /* VESC6 wire has no speed-source selector. Typed/internal callers may use
       FAST/FASTER, while standard VESC6 SET_MCCONF always deserializes PLL. */
    m->speed_pid_source = c->s_pid_speed_source;
    m->position_pid.kp = c->p_pid_kp;
    m->position_pid.ki = c->p_pid_ki;
    m->position_pid.kd = c->p_pid_kd;
    m->position_kd_proc = c->p_pid_kd_proc;
    m->position_kd_filter = c->p_pid_kd_filter;
    m->position_ang_div = c->p_pid_ang_div;
    m->position_gain_dec_angle = c->p_pid_gain_dec_angle;
    m->position_offset_deg = c->p_pid_offset;
    m->cc_min_current = c->cc_min_current;
    if (c->si_motor_poles >= 2U && (c->si_motor_poles & 1U) == 0U) m->pole_pairs = c->si_motor_poles / 2U;
    m->si_gear_ratio = c->si_gear_ratio;
    m->si_wheel_diameter = c->si_wheel_diameter;
    m->si_battery_type = c->si_battery_type;
    m->si_battery_cells = c->si_battery_cells;
    m->si_battery_ah = c->si_battery_ah;

    if (m->id == MOTOR_LEFT) {
        const uint16_t encoder_new_offset_u16 = foc_deg_to_u16(c->foc_encoder_offset);
        const uint32_t encoder_new_cpr = c->m_encoder_counts;
        const uint32_t encoder_ratio_q16 = (uint32_t)lrintf(c->foc_encoder_ratio * 65536.0f);
        const uint64_t step_q16 = ((uint64_t)encoder_ratio_q16 << 16) /
                                  (encoder_new_cpr == 0U ? 1U : encoder_new_cpr);
        /* Representation bounds were preflighted before MotorRuntime was
           touched; these casts are therefore lossless in the accepted API. */

        const bool encoder_hw_changed = encoder_ab &&
            (!encoder_was_active || encoder_old_cpr != encoder_new_cpr);
        const bool encoder_phase_changed = encoder_ab &&
            (encoder_old_inverted != c->foc_encoder_inverted ||
             encoder_old_offset_u16 != encoder_new_offset_u16 ||
             fabsf(encoder_old_ratio - c->foc_encoder_ratio) > 1.0e-6f);

        m->encoder.cpr = encoder_new_cpr;
        m->encoder.inverted = c->foc_encoder_inverted;
        m->encoder.elec_offset_u16 = encoder_new_offset_u16;
        m->encoder.electrical_ratio = c->foc_encoder_ratio;
        m->encoder.electrical_ratio_q16 = encoder_ratio_q16;
        m->encoder.phase_per_count_q16 = (uint32_t)step_q16;
        m->foc_sensor_mode = encoder_ab ? FOC_SENSOR_MODE_ENCODER_AB : c->foc_sensor_mode;
        m->sensor_request_mode = encoder_ab ? SENSOR_MODE_ENCODER : SENSOR_MODE_HALL;

        if (encoder_ab) {
            if (encoder_hw_changed) {
                (void)encoder_init(m);
            } else {
                m->sensor_mode = SENSOR_MODE_ENCODER;
            }
            if (encoder_hw_changed || encoder_phase_changed) {
                m->encoder.synced = false;
                m->encoder.motion_proved = false;
                m->encoder.sync_active = false;
                m->encoder.speed_sample_valid = false;
                m->using_encoder = false;
            }
        } else {
            encoder_deinit(m);
            motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
            if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL) motor_hall_edge_isr(m);
        }
    } else {
        m->foc_sensor_mode = c->foc_sensor_mode;
        m->sensor_mode = SENSOR_MODE_HALL;
        m->sensor_request_mode = SENSOR_MODE_HALL;
        motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
        if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL) motor_hall_edge_isr(m);
    }
    foc_precalc_values(m);
    foc_observer_reset(m, m->observer_phase_u16);
    mirror_from_runtime(m, &s_mcconf_mirror[m->id]);
}
unsigned mc_interface_calc_crc(mc_configuration *conf, bool is_motor_2) {
    /* This pinned VESC-6 runtime mirror intentionally has no embedded crc
     * member (unlike app_configuration). Support VESC's conf == NULL motor
     * selection while using the one canonical project CRC16 implementation.
     * Do not append a crc member here: that would silently change this port's
     * mc_configuration ABI and every sizeof-based persistence/test contract. */
    if (conf == NULL) {
        conf = &s_mcconf_mirror[is_motor_2 ? 1 : 0];
    }
    return vesc_crc16((const uint8_t *)conf, (uint16_t)sizeof(*conf));
}
bool mc_interface_dccal_done(void){return foc_calibration_done();}
void mc_interface_set_pwm_callback(void(*p)(void)){s_pwm_callback=p;}
void mc_interface_lock(void){s_mc_locked=true;}
void mc_interface_unlock(void){s_mc_locked=false;}
void mc_interface_lock_override_once(void){s_mc_lock_override_once=true;}
int mc_interface_try_input_motor(motor_id_t id){uint32_t now=osKernelGetTickCount();if((int32_t)(s_ignore_until[id]-now)>0)return 0;if(s_mc_locked&&!s_mc_lock_override_once)return 0;s_mc_lock_override_once=false;return 1;}
int mc_interface_try_input(void){return mc_interface_try_input_motor(mc_interface_motor_runtime_now()->id);}
mc_fault_code motor_fault_to_vesc(motor_fault_t f) {
    switch (f) {
    case MOTOR_FAULT_NONE:             return FAULT_CODE_NONE;
    case MOTOR_FAULT_OVER_VOLTAGE:     return FAULT_CODE_OVER_VOLTAGE;
    case MOTOR_FAULT_UNDER_VOLTAGE:    return FAULT_CODE_UNDER_VOLTAGE;
    case MOTOR_FAULT_ABS_OVER_CURRENT: return FAULT_CODE_ABS_OVER_CURRENT;
    case MOTOR_FAULT_CURRENT_OFFSET:   return FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1;
    case MOTOR_FAULT_HALL_INVALID:
    case MOTOR_FAULT_SENSOR_DETECT:
    case MOTOR_FAULT_SENSORLESS_OBSERVER:return FAULT_CODE_ENCODER_FAULT;
    case MOTOR_FAULT_ADC_DMA:
    case MOTOR_FAULT_FOC_ISR_OVERRUN:  return FAULT_CODE_DRV;
    case MOTOR_FAULT_COMMAND_TIMEOUT:  return FAULT_CODE_NONE; /* timeout is not a VESC fault */
    case MOTOR_FAULT_OVER_TEMP_BOARD:  return FAULT_CODE_OVER_TEMP_FET;
    case MOTOR_FAULT_OVER_TEMP_MOTOR:  return FAULT_CODE_OVER_TEMP_MOTOR;
    case MOTOR_FAULT_OVERSPEED:        return FAULT_CODE_OVERSPEED;
    case MOTOR_FAULT_UNDERSPEED:       return FAULT_CODE_UNDERSPEED;
    case MOTOR_FAULT_ABS_OVERSPEED:    return FAULT_CODE_ABS_OVERSPEED;
    case MOTOR_FAULT_ENCODER_SLIP:     return FAULT_CODE_ENCODER_SLIP;
    case MOTOR_FAULT_MCU_UNDER_VOLTAGE:return FAULT_CODE_MCU_UNDER_VOLTAGE;
    case MOTOR_FAULT_BREAK:            return FAULT_CODE_BRK;
    case MOTOR_FAULT_FLASH_CONFIG:     return FAULT_CODE_FLASH_CORRUPTION;
    default:                           return FAULT_CODE_DRV;
    }
}

motor_fault_t motor_fault_from_vesc(mc_fault_code f) {
    switch (f) {
    case FAULT_CODE_NONE:                         return MOTOR_FAULT_NONE;
    case FAULT_CODE_OVER_VOLTAGE:                 return MOTOR_FAULT_OVER_VOLTAGE;
    case FAULT_CODE_UNDER_VOLTAGE:                return MOTOR_FAULT_UNDER_VOLTAGE;
    case FAULT_CODE_ABS_OVER_CURRENT:             return MOTOR_FAULT_ABS_OVER_CURRENT;
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return MOTOR_FAULT_CURRENT_OFFSET;
    case FAULT_CODE_ENCODER_FAULT:
    case FAULT_CODE_ENCODER_NO_MAGNET:            return MOTOR_FAULT_SENSOR_DETECT;
    case FAULT_CODE_ENCODER_SLIP:                 return MOTOR_FAULT_ENCODER_SLIP;
    case FAULT_CODE_OVER_TEMP_FET:                return MOTOR_FAULT_OVER_TEMP_BOARD;
    case FAULT_CODE_OVER_TEMP_MOTOR:              return MOTOR_FAULT_OVER_TEMP_MOTOR;
    case FAULT_CODE_OVERSPEED:                    return MOTOR_FAULT_OVERSPEED;
    case FAULT_CODE_UNDERSPEED:                   return MOTOR_FAULT_UNDERSPEED;
    case FAULT_CODE_ABS_OVERSPEED:                return MOTOR_FAULT_ABS_OVERSPEED;
    case FAULT_CODE_MCU_UNDER_VOLTAGE:             return MOTOR_FAULT_MCU_UNDER_VOLTAGE;
    case FAULT_CODE_BRK:                           return MOTOR_FAULT_BREAK;
    case FAULT_CODE_FLASH_CORRUPTION:
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG:
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG:        return MOTOR_FAULT_FLASH_CONFIG;
    case FAULT_CODE_DRV:
    default:                                      return MOTOR_FAULT_ADC_DMA;
    }
}

mc_fault_code mc_interface_get_fault(void){return motor_fault_to_vesc(mc_interface_motor_runtime_now()->fault);}
const char* mc_interface_fault_to_string(mc_fault_code f){
    switch(f){
    case FAULT_CODE_NONE:return "FAULT_CODE_NONE";
    case FAULT_CODE_OVER_VOLTAGE:return "FAULT_CODE_OVER_VOLTAGE";
    case FAULT_CODE_UNDER_VOLTAGE:return "FAULT_CODE_UNDER_VOLTAGE";
    case FAULT_CODE_DRV:return "FAULT_CODE_DRV";
    case FAULT_CODE_ABS_OVER_CURRENT:return "FAULT_CODE_ABS_OVER_CURRENT";
    case FAULT_CODE_OVER_TEMP_FET:return "FAULT_CODE_OVER_TEMP_FET";
    case FAULT_CODE_OVER_TEMP_MOTOR:return "FAULT_CODE_OVER_TEMP_MOTOR";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
    case FAULT_CODE_UNBALANCED_CURRENTS:return "FAULT_CODE_UNBALANCED_CURRENTS";
    case FAULT_CODE_ENCODER_NO_MAGNET:return "FAULT_CODE_ENCODER_NO_MAGNET";
    case FAULT_CODE_ENCODER_FAULT:return "FAULT_CODE_ENCODER_FAULT";
    case FAULT_CODE_ENCODER_SLIP:return "FAULT_CODE_ENCODER_SLIP";
    case FAULT_CODE_OVERSPEED:return "FAULT_CODE_OVERSPEED";
    case FAULT_CODE_UNDERSPEED:return "FAULT_CODE_UNDERSPEED";
    case FAULT_CODE_ABS_OVERSPEED:return "FAULT_CODE_ABS_OVERSPEED";
    case FAULT_CODE_MCU_UNDER_VOLTAGE:return "FAULT_CODE_MCU_UNDER_VOLTAGE";
    case FAULT_CODE_BRK:return "FAULT_CODE_BRK";
    case FAULT_CODE_FLASH_CORRUPTION:return "FAULT_CODE_FLASH_CORRUPTION";
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG:return "FAULT_CODE_FLASH_CORRUPTION_APP_CFG";
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG:return "FAULT_CODE_FLASH_CORRUPTION_MC_CFG";
    default:return "FAULT_CODE_UNKNOWN";
    }
}
mc_state mc_interface_get_state(void){MotorRuntime*m=mc_interface_motor_runtime_now();if(m->detect.busy)return MC_STATE_DETECTING;if(m->full_brake_active)return MC_STATE_FULL_BRAKE;if(m->pwm_enabled)return MC_STATE_RUNNING;return MC_STATE_OFF;}
mc_control_mode mc_interface_get_control_mode(void){MotorRuntime*m=mc_interface_motor_runtime_now();switch(m->control_mode){case MOTOR_CTRL_DUTY:return CONTROL_MODE_DUTY;case MOTOR_CTRL_SPEED:return CONTROL_MODE_SPEED;case MOTOR_CTRL_CURRENT:return CONTROL_MODE_CURRENT;case MOTOR_CTRL_BRAKE_CURRENT:return CONTROL_MODE_CURRENT_BRAKE;case MOTOR_CTRL_POSITION:return CONTROL_MODE_POS;case MOTOR_CTRL_HANDBRAKE:return CONTROL_MODE_HANDBRAKE;case MOTOR_CTRL_OPENLOOP:return CONTROL_MODE_OPENLOOP;case MOTOR_CTRL_OPENLOOP_PHASE:return CONTROL_MODE_OPENLOOP_PHASE;case MOTOR_CTRL_OPENLOOP_DUTY:return CONTROL_MODE_OPENLOOP_DUTY;case MOTOR_CTRL_OPENLOOP_DUTY_PHASE:return CONTROL_MODE_OPENLOOP_DUTY_PHASE;default:return CONTROL_MODE_NONE;}}

void mc_interface_set_duty(float v){if(mc_interface_try_input())motor_set_duty(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_duty_noramp(float v){mc_interface_set_duty(v);}
void mc_interface_set_pid_speed(float v){if(mc_interface_try_input())motor_set_speed(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_pid_pos(float v){if(mc_interface_try_input())motor_set_position(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_current(float v){if(mc_interface_try_input())motor_set_current(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_brake_current(float v){if(mc_interface_try_input())motor_set_brake_current(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_current_rel(float v){if(mc_interface_try_input())motor_set_current_rel(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_brake_current_rel(float v){MotorRuntime*m=mc_interface_motor_runtime_now();v=foc_clampf(v,-1.0f,1.0f);mc_interface_set_brake_current(fabsf(v)*fmaxf(fabsf(m->current_min_a),fabsf(m->current_max_a)));}
void mc_interface_set_handbrake(float v){if(mc_interface_try_input())motor_set_handbrake(mc_interface_motor_runtime_now(),v);}
void mc_interface_set_handbrake_rel(float v){MotorRuntime*m=mc_interface_motor_runtime_now();mc_interface_set_handbrake(fabsf(foc_clampf(v,-1.0f,1.0f))*m->current_max_a);}
void mc_interface_set_openloop_current(float current,float rpm){MotorRuntime*m=mc_interface_motor_runtime_now();if(m->motor_type==MOTOR_TYPE_FOC&&mc_interface_try_input()){mcpwm_foc_set_openloop_current_motor(m,current,rpm);motor_touch_command(m);}}
void mc_interface_set_openloop_phase(float current,float phase){MotorRuntime*m=mc_interface_motor_runtime_now();if(m->motor_type==MOTOR_TYPE_FOC&&mc_interface_try_input()){mcpwm_foc_set_openloop_phase_motor(m,current,phase);motor_touch_command(m);}}
void mc_interface_set_openloop_duty(float duty,float rpm){MotorRuntime*m=mc_interface_motor_runtime_now();if(m->motor_type==MOTOR_TYPE_FOC&&mc_interface_try_input()){mcpwm_foc_set_openloop_duty_motor(m,duty,rpm);motor_touch_command(m);}}
void mc_interface_set_openloop_duty_phase(float duty,float phase){MotorRuntime*m=mc_interface_motor_runtime_now();if(m->motor_type==MOTOR_TYPE_FOC&&mc_interface_try_input()){mcpwm_foc_set_openloop_duty_phase_motor(m,duty,phase);motor_touch_command(m);}}
int mc_interface_set_tachometer_value(int steps){MotorRuntime*m=mc_interface_motor_runtime_now();m->stats.tachometer=steps;m->stats.tachometer_abs=steps<0?-steps:steps;return steps;}
void mc_interface_brake_now(void){MotorRuntime*m=mc_interface_motor_runtime_now();motor_set_brake_current(m,m->current_max_a);}
void mc_interface_release_motor(void){motor_stop(mc_interface_motor_runtime_now());}
void mc_interface_release_motor_override(void){mc_interface_release_motor();}
bool mc_interface_wait_for_motor_release(float timeout){uint32_t st=osKernelGetTickCount(),ms=(uint32_t)fmaxf(timeout*1000.0f,0.0f);MotorRuntime*m=mc_interface_motor_runtime_now();while(m->pwm_enabled){if((uint32_t)(osKernelGetTickCount()-st)>ms)return false;osDelay(1);}return true;}

float mc_interface_get_duty_cycle_set(void){return mc_interface_motor_runtime_now()->duty_command;}
float mc_interface_get_duty_cycle_now(void){return mc_interface_motor_runtime_now()->duty_now;}
float mc_interface_get_sampling_frequency_now(void){return (float)FOC_ISR_EVENT_HZ;}
float mc_interface_get_rpm(void){return mc_interface_motor_runtime_now()->erpm;}
static float get_reset_f(float *p,bool reset){float v=*p;if(reset)*p=0.0f;return v;}
float mc_interface_get_amp_hours(bool reset){return get_reset_f(&mc_interface_motor_runtime_now()->stats.amp_hours,reset);}
float mc_interface_get_amp_hours_charged(bool reset){return get_reset_f(&mc_interface_motor_runtime_now()->stats.amp_hours_charged,reset);}
float mc_interface_get_watt_hours(bool reset){return get_reset_f(&mc_interface_motor_runtime_now()->stats.watt_hours,reset);}
float mc_interface_get_watt_hours_charged(bool reset){return get_reset_f(&mc_interface_motor_runtime_now()->stats.watt_hours_charged,reset);}
float mc_interface_get_tot_current(void){return mcpwm_foc_get_tot_current_rt(mc_interface_motor_runtime_now());}
float mc_interface_get_tot_current_filtered(void){return mcpwm_foc_get_tot_current_filtered();}
float mc_interface_get_tot_current_directional(void){return mc_interface_motor_runtime_now()->motor_current;}
float mc_interface_get_tot_current_directional_filtered(void){return mc_interface_get_tot_current_directional();}
float mc_interface_get_tot_current_in(void){return mc_interface_motor_runtime_now()->input_current;}
float mc_interface_get_tot_current_in_filtered(void){return mc_interface_get_tot_current_in();}
float mc_interface_get_input_voltage_filtered(void){return mc_interface_motor_runtime_now()->vbus_filter;}
float mc_interface_get_abs_motor_current_unbalance(void){MotorRuntime*m=mc_interface_motor_runtime_now();return fabsf(m->ia+m->ib+m->ic);}
int mc_interface_get_tachometer_value(bool reset){MotorRuntime*m=mc_interface_motor_runtime_now();int v=m->stats.tachometer;if(reset)m->stats.tachometer=0;return v;}
int mc_interface_get_tachometer_abs_value(bool reset){MotorRuntime*m=mc_interface_motor_runtime_now();int v=m->stats.tachometer_abs;if(reset)m->stats.tachometer_abs=0;return v;}
float mc_interface_get_last_inj_adc_isr_duration(void){return foc_last_isr_duration_s();}
static motor_telemetry_avg_t read_reset_avg_mask(uint32_t mask){
    motor_telemetry_avg_t a;
    telemetry_read_reset_avg(mc_interface_motor_runtime_now()->id,mask,&a);
    return a;
}
float mc_interface_read_reset_avg_motor_current(void){return read_reset_avg_mask(1UL<<2).current_motor;}
float mc_interface_read_reset_avg_input_current(void){return read_reset_avg_mask(1UL<<3).current_in;}
float mc_interface_read_reset_avg_id(void){return read_reset_avg_mask(1UL<<4).id;}
float mc_interface_read_reset_avg_iq(void){return read_reset_avg_mask(1UL<<5).iq;}
float mc_interface_read_reset_avg_vd(void){return read_reset_avg_mask(1UL<<19).vd;}
float mc_interface_read_reset_avg_vq(void){return read_reset_avg_mask(1UL<<20).vq;}
float mc_interface_get_pid_pos_set(void){return mc_interface_motor_runtime_now()->position_target_deg;}
float mc_interface_get_pid_pos_now(void){MotorRuntime*m=mc_interface_motor_runtime_now();return m?m->position_deg+m->position_offset_deg:0.0f;}
void mc_interface_update_pid_pos_offset(float angle_now,bool store){
    MotorRuntime*m=mc_interface_motor_runtime_now(); if(!m)return;
    m->position_offset_deg=angle_now-m->position_deg;
    mirror_from_runtime(m,&s_mcconf_mirror[m->id]);
    if(store)(void)conf_general_store_mc_configuration(&s_mcconf_mirror[m->id],m->id==MOTOR_RIGHT);
}
float mc_interface_get_last_sample_adc_isr_duration(void){MotorRuntime*m=mc_interface_motor_runtime_now();return (float)m->isr_max_cycles/(float)CPU_CLOCK_HZ;}
void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len,
		uint8_t decimation, bool raw,
		void (*reply_func)(unsigned char *data, unsigned int len)) {
	/* Upstream retains this asynchronous route for the sample sender. A capture
	 * may complete long after COMM_SAMPLE_PRINT returned, so default UART TX
	 * would reply to the wrong peer for CAN-forwarded or alternate transports. */
	s_sample_reply_func = reply_func;
	MotorRuntime *motor = mc_interface_motor_runtime_now();
	if (motor != NULL) {
		(void)mc_interface_sample_control(mode, motor->id, len, decimation, raw);
	}
}
void (*mc_interface_sample_reply_func(void))(unsigned char *data, unsigned int len) {
	return s_sample_reply_func;
}
/* There is still no MOSFET NTC. VESC's FET-temperature API exposes the
 * explicitly documented MCU/board proxy. Motor temperature remains unavailable
 * unless a real sensor backend feeds the override API. */
#define MC_TEMP_SENSOR_UNAVAILABLE_C (-300.0f)
float mc_interface_temp_fet_filtered(void){MotorRuntime*m=mc_interface_motor_runtime_now();return (m&&m->board_temp_valid)?m->board_temp_filter_c:MC_TEMP_SENSOR_UNAVAILABLE_C;}
float mc_interface_temp_motor_filtered(void){return isfinite(s_temp_motor_override)?s_temp_motor_override:MC_TEMP_SENSOR_UNAVAILABLE_C;}
static float battery_level_linear(float cell_v,float empty_v,float full_v){
    if(!isfinite(cell_v)||full_v<=empty_v)return 0.0f;
    return foc_clampf((cell_v-empty_v)/(full_v-empty_v),0.0f,1.0f);
}

float mc_interface_get_battery_level(float*wh_left){
    MotorRuntime*m=mc_interface_motor_runtime_now();
    if(wh_left)*wh_left=0.0f;
    if(!m||m->si_battery_cells==0U||m->si_battery_ah<=0.0f)return 0.0f;
    float cell_v=m->vbus_filter/(float)m->si_battery_cells;
    float level=0.0f,nominal=0.0f;
    switch(m->si_battery_type){
        case 0U: /* BATTERY_TYPE_LIION_3_0__4_2 */
            level=battery_level_linear(cell_v,3.0f,4.2f); nominal=3.7f; break;
        case 1U: /* BATTERY_TYPE_LIIRON_2_6__3_6 */
            level=battery_level_linear(cell_v,2.6f,3.6f); nominal=3.3f; break;
        case 2U: /* BATTERY_TYPE_LEAD_ACID */
            level=battery_level_linear(cell_v,1.9f,2.15f); nominal=2.0f; break;
        default:
            return 0.0f;
    }
    if(wh_left)*wh_left=level*m->si_battery_ah*(float)m->si_battery_cells*nominal;
    return level;
}

float mc_interface_get_speed(void){
    if(s_wheel_speed_override)return s_wheel_speed_override_value;
    MotorRuntime*m=mc_interface_motor_runtime_now();
    float poles=(float)(m->pole_pairs*2U);
    float gear=m->si_gear_ratio;
    float diam=m->si_wheel_diameter;
    if(poles<2.0f||gear<=0.0f||diam<=0.0f)return 0.0f;
    float mech_rpm=m->erpm*(2.0f/poles);
    float wheel_rpm=mech_rpm/gear;
    return wheel_rpm*(3.14159265358979323846f*diam)/60.0f;
}

static float tach_to_distance(const MotorRuntime*m,int32_t tach){
    float poles=(float)(m->pole_pairs*2U);
    float gear=m->si_gear_ratio;
    float diam=m->si_wheel_diameter;
    if(poles<2.0f||gear<=0.0f||diam<=0.0f)return 0.0f;
    /* VESC tachometer uses six steps per electrical revolution. Therefore
       one mechanical revolution is 3*motor_poles tachometer steps. */
    return ((float)tach*(3.14159265358979323846f*diam))/(3.0f*poles*gear);
}
float mc_interface_get_distance(void){MotorRuntime*m=mc_interface_motor_runtime_now();return tach_to_distance(m,m->stats.tachometer);}
float mc_interface_get_distance_abs(void){MotorRuntime*m=mc_interface_motor_runtime_now();return tach_to_distance(m,m->stats.tachometer_abs);}
void mc_interface_odometer_add_tach_delta(motor_id_t id,uint32_t abs_tach_steps){
    unsigned idx=(id==MOTOR_RIGHT)?1U:0U;
    MotorRuntime*m=motor_get(id);
    if(!m||abs_tach_steps==0U)return;
    float dm=fabsf(tach_to_distance(m,(int32_t)abs_tach_steps));
    if(!isfinite(dm)||dm<=0.0f)return;
    float total=s_odometer_fraction_m[idx]+dm;
    uint64_t whole=(uint64_t)floorf(total);
    if(whole>0U){
        if(UINT64_MAX-s_odometer[idx]<whole)s_odometer[idx]=UINT64_MAX;
        else s_odometer[idx]+=whole;
        total-=floorf(total);
    }
    s_odometer_fraction_m[idx]=total;
}
void mc_interface_override_wheel_speed(bool ovr,float speed){s_wheel_speed_override=ovr;s_wheel_speed_override_value=speed;}
setup_values mc_interface_get_setup_values(void){
    /* VESC setup_values are controller-setup totals, not speed/power stats.
       This board has two local bridges on one MCU, so aggregate LEFT+RIGHT
       analogous to one VESC plus its locally-visible second controller. */
    setup_values v={0};
    MotorRuntime *l=&g_motor_left,*r=&g_motor_right;
    v.ah_tot=l->stats.amp_hours+r->stats.amp_hours;
    v.ah_charge_tot=l->stats.amp_hours_charged+r->stats.amp_hours_charged;
    v.wh_tot=l->stats.watt_hours+r->stats.watt_hours;
    v.wh_charge_tot=l->stats.watt_hours_charged+r->stats.watt_hours_charged;
    v.current_tot=l->motor_current+r->motor_current;
    v.current_in_tot=l->input_current+r->input_current;
    v.num_vescs=2U;
    return v;
}
volatile gnss_data*mc_interface_gnss(void){return &s_gnss;}
uint64_t mc_interface_get_odometer_motor(motor_id_t id){return s_odometer[id==MOTOR_RIGHT?1U:0U];}
void mc_interface_set_odometer_motor(motor_id_t id,uint64_t v){unsigned idx=id==MOTOR_RIGHT?1U:0U;s_odometer[idx]=v;s_odometer_fraction_m[idx]=0.0f;}
uint64_t mc_interface_get_odometer(void){return mc_interface_get_odometer_motor(mc_interface_get_motor_thread()==2?MOTOR_RIGHT:MOTOR_LEFT);}
void mc_interface_set_odometer(uint64_t v){mc_interface_set_odometer_motor(mc_interface_get_motor_thread()==2?MOTOR_RIGHT:MOTOR_LEFT,v);}
void mc_interface_ignore_input(int time_ms){MotorRuntime*m=mc_interface_motor_runtime_now();s_ignore_until[m->id]=osKernelGetTickCount()+(time_ms>0?(uint32_t)time_ms:0U);}
void mc_interface_set_current_off_delay(float d){mcpwm_foc_set_current_off_delay(d);}
void mc_interface_override_temp_motor(float t){s_temp_motor_override=t;}
void mc_interface_ignore_input_both(int time_ms){uint32_t u=osKernelGetTickCount()+(time_ms>0?(uint32_t)time_ms:0U);s_ignore_until[0]=u;s_ignore_until[1]=u;}
void mc_interface_release_motor_override_both(void){motor_stop(&g_motor_left);motor_stop(&g_motor_right);}
bool mc_interface_wait_for_motor_release_both(float timeout){uint32_t st=osKernelGetTickCount(),ms=(uint32_t)fmaxf(timeout*1000.0f,0.0f);while(g_motor_left.pwm_enabled||g_motor_right.pwm_enabled){if((uint32_t)(osKernelGetTickCount()-st)>ms)return false;osDelay(1);}return true;}

static setup_stats *setup_stats_now(void){return &s_setup_stats[mc_interface_get_motor_thread()==2?MOTOR_RIGHT:MOTOR_LEFT];}
float mc_interface_stat_speed_avg(void){setup_stats*s=setup_stats_now();return s->samples>0.0?(float)(s->speed_sum/s->samples):0.0f;}
float mc_interface_stat_speed_max(void){return setup_stats_now()->max_speed;}
float mc_interface_stat_power_avg(void){setup_stats*s=setup_stats_now();return s->samples>0.0?(float)(s->power_sum/s->samples):0.0f;}
float mc_interface_stat_power_max(void){return setup_stats_now()->max_power;}
float mc_interface_stat_current_avg(void){setup_stats*s=setup_stats_now();return s->samples>0.0?(float)(s->current_sum/s->samples):0.0f;}
float mc_interface_stat_current_max(void){return setup_stats_now()->max_current;}
float mc_interface_stat_temp_mosfet_avg(void){setup_stats*s=setup_stats_now();return s->samples>0.0?(float)(s->temp_mos_sum/s->samples):0.0f;}
float mc_interface_stat_temp_mosfet_max(void){return setup_stats_now()->max_temp_mos;}
float mc_interface_stat_temp_motor_avg(void){setup_stats*s=setup_stats_now();return s->samples>0.0?(float)(s->temp_motor_sum/s->samples):0.0f;}
float mc_interface_stat_temp_motor_max(void){return setup_stats_now()->max_temp_motor;}
float mc_interface_stat_count_time(void){uint32_t now=osKernelGetTickCount();return (float)(now-setup_stats_now()->time_start)/1000.0f;}
void mc_interface_stat_reset(void){setup_stats*s=setup_stats_now();memset(s,0,sizeof(*s));s->time_start=osKernelGetTickCount();}
void mc_interface_set_fault_info(const char*str,int argn,float arg0,float arg1){(void)str;(void)argn;(void)arg0;(void)arg1;}
void mc_interface_fault_stop(mc_fault_code f,bool is_second_motor,bool is_isr){MotorRuntime*m=motor_get(is_second_motor?MOTOR_RIGHT:MOTOR_LEFT);motor_fault_t native=motor_fault_from_vesc(f);if(is_isr)motor_request_fault_from_isr(m,native);else motor_raise_fault_from_task(m,native);}
void mc_interface_mc_timer_isr(bool is_second_motor,float dt){(void)is_second_motor;(void)dt;if(s_pwm_callback)s_pwm_callback();}


/* ============================================================================
 * VESC-standard consolidation: debug sampler + RTOS service/sample/fault threads.
 * Upstream VESC keeps these inside mc_interface.c (ChibiOS THD_FUNCTION + static
 * working areas). This F103 port uses CMSIS-RTOS2/FreeRTOS, so the same
 * ownership is retained here in the single compatibility translation unit.
 * ============================================================================ */

/* Some host compile stubs expose TaskHandle_t without the optional deletion
 * prototype. The real FreeRTOS task.h declaration has this exact signature. */
extern void vTaskDelete(TaskHandle_t task_to_delete);

#include <stddef.h>

/*
 * Upstream VESC owns the service, sample sender and fault-stop threads in
 * mc_interface.c. This F103 port keeps their implementation in this private
 * translation unit to keep the already large compatibility layer reviewable.
 * The public API and ownership still remain in mc_interface.
 */
#define RTOS_READY_HEAP_RESERVE_BYTES 2048U

static osThreadId_t s_timer_thread;
static osThreadId_t s_sample_send_thread;
static osThreadId_t s_fault_stop_thread;
static bool s_threads_started;

void timer_thread(void *argument);
void sample_send_thread(void *argument);
void fault_stop_thread(void *argument);

void mc_interface_set_thread_ids(osThreadId_t timer, osThreadId_t sample,
                                 osThreadId_t fault) {
    s_timer_thread = timer;
    s_sample_send_thread = sample;
    s_fault_stop_thread = fault;
}

static uint32_t thread_stack_free_bytes(osThreadId_t thread) {
	if (thread == NULL) {
		return 0U;
	}
	return (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)thread) *
			(uint32_t)sizeof(StackType_t);
}

static void fault_signal(motor_id_t motor) {
	if (s_fault_stop_thread != NULL) {
		(void)osThreadFlagsSet(s_fault_stop_thread, 1UL << (uint32_t)motor);
	}
}

static void sample_signal(void) {
	if (s_sample_send_thread != NULL) {
		(void)osThreadFlagsSet(s_sample_send_thread, 1UL);
	}
}

uint32_t mc_interface_free_heap_bytes(void) {
	return (uint32_t)xPortGetFreeHeapSize();
}

uint32_t mc_interface_min_ever_free_heap_bytes(void) {
	return (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

void mc_interface_get_resource_stats(mc_interface_resource_stats_t *stats) {
	if (stats == NULL) {
		return;
	}

	stats->heap_free_bytes = mc_interface_free_heap_bytes();
	stats->heap_min_ever_bytes = mc_interface_min_ever_free_heap_bytes();
	stats->motor_service_stack_free_bytes =
			thread_stack_free_bytes(s_timer_thread);
	stats->sample_sender_stack_free_bytes =
			thread_stack_free_bytes(s_sample_send_thread);
	stats->fault_stack_free_bytes =
			thread_stack_free_bytes(s_fault_stop_thread);
	stats->status_stack_free_bytes = hw_status_stack_free_bytes();
}

void timer_thread(void *argument) {
	(void)argument;
	bool current_offset_fault_reported = false;
	uint8_t calibration_divider = 0U;
	uint8_t ten_ms_divider = 0U;
	uint32_t next = osKernelGetTickCount();

	for (;;) {
		next += 1U;
		const uint32_t now = osKernelGetTickCount();
		timeout_heartbeat(TIMEOUT_HEARTBEAT_MOTOR_SERVICE);

		motor_slow_update_1khz(&g_motor_left, now);
		motor_slow_update_1khz(&g_motor_right, now);
		motor_rpm_update_1khz(&g_motor_left);
		motor_rpm_update_1khz(&g_motor_right);

		/* APP ADC and serial commands share this central command arbitration. */
		app_command_service_1khz(now);
		app_adc_service_1khz(now);

		motor_pid_update_1khz(&g_motor_left);
		motor_pid_update_1khz(&g_motor_right);

		ten_ms_divider++;
		if (ten_ms_divider >= 10U) {
			ten_ms_divider = 0U;
			timeout_update_10ms(now);
			timeout_watchdog_update_10ms(now);
			telemetry_stats_update_100hz();
			telemetry_snapshot_100hz();
			vesc_comm_periodic_100hz();
		}

		/* The ISR only accumulates fixed-point calibration statistics. */
		calibration_divider++;
		if (calibration_divider >= 5U) {
			calibration_divider = 0U;
			foc_calibration_service_task();
			if (foc_calibration_done()) {
				if (!foc_calibration_valid() && !current_offset_fault_reported) {
					current_offset_fault_reported = true;
					motor_raise_fault_from_task(&g_motor_left,
							MOTOR_FAULT_CURRENT_OFFSET);
					motor_raise_fault_from_task(&g_motor_right,
							MOTOR_FAULT_CURRENT_OFFSET);
					fault_signal(MOTOR_LEFT);
					fault_signal(MOTOR_RIGHT);
				} else if (foc_calibration_valid()) {
					current_offset_fault_reported = false;
					if (g_motor_left.fault == MOTOR_FAULT_CURRENT_OFFSET) {
						motor_clear_fault(&g_motor_left);
					}
					if (g_motor_right.fault == MOTOR_FAULT_CURRENT_OFFSET) {
						motor_clear_fault(&g_motor_right);
					}
				}
			}
		}

		const uint32_t pending = motor_take_pending_fault_mask();
		if ((pending & (1UL << MOTOR_LEFT)) != 0U) {
			fault_signal(MOTOR_LEFT);
		}
		if ((pending & (1UL << MOTOR_RIGHT)) != 0U) {
			fault_signal(MOTOR_RIGHT);
		}

		if (mc_interface_sample_ready()) {
			sample_signal();
		}
		osDelayUntil(next);
	}
}

void sample_send_thread(void *argument) {
	(void)argument;
	for (;;) {
		const uint32_t flags = osThreadFlagsWait(1UL, osFlagsWaitAny,
				osWaitForever);
		if ((flags & osFlagsError) != 0U) {
			continue;
		}
		if (mc_interface_sample_ready()) {
			void (*reply)(unsigned char *, unsigned int) =
					mc_interface_sample_reply_func();
			if (reply != NULL) {
				vesc_comm_send_sample_buffer_to(reply,
						mc_interface_sample_count());
			} else {
				vesc_comm_send_sample_buffer(mc_interface_sample_data(),
						mc_interface_sample_count());
			}
			mc_interface_sample_mark_sent();
		}
	}
}

void fault_stop_thread(void *argument) {
	(void)argument;
	const uint32_t mask = (1UL << MOTOR_LEFT) | (1UL << MOTOR_RIGHT);

	for (;;) {
		const uint32_t flags = osThreadFlagsWait(mask, osFlagsWaitAny, 50U);
		timeout_heartbeat(TIMEOUT_HEARTBEAT_FAULT);
		if ((flags & osFlagsError) != 0U) {
			continue;
		}
		/* A stale task-side fault-stop flag (for example FLASH_CONFIG raised at
		 * boot) must not tear down the safe 50% zero-vector while offset
		 * calibration is active. Hardware PVD/BKIN/ADC faults already clear MOE
		 * synchronously in their ISR/emergency path, so deferring this task-side
		 * stop during calibration does not weaken hard protection. */
		if (!foc_calibration_in_progress()) {
			if ((flags & (1UL << MOTOR_LEFT)) != 0U) {
				motor_hw_set_pwm_enabled(&g_motor_left, false);
			}
			if ((flags & (1UL << MOTOR_RIGHT)) != 0U) {
				motor_hw_set_pwm_enabled(&g_motor_right, false);
			}
		}
	}
}

bool mc_interface_start_threads(void) {
	if (s_threads_started) {
		return s_timer_thread != NULL && s_sample_send_thread != NULL &&
				s_fault_stop_thread != NULL;
	}

	/* Inisialisasi app_command/app_adc/timeout dilakukan di main.c SEBELUM
	 * spawn thread (lihat motor_boot_thread). Fungsi ini hanya memvalidasi
	 * handle yang didaftarkan lewat mc_interface_set_thread_ids() dan
	 * menjalankan pemeriksaan resource/heap. */
	const bool threads_ok = s_timer_thread != NULL &&
	        s_sample_send_thread != NULL && s_fault_stop_thread != NULL;
	const bool heap_ok = mc_interface_free_heap_bytes() >=
			RTOS_READY_HEAP_RESERVE_BYTES;
	if (!threads_ok || !heap_ok) {
		/* Do not advertise a half-started controller. The three workers touch
		 * shared motor state immediately after creation, so a failed allocation
		 * or reserve check is rolled back before a later retry. */
		if (s_timer_thread != NULL) {
			vTaskDelete((TaskHandle_t)s_timer_thread);
			s_timer_thread = NULL;
		}
		if (s_sample_send_thread != NULL) {
			vTaskDelete((TaskHandle_t)s_sample_send_thread);
			s_sample_send_thread = NULL;
		}
		if (s_fault_stop_thread != NULL) {
			vTaskDelete((TaskHandle_t)s_fault_stop_thread);
			s_fault_stop_thread = NULL;
		}
		return false;
	}

	s_threads_started = true;
	timeout_watchdog_start();
	return true;
}

#include "applications/appconf_default.h"
#include <limits.h>
#include <string.h>

static debug_sample_t s_samples[SAMPLE_BUFFER_LEN];
static volatile uint16_t s_target_len;
static volatile uint16_t s_wr;             /* physical next-write index */
static volatile uint16_t s_count;          /* valid samples in buffer */
static volatile uint16_t s_read_start;     /* oldest physical sample */
static volatile uint16_t s_decimation;
static volatile uint16_t s_decim_count;
static volatile uint16_t s_post_remaining;
static volatile motor_id_t s_motor;
static volatile debug_sampling_mode s_mode;
static volatile bool s_active;
static volatile bool s_armed;
static volatile bool s_triggered;
static volatile bool s_capture_valid;
static volatile bool s_send_pending;
static volatile bool s_auto_send;
static volatile bool s_raw;
static volatile bool s_prev_running;
static volatile bool s_prev_fault;

static int16_t sat_i16(int32_t value) {
	if (value > INT16_MAX) {
		return INT16_MAX;
	}
	if (value < INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)value;
}

static void sample_fill(debug_sample_t *d, MotorRuntime *m) {
	d->ia_cA = sat_i16((m->ia_q15 * 6400L) / 32768L);
	d->ib_cA = sat_i16((m->ib_q15 * 6400L) / 32768L);
	d->id_cA = sat_i16((m->id_q15 * 6400L) / 32768L);
	d->iq_cA = sat_i16((m->iq_q15 * 6400L) / 32768L);
	d->vd_cV = sat_i16((m->vd_q15 * 6400L) / 32768L);
	d->vq_cV = sat_i16((m->vq_q15 * 6400L) / 32768L);
	d->erpm = sat_i16(m->erpm_int);
	d->phase_u16 = motor_sensor_electrical_phase_u16(m);
	d->duty_u_q15 = m->duty_u_q15;
	d->duty_v_q15 = m->duty_v_q15;
	d->duty_w_q15 = m->duty_w_q15;
	d->current_raw_u = m->current_raw_u;
	d->current_raw_v = m->current_raw_v;

	int32_t vbus_dv = (m->vbus_q15 * 640L) / 32768L;
	if (vbus_dv < 0) {
		vbus_dv = 0;
	}
	if (vbus_dv > UINT16_MAX) {
		vbus_dv = UINT16_MAX;
	}
	d->vbus_dV = (uint16_t)vbus_dv;
	d->motor = (uint8_t)m->id;
	d->hall_raw = m->hall.raw_state;
}

static void finish_capture_isr(void) {
	s_active = false;
	s_armed = false;
	s_capture_valid = (s_count != 0U);
	s_read_start = (s_count >= s_target_len) ? s_wr : 0U;
	s_send_pending = s_capture_valid && s_auto_send;
}

void mc_interface_sample_init(void) {
	memset(s_samples, 0, sizeof(s_samples));
	s_target_len = SAMPLE_BUFFER_LEN;
	s_wr = 0U;
	s_count = 0U;
	s_read_start = 0U;
	s_decimation = SAMPLE_DEFAULT_DECIMATION;
	s_decim_count = 0U;
	s_post_remaining = 0U;
	s_motor = MOTOR_LEFT;
	s_mode = DEBUG_SAMPLING_OFF;
	s_active = false;
	s_armed = false;
	s_triggered = false;
	s_capture_valid = false;
	s_send_pending = false;
	s_auto_send = true;
	s_raw = false;
	s_prev_running = false;
	s_prev_fault = false;
}

bool mc_interface_sample_control(debug_sampling_mode mode, motor_id_t motor,
		uint16_t len, uint16_t decimation, bool raw) {
	/* debug_sampling_mode starts at DEBUG_SAMPLING_OFF. A lower-bound check
	 * would trigger -Wtype-limits when ARM GCC represents the enum unsigned. */
	if (mode > DEBUG_SAMPLING_SEND_SINGLE_SAMPLE ||
			(motor != MOTOR_LEFT && motor != MOTOR_RIGHT)) {
		return false;
	}
	if (len == 0U || len > SAMPLE_BUFFER_LEN) {
		len = SAMPLE_BUFFER_LEN;
	}
	if (decimation == 0U) {
		decimation = 1U;
	}

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	if (mode == DEBUG_SAMPLING_OFF) {
		s_active = false;
		s_armed = false;
		s_triggered = false;
		s_send_pending = false;
		s_mode = mode;
		if (primask == 0U) {
			__enable_irq();
		}
		return true;
	}

	if (mode == DEBUG_SAMPLING_SEND_LAST_SAMPLES) {
		const bool capture_valid = s_capture_valid;
		if (capture_valid) {
			s_send_pending = true;
		}
		if (primask == 0U) {
			__enable_irq();
		}
		return capture_valid;
	}

	/* Never overwrite a buffer while the UART worker is serializing it. */
	if (s_send_pending) {
		if (primask == 0U) {
			__enable_irq();
		}
		return false;
	}

	s_motor = motor;
	s_mode = mode;
	s_target_len = (mode == DEBUG_SAMPLING_SEND_SINGLE_SAMPLE) ? 1U : len;
	s_decimation = decimation;
	s_decim_count = 0U;
	s_wr = 0U;
	s_count = 0U;
	s_read_start = 0U;
	s_post_remaining = 0U;
	s_raw = raw;
	s_capture_valid = false;
	s_send_pending = false;
	s_triggered = false;
	s_prev_running = false;
	s_prev_fault = false;
	s_auto_send = (mode != DEBUG_SAMPLING_TRIGGER_START_NOSEND &&
			mode != DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND);

	switch (mode) {
	case DEBUG_SAMPLING_NOW:
	case DEBUG_SAMPLING_SEND_SINGLE_SAMPLE:
		s_active = true;
		s_armed = false;
		break;
	case DEBUG_SAMPLING_START:
	case DEBUG_SAMPLING_TRIGGER_START:
	case DEBUG_SAMPLING_TRIGGER_FAULT:
	case DEBUG_SAMPLING_TRIGGER_START_NOSEND:
	case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND:
		s_active = true;
		s_armed = true;
		break;
	default:
		s_active = false;
		s_armed = false;
		break;
	}

	if (primask == 0U) {
		__enable_irq();
	}
	return true;
}

void mc_interface_sample_start_ex(motor_id_t motor, uint16_t len,
		uint16_t decimation, bool raw) {
	(void)mc_interface_sample_control(DEBUG_SAMPLING_NOW, motor, len,
			decimation, raw);
}

void mc_interface_sample_start(motor_id_t motor, uint16_t len,
		uint16_t decimation) {
	mc_interface_sample_start_ex(motor, len, decimation, false);
}

void mc_interface_sample_capture_isr(MotorRuntime *active) {
	if (!s_active || active == NULL || active->id != s_motor) {
		return;
	}

	const bool running = active->pwm_enabled &&
			active->fault == MOTOR_FAULT_NONE;
	const bool fault_now = active->fault != MOTOR_FAULT_NONE;
	const bool start_edge = running && !s_prev_running;
	const bool fault_edge = fault_now && !s_prev_fault;
	s_prev_running = running;
	s_prev_fault = fault_now;

	if (s_armed && !s_triggered) {
		bool trigger = false;
		switch (s_mode) {
		case DEBUG_SAMPLING_START:
		case DEBUG_SAMPLING_TRIGGER_START:
		case DEBUG_SAMPLING_TRIGGER_START_NOSEND:
			trigger = start_edge;
			break;
		case DEBUG_SAMPLING_TRIGGER_FAULT:
		case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND:
			trigger = fault_edge;
			break;
		default:
			trigger = true;
			break;
		}
		if (trigger) {
			s_triggered = true;
			s_armed = false;
			if (s_mode == DEBUG_SAMPLING_START) {
				/* START begins a fresh capture on the run edge. */
				s_wr = 0U;
				s_count = 0U;
				s_read_start = 0U;
				s_post_remaining = s_target_len;
			} else {
				/* Keep circular pre-trigger history. Half a capture after the
				 * edge gives an approximately even pre/post split. */
				s_post_remaining =
						(uint16_t)((s_target_len + 1U) / 2U);
			}
		}
	}

	if (++s_decim_count < s_decimation) {
		return;
	}
	s_decim_count = 0U;

    /* START does not record before its start edge. Trigger modes do, as a
       circular pre-trigger history. */
	const bool trigger_mode =
			s_mode == DEBUG_SAMPLING_TRIGGER_START ||
			s_mode == DEBUG_SAMPLING_TRIGGER_FAULT ||
			s_mode == DEBUG_SAMPLING_TRIGGER_START_NOSEND ||
			s_mode == DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND;
	if (s_mode == DEBUG_SAMPLING_START && !s_triggered) {
		return;
	}

	sample_fill(&s_samples[s_wr], active);
	s_wr++;
	if (s_wr >= s_target_len) {
		s_wr = 0U;
	}
	if (s_count < s_target_len) {
		s_count++;
	}

	if (trigger_mode || s_mode == DEBUG_SAMPLING_START) {
		if (s_triggered && s_post_remaining > 0U) {
			s_post_remaining--;
			if (s_post_remaining == 0U) {
				finish_capture_isr();
			}
		}
		return;
	}

	if (s_count >= s_target_len) {
		finish_capture_isr();
	}
}

bool mc_interface_sample_ready(void) {
	return s_capture_valid && s_send_pending;
}

bool mc_interface_sample_has_capture(void) {
	return s_capture_valid;
}

uint16_t mc_interface_sample_count(void) {
	return s_count;
}

const debug_sample_t *mc_interface_sample_data(void) {
	return s_samples;
}

const debug_sample_t *mc_interface_sample_at(uint16_t logical_index) {
	if (!s_capture_valid || logical_index >= s_count || s_target_len == 0U) {
		return NULL;
	}

	uint16_t physical_index = (uint16_t)(s_read_start + logical_index);
	while (physical_index >= s_target_len) {
		physical_index = (uint16_t)(physical_index - s_target_len);
	}
	return &s_samples[physical_index];
}

void mc_interface_sample_mark_sent(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	s_send_pending = false;
	if (primask == 0U) {
		__enable_irq();
	}
}

bool mc_interface_sample_active(void) {
	return s_active;
}

bool mc_interface_sample_raw(void) {
	return s_raw;
}

debug_sampling_mode mc_interface_sample_mode(void) {
	return s_mode;
}

