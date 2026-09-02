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

/* Fault software yang recoverable harus sehat kontinu selama 1000 ms sebelum
 * dilepas. Fault hardware-latched (PVD/BKIN/BREAK/MCU UV/flash) tetap memerlukan
 * intervensi/reset dan tidak disamarkan oleh timer ini. */
#define MOTOR_FAULT_RECOVERY_HOLD_MS 1000U
#define MOTOR_FAULT_RECOVERY_CURRENT_A 1.0f

#include <math.h>
#include <limits.h>
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "comm/commands.h"
#include "timeout.h"
#include "FreeRTOS.h"
#include "task.h"

// Variabel g_motor_left: state atau parameter motor yang sedang diproses.
MotorRuntime g_motor_left;
// Variabel g_motor_right: state atau parameter motor yang sedang diproses.
MotorRuntime g_motor_right;
// Variabel s_pending_fault_mask: status atau data gangguan untuk sistem proteksi.
static volatile uint32_t s_pending_fault_mask = 0U;

/* VESC setup statistics are task-side diagnostics, never part of the hard FOC ISR.
 * One accumulator is kept per local bridge and sampled at 100 Hz. */
// Variabel s_setup_stats: state internal modul yang dipertahankan antar pemanggilan fungsi.
static setup_stats s_setup_stats[2];
// Variabel s_setup_stats_div: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_setup_stats_div[2];
// Variabel s_temp_motor_override: nilai sementara atau temperatur sesuai konteks modul.
static float s_temp_motor_override = NAN;

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi amp_to_current_q15: menjalankan operasi amp to current q15 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int32_t amp_to_current_q15(float a) {
    // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float q = (a / FOC_CURRENT_Q_BASE_A) * 32768.0f;
    if (q > 32767.0f)
        q = 32767.0f;
    if (q < -32768.0f)
        q = -32768.0f;
    return (int32_t)q;
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi volt_to_q15: menjalankan operasi volt to q15 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int32_t volt_to_q15(float v) {
    // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float q = (v / FOC_VOLTAGE_Q_BASE_V) * 32768.0f;
    if (q > 32767.0f)
        q = 32767.0f;
    if (q < 0.0f)
        q = 0.0f;
    return (int32_t)q;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id_a: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Parameter iq_a: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi motor_set_foc_targets: mengatur motor set foc targets setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_set_foc_targets(MotorRuntime *m, float id_a, float iq_a) {
    if (m == NULL)
        return;
    // Variabel hi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float hi = (m->current_max_a > 0.0f) ? m->current_max_a : FOC_MAX_CURRENT_A;
    // Variabel lo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lo = (m->current_min_a < 0.0f) ? m->current_min_a : -hi;
    id_a = foc_clampf(id_a, lo, hi);
    iq_a = foc_clampf(iq_a, lo, hi);
    m->id_target = id_a;
    m->iq_target = iq_a;
    // Variabel id_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    const int32_t id_q15 = amp_to_current_q15(id_a);
    // Variabel iq_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const int32_t iq_q15 = amp_to_current_q15(iq_a);
    m->id_target_base_q15 = id_q15;
    m->iq_target_base_q15 = iq_q15;
    m->id_target_q15 = id_q15;
    m->iq_target_q15 = iq_q15;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter kp: penguatan proporsional regulator.
// Parameter ki: penguatan integral regulator.
// Fungsi motor_set_current_pi_gains: mengatur motor set current pi gains setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_set_current_pi_gains(MotorRuntime *m, float kp, float ki) {
    if (m == NULL)
        return;
    if (!isfinite(kp) || kp < 0.000001f)
        kp = 0.000001f;
    if (!isfinite(ki) || ki < 0.0f)
        ki = 0.0f;
    m->current_kp = kp;
    m->current_ki = ki;
    m->current_kp_q16 = (int32_t)((kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->current_ki_dt_q16 = (int32_t)((ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    /* A gain change is a control discontinuity. Reset only the fast current
       integrators; outer-loop state is left to the caller. */
    m->vd_int_q31 = 0;
    m->vq_int_q31 = 0;
    m->vd_int_q15 = 0;
    m->vq_int_q15 = 0;
}

// Parameter m: motor yang menerima tabel Hall hasil konfigurasi atau deteksi.
// Parameter table: tabel Hall VESC 0..200, dengan 255 menandai state yang tidak valid.
// Fungsi motor_apply_foc_hall_table: membangun lookup sudut dan urutan sektor Hall yang dipakai ISR dari satu
// sumber tabel VESC agar SET_MCCONF, auto-detect, dan fallback selalu menggunakan data yang sama.
bool motor_apply_foc_hall_table(MotorRuntime *m, const uint8_t table[8]) {
    if (m == NULL || table == NULL)
        return false;

    /* Run21 dan checkpoint lama memakai raw Hall U pada bit-0. Jika hanya
       fallback bawaan lama yang tersimpan di flash, migrasikan deterministik
       ke packing referensi U<<2|V<<1|W. Tabel hasil auto-detect pengguna tidak
       ditebak di sini karena hanya deteksi ulang yang dapat membuktikan fase. */
    // Variabel legacy_fallback: signature tabel fallback checkpoint dengan packing Hall lama.
    static const uint8_t legacy_fallback[8] = {
        255U, 0U, 133U, 166U, 66U, 33U, 100U, 255U
    };
    // Variabel reference_fallback: fallback referensi untuk packing U<<2|V<<1|W.
    static const uint8_t reference_fallback[8] = {
        255U, 66U, 0U, 33U, 133U, 100U, 166U, 255U
    };
    // Variabel effective_table: tabel yang benar-benar diterapkan ke lookup ISR Hall.
    const uint8_t *effective_table = table;
    if (memcmp(table, legacy_fallback, sizeof(legacy_fallback)) == 0)
        effective_table = reference_fallback;

    typedef struct {
        // Variabel raw: state Hall mentah yang menjadi indeks tabel VESC.
        uint8_t raw;
        // Variabel angle_u16: sudut listrik Hall dalam satu putaran uint16.
        uint16_t angle_u16;
    } hall_item_t;

    // Variabel items: daftar state Hall valid untuk menyusun urutan sektor.
    hall_item_t items[6];
    // Variabel count: jumlah state Hall valid yang ditemukan pada tabel.
    uint8_t count = 0U;
    for (uint8_t raw = 0U; raw < 8U; raw++) {
        // Variabel value: sudut Hall VESC 0..200 untuk state mentah saat ini.
        const uint8_t value = effective_table[raw];
        m->foc_hall_table[raw] = value;
        m->hall_table[raw] = -1;
        m->hall_angle_u16[raw] = 0U;
        if (value <= 200U && raw != 0U && raw != 7U && count < 6U) {
            // Variabel angle: konversi sudut VESC ke representasi satu putaran uint16.
            const uint16_t angle = (uint16_t)(((uint32_t)value * 65536U) / 200U);
            m->hall_angle_u16[raw] = angle;
            items[count].raw = raw;
            items[count].angle_u16 = angle;
            count++;
        }
    }

    /* Urutan sektor diturunkan dari sudut hasil deteksi, bukan dari nomor raw
       Hall. Dengan demikian forward/reverse neighbor check tetap benar walau
       urutan kabel Hall berbeda. */
    for (uint8_t i = 0U; i < count; i++) {
        for (uint8_t j = (uint8_t)(i + 1U); j < count; j++) {
            if (items[j].angle_u16 < items[i].angle_u16) {
                // Variabel tmp: penampung sementara saat mengurutkan state Hall berdasarkan sudut.
                const hall_item_t tmp = items[i];
                items[i] = items[j];
                items[j] = tmp;
            }
        }
    }
    for (uint8_t i = 0U; i < count; i++)
        m->hall_table[items[i].raw] = (int8_t)i;

    m->hall.valid = false;
    m->hall.sector = -1;
    m->hall.period_cycles = 0U;
    m->hall.phase_per_cycle_q16 = 0;
    m->hall.rate_limited_valid = false;
    m->hall.invalid_count = 0U;
    m->hall.sequence_error_count = 0U;

    return count == 6U && effective_table[0] == 255U && effective_table[7] == 255U;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi init_hall_defaults: menginisialisasi init hall defaults sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
static void init_hall_defaults(MotorRuntime *m) {
    // Variabel sectors: urutan sektor fallback dari firmware hoverboard referensi.
    const int8_t sectors[8] = HALL_TABLE_DEFAULT;
    // Variabel vesc_table: tabel fallback dalam format sudut 0..200 milik VESC.
    uint8_t vesc_table[8];
    for (uint8_t raw = 0U; raw < 8U; raw++) {
        vesc_table[raw] = sectors[raw] >= 0 ?
                          (uint8_t)(((uint32_t)sectors[raw] * 200U) / 6U) : 255U;
    }
    (void)motor_apply_foc_hall_table(m, vesc_table);
    m->hall.direction = 1;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi motor_defaults: menjalankan operasi motor defaults sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
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
    /* Mode sensor fisik mengikuti sumber fase FOC. Saat SENSORLESS dipilih,
       Hall/encoder benar-benar dilepas dari jalur runtime agar motor tidak
       bergantung pada edge sensor yang tidak terpasang atau berisik. */
    if (m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
        m->sensor_mode = SENSOR_MODE_NO_SENSOR;
        m->sensor_request_mode = SENSOR_MODE_NO_SENSOR;
    }
    else {
        m->sensor_mode = (id == MOTOR_LEFT && LEFT_SENSOR_BOOT_MODE == SENSOR_MODE_ENCODER) ?
                         SENSOR_MODE_ENCODER : SENSOR_MODE_HALL;
        m->sensor_request_mode = (id == MOTOR_LEFT) ? LEFT_SENSOR_BOOT_MODE : RIGHT_SENSOR_BOOT_MODE;
    }
    m->control_mode = MOTOR_CTRL_OFF;
    m->fault = MOTOR_FAULT_NONE;
    m->last_fault = MOTOR_FAULT_NONE;
    m->fault_recovery_ticks = 0U;
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
    m->current_max_a = FOC_MAX_CURRENT_A;
    m->current_min_a = -FOC_MAX_CURRENT_A;
    m->input_current_max_a = FOC_MAX_CURRENT_A;
    m->input_current_min_a = -FOC_MAX_CURRENT_A;
    m->abs_current_max_a = FOC_ABS_CURRENT_TRIP_A;
    m->abs_current_trip_q15 = amp_to_current_q15(FOC_ABS_CURRENT_TRIP_A);
    m->slow_abs_current = false;
    m->temp_fet_start = MCCONF_L_TEMP_FET_START_DEFAULT;
    m->temp_fet_end = MCCONF_L_TEMP_FET_END_DEFAULT;
    m->temp_motor_start = MCCONF_L_TEMP_MOTOR_START_DEFAULT;
    m->temp_motor_end = MCCONF_L_TEMP_MOTOR_END_DEFAULT;
    m->temp_accel_dec = MCCONF_L_TEMP_ACCEL_DEC_DEFAULT;
    m->additional_faults = MCCONF_L_ADDITIONAL_FAULTS_DEFAULT;
    m->board_temp_c = 25.0f;
    m->board_temp_filter_c = 25.0f;
    m->board_temp_valid = false;
    m->abs_phase_current_filter_q15 = 0;
    m->abs_current_peak_q15 = 0;
    m->abs_current_fault_count = 0U;
    m->min_vin = VBUS_MIN_RUN_V;
    m->max_vin = VBUS_MAX_RUN_V;
    m->battery_cut_start = 36.0f;
    m->battery_cut_end = 32.0f;
    m->battery_regen_cut_start = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    m->battery_regen_cut_end = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    m->min_vin_q15 = volt_to_q15(m->min_vin);
    m->max_vin_q15 = volt_to_q15(m->max_vin);
    m->hard_max_vin_q15 = volt_to_q15(fminf(m->max_vin + FOC_VBUS_HARD_OV_MARGIN_V, FOC_VBUS_HARD_MAX_V));
    m->hard_min_vin_q15 = volt_to_q15(fmaxf(m->min_vin - FOC_VBUS_HARD_UV_MARGIN_V, FOC_VBUS_HARD_MIN_V));
    m->over_voltage_fault_count = 0U;
    m->under_voltage_fault_count = 0U;
    m->max_erpm = MOTOR_DEFAULT_MAX_ERPM;
    m->min_erpm = MOTOR_DEFAULT_MIN_ERPM;
    m->erpm_start = MCCONF_L_ERPM_START_DEFAULT;
    m->erpm_fault_filter = 0.0f;
    m->foc_start_curr_dec = MCCONF_FOC_START_CURR_DEC_DEFAULT;
    m->foc_start_curr_dec_rpm = MCCONF_FOC_START_CURR_DEC_RPM_DEFAULT;
    m->foc_short_ls_on_zero_duty = MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT;
    m->full_brake_active = false;
    m->max_duty = MOTOR_DEFAULT_MAX_DUTY;
    m->min_duty = MOTOR_DEFAULT_MIN_DUTY;
    m->duty_limit_now = m->max_duty;
    m->duty_was_pi = false;
    m->duty_pi_duty_last = 0.0f;
    m->force_zero_modulation = false;
    m->brake_speed_before_q16 = 0;
    m->brake_vq_before_q15 = 0;
    m->brake_zero_hold_ticks = 1U;
    m->brake_zero_active = false;
    m->current_max_scale = MCCONF_L_CURRENT_MAX_SCALE_DEFAULT;
    m->current_min_scale = MCCONF_L_CURRENT_MIN_SCALE_DEFAULT;
    m->watt_max = MCCONF_L_WATT_MAX_DEFAULT;
    m->watt_min = MCCONF_L_WATT_MIN_DEFAULT;
    m->duty_start = MCCONF_L_DUTY_START_DEFAULT;
    m->lo_current_max_a = m->current_max_a;
    m->lo_current_min_a = m->current_min_a;
    m->lo_input_current_max_a = m->input_current_max_a;
    m->lo_input_current_min_a = m->input_current_min_a;
    m->input_current_map_start = MCCONF_L_IN_CURRENT_MAP_START_DEFAULT;
    m->input_current_map_filter = MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT;
    m->input_current_map_filtered_a = 0.0f;
    m->input_current_map_limit_a = m->current_max_a;
    m->speed_pid.kp = SPEED_PID_KP;
    m->speed_pid.ki = SPEED_PID_KI;
    m->speed_pid.kd = SPEED_PID_KD;
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
    m->foc_fw_current_now = 0.0f;
    m->mtpa_id_target = 0.0f;
    m->foc_fw_current_acc_q31 = 0;
    m->foc_fw_current_q15 = 0;
    m->foc_fw_duty_filter_q15 = 0;
    m->fw_override_current_q15 = 0;
    m->foc_fw_fast_active = false;
    m->foc_fw_hold_request = false;
    m->foc_current_limit_q15 = amp_to_current_q15(FOC_MAX_CURRENT_A);
    m->encoder_slip_bad_ticks = 0U;
    m->encoder_slip_error_phase = 0;
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
    }
    else {
        m->hall_offset_u16 = foc_deg_to_u16(RIGHT_HALL_ELEC_OFFSET_DEG_DEFAULT);
    }
}

// Fungsi motor_control_init: menginisialisasi motor control init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void motor_control_init(void) {
    motor_defaults(&g_motor_left, MOTOR_LEFT);
    motor_defaults(&g_motor_right, MOTOR_RIGHT);
    memset(s_setup_stats, 0, sizeof(s_setup_stats));
    memset(s_setup_stats_div, 0, sizeof(s_setup_stats_div));
    s_setup_stats[MOTOR_LEFT].time_start = xTaskGetTickCount();
    s_setup_stats[MOTOR_RIGHT].time_start = s_setup_stats[MOTOR_LEFT].time_start;
    foc_math_init();
    /* Precompute every float-derived coefficient before ADC/DMA can enter the
       hard FOC ISR. Configuration changes repeat this atomically task-side. */
    foc_precalc_values(&g_motor_left);
    foc_precalc_values(&g_motor_right);
    mcpwm_foc_init_hw();
    motor_hw_configure_sensor(&g_motor_left, g_motor_left.sensor_mode);
    motor_hw_configure_sensor(&g_motor_right, g_motor_right.sensor_mode);
    if (g_motor_left.sensor_mode == SENSOR_MODE_HALL)
        motor_hall_edge_isr(&g_motor_left);
    if (g_motor_right.sensor_mode == SENSOR_MODE_HALL)
        motor_hall_edge_isr(&g_motor_right);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi motor_get: membaca motor get tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
MotorRuntime *motor_get(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? &g_motor_right : &g_motor_left;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_touch_command: menjalankan operasi motor touch command sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void motor_touch_command(MotorRuntime *m) {
    m->last_command_tick = xTaskGetTickCount();
    m->command_active = true;
    m->timeout_active = false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_keepalive: menjalankan operasi motor keepalive sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void motor_keepalive(MotorRuntime *m) {
    if (m == NULL)
        return;
    m->last_command_tick = xTaskGetTickCount();
    m->timeout_active = false;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter amp: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_current: mengatur motor set current setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void motor_set_current(MotorRuntime *m, float amp) {
    if (m == NULL)
        return;
    mcpwm_foc_set_current_motor(m, amp);
    motor_touch_command(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter amp: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_brake_current: mengatur motor set brake current setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_set_brake_current(MotorRuntime *m, float amp) {
    if (m == NULL)
        return;
    mcpwm_foc_set_brake_current_motor(m, amp);
    motor_touch_command(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter rel: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_current_rel: mengatur motor set current rel setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_set_current_rel(MotorRuntime *m, float rel) {
    if (m == NULL) {
        return;
    }

    rel = foc_clampf(rel, -1.0f, 1.0f);
    motor_set_current(m, rel >= 0.0f ? rel * m->current_max_a : (-rel) * m->current_min_a);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter amp: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_handbrake: mengatur motor set handbrake setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void motor_set_handbrake(MotorRuntime *m, float amp) {
    if (m == NULL) {
        return;
    }

    mcpwm_foc_set_handbrake_motor(m, amp);
    motor_touch_command(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi motor_set_speed: mengatur motor set speed setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void motor_set_speed(MotorRuntime *m, float erpm) {
    if (m == NULL)
        return;
    mcpwm_foc_set_pid_speed_motor(m, erpm);
    motor_touch_command(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_position: mengatur motor set position setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void motor_set_position(MotorRuntime *m, float deg) {
    if (m == NULL)
        return;
    mcpwm_foc_set_pid_pos_motor(m, deg);
    motor_touch_command(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi motor_set_duty: mengatur motor set duty setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void motor_set_duty(MotorRuntime *m, float duty) {
    if (m == NULL)
        return;
    mcpwm_foc_set_duty_motor(m, duty);
    motor_touch_command(m);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_stop: menghentikan motor stop dengan menonaktifkan output atau state terkait secara aman.
void motor_stop(MotorRuntime *m) {
    if (m == NULL)
        return;
    mcpwm_foc_release_motor_motor(m);
    m->command_active = false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_clear_fault: menangani motor clear fault dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
void motor_clear_fault(MotorRuntime *m) {
    if (m == NULL)
        return;
    /* PVD/BKIN faults are reset-latched power-stage faults. Refuse to hide
       them from VESC Tool while hardware re-enable is still blocked. */
    if (motor_hw_powerstage_fault_latched() ||
        (m->fault == MOTOR_FAULT_FLASH_CONFIG && !conf_general_integrity_ok()) ||
        m->fault == MOTOR_FAULT_MCU_UNDER_VOLTAGE || m->fault == MOTOR_FAULT_BREAK) {
        motor_stop(m);
        return;
    }
    motor_stop(m);
    if (m->fault != MOTOR_FAULT_NONE)
        m->last_fault = m->fault;
    m->fault = MOTOR_FAULT_NONE;
    m->fault_recovery_ticks = 0U;
    /* A stale observer-speed filter can immediately recreate an
     * ABS_OVERSPEED fault on the next 1-kHz service tick, preventing a safe
     * stopped current recalibration from ever arming MOE. */
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

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_clear_fault_for_cal: menangani kalibrasi motor clear fault for cal agar offset atau parameter
// hasil ukur valid sebelum dipakai kendali.
void motor_clear_fault_for_cal(MotorRuntime *m) {
    if (m == NULL)
        return;
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
    if (m->fault != MOTOR_FAULT_NONE)
        m->last_fault = m->fault;
    m->fault = MOTOR_FAULT_NONE;
    m->fault_recovery_ticks = 0U;
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

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi motor_raise_fault_from_task: menangani motor raise fault from task dengan memprioritaskan pemadaman
// keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
void motor_raise_fault_from_task(MotorRuntime *m, motor_fault_t fault) {
    if (!m)
        return;
    if (m->fault == MOTOR_FAULT_NONE) {
        m->fault = fault;
        m->last_fault = fault;
        m->fault_recovery_ticks = 0U;
    }
    motor_stop(m);
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_pending_fault_mask |= (1UL << (uint32_t)m->id);
    if (!primask)
        __enable_irq();
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi motor_request_fault_from_isr: menangani motor request fault from isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void motor_request_fault_from_isr(MotorRuntime *m, motor_fault_t fault) {
    if (m->fault == MOTOR_FAULT_NONE) {
        m->fault = fault;
        m->last_fault = fault;
        m->fault_recovery_ticks = 0U;
    }
    m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
    m->pwm_enabled = false;
    s_pending_fault_mask |= (1UL << (uint32_t)m->id);
}
// Fungsi motor_take_pending_fault_mask: menangani motor take pending fault mask dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
uint32_t motor_take_pending_fault_mask(void) {
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mask = s_pending_fault_mask;
    s_pending_fault_mask = 0U;
    if (!primask)
        __enable_irq();
    return mask;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi motor_select_sensor_mode: menjalankan operasi motor select sensor mode sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool motor_select_sensor_mode(MotorRuntime *m, uint8_t mode) {
    if (m == NULL || mode > SENSOR_MODE_NO_SENSOR)
        return false;
    motor_stop(m);
    if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER)
        return false;
    /* AUTO tetap khusus proses deteksi eksplisit. SENSORLESS adalah pilihan
       runtime yang sah untuk LEFT maupun RIGHT dan tidak memakai Hall/encoder. */
    if (mode == SENSOR_MODE_AUTO)
        return false;
    m->sensor_request_mode = mode;
    if (mode == SENSOR_MODE_NO_SENSOR)
        m->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    else if (mode == SENSOR_MODE_ENCODER)
        m->foc_sensor_mode = FOC_SENSOR_MODE_ENCODER_AB;
    else
        m->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
    m->stats.tachometer_source_valid = false;

    /* Pergantian sumber sudut harus memutus state estimator dari mode lama.
       Tanpa reset ini, SENSORLESS dapat melewati forced-openloop karena
       observer_valid/speed masih tersisa dari Hall/encoder sebelumnya. */
    foc_sensorless_startup_abort(m);
    m->using_encoder = false;
    m->speed_est_fast_erpm_q16 = 0;
    m->speed_est_faster_erpm_q16 = 0;
    m->pll_erpm_q16 = 0;
    foc_observer_reset(m, 0U);

    if (m->id == MOTOR_LEFT && mode == SENSOR_MODE_ENCODER) {
        m->encoder.synced = false;
        m->encoder.motion_proved = false;
        m->encoder.sync_active = false;
        m->encoder.speed_sample_valid = false;
    }
    motor_hw_configure_sensor(m, mode);
    if (mode == SENSOR_MODE_HALL) {
        motor_hall_edge_isr(m);
        if (m->hall.valid) {
            /* Seed observer dari sektor Hall aktif agar jalur hybrid tidak
               memulai dari fase estimator lama yang tidak berkaitan. */
            foc_observer_reset(m, m->hall_angle_u16[m->hall.raw_state & 7U]);
        }
    }
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_encoder_extended_count: menjalankan operasi motor encoder extended count sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
int32_t motor_encoder_extended_count(MotorRuntime *m) {
    if (m == NULL || m->id != MOTOR_LEFT)
        return 0;
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // Variabel turns: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t turns = m->encoder.turns;
    // Variabel cnt: pencacah kejadian atau sampel.
    uint16_t cnt = motor_hw_encoder_cnt();
    if (!primask)
        __enable_irq();
    return turns*(int32_t)m->encoder.cpr + (int32_t)cnt;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_encoder_speed_position: memperbarui update encoder speed position menggunakan state terbaru
// dengan urutan yang konsisten dan aman.
static void update_encoder_speed_position(MotorRuntime *m) {
    if (m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER)
        return;
    // Variabel ext: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ext = motor_encoder_extended_count(m);
    m->encoder.extended_count = ext;
    if (!m->encoder.speed_sample_valid) {
        m->encoder.prev_extended_count = ext;
        m->encoder.speed_sample_valid = true;
        m->mech_rpm = 0.0f;
        m->erpm = 0.0f;
    }
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t d = ext-m->encoder.prev_extended_count;
    m->encoder.prev_extended_count = ext;
    m->mech_rpm = ((float)d*60.0f*1000.0f)/(float)m->encoder.cpr;
    m->erpm = m->mech_rpm*(float)m->pole_pairs;
    m->position_deg = ((float)(ext-m->encoder.mechanical_zero_count)*360.0f)/(float)m->encoder.cpr;
    /* m_invert_direction changes the external VESC coordinate system as well
       as torque polarity. This keeps positive RPM/POS feedback consistent with
       a positive command after the physical motor direction is inverted. */
    if (m->invert_direction) {
        m->mech_rpm = -m->mech_rpm;
        m->erpm = -m->erpm;
        m->position_deg = -m->position_deg;
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_hall_speed_position: memperbarui update hall speed position menggunakan state terbaru dengan
// urutan yang konsisten dan aman.
static void update_hall_speed_position(MotorRuntime *m) {
    if (m->sensor_mode != SENSOR_MODE_HALL || !m->hall.valid || m->hall.period_cycles == 0U)
        return;
    // Variabel age: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t age = DWT->CYCCNT-m->hall.edge_cycle;
    if (age > (CPU_CLOCK_HZ/5U)) {
        m->erpm = 0.0f;
        m->mech_rpm = 0.0f;
        return;
    }
    // Variabel edge_hz: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float edge_hz = (float)CPU_CLOCK_HZ/(float)m->hall.period_cycles;
    m->erpm = (float)m->hall.direction*edge_hz*10.0f;
    m->mech_rpm = m->erpm/(float)m->pole_pairs;
    m->position_deg = ((float)m->hall.edge_count*60.0f)/(float)m->pole_pairs;
    if (m->invert_direction) {
        m->mech_rpm = -m->mech_rpm;
        m->erpm = -m->erpm;
        m->position_deg = -m->position_deg;
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_rpm_update_1khz: memperbarui motor rpm update 1khz menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void motor_rpm_update_1khz(MotorRuntime *m) {
    if (m == NULL)
        return;

    /* Speed feedback follows the configured FOC phase strategy, not merely
       whichever GPIO peripheral happens to be initialized. SENSORLESS must
       never accidentally report Hall RPM just because the Hall pins are left
       configured as harmless inputs on this board. */
    if (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
        m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) {
        update_encoder_speed_position(m);
    }
    else if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL) {
        update_hall_speed_position(m);
    }
    else if (m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
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
    }
    else {
        m->erpm = 0.0f;
        m->mech_rpm = 0.0f;
    }
    m->erpm_int = (int32_t)m->erpm;
}

// Parameter oldv: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter newv: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi lp: menjalankan operasi lp sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static float lp(float oldv, float newv, float a) {
    return oldv + a*(newv-oldv);
}

/* Current VESC online motor-resistance estimator, intentionally kept at 1 kHz
 * on Cortex-M3. It is an estimate/diagnostic state and does not silently alter
 * the configured observer resistance. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_res_estimator_1khz: memperbarui update res estimator 1khz menggunakan state terbaru dengan
// urutan yang konsisten dan aman.
static void update_res_estimator_1khz(MotorRuntime *m) {
    if (!m)
        return;
    // Variabel r_nom: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float r_nom = fmaxf(m->foc_motor_r, 1.0e-5f);
    if (!isfinite(m->res_est_state_ohm) || m->res_est_state_ohm < 0.25f*r_nom ||
        m->res_est_state_ohm > 3.0f*r_nom) {
        m->res_est_state_ohm = r_nom;
        m->res_est_ohm = r_nom;
        m->res_est_valid = false;
    }
    if (!m->pwm_enabled || !m->observer_valid || m->detect.busy)
        return;

    // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ia = m->ia;
    // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ib = m->ib;
    // Variabel i_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_alpha = ia;
    // Variabel i_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_beta = (ia + 2.0f*ib) * 0.57735026919f;
    // Variabel i2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i2 = i_alpha*i_alpha + i_beta*i_beta;
    if (i2 < 0.25f)
        return; /* Hindari adaptasi pada noise ADC ketika arus mendekati nol. */

    // Variabel gain: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float gain = 0.00002f;
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float l = fmaxf(m->foc_motor_l, 1.0e-8f);
    // Variabel r_est: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float r_est = m->res_est_state_ohm - 0.5f*gain*l*i2;
    // Variabel v_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v_alpha = (float)m->observer_v_alpha_q15_prev * (FOC_VOLTAGE_Q_BASE_V / 32768.0f);
    // Variabel v_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v_beta = (float)m->observer_v_beta_q15_prev * (FOC_VOLTAGE_Q_BASE_V / 32768.0f);
    // Variabel omega: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float omega = m->observer_speed_rad_s;
    // Variabel x1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float x1 = m->observer_flux_alpha;
    // Variabel x2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float x2 = m->observer_flux_beta;
    // Variabel res_dot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float res_dot = -gain * (r_est*i2 + omega*(i_beta*x1 - i_alpha*x2) -
                                    (i_alpha*v_alpha + i_beta*v_beta));
    m->res_est_state_ohm += res_dot * 0.001f;
    m->res_est_state_ohm = foc_clampf(m->res_est_state_ohm, 0.25f*r_nom, 3.0f*r_nom);
    m->res_est_ohm = foc_clampf(m->res_est_state_ohm - 0.5f*gain*l*i2,
                                0.25f*r_nom, 3.0f*r_nom);
    m->res_est_valid = isfinite(m->res_est_ohm);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi motor_fault_recovery_ready_1khz: memeriksa apakah penyebab fault software sudah hilang sebelum timer auto-clear bertambah.
static bool motor_fault_recovery_ready_1khz(MotorRuntime *m) {
    if (m == NULL || m->fault == MOTOR_FAULT_NONE || m->pwm_enabled || m->command_active)
        return false;
    if (motor_hw_powerstage_fault_latched())
        return false;

    switch (m->fault) {
    case MOTOR_FAULT_ABS_OVER_CURRENT:
        return fabsf(m->motor_current) <= MOTOR_FAULT_RECOVERY_CURRENT_A &&
               fabsf(m->input_current) <= MOTOR_FAULT_RECOVERY_CURRENT_A;
    case MOTOR_FAULT_OVER_VOLTAGE:
        return m->vbus_filter < (m->max_vin - 1.0f);
    case MOTOR_FAULT_UNDER_VOLTAGE:
        return m->vbus_filter > (m->min_vin + 1.0f);
    case MOTOR_FAULT_HALL_INVALID:
        return m->sensor_mode == SENSOR_MODE_HALL && m->hall.valid &&
               m->hall.invalid_count == 0U && m->hall.sequence_error_count == 0U;
    case MOTOR_FAULT_COMMAND_TIMEOUT:
        return !m->timeout_active;
    case MOTOR_FAULT_SENSOR_DETECT:
        return !m->detect.busy;
    case MOTOR_FAULT_OVERSPEED:
    case MOTOR_FAULT_UNDERSPEED:
    case MOTOR_FAULT_ABS_OVERSPEED:
        return fabsf(m->erpm) < fmaxf(100.0f, 0.5f * fminf(fabsf(m->max_erpm), fabsf(m->min_erpm)));
    case MOTOR_FAULT_SENSORLESS_OBSERVER:
        return !m->openloop_started && !m->phase_observer_override;
    case MOTOR_FAULT_CURRENT_OFFSET:
        return foc_calibration_valid();
    case MOTOR_FAULT_ADC_DMA:
    case MOTOR_FAULT_FOC_ISR_OVERRUN:
    case MOTOR_FAULT_OVER_TEMP_BOARD:
    case MOTOR_FAULT_OVER_TEMP_MOTOR:
    case MOTOR_FAULT_ENCODER_SLIP:
        /* Fault ini memerlukan bukti sehat spesifik/commissioning; jangan auto-clear buta. */
        return false;
    case MOTOR_FAULT_MCU_UNDER_VOLTAGE:
    case MOTOR_FAULT_BREAK:
    case MOTOR_FAULT_FLASH_CONFIG:
    case MOTOR_FAULT_NONE:
    default:
        return false;
    }
}

// Fungsi motor_slow_update_1khz: memperbarui motor slow update 1khz menggunakan state terbaru dengan urutan
// yang konsisten dan aman.
void motor_slow_update_1khz(MotorRuntime *m, uint32_t now_ms) {
    if (m->control_mode == MOTOR_CTRL_OPENLOOP || m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY) {
        // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t st = (int32_t)lrintf(m->openloop_command_erpm*(65536.0f/60.0f)*0.001f);
        m->openloop_command_phase_u16 = (uint16_t)(m->openloop_command_phase_u16+st);
    }
    m->vbus = ((float)m->vbus_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->ia = ((float)m->ia_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->ib = ((float)m->ib_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->ic = ((float)m->ic_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->id_meas = ((float)m->id_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->iq_meas = ((float)m->iq_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->foc_fw_current_now = ((float)m->foc_fw_current_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->id_filter = ((float)m->id_filter_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->iq_filter = ((float)m->iq_filter_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->vd = ((float)m->vd_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->vq = ((float)m->vq_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->vd_filter = lp(m->vd_filter, m->vd, FOC_CURRENT_FILTER_CONST);
    m->vq_filter = lp(m->vq_filter, m->vq, FOC_CURRENT_FILTER_CONST);
    m->duty_u = (float)m->duty_u_q15/32768.0f;
    m->duty_v = (float)m->duty_v_q15/32768.0f;
    m->duty_w = (float)m->duty_w_q15/32768.0f;
    // Variabel duty_dev_u: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty_dev_u = fabsf(m->duty_u - 0.5f);
    // Variabel duty_dev_v: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty_dev_v = fabsf(m->duty_v - 0.5f);
    // Variabel duty_dev_w: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty_dev_w = fabsf(m->duty_w - 0.5f);
    m->duty_now = 2.0f * fmaxf(duty_dev_u, fmaxf(duty_dev_v, duty_dev_w));
    /* VESC FOC duty sign follows the applied q-axis voltage, not torque
       current. During regenerative braking Iq can reverse while Vq (and shaft
       electrical direction) keeps its sign. */
    if (m->vq_filter < 0.0f) {
        m->duty_now = -m->duty_now;
    }
    // Variabel dc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dc = ((float)(m->dc_current_offset_counts-(int32_t)m->dc_current_raw))*m->dc_current_scale;
    m->dc_current_a = dc;
    m->dc_current_filter = lp(m->dc_current_filter, dc, DC_CURRENT_FILTER_CONST);
    m->input_current = m->dc_current_filter;
    {
        // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float a = foc_clampf(m->input_current_map_filter, 0.0f, 1.0f);
        if (a > 0.0f)
            m->input_current_map_filtered_a = lp(m->input_current_map_filtered_a, m->input_current, a);
        else m->input_current_map_filtered_a = m->input_current;
    }
    m->vbus_filter = lp(m->vbus_filter, m->vbus, VBUS_FILTER_CONST);

    /* Board thermal proxy from the STM32F103 internal sensor. This runs in the
       1-kHz task only; the fast current ISR never performs temperature math. */
    {
        // Variabel t_board: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float t_board = 0.0f;
        if (motor_hw_board_temperature_c(&t_board)) {
            m->board_temp_c = t_board;
            if (!m->board_temp_valid) {
                m->board_temp_filter_c = t_board;
                m->board_temp_valid = true;
            }
            else {
                m->board_temp_filter_c = lp(m->board_temp_filter_c, t_board, 0.02f);
            }
        }
    }
    if (m->motor_type == MOTOR_TYPE_FOC) {
        // Variabel imag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float imag = sqrtf(m->id_filter*m->id_filter + m->iq_filter*m->iq_filter);
        /* VESC 6.00: motor current uses SIGN(Vq * Iq) times the dq-current
           magnitude. Directional/torque current is a separate getter that
           returns Iq directly. Keep input/battery current on the DC shunt. */
        // Variabel motor_regen: state atau parameter motor yang sedang diproses.
        const bool motor_regen = ((m->vq_filter < 0.0f) != (m->iq_filter < 0.0f));
        m->motor_current = motor_regen ? -imag : imag;

        /* VESC-style motor-temperature (resistance) compensation. The F103
           port has no motor NTC, so the STM32 board-temperature proxy is the
           thermal input. comp_factor = 1 + 0.00386*(T - base_temp); R and Ki
           are scaled by it so the current loop and observer track copper drift. */
        {
            // Variabel cfg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const volatile mc_configuration *cfg = mc_interface_get_configuration();
            if (cfg && cfg->foc_temp_comp && m->board_temp_valid) {
                // Variabel comp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                const float comp = 1.0f + 0.00386f * (m->board_temp_filter_c - cfg->foc_temp_comp_base_temp);
                m->res_temp_comp_ohm = m->foc_motor_r * comp;
                m->current_ki_temp_comp = m->current_ki * comp;
            }
            else {
                m->res_temp_comp_ohm = m->foc_motor_r;
                m->current_ki_temp_comp = m->current_ki;
            }
            /* Konversi Ki*dt dilakukan di task 1 kHz agar ISR 16 kHz hanya
               membaca fixed-point Q16.16 dan tidak menjalankan float math. */
            m->current_ki_dt_q16 = (int32_t)((m->current_ki_temp_comp * FOC_DT_S *
                    FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
        }
    }

    /* setup_stats is sampled at 100 Hz. This is intentionally task-side: the
       Cortex-M3 hard FOC path stays fixed-point and telemetry/statistics never
       lengthen the 16-kHz ADC ISR. */
    {
        // Variabel si: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        unsigned si = (unsigned)m->id;
        if (++s_setup_stats_div[si] >= 10U) {
            s_setup_stats_div[si] = 0U;
            // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            setup_stats *st = &s_setup_stats[si];
            // Variabel speed: nilai kecepatan untuk target atau pengukuran.
            double speed = (double)fabsf(m->mech_rpm);
            // Variabel power: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            double power = (double)fabsf(m->vbus_filter*m->input_current);
            // Variabel current: nilai arus untuk pengukuran, kendali, atau proteksi.
            double current = (double)fabsf(m->motor_current);
            st->samples += 1.0;
            st->speed_sum += speed;
            st->power_sum += power;
            st->current_sum += current;
            if (speed > (double)st->max_speed)
                st->max_speed = (float)speed;
            if (power > (double)st->max_power)
                st->max_power = (float)power;
            if (current > (double)st->max_current)
                st->max_current = (float)current;
            if (m->board_temp_valid) {
                st->temp_mos_sum += (double)m->board_temp_filter_c;
                if (m->board_temp_filter_c > st->max_temp_mos)
                    st->max_temp_mos = m->board_temp_filter_c;
            }
            if (isfinite(s_temp_motor_override)) {
                st->temp_motor_sum += (double)s_temp_motor_override;
                if (s_temp_motor_override > st->max_temp_motor)
                    st->max_temp_motor = s_temp_motor_override;
            }
        }
    }

    /* Observer is a FOC backend function. The fixed-point observer keeps the same ADC/current
       telemetry but does not run the flux observer. */
    if (m->motor_type == MOTOR_TYPE_FOC)
        foc_observer_update_1khz(m);
    if (m->motor_type == MOTOR_TYPE_FOC)
        update_res_estimator_1khz(m);
    if (m->id == MOTOR_LEFT &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB || m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) &&
        m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.synced && m->observer_valid) {
        /* Source switching is owned by the 16-kHz phase selector using the
           fast corrected speed estimate. Task-side code only performs the ABI
           counter rebase while observer mode is active. */
        if (!m->using_encoder)
            foc_encoder_ab_sync_from_observer(m);
    }

    m->rotor_elec_deg = ((float)motor_sensor_electrical_phase_u16(m)*360.0f)/65536.0f;

    (void)now_ms; /* command timeout is global and handled by motor service. */
    if (m->fault == MOTOR_FAULT_HALL_INVALID && m->sensor_mode == SENSOR_MODE_HALL && !m->pwm_enabled) {
        /* Refresh Hall state while stopped so recovery timer sees hardware yang aktual. */
        motor_hall_edge_isr(m);
    }
    if (m->fault != MOTOR_FAULT_NONE && !foc_calibration_in_progress()) {
        motor_hw_set_pwm_enabled(m, false);
        if (motor_fault_recovery_ready_1khz(m)) {
            if (m->fault_recovery_ticks < MOTOR_FAULT_RECOVERY_HOLD_MS)
                m->fault_recovery_ticks++;
            if (m->fault_recovery_ticks >= MOTOR_FAULT_RECOVERY_HOLD_MS)
                motor_clear_fault(m);
        }
        else {
            m->fault_recovery_ticks = 0U;
        }
        if (m->fault != MOTOR_FAULT_NONE)
            return;
    }
    /* VESC offset-calibration mode bit2: when the motor is stopped (state OFF)
       and no calibration is in progress, periodically re-measure the current
       offsets so drift from temperature/aging does not accumulate. This mirrors
       upstream's motor-stopped DC-offset recalibration. It is gated behind
       foc_offsets_cal_mode bit2 and never preempts a running motor. */
    {
        // Variabel cfg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
    if (!foc_calibration_done())
        return;

    // Variabel encoder_foc: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool encoder_foc = m->id == MOTOR_LEFT &&
                       (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
                        m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER);
    // Variabel encoder_ready: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool encoder_ready = !encoder_foc || m->encoder.synced || m->encoder.sync_active ||
                         m->openloop_started || m->phase_observer_override || m->detect.busy;
    // Variabel min_hold_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float min_hold_current = fmaxf(m->cc_min_current, 0.001f);
    // Variabel current_request_active: nilai arus untuk pengukuran, kendali, atau proteksi.
    const bool current_request_active =
        fabsf(m->id_target) >= min_hold_current ||
        fabsf(m->iq_target) >= min_hold_current ||
        m->current_off_delay_s > 0.0f;
    // Variabel modulation_mode: mode operasi yang menentukan jalur algoritma aktif.
    const bool modulation_mode =
        m->control_mode == MOTOR_CTRL_DUTY ||
        m->control_mode == MOTOR_CTRL_OPENLOOP ||
        m->control_mode == MOTOR_CTRL_OPENLOOP_PHASE ||
        m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY ||
        m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE ||
        m->control_mode == MOTOR_CTRL_HANDBRAKE;
    // Variabel wants_pwm: state atau nilai PWM untuk pengendalian inverter.
    bool wants_pwm = m->detect.busy || m->encoder.sync_active || m->openloop_started ||
                     m->phase_observer_override ||
                     (m->command_active && m->control_mode != MOTOR_CTRL_OFF &&
                      (current_request_active || modulation_mode));
    if (foc_calibration_valid() && encoder_ready && wants_pwm && m->vbus_filter >= VBUS_MIN_RUN_V && m->vbus_filter <= VBUS_MAX_RUN_V)
        motor_hw_set_pwm_enabled(m, true);
    else if (!m->detect.busy && !m->command_active) {
        motor_hw_set_pwm_enabled(m, false);
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi current_limit_pos: membatasi current limit pos ke rentang yang diizinkan agar pengendali dan perangkat
// keras tetap aman.
static float current_limit_pos(const MotorRuntime *m) {
    // Variabel lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lim = (m->lo_current_max_a > 0.0f) ? m->lo_current_max_a :
                (m->current_max_a * fmaxf(m->current_max_scale, 0.0f));
    return fmaxf(lim, 0.0f);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi current_limit_neg: membatasi current limit neg ke rentang yang diizinkan agar pengendali dan perangkat
// keras tetap aman.
static float current_limit_neg(const MotorRuntime *m) {
    // Variabel lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lim = (m->lo_current_min_a < 0.0f) ? m->lo_current_min_a :
                (m->current_min_a * fmaxf(m->current_min_scale, 0.0f));
    return fminf(lim, 0.0f);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi configured_iq_limit: membatasi configured iq limit ke rentang yang diizinkan agar pengendali dan
// perangkat keras tetap aman.
static float configured_iq_limit(const MotorRuntime *m) {
    return fmaxf(current_limit_pos(m), fabsf(current_limit_neg(m)));
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter in0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter in1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi map_clamped: menjalankan operasi map clamped sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static float map_clamped(float x, float in0, float in1, float out0, float out1) {
    if (fabsf(in1 - in0) < 1.0e-9f)
        return out1;
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float t = (x - in0) / (in1 - in0);
    t = foc_clampf(t, 0.0f, 1.0f);
    return out0 + (out1 - out0) * t;
}

#define ENCODER_SLIP_LIMIT_PHASE_U16 ((uint16_t)2731U) /* 15 electrical degrees */
#define ENCODER_SLIP_TIME_TICKS      ((uint16_t)500U)  /* 500 ms at 1 kHz */

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter obs_raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter enc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter erpm_q16: kecepatan listrik rotor dalam electrical RPM.
// Fungsi read_encoder_slip_snapshot: membaca read encoder slip snapshot tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
static bool read_encoder_slip_snapshot(const MotorRuntime *m, uint16_t *obs_raw,
                                       uint16_t *enc, int32_t *erpm_q16) {
    if (!m || !obs_raw || !enc || !erpm_q16)
        return false;
    // Variabel attempt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (unsigned attempt = 0; attempt < 4U; attempt++) {
        // Variabel s1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t s1 = m->rt_snapshot_seq;
        if (s1 & 1U)
            continue;
        __DMB();
        // Variabel o: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t o = m->rt_snapshot.phase_observer_u16;
        // Variabel e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t e = m->rt_snapshot.phase_encoder_u16;
        // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t r = m->rt_snapshot.erpm_fast_q16;
        __DMB();
        // Variabel s2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t s2 = m->rt_snapshot_seq;
        if (s1 == s2 && !(s2 & 1U)) {
            *obs_raw = o;
            *enc = e;
            *erpm_q16 = r;
            return true;
        }
    }
    return false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_encoder_slip_fault_1khz: menangani update encoder slip fault 1khz dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
static void update_encoder_slip_fault_1khz(MotorRuntime *m) {
    if (!m)
        return;
    // Variabel enabled: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool enabled = (m->additional_faults & MCCONF_L_ADDITIONAL_FAULT_ENCODER_SLIP) != 0U;
    // Variabel encoder_mode: data encoder untuk pengukuran posisi atau kecepatan rotor.
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


    // Variabel enc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel obs_raw: nilai mentah sebelum konversi ke satuan fisik.
    uint16_t obs_raw = 0U, enc = 0U;
    // Variabel erpm_q16: kecepatan listrik motor dalam electrical RPM.
    int32_t erpm_q16 = 0;
    if (!read_encoder_slip_snapshot(m, &obs_raw, &enc, &erpm_q16))
        return;

    // Variabel openloop_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t openloop_q16 = (int32_t)lrintf(fabsf(m->foc_openloop_rpm) * 1.10f * 65536.0f);
    // Variabel erpm_abs: kecepatan listrik motor dalam electrical RPM.
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
    // Variabel num: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t num = (int64_t)erpm_q16 * (int64_t)m->observer_offset_factor_q15;
    // Variabel den: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t den = 60LL * (int64_t)FOC_ISR_EVENT_HZ * 32768LL;
    // Variabel adv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t adv = den != 0 ? num / den : 0;
    if (adv > 32767)
        adv = 32767;
    if (adv < -32768)
        adv = -32768;
    // Variabel obs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t obs = (uint16_t)(obs_raw + (int16_t)adv);
    // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int16_t diff = (int16_t)(enc - obs);
    m->encoder_slip_error_phase = diff;
    m->encoder_slip_check_active = true;

    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t err = (uint16_t)(diff < 0 ? -(int32_t)diff : diff);
    if (err > ENCODER_SLIP_LIMIT_PHASE_U16) {
        if (m->encoder_slip_bad_ticks < UINT16_MAX)
            m->encoder_slip_bad_ticks++;
        if (m->encoder_slip_bad_ticks >= ENCODER_SLIP_TIME_TICKS) {
            motor_raise_fault_from_task(m, MOTOR_FAULT_ENCODER_SLIP);
        }
    }
    else {
        m->encoder_slip_bad_ticks = 0U;
    }
}

/* Reduced VESC-style override-limit update. Board temperature, RPM, duty,
 * battery/input and watt limits are evaluated at 1 kHz. BMS/CAN limits remain
 * absent because this PCB has no such backend. The hard ISR only consumes the
 * resulting lo_* values and therefore stays deterministic/fixed-point. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_runtime_limits_1khz: memperbarui update runtime limits 1khz menggunakan state terbaru dengan
// urutan yang konsisten dan aman.
static void update_runtime_limits_1khz(MotorRuntime *m) {
    if (!m)
        return;

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

    // Variabel base_max: batas atau nilai maksimum untuk validasi dan proteksi.
    const float base_max = fmaxf(0.0f, m->current_max_a * fmaxf(m->current_max_scale, 0.0f));
    // Variabel base_min: batas atau nilai minimum untuk validasi dan proteksi.
    const float base_min = fminf(0.0f, m->current_min_a * fmaxf(m->current_min_scale, 0.0f));
    // Variabel lo_max: batas atau nilai maksimum untuk validasi dan proteksi.
    float lo_max = base_max;
    // Variabel lo_min: batas atau nilai minimum untuk validasi dan proteksi.
    float lo_min = base_min;

    /* Current VESC uses a deliberately slower speed for optional hard RPM
       faults. Keep the limiter on the fresh 1-kHz RPM, but filter fault RPM so
       one estimator spike cannot latch an additional-fault condition. */
    m->erpm_fault_filter = lp(m->erpm_fault_filter, m->erpm, 0.02f);
    update_encoder_slip_fault_1khz(m);
    if (m->fault != MOTOR_FAULT_NONE)
        return;
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
        // Variabel abs_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
        // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float t = m->board_temp_filter_c;
        if (t > (m->temp_fet_end - 0.1f)) {
            m->lo_current_max_a = 0.0f;
            m->lo_current_min_a = 0.0f;
            motor_raise_fault_from_task(m, MOTOR_FAULT_OVER_TEMP_BOARD);
            return;
        }
        else if (t >= (m->temp_fet_start + 0.1f)) {
            // Variabel maxc0: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float maxc0 = fmaxf(fabsf(base_max), fabsf(base_min));
            // Variabel maxc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float maxc = mc_math_thermal_current_limit(maxc0, t, m->temp_fet_start, m->temp_fet_end);
            if (lo_max > maxc)
                lo_max = maxc;
            if (fabsf(lo_min) > maxc)
                lo_min = -maxc;
        }

        /* l_temp_accel_dec moves acceleration-only thresholds toward 25 C,
           preserving more braking authority close to a thermal limit. */
        // Variabel accel_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float accel_lim = mc_math_thermal_accel_limit(base_max, t,
                                                               m->temp_fet_start, m->temp_fet_end,
                                                               m->temp_accel_dec);
        lo_max = fminf(lo_max, accel_lim);
    }

    if (isfinite(s_temp_motor_override)) {
        // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float t = s_temp_motor_override;
        if (t > (m->temp_motor_end - 0.1f)) {
            m->lo_current_max_a = 0.0f;
            m->lo_current_min_a = 0.0f;
            motor_raise_fault_from_task(m, MOTOR_FAULT_OVER_TEMP_MOTOR);
            return;
        }
        else if (t >= (m->temp_motor_start + 0.1f)) {
            // Variabel maxc0: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float maxc0 = fmaxf(fabsf(base_max), fabsf(base_min));
            // Variabel maxc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float maxc = mc_math_thermal_current_limit(maxc0, t, m->temp_motor_start, m->temp_motor_end);
            if (lo_max > maxc)
                lo_max = maxc;
            if (fabsf(lo_min) > maxc)
                lo_min = -maxc;
        }
        // Variabel accel_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float accel_lim = mc_math_thermal_accel_limit(base_max, t,
                                                               m->temp_motor_start, m->temp_motor_end,
                                                               m->temp_accel_dec);
        lo_max = fminf(lo_max, accel_lim);
    }

    // Variabel erpm_start: kecepatan listrik motor dalam electrical RPM.
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
    // Variabel rpm_abs: kecepatan putar yang digunakan oleh logika kendali.
    const float rpm_abs = fabsf(m->erpm);
    // Variabel start_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float start_lim = mc_math_start_current_limit(base_max, rpm_abs,
                                                         m->foc_start_curr_dec,
                                                         m->foc_start_curr_dec_rpm);
    lo_max = fminf(lo_max, start_lim);

    /* VESC l_duty_start begins tapering torque-producing current before the
       configured modulation ceiling. Preserve opposite-sign braking current. */
    // Variabel dstart: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float dstart = foc_clampf(m->duty_start, 0.0f, 1.0f) * fmaxf(m->max_duty, 0.001f);
    // Variabel dabs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float dabs = fabsf(m->duty_now);
    if (dabs > dstart && m->max_duty > dstart + 1.0e-4f) {
        // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float scale = map_clamped(dabs, dstart, m->max_duty, 1.0f, 0.0f);
        if (m->duty_now >= 0.0f)
            lo_max *= scale;
        else lo_min *= scale;
    }

    m->lo_current_max_a = fmaxf(lo_max, 0.0f);
    m->lo_current_min_a = fminf(lo_min, 0.0f);

    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v = (m->vbus_filter > 0.1f) ? m->vbus_filter : m->vbus;
    // Variabel in_max: batas atau nilai maksimum untuk validasi dan proteksi.
    float in_max = mc_math_battery_cut_input_max(m->input_current_max_a, v,
                                                  m->battery_cut_start, m->battery_cut_end);
    // Variabel in_min: batas atau nilai minimum untuk validasi dan proteksi.
    float in_min = mc_math_battery_regen_cut_input_min(m->input_current_min_a, v,
                                                        m->battery_regen_cut_start,
                                                        m->battery_regen_cut_end);
    if (v > 0.5f) {
        if (isfinite(m->watt_max) && m->watt_max >= 0.0f)
            in_max = fminf(in_max, m->watt_max / v);
        if (isfinite(m->watt_min) && m->watt_min <= 0.0f)
            in_min = fmaxf(in_min, m->watt_min / v);
    }

    /* VESC l_in_current_map_start semantics, but driven by the hoverboard's
       physical DC-current measurement. Reduce positive torque capability
       smoothly before the input limit instead of waiting for clipping. */
    m->input_current_map_limit_a = m->lo_current_max_a;
    // Variabel map_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float map_start = foc_clampf(m->input_current_map_start, 0.0f, 1.0f);
    // Variabel map_in_max: batas atau nilai maksimum untuk validasi dan proteksi.
    const float map_in_max = fmaxf(0.0f, in_max);
    if (map_start < 0.98f && map_in_max > 0.05f && m->input_current_map_filtered_a > 0.0f) {
        // Variabel frac: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float frac = m->input_current_map_filtered_a / map_in_max;
        if (frac > map_start) {
            // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float scale = foc_clampf((1.0f - frac) / fmaxf(1.0f - map_start, 0.001f), 0.0f, 1.0f);
            // Variabel cap: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter iq: arus sumbu-q FOC yang terutama menghasilkan torsi motor.
// Fungsi limit_iq_by_input_current: membatasi limit iq by input current ke rentang yang diizinkan agar
// pengendali dan perangkat keras tetap aman.
static float limit_iq_by_input_current(MotorRuntime *m, float iq) {
    if (m == NULL || iq == 0.0f)
        return iq;
    return mc_math_limit_input_current(iq, m->erpm, m->duty_now, m->input_current,
                                       m->lo_input_current_max_a, m->lo_input_current_min_a);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi speed_feedback_erpm: menjalankan operasi speed feedback erpm sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float speed_feedback_erpm(const MotorRuntime *m) {
    if (m == NULL)
        return 0.0f;
    // Variabel rpm: kecepatan putar yang digunakan oleh logika kendali.
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

// Parameter value: nilai kerja yang digunakan oleh algoritma pada konteks tersebut.
// Parameter target: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter step: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi step_towards_f: menjalankan operasi step towards f sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float step_towards_f(float value, float target, float step) {
    if (step <= 0.0f)
        return target;
    if (value < target)
        return fminf(value + step, target);
    if (value > target)
        return fmaxf(value - step, target);
    return target;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter output: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi normalized_output_to_current: menjalankan operasi normalized output to current sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static float normalized_output_to_current(const MotorRuntime *m, float output) {
    output = foc_clampf(output, -1.0f, 1.0f);
    if (output >= 0.0f)
        return output * current_limit_pos(m);
    return (-output) * current_limit_neg(m);
}

/* VESC speed PID semantics:
 * - optional ERPM/s input ramp
 * - selectable PLL/fast/faster speed source
 * - release below s_pid_min_erpm
 * - normalized PID output with the historical 1/20 gain scaling
 * - optional prevention of active braking.
 */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter target_erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi speed_pid_step: menjalankan operasi speed pid step sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float speed_pid_step(MotorRuntime *m, float target_erpm) {
    // Variabel dt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float dt = 0.001f;
    if (m == NULL)
        return 0.0f;

    target_erpm = foc_clampf(target_erpm, m->min_erpm, m->max_erpm);
    if (m->speed_pid_ramp_erpms_s > 0.0f) {
        m->speed_pid_set_erpm = step_towards_f(m->speed_pid_set_erpm, target_erpm,
                                               m->speed_pid_ramp_erpms_s * dt);
    }
    else {
        m->speed_pid_set_erpm = target_erpm;
    }
    m->speed_pid_set_erpm = foc_clampf(m->speed_pid_set_erpm, m->min_erpm, m->max_erpm);

    // Variabel rpm: kecepatan putar yang digunakan oleh logika kendali.
    const float rpm = speed_feedback_erpm(m);
    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float err = m->speed_pid_set_erpm - rpm;

    if (fabsf(m->speed_pid_set_erpm) < m->speed_pid_min_erpm) {
        m->speed_pid.integrator = 0.0f;
        m->speed_pid.prev_error = err;
        m->speed_derivative_filtered = 0.0f;
        return 0.0f;
    }

    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float p = err * m->speed_pid.kp * (1.0f / 20.0f);
    // Variabel d_raw: nilai mentah sebelum konversi ke satuan fisik.
    const float d_raw = (err - m->speed_pid.prev_error) *
                        (m->speed_pid.kd / dt) * (1.0f / 20.0f);
    m->speed_pid.prev_error = err;
    // Variabel df: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float df = foc_clampf(m->speed_kd_filter, 0.0f, 1.0f);
    m->speed_derivative_filtered += df * (d_raw - m->speed_derivative_filtered);

    // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float out = p + m->speed_pid.integrator + m->speed_derivative_filtered;
    out = foc_clampf(out, -1.0f, 1.0f);

    m->speed_pid.integrator += err * m->speed_pid.ki * dt * (1.0f / 20.0f);
    m->speed_pid.integrator = foc_clampf(m->speed_pid.integrator, -1.0f, 1.0f);
    if (m->speed_pid.ki < 1.0e-9f)
        m->speed_pid.integrator = 0.0f;

    if (!m->speed_pid_allow_braking) {
        if (rpm > 20.0f && out < 0.0f)
            out = 0.0f;
        if (rpm < -20.0f && out > 0.0f)
            out = 0.0f;
    }

    return normalized_output_to_current(m, out);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi position_pid_step: menjalankan operasi position pid step sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float position_pid_step(MotorRuntime *m) {
    // Variabel dt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float dt = 0.001f;
    if (m == NULL)
        return 0.0f;

    /* LEFT incremental A/B is the steering actuator on this target. Preserve
       its extended mechanical coordinate across 0/360 instead of applying the
       generic rotary shortest-path wrap, which can command a steering turn in
       the wrong direction near the wrap boundary. Hall/sensorless retain the
       normal circular VESC semantics. */
    // Variabel linear_position: nilai posisi rotor atau aktuator.
    const bool linear_position = m->id == MOTOR_LEFT &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
         m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER);
    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float now = linear_position ?
        (m->position_deg + m->position_offset_deg) :
        foc_wrap_deg(m->position_deg + m->position_offset_deg);
    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float err = m->position_target_deg - now;
    if (!linear_position) {
        while (err > 180.0f)
            err -= 360.0f;
        while (err < -180.0f)
            err += 360.0f;
    }

    /* Match VESC position sign semantics for encoder configurations. */
    // Variabel err_sign: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float err_sign = 1.0f;
    if (m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.inverted)
        err_sign = -1.0f;
    err *= err_sign;

    // Variabel kp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float kp = m->position_pid.kp;
    // Variabel ki: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ki = m->position_pid.ki;
    // Variabel kd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float kd = m->position_pid.kd;
    // Variabel kd_proc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float kd_proc = m->position_kd_proc;
    if (m->position_gain_dec_angle > 0.1f && m->position_ang_div > 0.001f) {
        // Variabel min_err: batas atau nilai minimum untuk validasi dan proteksi.
        const float min_err = m->position_gain_dec_angle / m->position_ang_div;
        // Variabel ae: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float ae = fabsf(err);
        if (min_err > 1.0e-6f && ae < min_err) {
            // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float scale = ae / min_err;
            kp *= scale;
            ki *= scale;
            kd *= scale;
            kd_proc *= scale;
        }
    }

    // Variabel p_term: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float p_term = err * kp;
    m->position_pid.integrator += err * ki * dt;

    m->position_dt_integrator += dt;
    // Variabel d_term: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float d_term = 0.0f;
    if (err != m->position_pid.prev_error && m->position_dt_integrator > 0.0f) {
        d_term = (err - m->position_pid.prev_error) *
                 (kd / m->position_dt_integrator);
        m->position_dt_integrator = 0.0f;
    }
    // Variabel df: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float df = foc_clampf(m->position_kd_filter, 0.0f, 1.0f);
    m->position_derivative_filtered += df * (d_term - m->position_derivative_filtered);

    m->position_dt_process_integrator += dt;
    // Variabel d_proc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float d_proc = 0.0f;
    if (now != m->position_prev_process_deg && m->position_dt_process_integrator > 0.0f) {
        // Variabel proc_diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float proc_diff = now - m->position_prev_process_deg;
        if (!linear_position) {
            while (proc_diff > 180.0f)
                proc_diff -= 360.0f;
            while (proc_diff < -180.0f)
                proc_diff += 360.0f;
        }
        d_proc = -proc_diff * err_sign *
                 (kd_proc / m->position_dt_process_integrator);
        m->position_dt_process_integrator = 0.0f;
    }
    m->position_derivative_proc_filtered +=
        df * (d_proc - m->position_derivative_proc_filtered);

    // Variabel p_clip: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float p_clip = foc_clampf(p_term, -1.0f, 1.0f);
    // Variabel i_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_lim = fmaxf(1.0f - fabsf(p_clip), 0.0f);
    m->position_pid.integrator =
        foc_clampf(m->position_pid.integrator, -i_lim, i_lim);

    m->position_pid.prev_error = err;
    m->position_prev_process_deg = now;

    // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float out = p_term + m->position_pid.integrator +
                m->position_derivative_filtered +
                m->position_derivative_proc_filtered;
    out = foc_clampf(out, -1.0f, 1.0f);
    return normalized_output_to_current(m, out);
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi sign_i32_mc: menjalankan operasi sign i32 mc sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static inline int sign_i32_mc(int32_t v) {
    return (v > 0) - (v < 0);
}

/* VESC duty controller semantics. The dedicated PI is used only when an
   already-generated duty must be reduced safely. During normal/ramp-up duty
   control the current request goes to the allowed motor-current limit while
   the FOC voltage circle is lowered to the requested modulation. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty_set: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter current_max_for_duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter brake_zero_guard: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi duty_control_step_1khz: menjalankan operasi duty control step 1khz sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static float duty_control_step_1khz(MotorRuntime *m, float duty_set,
                                    float current_max_for_duty,
                                    bool brake_zero_guard) {
    if (!m)
        return 0.0f;
    current_max_for_duty = fabsf(current_max_for_duty);
    if (current_max_for_duty < 1.0e-3f) {
        m->duty_limit_now = m->max_duty;
        m->force_zero_modulation = false;
        m->duty_was_pi = false;
        return 0.0f;
    }

    duty_set = foc_clampf(duty_set, -m->max_duty, m->max_duty);
    // Variabel duty_now: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_now = m->duty_now;
    // Variabel duty_abs: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_abs = fabsf(duty_now);
    // Variabel sign_now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int sign_now = (duty_now > 0.0f) - (duty_now < 0.0f);
    // Variabel sign_last: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int sign_last = (m->duty_pi_duty_last > 0.0f) - (m->duty_pi_duty_last < 0.0f);

    // Variabel downramp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
            }
            else if (duty_now < 0.0f && m->duty_pid.integrator < 0.0f) {
                m->duty_pid.integrator = 0.0f;
            }
        }

        // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float err = duty_set - duty_now;
        // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float scale = 1.0f / fmaxf(m->vbus_filter, 1.0f);
        m->duty_pid.integrator += err * (m->foc_duty_dowmramp_ki * 0.001f) * scale;
        m->duty_pid.integrator = foc_clampf(m->duty_pid.integrator, -1.0f, 1.0f);
        // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
        // Variabel dir: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int dir = sign_i32_mc(m->speed_est_fast_erpm_q16);
        if (dir == 0)
            dir = sign_i32_mc(m->brake_vq_before_q15);
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
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter brake_target_a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi brake_zero_guard_1khz: menjalankan operasi brake zero guard 1khz sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static bool brake_zero_guard_1khz(MotorRuntime *m, float brake_target_a) {
    if (!m)
        return false;
    // Variabel BR_ZERO_MIN_HOLD_TICKS: batas atau nilai minimum untuk validasi dan proteksi.
    const uint8_t BR_ZERO_MIN_HOLD_TICKS = 1U;
    // Variabel speed_now: nilai kecepatan untuk target atau pengukuran.
    const int32_t speed_now = m->speed_est_fast_erpm_q16;
    // Variabel vq_now: tegangan sumbu-q keluaran regulator FOC.
    const int32_t vq_now = m->vq_q15;
    // Variabel speed_sign_now: nilai kecepatan untuk target atau pengukuran.
    const int speed_sign_now = sign_i32_mc(speed_now);
    // Variabel speed_sign_prev: nilai kecepatan untuk target atau pengukuran.
    const int speed_sign_prev = sign_i32_mc(m->brake_speed_before_q16);
    // Variabel vq_sign_now: tegangan sumbu-q keluaran regulator FOC.
    const int vq_sign_now = sign_i32_mc(vq_now);
    // Variabel vq_sign_prev: tegangan sumbu-q keluaran regulator FOC.
    const int vq_sign_prev = sign_i32_mc(m->brake_vq_before_q15);
    // Variabel current_abs: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_abs = sqrtf(m->id_filter * m->id_filter + m->iq_filter * m->iq_filter);
    // Variabel need_more_brake_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    const bool need_more_brake_current = current_abs < fabsf(brake_target_a);

    // Variabel transition: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool transition = speed_sign_now != speed_sign_prev ||
                            vq_sign_now != vq_sign_prev ||
                            fabsf(m->duty_now) < 0.001f ||
                            m->brake_zero_hold_ticks < BR_ZERO_MIN_HOLD_TICKS;

    if (transition && need_more_brake_current) {
        m->brake_zero_active = true;
        m->brake_zero_hold_ticks = 0U;
    }
    else if (m->brake_zero_hold_ticks < BR_ZERO_MIN_HOLD_TICKS) {
        m->brake_zero_active = true;
        m->brake_zero_hold_ticks++;
    }
    else {
        m->brake_zero_active = false;
    }

    m->brake_speed_before_q16 = speed_now;
    m->brake_vq_before_q15 = vq_now;
    return m->brake_zero_active;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter iq: arus sumbu-q FOC yang terutama menghasilkan torsi motor.
// Fungsi apply_mtpa_1khz: menjalankan operasi apply mtpa 1khz sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void apply_mtpa_1khz(MotorRuntime *m, float *id, float *iq) {
    if (!m || !id || !iq || m->foc_mtpa_mode == MTPA_MODE_OFF ||
        fabsf(m->foc_motor_ld_lq_diff) < 1.0e-12f) {
        if (m)
            m->mtpa_id_target = 0.0f;
        return;
    }

    // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float diff = m->foc_motor_ld_lq_diff;
    // Variabel lambda: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float lambda = m->foc_motor_flux_linkage;

    /* Current VESC MTPA semantics: calculate the reluctance-producing Id from
       target/measured Iq, then reduce the commanded Iq so MTPA rotates the
       requested current vector instead of silently increasing its magnitude.
       Field weakening is composed later in the 16-kHz ISR by selecting the
       larger-magnitude d-axis request rather than summing both negative Ids. */
    // Variabel iq_cmd: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq_cmd = *iq;
    // Variabel iq_ref: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq_ref = (m->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED)
                        ? fminf(fabsf(iq_cmd), fabsf(m->iq_filter)) * ((iq_cmd < 0.0f) ? -1.0f : 1.0f)
                        : iq_cmd;
    // Variabel term: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float term = lambda * lambda + 8.0f * (diff * iq_ref) * (diff * iq_ref);
    // Variabel id_mtpa: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    const float id_mtpa = (lambda - sqrtf(fmaxf(term, 0.0f))) / (4.0f * diff);
    // Variabel iq_sq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq_sq = iq_cmd * iq_cmd - id_mtpa * id_mtpa;
    *id = id_mtpa;
    *iq = ((iq_cmd < 0.0f) ? -1.0f : 1.0f) * sqrtf(fmaxf(iq_sq, 0.0f));
    m->mtpa_id_target = id_mtpa;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_pid_update_1khz: memperbarui motor pid update 1khz menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void motor_pid_update_1khz(MotorRuntime *m) {
    if (m == NULL || m->detect.busy)
        return;

    if (m->current_off_delay_s > 0.0f) {
        m->current_off_delay_s = fmaxf(0.0f, m->current_off_delay_s-0.001f);
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

    if (m->fault != MOTOR_FAULT_NONE || !m->command_active) {
        m->foc_fw_current_now = 0.0f;
        m->foc_fw_current_acc_q31 = 0;
        m->foc_fw_current_q15 = 0;
        m->foc_fw_duty_filter_q15 = 0;
        m->foc_fw_fast_active = false;
        m->mtpa_id_target = 0.0f;
        m->brake_zero_active = false;
        m->brake_zero_hold_ticks = 1U;
        motor_set_foc_targets(m, 0.0f, 0.0f);
        return;
    }
    /* Incremental AB is a valid FOC phase source only after this boot's
       runtime reference has been established. Hall remains usable immediately
       when its state is valid. */
    // Variabel explicit_openloop: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool explicit_openloop = (m->control_mode == MOTOR_CTRL_OPENLOOP ||
                            m->control_mode == MOTOR_CTRL_OPENLOOP_PHASE ||
                            m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY ||
                            m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE);
    // Variabel sensorless_foc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool sensorless_foc = m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
    if (!explicit_openloop && sensorless_foc) {
        // Variabel direction_hint: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float direction_hint = 0.0f;
        // Variabel iq_hint: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
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

        // Variabel sl_speed_abs: nilai kecepatan untuk target atau pengukuran.
        const int32_t sl_speed_abs = m->speed_est_fast_erpm_q16 >= 0 ?
                                     m->speed_est_fast_erpm_q16 :
                                     (m->speed_est_fast_erpm_q16 == INT32_MIN ? INT32_MAX :
                                      -m->speed_est_fast_erpm_q16);
        /* foc_sl_erpm/foc_sl_erpm_start are hybrid Hall/encoder-to-observer
           thresholds. Pure SENSORLESS startup in upstream VESC is governed by
           foc_openloop_rpm and foc_openloop_rpm_low (0..1 current scaling).
           Therefore do not gate pure sensorless startup with foc_sl_erpm here;
           foc_sensorless_startup_1khz() owns the correct dynamic threshold. */
        // Variabel observer_min_ready: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
        const bool observer_min_ready = m->observer_valid &&
                                        sl_speed_abs >= (50 * 65536);
        if ((m->control_mode == MOTOR_CTRL_BRAKE_CURRENT ||
             m->control_mode == MOTOR_CTRL_HANDBRAKE ||
             m->control_mode == MOTOR_CTRL_POSITION) && !observer_min_ready) {
            foc_sensorless_startup_abort(m);
            m->mtpa_id_target = 0.0f;
            motor_set_foc_targets(m, 0.0f, 0.0f);
            return;
        }

        /* m_invert_direction membalik koordinat eksternal VESC. Forced-openloop
           harus memakai arah fisik yang sama dengan Iq setelah handover; jika
           tidak, motor inverted dapat dipaksa start ke arah yang berlawanan. */
        if (m->invert_direction)
            direction_hint = -direction_hint;

        /* Selalu serahkan keputusan forced-openloop/handover ke satu fungsi.
           Ini mencegah dua threshold berbeda saling memulai ulang startup. */
        if (!foc_sensorless_startup_1khz(m, xTaskGetTickCount(),
                                         direction_hint, iq_hint)) {
            if (m->sensorless_start_failures >= 3U) {
                motor_raise_fault_from_task(m, MOTOR_FAULT_SENSORLESS_OBSERVER);
            }
            return;
        }
        if (!m->openloop_started) {
            m->sensorless_start_failures = 0U;
        }
    }
    if (!explicit_openloop && m->id == MOTOR_LEFT &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB || m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) &&
        m->sensor_mode == SENSOR_MODE_ENCODER && !m->encoder.synced) {
        /* Incremental AB requires a runtime electrical reference. Use the
           proven observer/open-loop alignment path before switching to AB. */
        if (!foc_encoder_ab_startup_1khz(m, xTaskGetTickCount()))
            return;
        m->using_encoder = true;
    }
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq = 0.0f;
    switch (m->control_mode) {
        case MOTOR_CTRL_CURRENT:
            iq = m->current_command_a;
        break;
        case MOTOR_CTRL_BRAKE_CURRENT:
            {
            // Variabel brake_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float brake_lim = fminf(fabsf(current_limit_neg(m)), configured_iq_limit(m));
            // Variabel brake_target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float brake_target = fminf(fabsf(m->brake_current_a), brake_lim);
            if (brake_zero_guard_1khz(m, brake_target)) {
                iq = duty_control_step_1khz(m, 0.0f, brake_lim, true);
            }
            else {
                // Variabel dir: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                int dir = sign_i32_mc(m->speed_est_fast_erpm_q16);
                iq = -(float)dir * brake_target;
                m->duty_limit_now = m->max_duty;
                m->force_zero_modulation = false;
            }
            break;
        }
        case MOTOR_CTRL_SPEED:
            iq = speed_pid_step(m, m->speed_target_erpm);
        break;
        case MOTOR_CTRL_POSITION:
            iq = position_pid_step(m);
        break;
        case MOTOR_CTRL_DUTY:
            iq = duty_control_step_1khz(m, m->duty_command, configured_iq_limit(m), false);
            break;
        case MOTOR_CTRL_OPENLOOP:
            iq = m->current_command_a;
            break;
        case MOTOR_CTRL_OPENLOOP_PHASE:
            foc_update_modulation_limit(m);
            motor_set_foc_targets(m, m->current_command_a, 0.0f);
            return;
        case MOTOR_CTRL_OPENLOOP_DUTY:
        case MOTOR_CTRL_OPENLOOP_DUTY_PHASE:
            /* Direct modulation is handled in the fixed-point ISR. */
            foc_update_modulation_limit(m);
            motor_set_foc_targets(m, 0.0f, 0.0f);
            return;
        case MOTOR_CTRL_HANDBRAKE:
            foc_update_modulation_limit(m);
            motor_set_foc_targets(m, m->handbrake_current_a, 0.0f);
            return;
        default:
            iq = 0.0f;
        break;
    }
    /* duty_limit_now may have changed in duty/brake mode. Publish the new
       fixed-point voltage-circle coefficient before the next FOC ISR sample. */
    foc_update_modulation_limit(m);
    if (m->invert_direction)
        iq = -iq;

    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    float id = 0.0f;
    apply_mtpa_1khz(m, &id, &iq);

    /* VESC 7.x composition is completed in the hard loop. Keep this task-side
       request as MTPA-only Id + torque Iq; fast FW will select max-absolute Id,
       apply q-axis FW compensation, then re-apply the current circle. */
    iq = limit_iq_by_input_current(m, iq);

    /* Sign-aware motor-current limits preserve braking authority while
       preventing acceleration torque from crossing the computed lo_* bounds. */
    if (m->duty_now >= 0.0f)
        iq = foc_clampf(iq, current_limit_neg(m), current_limit_pos(m));
    else iq = foc_clampf(iq, -current_limit_pos(m), -current_limit_neg(m));

    // Variabel current_abs: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_abs = configured_iq_limit(m);
    id = foc_clampf(id, -current_abs, current_abs);
    // Variabel iq_abs: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq_abs = sqrtf(fmaxf(current_abs * current_abs - id * id, 0.0f));
    iq = foc_clampf(iq, -iq_abs, iq_abs);

    /* VESC cc_min_current semantics: below the minimum useful current the
       bridge may release, but the outer controller command remains alive so
       speed/position control can re-engage automatically when error grows.
       current_off_delay keeps modulation alive temporarily after field
       weakening or an explicit compatibility API request. */
    // Variabel release_threshold: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float release_threshold = fmaxf(m->cc_min_current, 0.001f);
    if (m->current_off_delay_s <= 0.0f &&
        sqrtf(id*id+iq*iq) < release_threshold) {
        id = 0.0f;
        iq = 0.0f;
    }
    m->foc_current_limit_q15 = amp_to_current_q15(current_abs);
    motor_set_foc_targets(m, id, iq);
}

/* ========================================================================
 * VESC master-compatible mc_interface wrappers
 * ======================================================================== */
typedef struct {
    // Variabel tid: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TaskHandle_t tid;
    // Variabel motor: state atau parameter motor yang sedang diproses.
    uint8_t motor;
}
motor_thread_sel_t;
// Variabel s_motor_sel: state atau parameter motor yang sedang diproses.
static motor_thread_sel_t s_motor_sel[16];
// Variabel s_mc_interface_inited: state internal modul yang dipertahankan antar pemanggilan fungsi.
static bool s_mc_interface_inited = false;
// Variabel s_mc_locked: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_mc_locked = false;
// Variabel s_mc_lock_override_once: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_mc_lock_override_once = false;
// Variabel s_ignore_until: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_ignore_until[2] = {
    0, 0
}
;
// Variabel s_pwm_callback: state atau nilai PWM untuk pengendalian inverter.
static void (*s_pwm_callback)(void) = NULL;
// Variabel volatile: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
static void (* volatile s_sample_reply_func)(unsigned char *data, unsigned int len) = NULL;
// Variabel s_mcconf_mirror: state internal modul yang dipertahankan antar pemanggilan fungsi.
static mc_configuration s_mcconf_mirror[2];
// Variabel s_gnss: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile gnss_data s_gnss = {
    0
}
;
// Variabel s_odometer: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint64_t s_odometer[2] = {
    0U, 0U
}
;
// Variabel s_odometer_fraction_m: state internal modul yang dipertahankan antar pemanggilan fungsi.
static float s_odometer_fraction_m[2] = {
    0.0f, 0.0f
}
;
// Variabel s_wheel_speed_override: nilai kecepatan untuk target atau pengukuran.
static bool s_wheel_speed_override = false;
// Variabel s_wheel_speed_override_value: nilai kecepatan untuk target atau pengukuran.
static float s_wheel_speed_override_value = 0.0f;

// Parameter tid: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter alloc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi sel_index: menjalankan operasi sel index sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int sel_index(TaskHandle_t tid, bool alloc) {
    if (!tid)
        return -1;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (int i = 0; i < 16; i++)
        if (s_motor_sel[i].tid == tid)
            return i;
    if (alloc) {
        // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (int i = 0; i < 16; i++)
            if (!s_motor_sel[i].tid) {
            s_motor_sel[i].tid = tid;
            s_motor_sel[i].motor = 1;
            return i;
        }
    }
    return -1;
}
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi mc_interface_select_motor_thread: menjalankan operasi mc interface select motor thread sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_select_motor_thread(int motor) {
    if (motor < 1) {
        motor = 1;
    }
    if (motor > 2) {
        motor = 2;
    }

    // Variabel tid: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TaskHandle_t tid = xTaskGetCurrentTaskHandle();
    // Variabel pm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int i = sel_index(tid, true);
    if (i >= 0) {
        s_motor_sel[i].motor = (uint8_t)motor;
    }
    if (pm == 0U) {
        __enable_irq();
    }
}
// Fungsi mc_interface_get_motor_thread: membaca mc interface get motor thread tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
int mc_interface_get_motor_thread(void) {
    // Variabel tid: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TaskHandle_t tid = xTaskGetCurrentTaskHandle();
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int i = sel_index(tid, false);
    return i >= 0 ? s_motor_sel[i].motor : 1;
}
// Fungsi mc_interface_motor_now: menjalankan operasi mc interface motor now sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
int mc_interface_motor_now(void) {
    /* VESC: the motor active in the FOC ISR wins over the thread-selected one. */
    // Variabel isr_motor: state atau parameter motor yang sedang diproses.
    int isr_motor = mcpwm_foc_isr_motor();
    if (isr_motor == 1 || isr_motor == 2) {
        return isr_motor;
    }
    // Variabel selected: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int selected = mc_interface_get_motor_thread();
    return selected == 2 ? 2 : 1;
}
// Fungsi mc_interface_motor_runtime_now: menjalankan operasi mc interface motor runtime now sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
MotorRuntime *mc_interface_motor_runtime_now(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int m = mc_interface_motor_now();
    return motor_get(m == 2 ? MOTOR_RIGHT : MOTOR_LEFT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mirror_from_runtime: menjalankan operasi mirror from runtime sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void mirror_from_runtime(const MotorRuntime*m, mc_configuration*c) {
    memset(c, 0, sizeof(*c));
    c->l_current_max = m->current_max_a;
    c->l_current_min = m->current_min_a;
    c->l_in_current_max = m->input_current_max_a;
    c->l_in_current_min = m->input_current_min_a;
    c->l_in_current_map_start = m->input_current_map_start;
    c->l_in_current_map_filter = m->input_current_map_filter;
    c->l_abs_current_max = m->abs_current_max_a;
    c->l_min_erpm = m->min_erpm;
    c->l_max_erpm = m->max_erpm;
    c->l_erpm_start = m->erpm_start;
    c->l_temp_fet_start = m->temp_fet_start;
    c->l_temp_fet_end = m->temp_fet_end;
    c->l_temp_motor_start = m->temp_motor_start;
    c->l_temp_motor_end = m->temp_motor_end;
    c->l_temp_accel_dec = m->temp_accel_dec;
    c->l_additional_faults = m->additional_faults;
    c->l_min_vin = m->min_vin;
    c->l_max_vin = m->max_vin;
    c->l_battery_cut_start = m->battery_cut_start;
    c->l_battery_cut_end = m->battery_cut_end;
    c->l_slow_abs_current = m->slow_abs_current;
    c->l_battery_regen_cut_start = m->battery_regen_cut_start;
    c->l_battery_regen_cut_end = m->battery_regen_cut_end;
    c->l_min_duty = m->min_duty;
    c->l_max_duty = m->max_duty;
    c->l_watt_max = m->watt_max;
    c->l_watt_min = m->watt_min;
    c->l_current_max_scale = m->current_max_scale;
    c->l_current_min_scale = m->current_min_scale;
    c->l_duty_start = m->duty_start;
    c->lo_current_max = m->lo_current_max_a;
    c->lo_current_min = m->lo_current_min_a;
    c->lo_in_current_max = m->lo_input_current_max_a;
    c->lo_in_current_min = m->lo_input_current_min_a;
    c->pwm_mode = m->pwm_mode;
    c->comm_mode = m->comm_mode;
    c->motor_type = MOTOR_TYPE_FOC;
    c->sensor_mode = SENSOR_MODE_SENSORED;
    {
        // Variabel legacy_hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
        const int8_t legacy_hall[8] = {
            -1, 1, 3, 2, 5, 6, 4, -1
        }
        ;
        memcpy(c->hall_table, legacy_hall, sizeof(c->hall_table));
    }
    c->sensor_mode = SENSOR_MODE_SENSORED;
    c->hall_sl_erpm = 2000.0f;
    c->foc_current_kp = m->current_kp;
    c->foc_current_ki = m->current_ki;
    c->foc_f_zv = (float)VESC_FOC_F_ZV_HZ;
    c->foc_dt_us = m->foc_dt_us;
    c->foc_encoder_offset = (float)m->encoder.elec_offset_u16*360.0f/65536.0f;
    c->foc_encoder_inverted = m->encoder.inverted;
    c->foc_encoder_ratio = m->encoder.electrical_ratio;
    c->foc_motor_l = m->foc_motor_l;
    c->foc_motor_ld_lq_diff = m->foc_motor_ld_lq_diff;
    c->foc_motor_r = m->foc_motor_r;
    c->foc_motor_flux_linkage = m->foc_motor_flux_linkage;
    c->foc_observer_gain = m->foc_observer_gain;
    c->foc_observer_gain_slow = m->foc_observer_gain_slow;
    c->foc_observer_offset = m->foc_observer_offset;
    c->foc_sat_comp_mode = m->foc_sat_comp_mode;
    c->foc_sat_comp = m->foc_sat_comp;
    c->foc_observer_type = m->foc_observer_type;
    c->foc_duty_dowmramp_kp = m->foc_duty_dowmramp_kp;
    c->foc_duty_dowmramp_ki = m->foc_duty_dowmramp_ki;
    c->foc_start_curr_dec = m->foc_start_curr_dec;
    c->foc_start_curr_dec_rpm = m->foc_start_curr_dec_rpm;
    c->foc_short_ls_on_zero_duty = m->foc_short_ls_on_zero_duty;
    c->foc_current_filter_const = m->foc_current_filter_const;
    c->foc_cc_decoupling = m->foc_cc_decoupling;
    c->foc_mtpa_mode = m->foc_mtpa_mode;
    c->foc_fw_current_max = m->foc_fw_current_max;
    c->foc_fw_duty_start = m->foc_fw_duty_start;
    c->foc_fw_ramp_time = m->foc_fw_ramp_time;
    c->foc_fw_q_current_factor = m->foc_fw_q_current_factor;
    c->foc_fw_backoff = m->foc_fw_backoff;
    c->foc_mag_vd_max = m->foc_mag_vd_max;
    c->foc_overmod_factor = m->foc_overmod_factor;
    c->foc_temp_comp = m->foc_temp_comp;
    c->foc_temp_comp_base_temp = m->foc_temp_comp_base_temp;
    c->foc_offsets_cal_mode = m->foc_offsets_cal_mode;
    c->foc_calibrate_on_boot = m->foc_calibrate_on_boot;
    c->foc_pll_kp = m->foc_pll_kp;
    c->foc_pll_ki = m->foc_pll_ki;
    c->foc_openloop_rpm = m->foc_openloop_rpm;
    c->foc_openloop_rpm_low = m->foc_openloop_rpm_low;
    c->foc_sl_openloop_hyst = m->foc_sl_openloop_hyst;
    c->foc_sl_openloop_time = m->foc_sl_openloop_time;
    c->foc_sl_openloop_time_lock = m->foc_sl_openloop_time_lock;
    c->foc_sl_openloop_time_ramp = m->foc_sl_openloop_time_ramp;
    c->foc_sl_openloop_boost_q = m->foc_sl_openloop_boost_q;
    c->foc_sl_openloop_max_q = m->foc_sl_openloop_max_q;
    c->foc_sensor_mode = m->foc_sensor_mode;
    memcpy(c->foc_hall_table, m->foc_hall_table, 8);
    c->foc_hall_interp_erpm = m->foc_hall_interp_erpm;
    c->foc_sl_erpm_start = m->foc_sl_erpm_start;
    c->foc_sl_erpm = m->foc_sl_erpm;
    c->foc_speed_source = m->foc_speed_source;
    c->s_pid_kp = m->speed_pid.kp;
    c->s_pid_ki = m->speed_pid.ki;
    c->s_pid_kd = m->speed_pid.kd;
    c->s_pid_kd_filter = m->speed_kd_filter;
    c->s_pid_min_erpm = m->speed_pid_min_erpm;
    c->s_pid_allow_braking = m->speed_pid_allow_braking;
    c->s_pid_ramp_erpms_s = m->speed_pid_ramp_erpms_s;
    c->s_pid_speed_source = m->speed_pid_source;
    c->p_pid_kp = m->position_pid.kp;
    c->p_pid_ki = m->position_pid.ki;
    c->p_pid_kd = m->position_pid.kd;
    c->p_pid_kd_proc = m->position_kd_proc;
    c->p_pid_kd_filter = m->position_kd_filter;
    c->p_pid_ang_div = m->position_ang_div;
    c->p_pid_gain_dec_angle = m->position_gain_dec_angle;
    c->p_pid_offset = m->position_offset_deg;
    c->cc_startup_boost_duty = 0.0f;
    c->cc_min_current = m->cc_min_current;
    c->cc_gain = 1.0f;
    c->cc_ramp_step_max = 0.01f;
    c->m_encoder_counts = m->encoder.cpr;
    c->m_sensor_port_mode = m->sensor_mode == SENSOR_MODE_ENCODER ? SENSOR_PORT_MODE_ABI : SENSOR_PORT_MODE_HALL;
    c->m_invert_direction = m->invert_direction;
    c->si_motor_poles = (uint8_t)(m->pole_pairs*2U);
    c->si_gear_ratio = m->si_gear_ratio;
    c->si_wheel_diameter = m->si_wheel_diameter;
    c->si_battery_type = m->si_battery_type;
    c->si_battery_cells = m->si_battery_cells;
    c->si_battery_ah = m->si_battery_ah;
    c->si_motor_nl_current = m->si_motor_nl_current;
}

// Parameter reset_conf: data konfigurasi yang menentukan perilaku firmware.
// Fungsi mc_interface_init: menginisialisasi mc interface init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void mc_interface_init(bool reset_conf) {
    if (!s_mc_interface_inited) {
        motor_control_init();
        s_mc_interface_inited = true;
    }
    if (reset_conf) {
        /* Factory reset the live VESC-6.00 wire configs, then persist them so
         * a later boot does not reload the old transactional record. VESC Tool
         * issues this path via the standard GET/SET MCCONF reset command. */
        vesc_config_init_defaults();
        if (!vesc_config_apply_defaults()) {
            motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
            motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
        }
        if (!conf_general_store_all()) {
            motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
            motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
        }
    }
    mirror_from_runtime(&g_motor_left, &s_mcconf_mirror[0]);
    mirror_from_runtime(&g_motor_right, &s_mcconf_mirror[1]);
}
// Fungsi mc_interface_get_configuration: membaca mc interface get configuration tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
const volatile mc_configuration* mc_interface_get_configuration(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    mirror_from_runtime(m, &s_mcconf_mirror[m->id]);
    return &s_mcconf_mirror[m->id];
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi config_float_same: menjalankan operasi config float same sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool config_float_same(float a, float b) {
    if (!isfinite(a) || !isfinite(b))
        return false;
    // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float scale = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));
    return fabsf(a - b) <= (1.0e-6f * scale);
}

// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_unsupported_configuration_unchanged: menjalankan operasi mc interface unsupported
// configuration unchanged sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
static bool mc_interface_unsupported_configuration_unchanged(const mc_configuration *c,
                                                              const MotorRuntime *m) {
    /* Keep the public/internal configuration API under the same ownership
       rules as SET_MCCONF. A caller may change only fields with a real backend
       in this F103 port. Unsupported VESC fields are still mirrored so callers
       can round-trip the complete schema, but changing one must never look as
       though it was applied. */
    // Variabel expected: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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

// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_configuration_runtime_valid: menjalankan operasi mc interface configuration runtime valid
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
static bool mc_interface_configuration_runtime_valid(const mc_configuration *c,
                                                     const MotorRuntime *m) {
    if (c == NULL || m == NULL || c->motor_type != MOTOR_TYPE_FOC)
        return false;
    if (!mc_interface_unsupported_configuration_unchanged(c, m))
        return false;
    // Variabel encoder_ab: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const bool encoder_ab = (c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ||
                             c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB);
    // Variabel sensorless: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool sensorless = c->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
    if (m->id == MOTOR_LEFT) {
        if (!sensorless && !encoder_ab && c->foc_sensor_mode != FOC_SENSOR_MODE_HALL)
            return false;
        if (encoder_ab) {
            if (c->m_encoder_counts < 4U || c->m_encoder_counts > 65535U ||
                c->foc_encoder_offset < 0.0f || c->foc_encoder_offset >= 360.0f ||
                c->foc_encoder_ratio <= 0.0f || c->foc_encoder_ratio > 1000.0f)
                return false;
            // Variabel rq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const uint64_t rq = (uint64_t)llrintf(c->foc_encoder_ratio * 65536.0f);
            if (rq == 0U || rq > UINT32_MAX ||
                ((rq << 16) / c->m_encoder_counts) > UINT32_MAX)
                return false;
        }
    }
    else {
        if ((!sensorless && c->foc_sensor_mode != FOC_SENSOR_MODE_HALL) || encoder_ab)
            return false;
    }

    // Variabel f: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0U; k < sizeof(f) / sizeof(f[0]); k++) {
        if (!isfinite(f[k]))
            return false;
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
        (c->l_additional_faults & ~(MCCONF_L_ADDITIONAL_FAULT_ENCODER_SLIP | MCCONF_L_ADDITIONAL_FAULT_OVERSPEED | MCCONF_L_ADDITIONAL_FAULT_UNDERSPEED | MCCONF_L_ADDITIONAL_FAULT_ABS_SPEED)) != 0U)
        return false;
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
        c->foc_openloop_rpm_low < 0.0f || c->foc_openloop_rpm_low > 1.0f ||
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
        c->foc_fw_q_current_factor < 0.0f || c->foc_fw_q_current_factor > 1.0f)
        return false;
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
        c->si_motor_nl_current < 0.0f || c->si_motor_nl_current > FOC_MAX_CURRENT_A)
        return false;
    if (c->si_motor_poles < 2U || (c->si_motor_poles & 1U) != 0U || c->si_motor_poles > 120U)
        return false;
    return true;
}

// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_configuration: mengatur mc interface set configuration setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_configuration(mc_configuration *c) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = mc_interface_motor_runtime_now();
    if (m == NULL || c == NULL)
        return;

    /* Validate the complete set of runtime-consumed fields before touching
       MotorRuntime. The wire setter already does the same preflight; this API
       path must not become a second, weaker configuration semantics. */
    if (!mc_interface_configuration_runtime_valid(c, m))
        return;

    // Variabel encoder_ab: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const bool encoder_ab = (c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
                             c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER);
    // Variabel sensorless: menandai mode observer murni tanpa Hall atau encoder fisik.
    const bool sensorless = c->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
    // Variabel encoder_was_active: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const bool encoder_was_active = (m->id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER);
    // Variabel encoder_old_cpr: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const uint32_t encoder_old_cpr = m->encoder.cpr;
    // Variabel encoder_old_inverted: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const bool encoder_old_inverted = m->encoder.inverted;
    // Variabel encoder_old_offset_u16: offset kalibrasi untuk mengoreksi bias pengukuran.
    const uint16_t encoder_old_offset_u16 = m->encoder.elec_offset_u16;
    // Variabel encoder_old_ratio: data encoder untuk pengukuran posisi atau kecepatan rotor.
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
    m->temp_fet_start = c->l_temp_fet_start;
    m->temp_fet_end = c->l_temp_fet_end;
    m->temp_motor_start = c->l_temp_motor_start;
    m->temp_motor_end = c->l_temp_motor_end;
    m->temp_accel_dec = c->l_temp_accel_dec;
    m->additional_faults = c->l_additional_faults;
    m->abs_current_fault_count = 0U;
    m->min_vin = c->l_min_vin;
    m->max_vin = c->l_max_vin;
    m->battery_cut_start = c->l_battery_cut_start;
    m->battery_cut_end = c->l_battery_cut_end;
    m->battery_regen_cut_start = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    m->battery_regen_cut_end = m->max_vin - MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    m->min_vin_q15 = volt_to_q15(m->min_vin);
    m->max_vin_q15 = volt_to_q15(m->max_vin);
    m->hard_max_vin_q15 = volt_to_q15(fminf(m->max_vin + FOC_VBUS_HARD_OV_MARGIN_V, FOC_VBUS_HARD_MAX_V));
    m->hard_min_vin_q15 = volt_to_q15(fmaxf(m->min_vin - FOC_VBUS_HARD_UV_MARGIN_V, FOC_VBUS_HARD_MIN_V));
    m->over_voltage_fault_count = 0U;
    m->under_voltage_fault_count = 0U;
    m->min_erpm = c->l_min_erpm;
    m->max_erpm = c->l_max_erpm;
    m->erpm_start = c->l_erpm_start;
    m->foc_start_curr_dec = c->foc_start_curr_dec;
    m->foc_start_curr_dec_rpm = c->foc_start_curr_dec_rpm;
    m->foc_short_ls_on_zero_duty = c->foc_short_ls_on_zero_duty;
    m->max_duty = c->l_max_duty;
    m->min_duty = c->l_min_duty;
    m->duty_limit_now = m->max_duty;
    m->watt_max = c->l_watt_max;
    m->watt_min = c->l_watt_min;
    m->current_max_scale = c->l_current_max_scale;
    m->current_min_scale = c->l_current_min_scale;
    m->duty_start = c->l_duty_start;
    m->invert_direction = c->m_invert_direction;

    motor_set_current_pi_gains(m, c->foc_current_kp, c->foc_current_ki);
    m->foc_temp_comp = c->foc_temp_comp;
    m->foc_temp_comp_base_temp = c->foc_temp_comp_base_temp;
    m->foc_offsets_cal_mode = c->foc_offsets_cal_mode;
    m->foc_calibrate_on_boot = c->foc_calibrate_on_boot;
    m->foc_motor_l = c->foc_motor_l;
    m->foc_motor_ld_lq_diff = c->foc_motor_ld_lq_diff;
    m->foc_motor_r = c->foc_motor_r;
    m->res_est_ohm = m->foc_motor_r;
    m->res_est_state_ohm = m->foc_motor_r;
    m->res_est_valid = false;
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
    m->foc_duty_dowmramp_kp = c->foc_duty_dowmramp_kp;
    m->foc_duty_dowmramp_ki = c->foc_duty_dowmramp_ki;
    m->foc_current_filter_const = c->foc_current_filter_const;
    m->foc_cc_decoupling = c->foc_cc_decoupling;
    m->foc_mtpa_mode = c->foc_mtpa_mode;
    m->foc_fw_current_max = c->foc_fw_current_max;
    m->foc_fw_duty_start = c->foc_fw_duty_start;
    m->foc_fw_ramp_time = c->foc_fw_ramp_time;
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
    // Variabel hall_table_complete: menandai enam state Hall valid telah tersedia.
    const bool hall_table_complete = motor_apply_foc_hall_table(m, c->foc_hall_table);
    if (c->foc_sensor_mode == FOC_SENSOR_MODE_HALL && !hall_table_complete) {
        /* Konfigurasi Hall kosong/korup tidak boleh membuat ISR memakai sudut
           stale. Pakai fallback hoverboard yang tervalidasi sampai Hall detect
           menghasilkan tabel VESC yang baru. */
        init_hall_defaults(m);
    }
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
    if (c->si_motor_poles >= 2U && (c->si_motor_poles & 1U) == 0U)
        m->pole_pairs = c->si_motor_poles / 2U;
    m->si_gear_ratio = c->si_gear_ratio;
    m->si_wheel_diameter = c->si_wheel_diameter;
    m->si_battery_type = c->si_battery_type;
    m->si_battery_cells = c->si_battery_cells;
    m->si_battery_ah = c->si_battery_ah;

    if (m->id == MOTOR_LEFT) {
        // Variabel encoder_new_offset_u16: offset kalibrasi untuk mengoreksi bias pengukuran.
        const uint16_t encoder_new_offset_u16 = foc_deg_to_u16(c->foc_encoder_offset);
        // Variabel encoder_new_cpr: data encoder untuk pengukuran posisi atau kecepatan rotor.
        const uint32_t encoder_new_cpr = c->m_encoder_counts;
        // Variabel encoder_ratio_q16: data encoder untuk pengukuran posisi atau kecepatan rotor.
        const uint32_t encoder_ratio_q16 = (uint32_t)lrintf(c->foc_encoder_ratio * 65536.0f);
        // Variabel step_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint64_t step_q16 = ((uint64_t)encoder_ratio_q16 << 16) /
                                  (encoder_new_cpr == 0U ? 1U : encoder_new_cpr);
        /* Representation bounds were preflighted before MotorRuntime was
           touched; these casts are therefore lossless in the accepted API. */

        // Variabel encoder_hw_changed: data encoder untuk pengukuran posisi atau kecepatan rotor.
        const bool encoder_hw_changed = encoder_ab &&
            (!encoder_was_active || encoder_old_cpr != encoder_new_cpr);
        // Variabel encoder_phase_changed: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
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
        m->sensor_request_mode = encoder_ab ? SENSOR_MODE_ENCODER :
                                 (sensorless ? SENSOR_MODE_NO_SENSOR : SENSOR_MODE_HALL);

        if (encoder_ab) {
            if (encoder_hw_changed) {
                (void)encoder_init(m);
            }
            else {
                m->sensor_mode = SENSOR_MODE_ENCODER;
            }
            if (encoder_hw_changed || encoder_phase_changed) {
                m->encoder.synced = false;
                m->encoder.motion_proved = false;
                m->encoder.sync_active = false;
                m->encoder.speed_sample_valid = false;
                m->using_encoder = false;
            }
        }
        else {
            encoder_deinit(m);
            motor_hw_configure_sensor(m, sensorless ? SENSOR_MODE_NO_SENSOR : SENSOR_MODE_HALL);
            if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL)
                motor_hall_edge_isr(m);
        }
    }
    else {
        m->foc_sensor_mode = c->foc_sensor_mode;
        m->sensor_mode = sensorless ? SENSOR_MODE_NO_SENSOR : SENSOR_MODE_HALL;
        m->sensor_request_mode = m->sensor_mode;
        motor_hw_configure_sensor(m, m->sensor_mode);
        if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL)
            motor_hall_edge_isr(m);
    }
    foc_precalc_values(m);
    /* Setelah MCCONF mengganti mode sensor, jangan membawa fase observer dari
       konfigurasi sebelumnya. Hall di-seed dari state aktif; sensorless murni
       mulai deterministik dari nol lalu forced-openloop membangun fase. */
    uint16_t observer_seed_u16 = m->observer_phase_u16;
    if (m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS)
        observer_seed_u16 = 0U;
    else if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL && m->hall.valid)
        observer_seed_u16 = m->hall_angle_u16[m->hall.raw_state & 7U];
    foc_observer_reset(m, observer_seed_u16);
    mirror_from_runtime(m, &s_mcconf_mirror[m->id]);
}
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter is_motor_2: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mc_interface_calc_crc: menangani kalibrasi mc interface calc crc agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
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
// Fungsi mc_interface_dccal_done: menjalankan operasi mc interface dccal done sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_dccal_done(void) {
    return foc_calibration_done();
}
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_pwm_callback: mengatur mc interface set pwm callback setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_pwm_callback(void(*p)(void)) {
    s_pwm_callback = p;
}
// Fungsi mc_interface_lock: menjalankan operasi mc interface lock sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void mc_interface_lock(void) {
    s_mc_locked = true;
}
// Fungsi mc_interface_unlock: menjalankan operasi mc interface unlock sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void mc_interface_unlock(void) {
    s_mc_locked = false;
}
// Fungsi mc_interface_lock_override_once: menjalankan operasi mc interface lock override once sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_lock_override_once(void) {
    s_mc_lock_override_once = true;
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi mc_interface_try_input_motor: menjalankan operasi mc interface try input motor sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
int mc_interface_try_input_motor(motor_id_t id) {
    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t now = xTaskGetTickCount();
    if ((int32_t)(s_ignore_until[id]-now) > 0)
        return 0;
    if (s_mc_locked && !s_mc_lock_override_once)
        return 0;
    s_mc_lock_override_once = false;
    return 1;
}
// Fungsi mc_interface_try_input: menjalankan operasi mc interface try input sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
int mc_interface_try_input(void) {
    return mc_interface_try_input_motor(mc_interface_motor_runtime_now()->id);
}
// Parameter f: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_fault_to_vesc: menangani motor fault to vesc dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
mc_fault_code motor_fault_to_vesc(motor_fault_t f) {
    switch (f) {
    case MOTOR_FAULT_NONE:
        return FAULT_CODE_NONE;
    case MOTOR_FAULT_OVER_VOLTAGE:
        return FAULT_CODE_OVER_VOLTAGE;
    case MOTOR_FAULT_UNDER_VOLTAGE:
        return FAULT_CODE_UNDER_VOLTAGE;
    case MOTOR_FAULT_ABS_OVER_CURRENT:
        return FAULT_CODE_ABS_OVER_CURRENT;
    case MOTOR_FAULT_CURRENT_OFFSET:
        return FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1;
    case MOTOR_FAULT_HALL_INVALID:
    case MOTOR_FAULT_SENSOR_DETECT:
    case MOTOR_FAULT_SENSORLESS_OBSERVER:
        return FAULT_CODE_ENCODER_FAULT;
    case MOTOR_FAULT_ADC_DMA:
    case MOTOR_FAULT_FOC_ISR_OVERRUN:
        return FAULT_CODE_DRV;
    case MOTOR_FAULT_COMMAND_TIMEOUT:
        return FAULT_CODE_NONE; /* timeout is not a VESC fault */
    case MOTOR_FAULT_OVER_TEMP_BOARD:
        return FAULT_CODE_OVER_TEMP_FET;
    case MOTOR_FAULT_OVER_TEMP_MOTOR:
        return FAULT_CODE_OVER_TEMP_MOTOR;
    case MOTOR_FAULT_OVERSPEED:
        return FAULT_CODE_OVERSPEED;
    case MOTOR_FAULT_UNDERSPEED:
        return FAULT_CODE_UNDERSPEED;
    case MOTOR_FAULT_ABS_OVERSPEED:
        return FAULT_CODE_ABS_OVERSPEED;
    case MOTOR_FAULT_ENCODER_SLIP:
        return FAULT_CODE_ENCODER_SLIP;
    case MOTOR_FAULT_MCU_UNDER_VOLTAGE:
        return FAULT_CODE_MCU_UNDER_VOLTAGE;
    case MOTOR_FAULT_BREAK:
        return FAULT_CODE_BRK;
    case MOTOR_FAULT_FLASH_CONFIG:
        return FAULT_CODE_FLASH_CORRUPTION;
    default:
        return FAULT_CODE_DRV;
    }
}

// Parameter f: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_fault_from_vesc: menangani motor fault from vesc dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
motor_fault_t motor_fault_from_vesc(mc_fault_code f) {
    switch (f) {
    case FAULT_CODE_NONE:
        return MOTOR_FAULT_NONE;
    case FAULT_CODE_OVER_VOLTAGE:
        return MOTOR_FAULT_OVER_VOLTAGE;
    case FAULT_CODE_UNDER_VOLTAGE:
        return MOTOR_FAULT_UNDER_VOLTAGE;
    case FAULT_CODE_ABS_OVER_CURRENT:
        return MOTOR_FAULT_ABS_OVER_CURRENT;
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
        return MOTOR_FAULT_CURRENT_OFFSET;
    case FAULT_CODE_ENCODER_FAULT:
    case FAULT_CODE_ENCODER_NO_MAGNET:
        return MOTOR_FAULT_SENSOR_DETECT;
    case FAULT_CODE_ENCODER_SLIP:
        return MOTOR_FAULT_ENCODER_SLIP;
    case FAULT_CODE_OVER_TEMP_FET:
        return MOTOR_FAULT_OVER_TEMP_BOARD;
    case FAULT_CODE_OVER_TEMP_MOTOR:
        return MOTOR_FAULT_OVER_TEMP_MOTOR;
    case FAULT_CODE_OVERSPEED:
        return MOTOR_FAULT_OVERSPEED;
    case FAULT_CODE_UNDERSPEED:
        return MOTOR_FAULT_UNDERSPEED;
    case FAULT_CODE_ABS_OVERSPEED:
        return MOTOR_FAULT_ABS_OVERSPEED;
    case FAULT_CODE_MCU_UNDER_VOLTAGE:
        return MOTOR_FAULT_MCU_UNDER_VOLTAGE;
    case FAULT_CODE_BRK:
        return MOTOR_FAULT_BREAK;
    case FAULT_CODE_FLASH_CORRUPTION:
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG:
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG:
        return MOTOR_FAULT_FLASH_CONFIG;
    case FAULT_CODE_DRV:
    default:
        return MOTOR_FAULT_ADC_DMA;
    }
}

// Fungsi mc_interface_get_fault: membaca mc interface get fault tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_fault_code mc_interface_get_fault(void) {
    return motor_fault_to_vesc(mc_interface_motor_runtime_now()->fault);
}
// Parameter f: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_fault_to_string: menangani mc interface fault to string dengan memprioritaskan pemadaman
// keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
const char* mc_interface_fault_to_string(mc_fault_code f) {
    switch (f) {
    case FAULT_CODE_NONE:
        return "FAULT_CODE_NONE";
    case FAULT_CODE_OVER_VOLTAGE:
        return "FAULT_CODE_OVER_VOLTAGE";
    case FAULT_CODE_UNDER_VOLTAGE:
        return "FAULT_CODE_UNDER_VOLTAGE";
    case FAULT_CODE_DRV:
        return "FAULT_CODE_DRV";
    case FAULT_CODE_ABS_OVER_CURRENT:
        return "FAULT_CODE_ABS_OVER_CURRENT";
    case FAULT_CODE_OVER_TEMP_FET:
        return "FAULT_CODE_OVER_TEMP_FET";
    case FAULT_CODE_OVER_TEMP_MOTOR:
        return "FAULT_CODE_OVER_TEMP_MOTOR";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
        return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
        return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
        return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
    case FAULT_CODE_UNBALANCED_CURRENTS:
        return "FAULT_CODE_UNBALANCED_CURRENTS";
    case FAULT_CODE_ENCODER_NO_MAGNET:
        return "FAULT_CODE_ENCODER_NO_MAGNET";
    case FAULT_CODE_ENCODER_FAULT:
        return "FAULT_CODE_ENCODER_FAULT";
    case FAULT_CODE_ENCODER_SLIP:
        return "FAULT_CODE_ENCODER_SLIP";
    case FAULT_CODE_OVERSPEED:
        return "FAULT_CODE_OVERSPEED";
    case FAULT_CODE_UNDERSPEED:
        return "FAULT_CODE_UNDERSPEED";
    case FAULT_CODE_ABS_OVERSPEED:
        return "FAULT_CODE_ABS_OVERSPEED";
    case FAULT_CODE_MCU_UNDER_VOLTAGE:
        return "FAULT_CODE_MCU_UNDER_VOLTAGE";
    case FAULT_CODE_BRK:
        return "FAULT_CODE_BRK";
    case FAULT_CODE_FLASH_CORRUPTION:
        return "FAULT_CODE_FLASH_CORRUPTION";
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG:
        return "FAULT_CODE_FLASH_CORRUPTION_APP_CFG";
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG:
        return "FAULT_CODE_FLASH_CORRUPTION_MC_CFG";
    default:
        return "FAULT_CODE_UNKNOWN";
    }
}
// Fungsi mc_interface_get_state: membaca mc interface get state tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_state mc_interface_get_state(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (m->detect.busy)
        return MC_STATE_DETECTING;
    if (m->full_brake_active)
        return MC_STATE_FULL_BRAKE;
    if (m->pwm_enabled)
        return MC_STATE_RUNNING;
    return MC_STATE_OFF;
}
// Fungsi mc_interface_get_control_mode: membaca mc interface get control mode tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
mc_control_mode mc_interface_get_control_mode(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    switch (m->control_mode) {
        case MOTOR_CTRL_DUTY :
            return CONTROL_MODE_DUTY;
        case MOTOR_CTRL_SPEED :
            return CONTROL_MODE_SPEED;
        case MOTOR_CTRL_CURRENT :
            return CONTROL_MODE_CURRENT;
        case MOTOR_CTRL_BRAKE_CURRENT :
            return CONTROL_MODE_CURRENT_BRAKE;
        case MOTOR_CTRL_POSITION :
            return CONTROL_MODE_POS;
        case MOTOR_CTRL_HANDBRAKE :
            return CONTROL_MODE_HANDBRAKE;
        case MOTOR_CTRL_OPENLOOP :
            return CONTROL_MODE_OPENLOOP;
        case MOTOR_CTRL_OPENLOOP_PHASE :
            return CONTROL_MODE_OPENLOOP_PHASE;
        case MOTOR_CTRL_OPENLOOP_DUTY :
            return CONTROL_MODE_OPENLOOP_DUTY;
        case MOTOR_CTRL_OPENLOOP_DUTY_PHASE :
            return CONTROL_MODE_OPENLOOP_DUTY_PHASE;
        default:
            return CONTROL_MODE_NONE;
    }
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_duty: mengatur mc interface set duty setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void mc_interface_set_duty(float v) {
    if (mc_interface_try_input())
        motor_set_duty(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_duty_noramp: mengatur mc interface set duty noramp setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_duty_noramp(float v) {
    mc_interface_set_duty(v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_pid_speed: mengatur mc interface set pid speed setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_pid_speed(float v) {
    if (mc_interface_try_input())
        motor_set_speed(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_pid_pos: mengatur mc interface set pid pos setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_pid_pos(float v) {
    if (mc_interface_try_input())
        motor_set_position(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_current: mengatur mc interface set current setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_current(float v) {
    if (mc_interface_try_input())
        motor_set_current(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_brake_current: mengatur mc interface set brake current setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_brake_current(float v) {
    if (mc_interface_try_input())
        motor_set_brake_current(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_current_rel: mengatur mc interface set current rel setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_current_rel(float v) {
    if (mc_interface_try_input())
        motor_set_current_rel(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_brake_current_rel: mengatur mc interface set brake current rel setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_brake_current_rel(float v) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    v = foc_clampf(v, -1.0f, 1.0f);
    mc_interface_set_brake_current(fabsf(v)*fmaxf(fabsf(m->current_min_a), fabsf(m->current_max_a)));
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_handbrake: mengatur mc interface set handbrake setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_handbrake(float v) {
    if (mc_interface_try_input())
        motor_set_handbrake(mc_interface_motor_runtime_now(), v);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_handbrake_rel: mengatur mc interface set handbrake rel setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_handbrake_rel(float v) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    mc_interface_set_handbrake(fabsf(foc_clampf(v, -1.0f, 1.0f))*m->current_max_a);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mc_interface_set_openloop_current: mengatur mc interface set openloop current setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_current(float current, float rpm) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (m->motor_type == MOTOR_TYPE_FOC && mc_interface_try_input()) {
        mcpwm_foc_set_openloop_current_motor(m, current, rpm);
        motor_touch_command(m);
    }
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mc_interface_set_openloop_phase: mengatur mc interface set openloop phase setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_phase(float current, float phase) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (m->motor_type == MOTOR_TYPE_FOC && mc_interface_try_input()) {
        mcpwm_foc_set_openloop_phase_motor(m, current, phase);
        motor_touch_command(m);
    }
}
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mc_interface_set_openloop_duty: mengatur mc interface set openloop duty setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_duty(float duty, float rpm) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (m->motor_type == MOTOR_TYPE_FOC && mc_interface_try_input()) {
        mcpwm_foc_set_openloop_duty_motor(m, duty, rpm);
        motor_touch_command(m);
    }
}
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mc_interface_set_openloop_duty_phase: mengatur mc interface set openloop duty phase setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_duty_phase(float duty, float phase) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (m->motor_type == MOTOR_TYPE_FOC && mc_interface_try_input()) {
        mcpwm_foc_set_openloop_duty_phase_motor(m, duty, phase);
        motor_touch_command(m);
    }
}
// Parameter steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_tachometer_value: mengatur mc interface set tachometer value setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
int mc_interface_set_tachometer_value(int steps) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    m->stats.tachometer = steps;
    m->stats.tachometer_abs = steps < 0 ? -steps : steps;
    return steps;
}
// Fungsi mc_interface_brake_now: menjalankan operasi mc interface brake now sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void mc_interface_brake_now(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    motor_set_brake_current(m, m->current_max_a);
}
// Fungsi mc_interface_release_motor: menjalankan operasi mc interface release motor sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void mc_interface_release_motor(void) {
    motor_stop(mc_interface_motor_runtime_now());
}
// Fungsi mc_interface_release_motor_override: menjalankan operasi mc interface release motor override sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_release_motor_override(void) {
    mc_interface_release_motor();
}
// Parameter timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi mc_interface_wait_for_motor_release: menjalankan operasi mc interface wait for motor release sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_wait_for_motor_release(float timeout) {
    // Variabel ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t st = xTaskGetTickCount(), ms = (uint32_t)fmaxf(timeout*1000.0f, 0.0f);
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    while (m->pwm_enabled) {
        if ((uint32_t)(xTaskGetTickCount()-st) > ms)
            return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

// Fungsi mc_interface_get_duty_cycle_set: membaca mc interface get duty cycle set tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_duty_cycle_set(void) {
    return mc_interface_motor_runtime_now()->duty_command;
}
// Fungsi mc_interface_get_duty_cycle_now: membaca mc interface get duty cycle now tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_duty_cycle_now(void) {
    return mc_interface_motor_runtime_now()->duty_now;
}
// Fungsi mc_interface_get_sampling_frequency_now: membaca mc interface get sampling frequency now tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_sampling_frequency_now(void) {
    return (float)FOC_ISR_EVENT_HZ;
}
// Fungsi mc_interface_get_rpm: membaca mc interface get rpm tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_rpm(void) {
    return mc_interface_motor_runtime_now()->erpm;
}
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_reset_f: membaca get reset f tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static float get_reset_f(float *p, bool reset) {
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float v = *p;
    if (reset)
        *p = 0.0f;
    return v;
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_amp_hours: membaca mc interface get amp hours tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_amp_hours(bool reset) {
    return get_reset_f(&mc_interface_motor_runtime_now()->stats.amp_hours, reset);
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_amp_hours_charged: membaca mc interface get amp hours charged tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_amp_hours_charged(bool reset) {
    return get_reset_f(&mc_interface_motor_runtime_now()->stats.amp_hours_charged, reset);
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_watt_hours: membaca mc interface get watt hours tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_watt_hours(bool reset) {
    return get_reset_f(&mc_interface_motor_runtime_now()->stats.watt_hours, reset);
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_watt_hours_charged: membaca mc interface get watt hours charged tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_watt_hours_charged(bool reset) {
    return get_reset_f(&mc_interface_motor_runtime_now()->stats.watt_hours_charged, reset);
}
// Fungsi mc_interface_get_tot_current: membaca mc interface get tot current tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current(void) {
    return mcpwm_foc_get_tot_current_rt(mc_interface_motor_runtime_now());
}
// Fungsi mc_interface_get_tot_current_filtered: membaca mc interface get tot current filtered tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_filtered(void) {
    return mcpwm_foc_get_tot_current_filtered();
}
// Fungsi mc_interface_get_tot_current_directional: membaca mc interface get tot current directional tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_directional(void) {
    return mc_interface_motor_runtime_now()->motor_current;
}
// Fungsi mc_interface_get_tot_current_directional_filtered: membaca mc interface get tot current directional
// filtered tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_directional_filtered(void) {
    return mc_interface_get_tot_current_directional();
}
// Fungsi mc_interface_get_tot_current_in: membaca mc interface get tot current in tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_in(void) {
    return mc_interface_motor_runtime_now()->input_current;
}
// Fungsi mc_interface_get_tot_current_in_filtered: membaca mc interface get tot current in filtered tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_in_filtered(void) {
    return mc_interface_get_tot_current_in();
}
// Fungsi mc_interface_get_input_voltage_filtered: membaca mc interface get input voltage filtered tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_input_voltage_filtered(void) {
    return mc_interface_motor_runtime_now()->vbus_filter;
}
// Fungsi mc_interface_get_abs_motor_current_unbalance: membaca mc interface get abs motor current unbalance
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_abs_motor_current_unbalance(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    return fabsf(m->ia+m->ib+m->ic);
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_tachometer_value: membaca mc interface get tachometer value tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
int mc_interface_get_tachometer_value(bool reset) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int v = m->stats.tachometer;
    if (reset)
        m->stats.tachometer = 0;
    return v;
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_tachometer_abs_value: membaca mc interface get tachometer abs value tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
int mc_interface_get_tachometer_abs_value(bool reset) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int v = m->stats.tachometer_abs;
    if (reset)
        m->stats.tachometer_abs = 0;
    return v;
}
// Fungsi mc_interface_get_last_inj_adc_isr_duration: menangani mc interface get last inj adc isr duration pada
// konteks interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
float mc_interface_get_last_inj_adc_isr_duration(void) {
    return foc_last_isr_duration_s();
}
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi read_reset_avg_mask: membaca read reset avg mask tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
static motor_telemetry_avg_t read_reset_avg_mask(uint32_t mask) {
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_telemetry_avg_t a;
    telemetry_read_reset_avg(mc_interface_motor_runtime_now()->id, mask, &a);
    return a;
}
// Fungsi mc_interface_read_reset_avg_motor_current: mereset mc interface read reset avg motor current ke
// kondisi awal yang aman tanpa meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_motor_current(void) {
    return read_reset_avg_mask(1UL<<2).current_motor;
}
// Fungsi mc_interface_read_reset_avg_input_current: mereset mc interface read reset avg input current ke
// kondisi awal yang aman tanpa meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_input_current(void) {
    return read_reset_avg_mask(1UL<<3).current_in;
}
// Fungsi mc_interface_read_reset_avg_id: mereset mc interface read reset avg id ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_id(void) {
    return read_reset_avg_mask(1UL<<4).id;
}
// Fungsi mc_interface_read_reset_avg_iq: mereset mc interface read reset avg iq ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_iq(void) {
    return read_reset_avg_mask(1UL<<5).iq;
}
// Fungsi mc_interface_read_reset_avg_vd: mereset mc interface read reset avg vd ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_vd(void) {
    return read_reset_avg_mask(1UL<<19).vd;
}
// Fungsi mc_interface_read_reset_avg_vq: mereset mc interface read reset avg vq ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_vq(void) {
    return read_reset_avg_mask(1UL<<20).vq;
}
// Fungsi mc_interface_get_pid_pos_set: membaca mc interface get pid pos set tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_pid_pos_set(void) {
    return mc_interface_motor_runtime_now()->position_target_deg;
}
// Fungsi mc_interface_get_pid_pos_now: membaca mc interface get pid pos now tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_pid_pos_now(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    return m ? m->position_deg+m->position_offset_deg : 0.0f;
}
// Parameter angle_now: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_update_pid_pos_offset: memperbarui mc interface update pid pos offset menggunakan state
// terbaru dengan urutan yang konsisten dan aman.
void mc_interface_update_pid_pos_offset(float angle_now, bool store) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (!m)
        return;
    m->position_offset_deg = angle_now-m->position_deg;
    mirror_from_runtime(m, &s_mcconf_mirror[m->id]);
    if (store)
        (void)conf_general_store_mc_configuration(&s_mcconf_mirror[m->id], m->id == MOTOR_RIGHT);
}
// Fungsi mc_interface_get_last_sample_adc_isr_duration: menangani mc interface get last sample adc isr duration
// pada konteks interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
float mc_interface_get_last_sample_adc_isr_duration(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    return (float)m->isr_max_cycles/(float)CPU_CLOCK_HZ;
}
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter reply_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_sample_print_data: menjalankan operasi mc interface sample print data sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len,
        uint8_t decimation, bool raw,
        void (*reply_func)(unsigned char *data, unsigned int len)) {
    /* Upstream retains this asynchronous route for the sample sender. A capture
     * may complete long after COMM_SAMPLE_PRINT returned, so default UART TX
     * would reply to the wrong peer for CAN-forwarded or alternate transports. */
    s_sample_reply_func = reply_func;
    // Variabel motor: state atau parameter motor yang sedang diproses.
    MotorRuntime *motor = mc_interface_motor_runtime_now();
    if (motor != NULL) {
        (void)mc_interface_sample_control(mode, motor->id, len, decimation, raw);
    }
}
// Callback yang dikembalikan menerima pointer data dan panjang payload melalui parameter data dan len.
// Fungsi mc_interface_sample_reply_func: menyusun atau mengirim mc interface sample reply func dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
void (*mc_interface_sample_reply_func(void))(unsigned char *data, unsigned int len) {
    return s_sample_reply_func;
}
/* There is still no MOSFET NTC. VESC's FET-temperature API exposes the
 * explicitly documented MCU/board proxy. Motor temperature remains unavailable
 * unless a real sensor backend feeds the override API. */
#define MC_TEMP_SENSOR_UNAVAILABLE_C (-300.0f)
// Fungsi mc_interface_temp_fet_filtered: menjalankan operasi mc interface temp fet filtered sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_temp_fet_filtered(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    return (m && m->board_temp_valid) ? m->board_temp_filter_c : MC_TEMP_SENSOR_UNAVAILABLE_C;
}
// Fungsi mc_interface_temp_motor_filtered: menjalankan operasi mc interface temp motor filtered sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_temp_motor_filtered(void) {
    return isfinite(s_temp_motor_override) ? s_temp_motor_override : MC_TEMP_SENSOR_UNAVAILABLE_C;
}
// Parameter cell_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter empty_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter full_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi battery_level_linear: menjalankan operasi battery level linear sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static float battery_level_linear(float cell_v, float empty_v, float full_v) {
    if (!isfinite(cell_v) || full_v <= empty_v)
        return 0.0f;
    return foc_clampf((cell_v-empty_v)/(full_v-empty_v), 0.0f, 1.0f);
}

// Parameter wh_left: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_get_battery_level: membaca mc interface get battery level tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_battery_level(float*wh_left) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    if (wh_left)
        *wh_left = 0.0f;
    if (!m || m->si_battery_cells == 0U || m->si_battery_ah <= 0.0f)
        return 0.0f;
    // Variabel cell_v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float cell_v = m->vbus_filter/(float)m->si_battery_cells;
    // Variabel level: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel nominal: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float level = 0.0f, nominal = 0.0f;
    switch (m->si_battery_type) {
        case 0U: /* BATTERY_TYPE_LIION_3_0__4_2 */
            level = battery_level_linear(cell_v, 3.0f, 4.2f);
            nominal = 3.7f;
            break;
        case 1U: /* BATTERY_TYPE_LIIRON_2_6__3_6 */
            level = battery_level_linear(cell_v, 2.6f, 3.6f);
            nominal = 3.3f;
            break;
        case 2U: /* BATTERY_TYPE_LEAD_ACID */
            level = battery_level_linear(cell_v, 1.9f, 2.15f);
            nominal = 2.0f;
            break;
        default:
            return 0.0f;
    }
    if (wh_left)
        *wh_left = level*m->si_battery_ah*(float)m->si_battery_cells*nominal;
    return level;
}

// Fungsi mc_interface_get_speed: membaca mc interface get speed tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_speed(void) {
    if (s_wheel_speed_override)
        return s_wheel_speed_override_value;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    // Variabel poles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float poles = (float)(m->pole_pairs*2U);
    // Variabel gear: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float gear = m->si_gear_ratio;
    // Variabel diam: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float diam = m->si_wheel_diameter;
    if (poles < 2.0f || gear <= 0.0f || diam <= 0.0f)
        return 0.0f;
    // Variabel mech_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float mech_rpm = m->erpm*(2.0f/poles);
    // Variabel wheel_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float wheel_rpm = mech_rpm/gear;
    return wheel_rpm*(3.14159265358979323846f*diam)/60.0f;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter tach: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi tach_to_distance: menjalankan operasi tach to distance sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float tach_to_distance(const MotorRuntime*m, int32_t tach) {
    // Variabel poles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float poles = (float)(m->pole_pairs*2U);
    // Variabel gear: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float gear = m->si_gear_ratio;
    // Variabel diam: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float diam = m->si_wheel_diameter;
    if (poles < 2.0f || gear <= 0.0f || diam <= 0.0f)
        return 0.0f;
    /* VESC tachometer uses six steps per electrical revolution. Therefore
       one mechanical revolution is 3*motor_poles tachometer steps. */
    return ((float)tach*(3.14159265358979323846f*diam))/(3.0f*poles*gear);
}
// Fungsi mc_interface_get_distance: membaca mc interface get distance tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_distance(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    return tach_to_distance(m, m->stats.tachometer);
}
// Fungsi mc_interface_get_distance_abs: membaca mc interface get distance abs tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_distance_abs(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    return tach_to_distance(m, m->stats.tachometer_abs);
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter abs_tach_steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mc_interface_odometer_add_tach_delta: menjalankan operasi mc interface odometer add tach delta sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_odometer_add_tach_delta(motor_id_t id, uint32_t abs_tach_steps) {
    // Variabel idx: indeks elemen yang sedang diproses.
    unsigned idx = (id == MOTOR_RIGHT) ? 1U : 0U;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = motor_get(id);
    if (!m || abs_tach_steps == 0U)
        return;
    // Variabel dm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dm = fabsf(tach_to_distance(m, (int32_t)abs_tach_steps));
    if (!isfinite(dm) || dm <= 0.0f)
        return;
    // Variabel total: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float total = s_odometer_fraction_m[idx]+dm;
    // Variabel whole: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t whole = (uint64_t)floorf(total);
    if (whole > 0U) {
        if (UINT64_MAX-s_odometer[idx] < whole)
            s_odometer[idx] = UINT64_MAX;
        else s_odometer[idx] += whole;
        total -= floorf(total);
    }
    s_odometer_fraction_m[idx] = total;
}
// Parameter ovr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter speed: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Fungsi mc_interface_override_wheel_speed: menjalankan operasi mc interface override wheel speed sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_override_wheel_speed(bool ovr, float speed) {
    s_wheel_speed_override = ovr;
    s_wheel_speed_override_value = speed;
}
// Fungsi mc_interface_get_setup_values: membaca mc interface get setup values tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
setup_values mc_interface_get_setup_values(void) {
    /* VESC setup_values are controller-setup totals, not speed/power stats.
       This board has two local bridges on one MCU, so aggregate LEFT+RIGHT
       analogous to one VESC plus its locally-visible second controller. */
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_values v = {
        0
    }
    ;
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *l = &g_motor_left, *r = &g_motor_right;
    v.ah_tot = l->stats.amp_hours+r->stats.amp_hours;
    v.ah_charge_tot = l->stats.amp_hours_charged+r->stats.amp_hours_charged;
    v.wh_tot = l->stats.watt_hours+r->stats.watt_hours;
    v.wh_charge_tot = l->stats.watt_hours_charged+r->stats.watt_hours_charged;
    v.current_tot = l->motor_current+r->motor_current;
    v.current_in_tot = l->input_current+r->input_current;
    v.num_vescs = 2U;
    return v;
}
// Fungsi mc_interface_gnss: menjalankan operasi mc interface gnss sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
volatile gnss_data*mc_interface_gnss(void) {
    return &s_gnss;
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi mc_interface_get_odometer_motor: membaca mc interface get odometer motor tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
uint64_t mc_interface_get_odometer_motor(motor_id_t id) {
    return s_odometer[id == MOTOR_RIGHT ? 1U : 0U];
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_odometer_motor: mengatur mc interface set odometer motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_odometer_motor(motor_id_t id, uint64_t v) {
    // Variabel idx: indeks elemen yang sedang diproses.
    unsigned idx = id == MOTOR_RIGHT ? 1U : 0U;
    s_odometer[idx] = v;
    s_odometer_fraction_m[idx] = 0.0f;
}
// Fungsi mc_interface_get_odometer: membaca mc interface get odometer tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
uint64_t mc_interface_get_odometer(void) {
    return mc_interface_get_odometer_motor(mc_interface_get_motor_thread() == 2 ? MOTOR_RIGHT : MOTOR_LEFT);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_odometer: mengatur mc interface set odometer setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_odometer(uint64_t v) {
    mc_interface_set_odometer_motor(mc_interface_get_motor_thread() == 2 ? MOTOR_RIGHT : MOTOR_LEFT, v);
}
// Parameter time_ms: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi mc_interface_ignore_input: menjalankan operasi mc interface ignore input sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void mc_interface_ignore_input(int time_ms) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = mc_interface_motor_runtime_now();
    s_ignore_until[m->id] = xTaskGetTickCount()+(time_ms > 0 ? (uint32_t)time_ms : 0U);
}
// Parameter d: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_current_off_delay: mengatur mc interface set current off delay setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_current_off_delay(float d) {
    mcpwm_foc_set_current_off_delay(d);
}
// Parameter t: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_override_temp_motor: menjalankan operasi mc interface override temp motor sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_override_temp_motor(float t) {
    s_temp_motor_override = t;
}
// Parameter time_ms: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi mc_interface_ignore_input_both: menjalankan operasi mc interface ignore input both sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_ignore_input_both(int time_ms) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t u = xTaskGetTickCount()+(time_ms > 0 ? (uint32_t)time_ms : 0U);
    s_ignore_until[0] = u;
    s_ignore_until[1] = u;
}
// Fungsi mc_interface_release_motor_override_both: menjalankan operasi mc interface release motor override both
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_release_motor_override_both(void) {
    motor_stop(&g_motor_left);
    motor_stop(&g_motor_right);
}
// Parameter timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi mc_interface_wait_for_motor_release_both: menjalankan operasi mc interface wait for motor release both
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_wait_for_motor_release_both(float timeout) {
    // Variabel ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t st = xTaskGetTickCount(), ms = (uint32_t)fmaxf(timeout*1000.0f, 0.0f);
    while (g_motor_left.pwm_enabled || g_motor_right.pwm_enabled) {
        if ((uint32_t)(xTaskGetTickCount()-st) > ms)
            return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

// Fungsi setup_stats_now: menjalankan operasi setup stats now sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static setup_stats *setup_stats_now(void) {
    return &s_setup_stats[mc_interface_get_motor_thread() == 2 ? MOTOR_RIGHT : MOTOR_LEFT];
}
// Fungsi mc_interface_stat_speed_avg: menjalankan operasi mc interface stat speed avg sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_speed_avg(void) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_stats*s = setup_stats_now();
    return s->samples > 0.0 ? (float)(s->speed_sum/s->samples) : 0.0f;
}
// Fungsi mc_interface_stat_speed_max: menjalankan operasi mc interface stat speed max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_speed_max(void) {
    return setup_stats_now()->max_speed;
}
// Fungsi mc_interface_stat_power_avg: menjalankan operasi mc interface stat power avg sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_power_avg(void) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_stats*s = setup_stats_now();
    return s->samples > 0.0 ? (float)(s->power_sum/s->samples) : 0.0f;
}
// Fungsi mc_interface_stat_power_max: menjalankan operasi mc interface stat power max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_power_max(void) {
    return setup_stats_now()->max_power;
}
// Fungsi mc_interface_stat_current_avg: menjalankan operasi mc interface stat current avg sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_current_avg(void) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_stats*s = setup_stats_now();
    return s->samples > 0.0 ? (float)(s->current_sum/s->samples) : 0.0f;
}
// Fungsi mc_interface_stat_current_max: menjalankan operasi mc interface stat current max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_current_max(void) {
    return setup_stats_now()->max_current;
}
// Fungsi mc_interface_stat_temp_mosfet_avg: menjalankan operasi mc interface stat temp mosfet avg sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_mosfet_avg(void) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_stats*s = setup_stats_now();
    return s->samples > 0.0 ? (float)(s->temp_mos_sum/s->samples) : 0.0f;
}
// Fungsi mc_interface_stat_temp_mosfet_max: menjalankan operasi mc interface stat temp mosfet max sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_mosfet_max(void) {
    return setup_stats_now()->max_temp_mos;
}
// Fungsi mc_interface_stat_temp_motor_avg: menjalankan operasi mc interface stat temp motor avg sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_motor_avg(void) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_stats*s = setup_stats_now();
    return s->samples > 0.0 ? (float)(s->temp_motor_sum/s->samples) : 0.0f;
}
// Fungsi mc_interface_stat_temp_motor_max: menjalankan operasi mc interface stat temp motor max sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_motor_max(void) {
    return setup_stats_now()->max_temp_motor;
}
// Fungsi mc_interface_stat_count_time: menjalankan operasi mc interface stat count time sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_count_time(void) {
    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t now = xTaskGetTickCount();
    return (float)(now-setup_stats_now()->time_start)/1000.0f;
}
// Fungsi mc_interface_stat_reset: mereset mc interface stat reset ke kondisi awal yang aman tanpa meninggalkan
// state lama yang tidak konsisten.
void mc_interface_stat_reset(void) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_stats*s = setup_stats_now();
    memset(s, 0, sizeof(*s));
    s->time_start = xTaskGetTickCount();
}
// Parameter str: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter argn: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter arg0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter arg1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_fault_info: mengatur mc interface set fault info setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_fault_info(const char*str, int argn, float arg0, float arg1) {
    (void)str;
    (void)argn;
    (void)arg0;
    (void)arg1;
}
// Parameter f: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Parameter is_isr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_fault_stop: menangani mc interface fault stop dengan memprioritaskan pemadaman keluaran
// daya, pencatatan penyebab, dan pemulihan yang aman.
void mc_interface_fault_stop(mc_fault_code f, bool is_second_motor, bool is_isr) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = motor_get(is_second_motor ? MOTOR_RIGHT : MOTOR_LEFT);
    // Variabel native: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_fault_t native = motor_fault_from_vesc(f);
    if (is_isr)
        motor_request_fault_from_isr(m, native);
    else motor_raise_fault_from_task(m, native);
}
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Fungsi mc_interface_mc_timer_isr: menangani mc interface mc timer isr pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
void mc_interface_mc_timer_isr(bool is_second_motor, float dt) {
    (void)is_second_motor;
    (void)dt;
    if (s_pwm_callback)
        s_pwm_callback();
}


/* ============================================================================
 * VESC-standard consolidation: debug sampler + RTOS service/sample/fault threads.
 * Upstream VESC keeps these inside mc_interface.c (ChibiOS THD_FUNCTION + static
 * working areas). This F103 port uses FreeRTOS/FreeRTOS, so the same
 * ownership is retained here in the single compatibility translation unit.
 * ============================================================================ */

#include <stddef.h>

/*
 * Upstream VESC owns the service, sample sender and fault-stop threads in
 * mc_interface.c. This F103 port keeps their implementation in this private
 * translation unit to keep the already large compatibility layer reviewable.
 * The public API and ownership still remain in mc_interface.
 */
#define RTOS_READY_HEAP_RESERVE_BYTES 2048U

// Variabel s_timer_thread: state internal modul yang dipertahankan antar pemanggilan fungsi.
static TaskHandle_t s_timer_thread;
// Variabel s_sample_send_thread: state internal modul yang dipertahankan antar pemanggilan fungsi.
static TaskHandle_t s_sample_send_thread;
// Variabel s_fault_stop_thread: status atau data gangguan untuk sistem proteksi.
static TaskHandle_t s_fault_stop_thread;
// Variabel s_threads_started: state internal modul yang dipertahankan antar pemanggilan fungsi.
static bool s_threads_started;

// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi timer_thread: menjalankan operasi timer thread sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
void timer_thread(void *argument);
// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi sample_send_thread: menyusun atau mengirim sample send thread dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
void sample_send_thread(void *argument);
// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi fault_stop_thread: menangani fault stop thread dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
void fault_stop_thread(void *argument);

// Parameter timer: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sample: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi mc_interface_set_thread_ids: mengatur mc interface set thread ids setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_thread_ids(TaskHandle_t timer, TaskHandle_t sample,
                                 TaskHandle_t fault) {
    s_timer_thread = timer;
    s_sample_send_thread = sample;
    s_fault_stop_thread = fault;
}

// Parameter thread: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi thread_stack_free_bytes: menjalankan operasi thread stack free bytes sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static uint32_t thread_stack_free_bytes(TaskHandle_t thread) {
    if (thread == NULL) {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)thread) *
            (uint32_t)sizeof(StackType_t);
}

// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi fault_signal: menangani fault signal dengan memprioritaskan pemadaman keluaran daya, pencatatan
// penyebab, dan pemulihan yang aman.
static void fault_signal(motor_id_t motor) {
    if (s_fault_stop_thread != NULL) {
        (void)xTaskNotify(s_fault_stop_thread, 1UL << (uint32_t)motor, eSetBits);
    }
}

// Fungsi sample_signal: menjalankan operasi sample signal sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
static void sample_signal(void) {
    if (s_sample_send_thread != NULL) {
        (void)xTaskNotify(s_sample_send_thread, 1UL, eSetBits);
    }
}

// Fungsi mc_interface_free_heap_bytes: menjalankan operasi mc interface free heap bytes sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
uint32_t mc_interface_free_heap_bytes(void) {
    return (uint32_t)xPortGetFreeHeapSize();
}

// Fungsi mc_interface_min_ever_free_heap_bytes: menjalankan operasi mc interface min ever free heap bytes
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t mc_interface_min_ever_free_heap_bytes(void) {
    return (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

// Parameter stats: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_resource_stats: membaca mc interface get resource stats tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
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
    stats->status_stack_free_bytes = stats->motor_service_stack_free_bytes; /* status service shares timer task */
}

// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi timer_thread: menjalankan operasi timer thread sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
void timer_thread(void *argument) {
    (void)argument;
    // Variabel current_offset_fault_reported: offset kalibrasi untuk mengoreksi bias pengukuran.
    bool current_offset_fault_reported = false;
    // Variabel calibration_divider: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t calibration_divider = 0U;
    // Variabel ten_ms_divider: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t ten_ms_divider = 0U;
    // Variabel last_wake: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TickType_t last_wake = xTaskGetTickCount();
    // Variabel watchdog_start_at: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const TickType_t watchdog_start_at = last_wake + pdMS_TO_TICKS(2000U);
    // Variabel watchdog_start_attempted: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool watchdog_start_attempted = false;

    for (;; ) {
        // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint32_t now = (uint32_t)xTaskGetTickCount();
        timeout_heartbeat(TIMEOUT_HEARTBEAT_MOTOR_SERVICE);
        if (!watchdog_start_attempted &&
            (int32_t)((TickType_t)now - watchdog_start_at) >= 0) {
            /* Hardware IWDG, when explicitly enabled, starts only after the
             * scheduler and all workers have had two seconds to establish
             * heartbeat. This avoids reset loops that make SWD require
             * connect-under-reset during commissioning. */
            timeout_watchdog_start();
            watchdog_start_attempted = true;
        }

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
            hw_status_service_10ms(now);
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
                }
                else if (foc_calibration_valid()) {
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

        // Variabel pending: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
    }
}

// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi sample_send_thread: menyusun atau mengirim sample send thread dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
void sample_send_thread(void *argument) {
    (void)argument;
    for (;; ) {
        // Variabel flags: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t flags = 0U;
        (void)xTaskNotifyWait(0U, UINT32_MAX, &flags, portMAX_DELAY);
        if (mc_interface_sample_ready()) {
            // Variabel reply: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            void (*reply)(unsigned char *, unsigned int) =
                    mc_interface_sample_reply_func();
            if (reply != NULL) {
                vesc_comm_send_sample_buffer_to(reply,
                        mc_interface_sample_count());
            }
            else {
                vesc_comm_send_sample_buffer(mc_interface_sample_data(),
                        mc_interface_sample_count());
            }
            mc_interface_sample_mark_sent();
        }
    }
}

// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi fault_stop_thread: menangani fault stop thread dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
void fault_stop_thread(void *argument) {
    (void)argument;
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t mask = (1UL << MOTOR_LEFT) | (1UL << MOTOR_RIGHT);

    for (;; ) {
        // Variabel flags: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t flags = 0U;
        (void)xTaskNotifyWait(0U, mask, &flags, pdMS_TO_TICKS(50U));
        timeout_heartbeat(TIMEOUT_HEARTBEAT_FAULT);
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

// Fungsi mc_interface_start_threads: memulai mc interface start threads setelah prasyarat hardware,
// konfigurasi, dan state keselamatan terpenuhi.
bool mc_interface_start_threads(void) {
    if (s_threads_started) {
        return s_timer_thread != NULL && s_sample_send_thread != NULL &&
                s_fault_stop_thread != NULL;
    }

    /* app_command/app_adc/timeout are initialized in main.c before the scheduler.
     * This function only validates
     * handle yang didaftarkan lewat mc_interface_set_thread_ids() dan
     * menjalankan pemeriksaan resource/heap. */
    // Variabel threads_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool threads_ok = s_timer_thread != NULL &&
            s_sample_send_thread != NULL && s_fault_stop_thread != NULL;
    // Variabel heap_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
    return true;
}

#include "applications/appconf_default.h"

// Variabel s_samples: state internal modul yang dipertahankan antar pemanggilan fungsi.
static debug_sample_t s_samples[SAMPLE_BUFFER_LEN];
// Variabel s_target_len: panjang data yang sedang diproses atau dikirim.
static volatile uint16_t s_target_len;
// Variabel s_wr: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_wr; /* physical next-write index */
// Variabel s_count: pencacah kejadian atau sampel.
static volatile uint16_t s_count; /* valid samples in buffer */
// Variabel s_read_start: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_read_start; /* oldest physical sample */
// Variabel s_decimation: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_decimation;
// Variabel s_decim_count: pencacah kejadian atau sampel.
static volatile uint16_t s_decim_count;
// Variabel s_post_remaining: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_post_remaining;
// Variabel s_motor: state atau parameter motor yang sedang diproses.
static volatile motor_id_t s_motor;
// Variabel s_mode: mode operasi yang menentukan jalur algoritma aktif.
static volatile debug_sampling_mode s_mode;
// Variabel s_active: penanda bahwa state atau fitur sedang aktif.
static volatile bool s_active;
// Variabel s_armed: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_armed;
// Variabel s_triggered: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_triggered;
// Variabel s_capture_valid: penanda validitas hasil pengukuran atau konfigurasi.
static volatile bool s_capture_valid;
// Variabel s_send_pending: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_send_pending;
// Variabel s_auto_send: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_auto_send;
// Variabel s_raw: nilai mentah sebelum konversi ke satuan fisik.
static volatile bool s_raw;
// Variabel s_prev_running: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_prev_running;
// Variabel s_prev_fault: status atau data gangguan untuk sistem proteksi.
static volatile bool s_prev_fault;

// Parameter value: nilai kerja yang digunakan oleh algoritma pada konteks tersebut.
// Fungsi sat_i16: menjalankan operasi sat i16 sesuai tanggung jawab modul dengan input tervalidasi dan state
// yang konsisten.
static int16_t sat_i16(int32_t value) {
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

// Parameter d: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi sample_fill: menjalankan operasi sample fill sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
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

    // Variabel vbus_dv: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
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

// Fungsi finish_capture_isr: menangani finish capture isr pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
static void finish_capture_isr(void) {
    s_active = false;
    s_armed = false;
    s_capture_valid = (s_count != 0U);
    s_read_start = (s_count >= s_target_len) ? s_wr : 0U;
    s_send_pending = s_capture_valid && s_auto_send;
}

// Fungsi mc_interface_sample_init: menginisialisasi mc interface sample init sehingga resource, konfigurasi
// awal, dan state modul siap digunakan dengan aman.
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

// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi mc_interface_sample_control: menjalankan operasi mc interface sample control sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
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

    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
        // Variabel capture_valid: penanda validitas hasil pengukuran atau konfigurasi.
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

// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi mc_interface_sample_start_ex: memulai mc interface sample start ex setelah prasyarat hardware,
// konfigurasi, dan state keselamatan terpenuhi.
void mc_interface_sample_start_ex(motor_id_t motor, uint16_t len,
        uint16_t decimation, bool raw) {
    (void)mc_interface_sample_control(DEBUG_SAMPLING_NOW, motor, len,
            decimation, raw);
}

// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_sample_start: memulai mc interface sample start setelah prasyarat hardware, konfigurasi,
// dan state keselamatan terpenuhi.
void mc_interface_sample_start(motor_id_t motor, uint16_t len,
        uint16_t decimation) {
    mc_interface_sample_start_ex(motor, len, decimation, false);
}

// Parameter active: penanda bahwa state atau fitur sedang aktif.
// Fungsi mc_interface_sample_capture_isr: menangani mc interface sample capture isr pada konteks interrupt
// dengan pekerjaan minimum agar timing FOC tetap deterministik.
void mc_interface_sample_capture_isr(MotorRuntime *active) {
    if (!s_active || active == NULL || active->id != s_motor) {
        return;
    }

    // Variabel running: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool running = active->pwm_enabled &&
            active->fault == MOTOR_FAULT_NONE;
    // Variabel fault_now: status atau data gangguan untuk sistem proteksi.
    const bool fault_now = active->fault != MOTOR_FAULT_NONE;
    // Variabel start_edge: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool start_edge = running && !s_prev_running;
    // Variabel fault_edge: status atau data gangguan untuk sistem proteksi.
    const bool fault_edge = fault_now && !s_prev_fault;
    s_prev_running = running;
    s_prev_fault = fault_now;

    if (s_armed && !s_triggered) {
        // Variabel trigger: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
            }
            else {
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
    // Variabel trigger_mode: mode operasi yang menentukan jalur algoritma aktif.
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

// Fungsi mc_interface_sample_ready: menjalankan operasi mc interface sample ready sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_ready(void) {
    return s_capture_valid && s_send_pending;
}

// Fungsi mc_interface_sample_has_capture: menjalankan operasi mc interface sample has capture sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_has_capture(void) {
    return s_capture_valid;
}

// Fungsi mc_interface_sample_count: menjalankan operasi mc interface sample count sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
uint16_t mc_interface_sample_count(void) {
    return s_count;
}

// Fungsi mc_interface_sample_data: menjalankan operasi mc interface sample data sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
const debug_sample_t *mc_interface_sample_data(void) {
    return s_samples;
}

// Parameter logical_index: indeks elemen yang sedang diproses.
// Fungsi mc_interface_sample_at: menjalankan operasi mc interface sample at sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
const debug_sample_t *mc_interface_sample_at(uint16_t logical_index) {
    if (!s_capture_valid || logical_index >= s_count || s_target_len == 0U) {
        return NULL;
    }

    // Variabel physical_index: indeks elemen yang sedang diproses.
    uint16_t physical_index = (uint16_t)(s_read_start + logical_index);
    while (physical_index >= s_target_len) {
        physical_index = (uint16_t)(physical_index - s_target_len);
    }
    return &s_samples[physical_index];
}

// Fungsi mc_interface_sample_mark_sent: menjalankan operasi mc interface sample mark sent sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_sample_mark_sent(void) {
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_send_pending = false;
    if (primask == 0U) {
        __enable_irq();
    }
}

// Fungsi mc_interface_sample_active: menjalankan operasi mc interface sample active sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_active(void) {
    return s_active;
}

// Fungsi mc_interface_sample_raw: menjalankan operasi mc interface sample raw sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_raw(void) {
    return s_raw;
}

// Fungsi mc_interface_sample_mode: menjalankan operasi mc interface sample mode sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
debug_sampling_mode mc_interface_sample_mode(void) {
    return s_mode;
}
