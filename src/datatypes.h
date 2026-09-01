#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_id_t;


/* Mux sensor fisik khusus port STM32F103. Enum ini sengaja dipisahkan dari
 * mc_foc_sensor_mode/sensor_port_mode milik ABI VESC: nilainya hanya menjelaskan
 * periferal input fisik yang aktif pada board. SENSOR_MODE_NO_SENSOR berarti
 * Hall EXTI dan encoder tidak menjadi sumber input FOC. */
typedef enum {
    SENSOR_MODE_AUTO = 0,
    SENSOR_MODE_HALL = 1,
    SENSOR_MODE_ENCODER = 2,
    SENSOR_MODE_NO_SENSOR = 3
} motor_sensor_runtime_mode_t;

/* Nilai enum kanonik VESC untuk MCCONF dan pembandingan logika. Entri HFI
 * tetap dipertahankan pada wire schema agar layout field berikutnya kompatibel,
 * tetapi backend STM32F103 ini tidak menjalankan HFI. */
typedef enum {
    FOC_SENSOR_MODE_SENSORLESS = 0,
    FOC_SENSOR_MODE_ENCODER = 1,
    FOC_SENSOR_MODE_HALL = 2,
    FOC_SENSOR_MODE_HFI = 3,
    FOC_SENSOR_MODE_HFI_START = 4,
    FOC_SENSOR_MODE_HFI_V2 = 5,
    FOC_SENSOR_MODE_HFI_V3 = 6,
    FOC_SENSOR_MODE_HFI_V4 = 7,
    FOC_SENSOR_MODE_HFI_V5 = 8,
    FOC_SENSOR_MODE_ENCODER_AB = 9
} mc_foc_sensor_mode;

typedef enum {
    SENSOR_PORT_MODE_HALL = 0,
    SENSOR_PORT_MODE_ABI = 1
} sensor_port_mode;

typedef enum {
    MOTOR_CTRL_OFF = 0,
    MOTOR_CTRL_CURRENT,
    MOTOR_CTRL_BRAKE_CURRENT,
    MOTOR_CTRL_SPEED,
    MOTOR_CTRL_POSITION,
    MOTOR_CTRL_DUTY,
    MOTOR_CTRL_HANDBRAKE,
    MOTOR_CTRL_OPENLOOP,
    MOTOR_CTRL_OPENLOOP_PHASE,
    MOTOR_CTRL_OPENLOOP_DUTY,
    MOTOR_CTRL_OPENLOOP_DUTY_PHASE,
    MOTOR_CTRL_DETECT
} motor_control_mode_t;

/* Board-internal faults. These values are NEVER placed directly on the VESC
 * wire protocol. Keep board diagnostics independent from the upstream
 * mc_fault_code numbering so new local diagnostics cannot silently change
 * what VESC Tool displays. */
typedef enum {
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_ADC_DMA,
    MOTOR_FAULT_ABS_OVER_CURRENT,
    MOTOR_FAULT_OVER_VOLTAGE,
    MOTOR_FAULT_UNDER_VOLTAGE,
    MOTOR_FAULT_HALL_INVALID,
    MOTOR_FAULT_FOC_ISR_OVERRUN,
    MOTOR_FAULT_COMMAND_TIMEOUT,
    MOTOR_FAULT_CURRENT_OFFSET,
    MOTOR_FAULT_SENSOR_DETECT,
    MOTOR_FAULT_OVER_TEMP_BOARD,
    MOTOR_FAULT_OVER_TEMP_MOTOR,
    MOTOR_FAULT_OVERSPEED,
    MOTOR_FAULT_UNDERSPEED,
    MOTOR_FAULT_ABS_OVERSPEED,
    MOTOR_FAULT_ENCODER_SLIP,
    MOTOR_FAULT_MCU_UNDER_VOLTAGE,
    MOTOR_FAULT_BREAK,
    MOTOR_FAULT_SENSORLESS_OBSERVER,
    MOTOR_FAULT_FLASH_CONFIG
} motor_fault_t;

/* Upstream-compatible mc_fault_code numbering superset. VESC Tool interprets
 * the numeric value, so board-private faults are mapped explicitly and the
 * VESC 6.00 protocol only emits the compatible subset used by this target. */
typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR,
    FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
    FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
    FAULT_CODE_MCU_UNDER_VOLTAGE,
    FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
    FAULT_CODE_ENCODER_SPI,
    FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
    FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
    FAULT_CODE_FLASH_CORRUPTION,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
    FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
    FAULT_CODE_UNBALANCED_CURRENTS,
    FAULT_CODE_BRK,
    FAULT_CODE_RESOLVER_LOT,
    FAULT_CODE_RESOLVER_DOS,
    FAULT_CODE_RESOLVER_LOS,
    FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
    FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
    FAULT_CODE_ENCODER_NO_MAGNET,
    FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
    FAULT_CODE_PHASE_FILTER,
    FAULT_CODE_ENCODER_FAULT,
    FAULT_CODE_LV_OUTPUT_FAULT,
    FAULT_CODE_ENCODER_SLIP,
    FAULT_CODE_OVERSPEED,
    FAULT_CODE_UNDERSPEED,
    FAULT_CODE_ABS_OVERSPEED
} mc_fault_code;

/* VESC motor backend/config enums. Values intentionally match upstream. */
typedef enum {
    PWM_MODE_NONSYNCHRONOUS_HISW = 0,
    PWM_MODE_SYNCHRONOUS = 1,
    PWM_MODE_BIPOLAR = 2
} mc_pwm_mode;

typedef enum {
    COMM_MODE_INTEGRATE = 0, COMM_MODE_DELAY = 1
}
mc_comm_mode;
typedef enum {
    SENSOR_MODE_SENSORLESS = 0, SENSOR_MODE_SENSORED = 1, SENSOR_MODE_HYBRID = 2
}
mc_sensor_mode;
typedef enum {
    MOTOR_TYPE_BLDC = 0, MOTOR_TYPE_DC = 1, MOTOR_TYPE_FOC = 2
}
mc_motor_type;
typedef enum {
    MC_STATE_OFF = 0, MC_STATE_DETECTING, MC_STATE_RUNNING, MC_STATE_FULL_BRAKE
}
mc_state;
typedef enum {
    CONTROL_MODE_DUTY = 0, CONTROL_MODE_SPEED, CONTROL_MODE_CURRENT,
    CONTROL_MODE_CURRENT_BRAKE, CONTROL_MODE_POS, CONTROL_MODE_HANDBRAKE,
    CONTROL_MODE_OPENLOOP, CONTROL_MODE_OPENLOOP_PHASE,
    CONTROL_MODE_OPENLOOP_DUTY, CONTROL_MODE_OPENLOOP_DUTY_PHASE, CONTROL_MODE_NONE
} mc_control_mode;

typedef struct {
    // Variabel cycle_int_limit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float cycle_int_limit;
    // Variabel cycle_int_limit_running: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float cycle_int_limit_running;
    // Variabel cycle_int_limit_max: batas atau nilai maksimum untuk validasi dan proteksi.
    float cycle_int_limit_max;
    // Variabel comm_time_sum: nilai waktu untuk penjadwalan atau pengawasan.
    float comm_time_sum;
    // Variabel comm_time_sum_min_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float comm_time_sum_min_rpm;
    // Variabel comms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t comms;
    // Variabel time_at_comm: nilai waktu untuk penjadwalan atau pengawasan.
    float time_at_comm;
} mc_rpm_dep_struct;

typedef enum {
    DEBUG_SAMPLING_OFF = 0, DEBUG_SAMPLING_NOW, DEBUG_SAMPLING_START,
    DEBUG_SAMPLING_TRIGGER_START, DEBUG_SAMPLING_TRIGGER_FAULT,
    DEBUG_SAMPLING_TRIGGER_START_NOSEND, DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,
    DEBUG_SAMPLING_SEND_LAST_SAMPLES, DEBUG_SAMPLING_SEND_SINGLE_SAMPLE
} debug_sampling_mode;

typedef enum {
    FOC_CC_DECOUPLING_DISABLED = 0,
    FOC_CC_DECOUPLING_CROSS,
    FOC_CC_DECOUPLING_BEMF,
    FOC_CC_DECOUPLING_CROSS_BEMF
} mc_foc_cc_decoupling_mode;

typedef enum {
    MTPA_MODE_OFF = 0,
    MTPA_MODE_IQ_TARGET,
    MTPA_MODE_IQ_MEASURED
} MTPA_MODE;

typedef enum {
    SAT_COMP_DISABLED = 0,
    SAT_COMP_FACTOR,
    SAT_COMP_LAMBDA,
    SAT_COMP_LAMBDA_AND_FACTOR
} SAT_COMP_MODE;

/* Current VESC speed-PID source numbering. VESC 6.00 only serializes the
 * legacy speed-PID fields, so this selector is runtime-only in this port and
 * defaults to PLL. Keeping the canonical enum avoids inventing a different
 * internal meaning while preserving the fixed 481-byte MCCONF ABI. */
typedef enum {
    S_PID_SPEED_SRC_PLL = 0,
    S_PID_SPEED_SRC_FAST,
    S_PID_SPEED_SRC_FASTER
} S_PID_SPEED_SRC;

/* VESC 6.00 already serializes the FOC phase/speed source at MCCONF byte 314.
 * This is separate from the outer speed-PID source. */
typedef enum {
    FOC_SPEED_SRC_CORRECTED = 0,
    FOC_SPEED_SRC_OBSERVER
} FOC_SPEED_SRC;

/* Canonical VESC observer numbering. Part 2 implements every non-HFI FOC
 * observer below in the fixed-point 16-kHz backend. Keep the upstream values
 * unchanged so the VESC 6.00 MCCONF wire image remains byte-compatible. */
typedef enum {
    FOC_OBSERVER_ORTEGA_ORIGINAL = 0,
    FOC_OBSERVER_MXLEMMING,
    FOC_OBSERVER_ORTEGA_LAMBDA_COMP,
    FOC_OBSERVER_MXLEMMING_LAMBDA_COMP,
    FOC_OBSERVER_MXV,
    FOC_OBSERVER_MXV_LAMBDA_COMP,
    FOC_OBSERVER_MXV_LAMBDA_COMP_LIN
} mc_foc_observer_type;

/* Compact VESC-compatible runtime configuration mirror. The canonical VESC
 * 6 wire image remains the persistent/source-of-truth representation in
 * confgenerator.c. This mirror contains every field consumed by the F103
 * motor backends, while unsupported VESC fields still round-trip byte-exact
 * in the wire image. */
typedef struct mc_configuration {
    // Variabel l_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_current_min: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_in_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_in_current_min: nilai arus untuk pengukuran, kendali, atau proteksi.
    float l_current_max, l_current_min, l_in_current_max, l_in_current_min;
    // Variabel l_in_current_map_filter: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_in_current_map_start: nilai arus untuk pengukuran, kendali, atau proteksi.
    float l_in_current_map_start, l_in_current_map_filter;
    // Variabel l_abs_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_erpm_start: kecepatan listrik motor dalam electrical RPM.
    // Variabel l_max_erpm: kecepatan listrik motor dalam electrical RPM.
    // Variabel l_max_vin: batas atau nilai maksimum untuk validasi dan proteksi.
    // Variabel l_min_erpm: kecepatan listrik motor dalam electrical RPM.
    // Variabel l_min_vin: batas atau nilai minimum untuk validasi dan proteksi.
    float l_abs_current_max, l_min_erpm, l_max_erpm, l_erpm_start, l_min_vin, l_max_vin;
    // Variabel l_battery_cut_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel l_battery_cut_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l_battery_cut_start, l_battery_cut_end;
    // Variabel l_slow_abs_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    bool l_slow_abs_current;
    // Variabel l_temp_fet_end: nilai sementara atau temperatur sesuai konteks modul.
    // Variabel l_temp_fet_start: nilai sementara atau temperatur sesuai konteks modul.
    float l_temp_fet_start, l_temp_fet_end;
    // Variabel l_temp_motor_end: nilai sementara atau temperatur sesuai konteks modul.
    // Variabel l_temp_motor_start: nilai sementara atau temperatur sesuai konteks modul.
    float l_temp_motor_start, l_temp_motor_end;
    // Variabel l_temp_accel_dec: nilai sementara atau temperatur sesuai konteks modul.
    float l_temp_accel_dec;
    /* Not present in the pinned VESC-6.00 wire image. Runtime-only until a
       dedicated protocol migration is performed. */
    // Variabel l_additional_faults: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t l_additional_faults;
    /* VESC-6 wire already contains watt/current scaling. Regen-cut fields were
       added later; they remain runtime-only defaults while FW reports 6.00. */
    // Variabel l_battery_regen_cut_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel l_battery_regen_cut_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l_battery_regen_cut_start, l_battery_regen_cut_end;
    // Variabel l_max_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    // Variabel l_min_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float l_min_duty, l_max_duty;
    // Variabel l_watt_max: batas atau nilai maksimum untuk validasi dan proteksi.
    // Variabel l_watt_min: batas atau nilai minimum untuk validasi dan proteksi.
    float l_watt_max, l_watt_min;
    // Variabel l_current_max_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_current_min_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel l_duty_start: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float l_current_max_scale, l_current_min_scale, l_duty_start;
    // Variabel lo_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel lo_current_min: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel lo_in_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel lo_in_current_min: nilai arus untuk pengukuran, kendali, atau proteksi.
    float lo_current_max, lo_current_min, lo_in_current_max, lo_in_current_min;
    // Variabel pwm_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_pwm_mode pwm_mode;
    // Variabel comm_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_comm_mode comm_mode;
    // Variabel motor_type: state atau parameter motor yang sedang diproses.
    mc_motor_type motor_type;
    // Variabel sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_sensor_mode sensor_mode;
    // Variabel sl_max_fullbreak_current_dir_change: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel sl_min_erpm: kecepatan listrik motor dalam electrical RPM.
    // Variabel sl_min_erpm_cycle_int_limit: kecepatan listrik motor dalam electrical RPM.
    float sl_min_erpm, sl_min_erpm_cycle_int_limit, sl_max_fullbreak_current_dir_change;
    // Variabel sl_bemf_coupling_k: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel sl_cycle_int_limit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel sl_cycle_int_rpm_br: kecepatan putar yang digunakan oleh logika kendali.
    // Variabel sl_phase_advance_at_br: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    float sl_cycle_int_limit, sl_phase_advance_at_br, sl_cycle_int_rpm_br, sl_bemf_coupling_k;
    // Variabel hall_table: data sensor Hall untuk menentukan sektor atau posisi rotor.
    int8_t hall_table[8];
    // Variabel hall_sl_erpm: kecepatan listrik motor dalam electrical RPM.
    float hall_sl_erpm;
    // Variabel foc_current_ki: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel foc_current_kp: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel foc_dt_us: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel foc_f_zv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_current_kp, foc_current_ki, foc_f_zv, foc_dt_us;
    // Variabel foc_encoder_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    float foc_encoder_offset;
    // Variabel foc_encoder_inverted: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool foc_encoder_inverted;
    // Variabel foc_encoder_ratio: data encoder untuk pengukuran posisi atau kecepatan rotor.
    float foc_encoder_ratio;
    // Variabel foc_motor_flux_linkage: state atau parameter motor yang sedang diproses.
    // Variabel foc_motor_l: state atau parameter motor yang sedang diproses.
    // Variabel foc_motor_ld_lq_diff: state atau parameter motor yang sedang diproses.
    // Variabel foc_motor_r: state atau parameter motor yang sedang diproses.
    float foc_motor_l, foc_motor_ld_lq_diff, foc_motor_r, foc_motor_flux_linkage;
    // Variabel foc_observer_gain: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    // Variabel foc_observer_gain_slow: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    // Variabel foc_observer_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    float foc_observer_gain, foc_observer_gain_slow, foc_observer_offset;
    // Variabel foc_duty_dowmramp_ki: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    // Variabel foc_duty_dowmramp_kp: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float foc_duty_dowmramp_kp, foc_duty_dowmramp_ki;
    // Variabel foc_start_curr_dec: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel foc_start_curr_dec_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float foc_start_curr_dec, foc_start_curr_dec_rpm;
    /* Newer VESC field with a real backend in Batch 9.3. It is runtime-only
       while the wire ABI remains VESC 6.00; serialization rejects true. */
    // Variabel foc_short_ls_on_zero_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    bool foc_short_ls_on_zero_duty;
    // Variabel foc_pll_ki: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel foc_pll_kp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_pll_kp, foc_pll_ki;
    // Variabel foc_openloop_rpm: kecepatan putar yang digunakan oleh logika kendali.
    // Variabel foc_openloop_rpm_low: kecepatan putar yang digunakan oleh logika kendali.
    // Variabel foc_sl_openloop_hyst: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_openloop_rpm, foc_openloop_rpm_low, foc_sl_openloop_hyst;
    // Variabel foc_sl_openloop_time: nilai waktu untuk penjadwalan atau pengawasan.
    // Variabel foc_sl_openloop_time_lock: nilai waktu untuk penjadwalan atau pengawasan.
    // Variabel foc_sl_openloop_time_ramp: nilai waktu untuk penjadwalan atau pengawasan.
    float foc_sl_openloop_time, foc_sl_openloop_time_lock, foc_sl_openloop_time_ramp;
    // Variabel foc_sl_openloop_boost_q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel foc_sl_openloop_max_q: batas atau nilai maksimum untuk validasi dan proteksi.
    float foc_sl_openloop_boost_q, foc_sl_openloop_max_q;
    // Variabel foc_sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_foc_sensor_mode foc_sensor_mode;
    // Variabel foc_hall_table: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint8_t foc_hall_table[8];
    // Variabel foc_hall_interp_erpm: kecepatan listrik motor dalam electrical RPM.
    // Variabel foc_sl_erpm: kecepatan listrik motor dalam electrical RPM.
    // Variabel foc_sl_erpm_start: kecepatan listrik motor dalam electrical RPM.
    float foc_hall_interp_erpm, foc_sl_erpm_start, foc_sl_erpm;
    // Variabel foc_sample_high_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel foc_sample_v0_v7: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool foc_sample_v0_v7, foc_sample_high_current;
    // Variabel foc_speed_source: nilai kecepatan untuk target atau pengukuran.
    FOC_SPEED_SRC foc_speed_source;
    // Variabel foc_sat_comp_mode: mode operasi yang menentukan jalur algoritma aktif.
    SAT_COMP_MODE foc_sat_comp_mode;
    // Variabel foc_current_filter_const: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel foc_sat_comp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_sat_comp, foc_current_filter_const;
    // Variabel foc_cc_decoupling: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    mc_foc_cc_decoupling_mode foc_cc_decoupling;
    // Variabel foc_observer_type: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    mc_foc_observer_type foc_observer_type;
    // Variabel foc_mtpa_mode: mode operasi yang menentukan jalur algoritma aktif.
    MTPA_MODE foc_mtpa_mode;
    // Variabel foc_fw_backoff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel foc_fw_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel foc_fw_duty_start: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    // Variabel foc_fw_q_current_factor: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel foc_fw_ramp_time: nilai waktu untuk penjadwalan atau pengawasan.
    float foc_fw_current_max, foc_fw_duty_start, foc_fw_ramp_time, foc_fw_q_current_factor, foc_fw_backoff;
    // Variabel foc_mag_vd_max: tegangan sumbu-d keluaran regulator FOC.
    // Variabel foc_overmod_factor: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_mag_vd_max, foc_overmod_factor;
    /* VESC-style motor-temperature (resistance) compensation. The F103 port has
       no motor NTC, so the STM32 board-temperature proxy (board_temp_filter_c)
       is used as the thermal input. comp_factor = 1 + 0.00386*(T - base_temp). */
    // Variabel foc_temp_comp: nilai sementara atau temperatur sesuai konteks modul.
    bool foc_temp_comp;
    // Variabel foc_temp_comp_base_temp: nilai sementara atau temperatur sesuai konteks modul.
    float foc_temp_comp_base_temp;
    /* VESC offset-calibration mode bits: bit0 = driven, bit1 = undriven,
       bit2 = periodic re-calibration when motor stopped (state OFF). */
    // Variabel foc_offsets_cal_mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t foc_offsets_cal_mode;
    /* When false, the entire boot current-offset calibration pipeline is
       skipped and the motor runs with stored/gross-default offsets instead.
       VESC Tool can read/write this flag. Default is true. */
    // Variabel foc_calibrate_on_boot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool foc_calibrate_on_boot;
    // Variabel s_pid_kd: state internal modul yang dipertahankan antar pemanggilan fungsi.
    // Variabel s_pid_kd_filter: state internal modul yang dipertahankan antar pemanggilan fungsi.
    // Variabel s_pid_ki: state internal modul yang dipertahankan antar pemanggilan fungsi.
    // Variabel s_pid_kp: state internal modul yang dipertahankan antar pemanggilan fungsi.
    // Variabel s_pid_min_erpm: kecepatan listrik motor dalam electrical RPM.
    float s_pid_kp, s_pid_ki, s_pid_kd, s_pid_kd_filter, s_pid_min_erpm;
    // Variabel s_pid_allow_braking: state internal modul yang dipertahankan antar pemanggilan fungsi.
    bool s_pid_allow_braking;
    // Variabel s_pid_ramp_erpms_s: state internal modul yang dipertahankan antar pemanggilan fungsi.
    float s_pid_ramp_erpms_s;
    // Variabel s_pid_speed_source: nilai kecepatan untuk target atau pengukuran.
    S_PID_SPEED_SRC s_pid_speed_source;
    // Variabel p_pid_ang_div: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel p_pid_gain_dec_angle: nilai sudut untuk posisi atau transformasi koordinat.
    // Variabel p_pid_kd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel p_pid_kd_filter: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel p_pid_kd_proc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel p_pid_ki: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel p_pid_kp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel p_pid_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    float p_pid_kp, p_pid_ki, p_pid_kd, p_pid_kd_proc, p_pid_kd_filter, p_pid_ang_div, p_pid_gain_dec_angle, p_pid_offset;
    // Variabel cc_gain: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel cc_min_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel cc_ramp_step_max: batas atau nilai maksimum untuk validasi dan proteksi.
    // Variabel cc_startup_boost_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float cc_startup_boost_duty, cc_min_current, cc_gain, cc_ramp_step_max;
    // Variabel m_encoder_counts: data encoder untuk pengukuran posisi atau kecepatan rotor.
    uint32_t m_encoder_counts;
    // Variabel m_sensor_port_mode: mode operasi yang menentukan jalur algoritma aktif.
    sensor_port_mode m_sensor_port_mode;
    // Variabel m_invert_direction: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool m_invert_direction;
    // Variabel si_motor_poles: state atau parameter motor yang sedang diproses.
    uint8_t si_motor_poles;
    // Variabel si_gear_ratio: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float si_gear_ratio;
    // Variabel si_wheel_diameter: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float si_wheel_diameter;
    // Variabel si_battery_type: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t si_battery_type;
    // Variabel si_battery_cells: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t si_battery_cells;
    // Variabel si_battery_ah: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float si_battery_ah;
    // Variabel si_motor_nl_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float si_motor_nl_current;
} mc_configuration;

/* Canonical VESC setup aggregate. In this dual-bridge port these values are
 * aggregated over both local motors, analogous to VESC setup values including
 * local/CAN controllers. */
typedef struct {
    // Variabel ah_tot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ah_tot;
    // Variabel ah_charge_tot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ah_charge_tot;
    // Variabel wh_tot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float wh_tot;
    // Variabel wh_charge_tot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float wh_charge_tot;
    // Variabel current_tot: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_tot;
    // Variabel current_in_tot: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_in_tot;
    // Variabel num_vescs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t num_vescs;
} setup_values;

/* Canonical VESC statistics state. FreeRTOS tick time replaces ChibiOS
 * systime_t, while the numeric fields retain upstream semantics. */
typedef struct {
    // Variabel time_start: nilai waktu untuk penjadwalan atau pengawasan.
    uint32_t time_start;
    // Variabel samples: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    double samples;
    // Variabel speed_sum: akumulator penjumlahan untuk averaging atau statistik.
    double speed_sum;
    // Variabel max_speed: batas atau nilai maksimum untuk validasi dan proteksi.
    float max_speed;
    // Variabel power_sum: akumulator penjumlahan untuk averaging atau statistik.
    double power_sum;
    // Variabel max_power: batas atau nilai maksimum untuk validasi dan proteksi.
    float max_power;
    // Variabel temp_motor_sum: akumulator penjumlahan untuk averaging atau statistik.
    double temp_motor_sum;
    // Variabel max_temp_motor: batas atau nilai maksimum untuk validasi dan proteksi.
    float max_temp_motor;
    // Variabel temp_mos_sum: akumulator penjumlahan untuk averaging atau statistik.
    double temp_mos_sum;
    // Variabel max_temp_mos: batas atau nilai maksimum untuk validasi dan proteksi.
    float max_temp_mos;
    // Variabel current_sum: nilai arus untuk pengukuran, kendali, atau proteksi.
    double current_sum;
    // Variabel max_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float max_current;
} setup_stats;

/* Canonical VESC application selector numbering. This target implements
 * APP_NONE, APP_ADC, APP_UART and APP_ADC_UART. Other application enums stay
 * present only to preserve the VESC-6 protocol schema. */
typedef enum {
    APP_NONE = 0, APP_PPM, APP_ADC, APP_UART, APP_PPM_UART, APP_ADC_UART,
    APP_NUNCHUK, APP_NRF, APP_CUSTOM, APP_PAS, APP_ADC_PAS
} app_use;

typedef enum {
    THR_EXP_EXPO = 0,
    THR_EXP_NATURAL,
    THR_EXP_POLY
} thr_exp_mode;

typedef enum {
    SAFE_START_DISABLED = 0,
    SAFE_START_REGULAR,
    SAFE_START_NO_FAULT
} SAFE_START_MODE;

/* Keep canonical VESC numbering. The F103 backend intentionally implements
 * only NONE, CURRENT, CURRENT_NOREV_BRAKE_ADC, DUTY and PID(speed). */
typedef enum {
    ADC_CTRL_TYPE_NONE = 0,
    ADC_CTRL_TYPE_CURRENT,
    ADC_CTRL_TYPE_CURRENT_REV_CENTER,
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON,
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC,
    ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER,
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER,
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON,
    ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC,
    ADC_CTRL_TYPE_DUTY,
    ADC_CTRL_TYPE_DUTY_REV_CENTER,
    ADC_CTRL_TYPE_DUTY_REV_BUTTON,
    ADC_CTRL_TYPE_PID,
    ADC_CTRL_TYPE_PID_REV_CENTER,
    ADC_CTRL_TYPE_PID_REV_BUTTON
} adc_control_type;

typedef struct {
    // Variabel ctrl_type: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    adc_control_type ctrl_type;
    // Variabel hyst: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float hyst;
    // Variabel voltage_start: nilai tegangan untuk pengukuran atau kendali.
    float voltage_start;
    // Variabel voltage_end: nilai tegangan untuk pengukuran atau kendali.
    float voltage_end;
    // Variabel voltage_min: batas atau nilai minimum untuk validasi dan proteksi.
    float voltage_min;
    // Variabel voltage_max: batas atau nilai maksimum untuk validasi dan proteksi.
    float voltage_max;
    // Variabel voltage_center: nilai tegangan untuk pengukuran atau kendali.
    float voltage_center;
    // Variabel voltage2_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float voltage2_start;
    // Variabel voltage2_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float voltage2_end;
    // Variabel use_filter: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool use_filter;
    // Variabel safe_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    SAFE_START_MODE safe_start;
    // Variabel buttons: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t buttons;
    // Variabel voltage_inverted: nilai tegangan untuk pengukuran atau kendali.
    bool voltage_inverted;
    // Variabel voltage2_inverted: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool voltage2_inverted;
    // Variabel throttle_exp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float throttle_exp;
    // Variabel throttle_exp_brake: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float throttle_exp_brake;
    // Variabel throttle_exp_mode: mode operasi yang menentukan jalur algoritma aktif.
    thr_exp_mode throttle_exp_mode;
    // Variabel ramp_time_pos: nilai waktu untuk penjadwalan atau pengawasan.
    float ramp_time_pos;
    // Variabel ramp_time_neg: nilai waktu untuk penjadwalan atau pengawasan.
    float ramp_time_neg;
    // Variabel multi_esc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool multi_esc;
    // Variabel tc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool tc;
    // Variabel tc_max_diff: batas atau nilai maksimum untuk validasi dan proteksi.
    float tc_max_diff;
    // Variabel update_rate_hz: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t update_rate_hz;
} adc_config;

/* Reduced canonical app_configuration view. The full VESC-6 493-byte wire
 * image remains the source of truth in confgenerator.c. APP ADC fields are now
 * represented because PA2/PA3 have a real backend; unsupported PPM/NRF/etc.
 * fields remain byte-preserved and immutable. */
typedef struct {
    // Variabel controller_id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    uint8_t controller_id;
    // Variabel timeout_msec: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    uint32_t timeout_msec;
    // Variabel timeout_brake_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float timeout_brake_current;
    // Variabel permanent_uart_enabled: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool permanent_uart_enabled;
    // Variabel app_to_use: state atau konfigurasi aplikasi VESC.
    app_use app_to_use;
    // Variabel app_adc_conf: nilai atau state ADC pada jalur pengukuran.
    adc_config app_adc_conf;
    // Variabel app_uart_baudrate: state atau konfigurasi aplikasi VESC.
    uint32_t app_uart_baudrate;
    // Variabel crc: nilai CRC untuk memeriksa integritas data.
    uint16_t crc;
} app_configuration;

/* Canonical VESC command IDs. Keeping the enum complete does not enable
 * unsupported subsystems; commands.c explicitly implements only commands
 * with a real backend on this hardware. */
typedef enum {
    COMM_FW_VERSION = 0, COMM_JUMP_TO_BOOTLOADER = 1, COMM_ERASE_NEW_APP = 2,
    COMM_WRITE_NEW_APP_DATA = 3, COMM_GET_VALUES = 4, COMM_SET_DUTY = 5,
    COMM_SET_CURRENT = 6, COMM_SET_CURRENT_BRAKE = 7, COMM_SET_RPM = 8,
    COMM_SET_POS = 9, COMM_SET_HANDBRAKE = 10, COMM_SET_DETECT = 11,
    COMM_SET_SERVO_POS = 12, COMM_SET_MCCONF = 13, COMM_GET_MCCONF = 14,
    COMM_GET_MCCONF_DEFAULT = 15, COMM_SET_APPCONF = 16, COMM_GET_APPCONF = 17,
    COMM_GET_APPCONF_DEFAULT = 18, COMM_SAMPLE_PRINT = 19, COMM_TERMINAL_CMD = 20,
    COMM_PRINT = 21, COMM_ROTOR_POSITION = 22, COMM_EXPERIMENT_SAMPLE = 23,
    COMM_DETECT_MOTOR_PARAM = 24, COMM_DETECT_MOTOR_R_L = 25,
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26, COMM_DETECT_ENCODER = 27,
    COMM_DETECT_HALL_FOC = 28, COMM_REBOOT = 29, COMM_ALIVE = 30,
    COMM_GET_DECODED_PPM = 31, COMM_GET_DECODED_ADC = 32, COMM_GET_DECODED_CHUK = 33,
    COMM_FORWARD_CAN = 34, COMM_SET_CHUCK_DATA = 35, COMM_CUSTOM_APP_DATA = 36,
    COMM_NRF_START_PAIRING = 37, COMM_GPD_SET_FSW = 38, COMM_GPD_BUFFER_NOTIFY = 39,
    COMM_GPD_BUFFER_SIZE_LEFT = 40, COMM_GPD_FILL_BUFFER = 41, COMM_GPD_OUTPUT_SAMPLE = 42,
    COMM_GPD_SET_MODE = 43, COMM_GPD_FILL_BUFFER_INT8 = 44, COMM_GPD_FILL_BUFFER_INT16 = 45,
    COMM_GPD_SET_BUFFER_INT_SCALE = 46, COMM_GET_VALUES_SETUP = 47,
    COMM_SET_MCCONF_TEMP = 48, COMM_SET_MCCONF_TEMP_SETUP = 49,
    COMM_GET_VALUES_SELECTIVE = 50, COMM_GET_VALUES_SETUP_SELECTIVE = 51,
    COMM_EXT_NRF_PRESENT = 52, COMM_EXT_NRF_ESB_SET_CH_ADDR = 53,
    COMM_EXT_NRF_ESB_SEND_DATA = 54, COMM_EXT_NRF_ESB_RX_DATA = 55,
    COMM_EXT_NRF_SET_ENABLED = 56, COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57,
    COMM_DETECT_APPLY_ALL_FOC = 58, COMM_JUMP_TO_BOOTLOADER_ALL_CAN = 59,
    COMM_ERASE_NEW_APP_ALL_CAN = 60, COMM_WRITE_NEW_APP_DATA_ALL_CAN = 61,
    COMM_PING_CAN = 62, COMM_APP_DISABLE_OUTPUT = 63, COMM_TERMINAL_CMD_SYNC = 64,
    COMM_GET_IMU_DATA = 65, COMM_BM_CONNECT = 66, COMM_BM_ERASE_FLASH_ALL = 67,
    COMM_BM_WRITE_FLASH = 68, COMM_BM_REBOOT = 69, COMM_BM_DISCONNECT = 70,
    COMM_BM_MAP_PINS_DEFAULT = 71, COMM_BM_MAP_PINS_NRF5X = 72, COMM_ERASE_BOOTLOADER = 73,
    COMM_ERASE_BOOTLOADER_ALL_CAN = 74, COMM_PLOT_INIT = 75, COMM_PLOT_DATA = 76,
    COMM_PLOT_ADD_GRAPH = 77, COMM_PLOT_SET_GRAPH = 78, COMM_GET_DECODED_BALANCE = 79,
    COMM_BM_MEM_READ = 80, COMM_WRITE_NEW_APP_DATA_LZO = 81,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO = 82, COMM_BM_WRITE_FLASH_LZO = 83,
    COMM_SET_CURRENT_REL = 84, COMM_CAN_FWD_FRAME = 85, COMM_SET_BATTERY_CUT = 86,
    COMM_SET_BLE_NAME = 87, COMM_SET_BLE_PIN = 88, COMM_SET_CAN_MODE = 89,
    COMM_GET_IMU_CALIBRATION = 90, COMM_GET_MCCONF_TEMP = 91,
    COMM_GET_CUSTOM_CONFIG_XML = 92, COMM_GET_CUSTOM_CONFIG = 93,
    COMM_GET_CUSTOM_CONFIG_DEFAULT = 94, COMM_SET_CUSTOM_CONFIG = 95,
    COMM_BMS_GET_VALUES = 96, COMM_BMS_SET_CHARGE_ALLOWED = 97,
    COMM_BMS_SET_BALANCE_OVERRIDE = 98, COMM_BMS_RESET_COUNTERS = 99,
    COMM_BMS_FORCE_BALANCE = 100, COMM_BMS_ZERO_CURRENT_OFFSET = 101,
    COMM_JUMP_TO_BOOTLOADER_HW = 102, COMM_ERASE_NEW_APP_HW = 103,
    COMM_WRITE_NEW_APP_DATA_HW = 104, COMM_ERASE_BOOTLOADER_HW = 105,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW = 106, COMM_ERASE_NEW_APP_ALL_CAN_HW = 107,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW = 108, COMM_ERASE_BOOTLOADER_ALL_CAN_HW = 109,
    COMM_SET_ODOMETER = 110, COMM_PSW_GET_STATUS = 111, COMM_PSW_SWITCH = 112,
    COMM_BMS_FWD_CAN_RX = 113, COMM_BMS_HW_DATA = 114, COMM_GET_BATTERY_CUT = 115,
    COMM_BM_HALT_REQ = 116, COMM_GET_QML_UI_HW = 117, COMM_GET_QML_UI_APP = 118,
    COMM_CUSTOM_HW_DATA = 119, COMM_QMLUI_ERASE = 120, COMM_QMLUI_WRITE = 121,
    COMM_IO_BOARD_GET_ALL = 122, COMM_IO_BOARD_SET_PWM = 123, COMM_IO_BOARD_SET_DIGITAL = 124,
    COMM_BM_MEM_WRITE = 125, COMM_BMS_BLNC_SELFTEST = 126, COMM_GET_EXT_HUM_TMP = 127,
    COMM_GET_STATS = 128, COMM_RESET_STATS = 129, COMM_LISP_READ_CODE = 130,
    COMM_LISP_WRITE_CODE = 131, COMM_LISP_ERASE_CODE = 132, COMM_LISP_SET_RUNNING = 133,
    COMM_LISP_GET_STATS = 134, COMM_LISP_PRINT = 135, COMM_BMS_SET_BATT_TYPE = 136,
    COMM_BMS_GET_BATT_TYPE = 137, COMM_LISP_REPL_CMD = 138, COMM_LISP_STREAM_CODE = 139,
    COMM_FILE_LIST = 140, COMM_FILE_READ = 141, COMM_FILE_WRITE = 142, COMM_FILE_MKDIR = 143,
    COMM_FILE_REMOVE = 144, COMM_LOG_START = 145, COMM_LOG_STOP = 146,
    COMM_LOG_CONFIG_FIELD = 147, COMM_LOG_DATA_F32 = 148, COMM_SET_APPCONF_NO_STORE = 149,
    COMM_GET_GNSS = 150, COMM_LOG_DATA_F64 = 151, COMM_LISP_RMSG = 152,
    COMM_SHUTDOWN = 156, COMM_FW_INFO = 157, COMM_CAN_UPDATE_BAUD_ALL = 158,
    COMM_MOTOR_ESTOP = 159
} COMM_PACKET_ID;

typedef struct {
    // Variabel hdop: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel height: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel lat: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel lon: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel speed: nilai kecepatan untuk target atau pengukuran.
    double lat, lon, height, speed, hdop;
    // Variabel ms_today: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t ms_today;
    // Variabel dd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel mo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel yy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int16_t yy, mo, dd;
} gnss_data;
typedef enum {
    SENSOR_DETECT_IDLE = 0,
    SENSOR_DETECT_PREPARE,
    SENSOR_DETECT_HALL_LOCK,
    SENSOR_DETECT_HALL_FWD,
    SENSOR_DETECT_HALL_REV,
    SENSOR_DETECT_HALL_EVAL,
    SENSOR_DETECT_ENCODER_PREP,
    SENSOR_DETECT_ENCODER_LOCK0,
    SENSOR_DETECT_ENCODER_SWEEP,
    SENSOR_DETECT_ENCODER_EVAL,
    SENSOR_DETECT_ENCODER_RETURN0,
    SENSOR_DETECT_ENCODER_ALIGN,
    SENSOR_DETECT_DONE,
    SENSOR_DETECT_FAILED
} sensor_detect_state_t;

typedef enum {
    DISP_POS_MODE_NONE = 0,
    DISP_POS_MODE_INDUCTANCE = 1,
    DISP_POS_MODE_OBSERVER = 2,
    DISP_POS_MODE_ENCODER = 3,
    DISP_POS_MODE_PID_POS = 4,
    DISP_POS_MODE_PID_POS_ERROR = 5,
    DISP_POS_MODE_ENCODER_OBSERVER_ERROR = 6,
    DISP_POS_MODE_HALL_OBSERVER_ERROR = 7
} disp_pos_mode_t;

typedef struct {
    // Variabel base_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t base_phase_u16;
    // Variabel phase_per_cycle_q16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile int32_t phase_per_cycle_q16;
    // Variabel edge_cycle: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t edge_cycle;
    // Variabel period_cycles: periode pengukuran atau eksekusi modul.
    volatile uint32_t period_cycles;
    // Variabel direction: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int8_t direction;
    // Variabel sector: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int8_t sector;
    // Variabel raw_state: nilai mentah sebelum konversi ke satuan fisik.
    volatile uint8_t raw_state;
    // Variabel valid: penanda validitas hasil pengukuran atau konfigurasi.
    volatile bool valid;
    // Variabel invalid_count: pencacah kejadian atau sampel.
    volatile uint16_t invalid_count;
    // Variabel sequence_error_count: pencacah kejadian atau sampel.
    volatile uint16_t sequence_error_count;
    // Variabel recovery_valid_ticks: penanda validitas hasil pengukuran atau konfigurasi.
    volatile uint16_t recovery_valid_ticks;
    // Variabel edge_count: pencacah kejadian atau sampel.
    volatile int32_t edge_count;
    /* Explicit Hall electrical-angle slew limiter, matching current VESC's
       rate-limited Hall phase while remaining integer-only in the 16-kHz path. */
    // Variabel rate_limited_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t rate_limited_phase_u16;
    // Variabel rate_limited_valid: penanda validitas hasil pengukuran atau konfigurasi.
    volatile bool rate_limited_valid;
    // Variabel rate_limit_frame: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t rate_limit_frame;
} hall_state_t;

typedef struct {
    // Variabel turns: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int32_t turns;
    // Variabel last_cnt: pencacah kejadian atau sampel.
    volatile uint16_t last_cnt;
    // Variabel extended_count: pencacah kejadian atau sampel.
    volatile int32_t extended_count;
    // Variabel prev_extended_count: pencacah kejadian atau sampel.
    volatile int32_t prev_extended_count;
    // Variabel speed_sample_valid: nilai kecepatan untuk target atau pengukuran.
    volatile bool speed_sample_valid;
    /* Incremental AB has no absolute phase after reset. session_zero_count is
       rebuilt from the sensorless observer via encoder_set_deg() after the
       VESC-style open-loop startup reaches a trustworthy observer speed. */
    // Variabel session_zero_count: pencacah kejadian atau sampel.
    volatile int32_t session_zero_count;
    // Variabel mechanical_zero_count: pencacah kejadian atau sampel.
    volatile int32_t mechanical_zero_count;
    // Variabel synced: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool synced;
    // Variabel motion_proved: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool motion_proved;
    // Variabel sync_active: penanda bahwa state atau fitur sedang aktif.
    volatile bool sync_active;
    // Variabel sync_start_tick: nilai tick scheduler untuk pengukuran waktu.
    volatile uint32_t sync_start_tick;
    // Variabel cpr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t cpr;
    /* Electrical revolutions per one mechanical encoder revolution.
       Kept separate from physical motor pole_pairs as in VESC mcconf.
       The float is task/config-side; Q16.16 and phase-per-count are the ISR
       representations, so non-integer VESC encoder ratios remain supported. */
    // Variabel electrical_ratio: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float electrical_ratio;
    // Variabel electrical_ratio_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t electrical_ratio_q16;
    // Variabel phase_per_count_q16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint32_t phase_per_count_q16;
    // Variabel elec_offset_u16: offset kalibrasi untuk mengoreksi bias pengukuran.
    uint16_t elec_offset_u16;
    // Variabel inverted: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool inverted;
} encoder_state_t;

typedef struct {
    // Variabel kp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float kp;
    // Variabel ki: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ki;
    // Variabel kd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float kd;
    // Variabel integrator: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float integrator;
    // Variabel prev_error: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float prev_error;
} pid_state_t;

#define FOC_DETECT_L_CAPTURE_MAX 32U

typedef struct {
    // Variabel state: state mesin keadaan yang menentukan tahap operasi.
    volatile sensor_detect_state_t state;
    // Variabel requested: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool requested;
    // Variabel busy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool busy;
    // Variabel success: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool success;
    // Variabel result_mode: mode operasi yang menentukan jalur algoritma aktif.
    volatile uint8_t result_mode;
    // Variabel apply_result: hasil sementara atau akhir suatu operasi.
    volatile bool apply_result;
    // Variabel drive_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float drive_current_a;
    // Variabel step_tick: nilai tick scheduler untuk pengukuran waktu.
    volatile uint32_t step_tick;
    // Variabel sweep_index: indeks elemen yang sedang diproses.
    volatile uint32_t sweep_index;
    // Variabel sweep_pass: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t sweep_pass;
    // Variabel forced_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t forced_phase_u16;
    // Variabel encoder_start_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    volatile int32_t encoder_start_count;
    // Variabel encoder_end_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    volatile int32_t encoder_end_count;
    // Variabel hall_valid_states: data sensor Hall untuk menentukan sektor atau posisi rotor.
    volatile uint8_t hall_valid_states;

    /* Detection is a temporary motor-control transaction. Preserve everything
       it changes so standard VESC COMM_DETECT_* can restore the previous
       configuration after returning its result. */
    // Variabel saved_sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t saved_sensor_mode;
    // Variabel saved_sensor_request_mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t saved_sensor_request_mode;
    // Variabel saved_current_kp: nilai arus untuk pengukuran, kendali, atau proteksi.
    float saved_current_kp;
    // Variabel saved_current_ki: nilai arus untuk pengukuran, kendali, atau proteksi.
    float saved_current_ki;
    // Variabel saved_timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    uint32_t saved_timeout_ms;
    // Variabel saved_timeout_brake_a: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    float saved_timeout_brake_a;

    /* LEFT Hall and A/B encoder share PB6/PB7. A temporary Hall detect must
       restore the incremental coordinate when TIM4 ownership returns to the
       encoder. Hall detection performs equal forward/reverse electrical sweeps,
       so the expected final shaft coordinate is the transaction entry point. */
    // Variabel saved_encoder_hw_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    uint16_t saved_encoder_hw_count;
    // Variabel saved_encoder_turns: data encoder untuk pengukuran posisi atau kecepatan rotor.
    int32_t saved_encoder_turns;
    // Variabel saved_encoder_last_cnt: data encoder untuk pengukuran posisi atau kecepatan rotor.
    uint16_t saved_encoder_last_cnt;
    // Variabel saved_encoder_extended_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    int32_t saved_encoder_extended_count;
    // Variabel saved_encoder_prev_extended_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    int32_t saved_encoder_prev_extended_count;
    // Variabel saved_encoder_session_zero_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    int32_t saved_encoder_session_zero_count;
    // Variabel saved_encoder_mechanical_zero_count: data encoder untuk pengukuran posisi atau kecepatan rotor.
    int32_t saved_encoder_mechanical_zero_count;
    // Variabel saved_encoder_speed_sample_valid: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool saved_encoder_speed_sample_valid;
    // Variabel saved_encoder_synced: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool saved_encoder_synced;
    // Variabel saved_encoder_motion_proved: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool saved_encoder_motion_proved;
    // Variabel saved_encoder_sync_active: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool saved_encoder_sync_active;
    // Variabel saved_encoder_sync_start_tick: data encoder untuk pengukuran posisi atau kecepatan rotor.
    uint32_t saved_encoder_sync_start_tick;
    // Variabel saved_encoder_coordinate_valid: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool saved_encoder_coordinate_valid;

    /* Result lives separately from active runtime config. This is required by
       VESC semantics: detect returns parameters; the Tool decides whether to
       apply/write them afterwards. */
    // Variabel result_hall_table: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint8_t result_hall_table[8];
    // Variabel result_encoder_offset_deg: offset kalibrasi untuk mengoreksi bias pengukuran.
    float result_encoder_offset_deg;
    // Variabel result_encoder_ratio: data encoder untuk pengukuran posisi atau kecepatan rotor.
    float result_encoder_ratio;
    // Variabel result_encoder_inverted: data encoder untuk pengukuran posisi atau kecepatan rotor.
    bool result_encoder_inverted;

    // Variabel hall_sin_sum: data sensor Hall untuk menentukan sektor atau posisi rotor.
    int64_t hall_sin_sum[8];
    // Variabel hall_cos_sum: data sensor Hall untuk menentukan sektor atau posisi rotor.
    int64_t hall_cos_sum[8];
    // Variabel hall_samples: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint32_t hall_samples[8];

    /* PWM-rate inductance step capture. The task selects d or q before arming;
       the hard ISR stores only that axis current and the causally preceding
       applied axis voltage. The blocking detect task estimates L afterwards. */
    // Variabel l_capture_active: penanda bahwa state atau fitur sedang aktif.
    volatile bool l_capture_active;
    // Variabel l_capture_done: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool l_capture_done;
    // Variabel l_capture_axis: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t l_capture_axis; /* 0=d, 1=q */
    // Variabel l_capture_count: pencacah kejadian atau sampel.
    volatile uint16_t l_capture_count;
    // Variabel l_capture_i_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t l_capture_i_q15[FOC_DETECT_L_CAPTURE_MAX];
    // Variabel l_capture_v_prev_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t l_capture_v_prev_q15[FOC_DETECT_L_CAPTURE_MAX];
} foc_detect_state_t;

typedef struct {
    // Variabel amp_hours: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float amp_hours;
    // Variabel amp_hours_charged: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float amp_hours_charged;
    // Variabel watt_hours: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float watt_hours;
    // Variabel watt_hours_charged: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float watt_hours_charged;
    // Variabel tachometer: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t tachometer;
    // Variabel tachometer_abs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t tachometer_abs;
    // Variabel tachometer_last: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t tachometer_last;
    // Variabel tachometer_source_valid: penanda validitas hasil pengukuran atau konfigurasi.
    bool tachometer_source_valid;
    // Variabel max_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float max_current;
    // Variabel max_input_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float max_input_current;
    // Variabel max_erpm: kecepatan listrik motor dalam electrical RPM.
    float max_erpm;
    // Variabel runtime_ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t runtime_ms;
} motor_stats_t;

typedef struct {
    /* Coherent data published by one FOC ISR pass. All values use the same
       fixed-point bases as MotorRuntime fast fields. */
    // Variabel ia_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ib_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ic_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ia_q15, ib_q15, ic_q15;
    // Variabel id_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    // Variabel iq_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t id_q15, iq_q15;
    // Variabel id_filter_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    // Variabel iq_filter_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t id_filter_q15, iq_filter_q15;
    // Variabel id_target_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    // Variabel iq_target_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t id_target_q15, iq_target_q15;
    // Variabel vd_q15: tegangan sumbu-d keluaran regulator FOC.
    // Variabel vq_q15: tegangan sumbu-q keluaran regulator FOC.
    int32_t vd_q15, vq_q15;
    // Variabel dc_current_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    // Variabel vbus_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    int32_t vbus_q15, dc_current_q15;
    // Variabel erpm_fast_q16: kecepatan listrik motor dalam electrical RPM.
    int32_t erpm_fast_q16;
    // Variabel duty_u_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    // Variabel duty_v_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    // Variabel duty_w_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    uint16_t duty_u_q15, duty_v_q15, duty_w_q15;
    // Variabel phase_control_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    // Variabel phase_observer_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint16_t phase_control_u16, phase_observer_u16;
    // Variabel phase_encoder_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    // Variabel phase_hall_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint16_t phase_encoder_u16, phase_hall_u16;
    // Variabel adc_frame: nilai atau state ADC pada jalur pengukuran.
    uint32_t adc_frame;
    // Variabel cycle_counter: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t cycle_counter;
} foc_rt_snapshot_t;

typedef struct MotorRuntime {
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    motor_id_t id;
    // Variabel pwm_tim: state atau nilai PWM untuk pengendalian inverter.
    TIM_TypeDef *pwm_tim;
    // Variabel pole_pairs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t pole_pairs;

    /* VESC backend selection from MCCONF. */
    // Variabel motor_type: state atau parameter motor yang sedang diproses.
    mc_motor_type motor_type;
    // Variabel pwm_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_pwm_mode pwm_mode;
    // Variabel comm_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_comm_mode comm_mode;
    /* VESC FOC angle-source strategy. Keep this distinct from sensor_mode,
       which only describes the physical Hall/ABI peripheral currently
       configured on this STM32F103 board. */
    // Variabel foc_sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    mc_foc_sensor_mode foc_sensor_mode;
    // Variabel state: state mesin keadaan yang menentukan tahap operasi.
    volatile mc_state state;


    /* Runtime-selected sensor mode. 0=AUTO, 1=Hall, 2=Encoder. */
    // Variabel sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    volatile uint8_t sensor_mode;
    // Variabel sensor_request_mode: mode operasi yang menentukan jalur algoritma aktif.
    volatile uint8_t sensor_request_mode;

    // Variabel control_mode: mode operasi yang menentukan jalur algoritma aktif.
    volatile motor_control_mode_t control_mode;
    // Variabel fault: status atau data gangguan untuk sistem proteksi.
    volatile motor_fault_t fault;
    // Variabel last_fault: fault terakhir yang tersimpan untuk diagnostik walau fault aktif sudah pulih.
    volatile motor_fault_t last_fault;
    // Variabel fault_recovery_ticks: menghitung 1-ms kondisi sehat setelah fault sebelum auto-clear.
    volatile uint16_t fault_recovery_ticks;
    // Variabel pwm_enabled: state atau nilai PWM untuk pengendalian inverter.
    volatile bool pwm_enabled;
    /* Short zero-vector blanking window after MOE is asserted. During this
       window the ISR holds 50%/50%/50% and only gross DC-current safety is
       active; phase-current PI/ABS fault starts after the analog front-end
       has settled. */
    // Variabel pwm_enable_blank_cycles: state atau nilai PWM untuk pengendalian inverter.
    volatile uint16_t pwm_enable_blank_cycles;
    /* MOE is asserted only from the ADC/DMA ISR after the 50% preload has
       crossed at least two hardware update boundaries. This prevents a stale
       CCR triplet from being exposed for a partial PWM cycle on re-enable. */
    // Variabel pwm_enable_pending_events: state atau nilai PWM untuk pengendalian inverter.
    volatile uint8_t pwm_enable_pending_events;
    /* VESC-style full low-side brake state used only for an exact zero vector.
       Default is disabled until the hoverboard gate-driver bootstrap behavior
       has been verified on real hardware. */
    // Variabel foc_short_ls_on_zero_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile bool foc_short_ls_on_zero_duty;
    // Variabel full_brake_active: penanda bahwa state atau fitur sedang aktif.
    volatile bool full_brake_active;
    // Variabel command_active: penanda bahwa state atau fitur sedang aktif.
    volatile bool command_active;
    // Variabel timeout_active: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    volatile bool timeout_active;
    // Variabel detect_force_angle: nilai sudut untuk posisi atau transformasi koordinat.
    volatile bool detect_force_angle;
    // Variabel detect_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t detect_phase_u16;

    // Variabel id_target: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile float id_target;
    // Variabel iq_target: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile float iq_target;
    // Variabel id_target_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile int32_t id_target_q15;
    // Variabel iq_target_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile int32_t iq_target_q15;
    /* Slow-loop torque/MTPA request before fast field weakening. The 16-kHz
       FOC ISR derives the effective id/iq targets from these values. Direct
       detection/open-loop helpers still initialize both base and effective
       targets atomically through motor_set_foc_targets(). */
    // Variabel id_target_base_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile int32_t id_target_base_q15;
    // Variabel iq_target_base_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile int32_t iq_target_base_q15;
    // Variabel current_command_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_command_a;
    // Variabel brake_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float brake_current_a;
    // Variabel handbrake_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float handbrake_current_a;
    // Variabel fw_override_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float fw_override_current_a;
    // Variabel current_off_delay_s: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_off_delay_s;
    // Variabel duty_command: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_command;
    // Variabel duty_command_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile int32_t duty_command_q15;
    // Variabel speed_target_erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float speed_target_erpm;
    // Variabel speed_pid_set_erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float speed_pid_set_erpm;
    // Variabel position_target_deg: nilai posisi rotor atau aktuator.
    volatile float position_target_deg;
    /* Explicit VESC FOC open-loop controls (not sensorless-startup state). */
    // Variabel openloop_command_erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float openloop_command_erpm;
    // Variabel openloop_command_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t openloop_command_phase_u16;

    /* Runtime limits/configuration applied from VESC MCCONF. */
    // Variabel current_max_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_max_a;
    // Variabel current_min_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_min_a;
    // Variabel input_current_max_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current_max_a;
    // Variabel input_current_min_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current_min_a;
    // Variabel current_max_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_max_scale;
    // Variabel current_min_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_min_scale;
    // Variabel watt_max: batas atau nilai maksimum untuk validasi dan proteksi.
    volatile float watt_max;
    // Variabel watt_min: batas atau nilai minimum untuk validasi dan proteksi.
    volatile float watt_min;
    // Variabel duty_start: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_start;
    // Variabel lo_current_max_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float lo_current_max_a;
    // Variabel lo_current_min_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float lo_current_min_a;
    // Variabel lo_input_current_max_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float lo_input_current_max_a;
    // Variabel lo_input_current_min_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float lo_input_current_min_a;
    // Variabel abs_current_max_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float abs_current_max_a;
    // Variabel slow_abs_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile bool slow_abs_current;
    /* The stock hoverboard has no validated MOSFET NTC. l_temp_fet_* is
       therefore applied to the STM32 internal-temperature board proxy. */
    // Variabel temp_fet_start: nilai sementara atau temperatur sesuai konteks modul.
    volatile float temp_fet_start;
    // Variabel temp_fet_end: nilai sementara atau temperatur sesuai konteks modul.
    volatile float temp_fet_end;
    // Variabel temp_motor_start: nilai sementara atau temperatur sesuai konteks modul.
    volatile float temp_motor_start;
    // Variabel temp_motor_end: nilai sementara atau temperatur sesuai konteks modul.
    volatile float temp_motor_end;
    // Variabel temp_accel_dec: nilai sementara atau temperatur sesuai konteks modul.
    volatile float temp_accel_dec;
    // Variabel additional_faults: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t additional_faults;
    // Variabel min_vin: batas atau nilai minimum untuk validasi dan proteksi.
    volatile float min_vin;
    // Variabel max_vin: batas atau nilai maksimum untuk validasi dan proteksi.
    volatile float max_vin;
    // Variabel battery_cut_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float battery_cut_start;
    // Variabel battery_cut_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float battery_cut_end;
    /* Runtime-only high-voltage regenerative-current taper. These fields are
     * intentionally not serialized while the protocol ABI is VESC 6.00. */
    // Variabel battery_regen_cut_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float battery_regen_cut_start;
    // Variabel battery_regen_cut_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float battery_regen_cut_end;
    // Variabel min_vin_q15: batas atau nilai minimum untuk validasi dan proteksi.
    volatile int32_t min_vin_q15;
    // Variabel max_vin_q15: batas atau nilai maksimum untuk validasi dan proteksi.
    volatile int32_t max_vin_q15;
    // Variabel hard_min_vin_q15: batas atau nilai minimum untuk validasi dan proteksi.
    volatile int32_t hard_min_vin_q15;
    // Variabel hard_max_vin_q15: batas atau nilai maksimum untuk validasi dan proteksi.
    volatile int32_t hard_max_vin_q15;
    // Variabel under_voltage_fault_count: status atau data gangguan untuk sistem proteksi.
    volatile uint8_t under_voltage_fault_count;
    // Variabel over_voltage_fault_count: status atau data gangguan untuk sistem proteksi.
    volatile uint8_t over_voltage_fault_count;
    // Variabel max_erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float max_erpm;
    // Variabel min_erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float min_erpm;
    // Variabel erpm_start: kecepatan listrik motor dalam electrical RPM.
    volatile float erpm_start;
    // Variabel erpm_fault_filter: kecepatan listrik motor dalam electrical RPM.
    volatile float erpm_fault_filter;
    // Variabel foc_start_curr_dec: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float foc_start_curr_dec;
    // Variabel foc_start_curr_dec_rpm: kecepatan putar yang digunakan oleh logika kendali.
    volatile float foc_start_curr_dec_rpm;
    // Variabel max_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float max_duty;
    // Variabel min_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float min_duty;
    /* Dynamic modulation ceiling. In ordinary current/speed/position modes it
       equals max_duty. VESC duty mode lowers it to the requested duty only
       after the down-ramp PI no longer has to reduce an already larger duty. */
    // Variabel duty_limit_now: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_limit_now;
    /* Precomputed voltage-circle coefficient:
       1/sqrt(3) * duty_limit_now * foc_overmod_factor, additionally capped by
       the physical 10..90% current-sampling duty window. */
    // Variabel vmax_coeff_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int32_t vmax_coeff_q15;
    // Variabel invert_direction: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool invert_direction;

    /* VESC 6.00 SI configuration used by GET_VALUES_SETUP. */
    // Variabel si_gear_ratio: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float si_gear_ratio;
    // Variabel si_wheel_diameter: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float si_wheel_diameter;
    // Variabel si_battery_type: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t si_battery_type;
    // Variabel si_battery_cells: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t si_battery_cells;
    // Variabel si_battery_ah: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float si_battery_ah;
    // Variabel si_motor_nl_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float si_motor_nl_current;

    // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float ia;
    // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float ib;
    // Variabel ic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float ic;
    // Variabel id_meas: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile float id_meas;
    // Variabel iq_meas: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile float iq_meas;
    // Variabel id_filter: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile float id_filter;
    // Variabel iq_filter: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile float iq_filter;
    // Variabel vd: tegangan sumbu-d keluaran regulator FOC.
    volatile float vd;
    // Variabel vq: tegangan sumbu-q keluaran regulator FOC.
    volatile float vq;
    // Variabel vd_filter: tegangan sumbu-d keluaran regulator FOC.
    volatile float vd_filter;
    // Variabel vq_filter: tegangan sumbu-q keluaran regulator FOC.
    volatile float vq_filter;
    // Variabel duty_u: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_u;
    // Variabel duty_v: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_v;
    // Variabel duty_w: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_w;
    // Variabel duty_now: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_now;
    // Variabel dc_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float dc_current_a;
    // Variabel dc_current_filter: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float dc_current_filter;
    // Variabel vbus: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    volatile float vbus;
    // Variabel vbus_filter: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    volatile float vbus_filter;
    // Variabel motor_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float motor_current;
    // Variabel input_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current;
    /* VESC-style pre-emptive input-current map. This uses the hoverboard's
       physical per-motor DC-current sensor, which is preferable to estimating
       battery current from Iq*duty. */
    // Variabel input_current_map_start: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current_map_start;
    // Variabel input_current_map_filter: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current_map_filter;
    // Variabel input_current_map_filtered_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current_map_filtered_a;
    // Variabel input_current_map_limit_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float input_current_map_limit_a;
    // Variabel board_temp_c: nilai sementara atau temperatur sesuai konteks modul.
    volatile float board_temp_c;
    // Variabel board_temp_filter_c: nilai sementara atau temperatur sesuai konteks modul.
    volatile float board_temp_filter_c;
    // Variabel board_temp_valid: nilai sementara atau temperatur sesuai konteks modul.
    volatile bool board_temp_valid;

    /* Hard real-time representations. */
    // Variabel ia_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int32_t ia_q15;
    // Variabel ib_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int32_t ib_q15;
    // Variabel ic_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile int32_t ic_q15;
    // Variabel id_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile int32_t id_q15;
    // Variabel iq_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile int32_t iq_q15;
    // Variabel id_filter_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile int32_t id_filter_q15;
    // Variabel iq_filter_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    volatile int32_t iq_filter_q15;
    // Variabel vd_q15: tegangan sumbu-d keluaran regulator FOC.
    volatile int32_t vd_q15;
    // Variabel vq_q15: tegangan sumbu-q keluaran regulator FOC.
    volatile int32_t vq_q15;
    // Variabel duty_u_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile uint16_t duty_u_q15;
    // Variabel duty_v_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile uint16_t duty_v_q15;
    // Variabel duty_w_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile uint16_t duty_w_q15;
    /* Shared ADC1/2 current sampling has one hardware-valid 10..90% window.
       These counters make modulation saturation visible during commissioning
       instead of pretending unsupported V0/V7/high-current sample modes work. */
    // Variabel sampling_window_clamp_count: pencacah kejadian atau sampel.
    volatile uint32_t sampling_window_clamp_count;
    // Variabel sampling_margin_min_q15: batas atau nilai minimum untuk validasi dan proteksi.
    volatile uint16_t sampling_margin_min_q15;
    // Variabel vbus_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    volatile int32_t vbus_q15;
    // Variabel abs_current_trip_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t abs_current_trip_q15;
    // Variabel abs_phase_current_filter_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t abs_phase_current_filter_q15;
    // Variabel abs_current_peak_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t abs_current_peak_q15;
    // Variabel abs_current_fault_count: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile uint8_t abs_current_fault_count;
    // Variabel dc_current_raw: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile uint16_t dc_current_raw;
    // Variabel dc_current_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t dc_current_q15;

    // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float erpm;
    // Variabel erpm_int: kecepatan listrik motor dalam electrical RPM.
    volatile int32_t erpm_int; /* slow-loop integer mirror for ISR debug capture */
    /* VESC-style low-latency electrical-speed estimators. They are updated
       directly from electrical phase deltas in the hard FOC ISR using Q16.16
       ERPM, so mcpwm_foc_get_rpm_fast/faster do not alias the slow 1-kHz RPM. */
    // Variabel speed_est_fast_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    volatile int32_t speed_est_fast_erpm_q16;
    // Variabel speed_est_faster_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    volatile int32_t speed_est_faster_erpm_q16;
    // Variabel phase_before_speed_est_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t phase_before_speed_est_u16;
    // Variabel speed_est_phase_valid: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile bool speed_est_phase_valid;
    // Variabel speed_est_fast_corrected_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    volatile int32_t speed_est_fast_corrected_erpm_q16;
    // Variabel phase_before_speed_est_corrected_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t phase_before_speed_est_corrected_u16;
    // Variabel speed_est_corrected_valid: nilai kecepatan untuk target atau pengukuran.
    volatile bool speed_est_corrected_valid;
    // Variabel foc_speed_source: nilai kecepatan untuk target atau pengukuran.
    FOC_SPEED_SRC foc_speed_source;
    // Variabel mech_rpm: kecepatan putar yang digunakan oleh logika kendali.
    volatile float mech_rpm;
    // Variabel position_deg: nilai posisi rotor atau aktuator.
    volatile float position_deg;
    // Variabel rotor_elec_deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float rotor_elec_deg;

    // Variabel current_kp: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_kp;
    // Variabel current_ki: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_ki;
    /* Cached temp-comp / offset-cal mode flags (mirrored from mc_configuration
       on every SET_MCCONF). The 1-kHz loop and 16-kHz ISR read these directly
       instead of dereferencing the configuration mirror each tick. */
    // Variabel foc_temp_comp: nilai sementara atau temperatur sesuai konteks modul.
    bool foc_temp_comp;
    // Variabel foc_temp_comp_base_temp: nilai sementara atau temperatur sesuai konteks modul.
    float foc_temp_comp_base_temp;
    // Variabel foc_offsets_cal_mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t foc_offsets_cal_mode;
    /* Cached mirror of mc_configuration.foc_calibrate_on_boot. The 1-kHz loop
       and boot skip logic read this directly instead of dereferencing the
       configuration mirror each tick. */
    // Variabel foc_calibrate_on_boot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool foc_calibrate_on_boot;
    /* VESC-style temperature-compensated R and Ki. The F103 port has no motor
       NTC, so the STM32 board-temperature proxy (board_temp_filter_c) is the
       thermal input. comp_factor = 1 + 0.00386*(T - base_temp); these hold
       foc_motor_r*comp and current_ki*comp, consumed by the observer/PI. */
    // Variabel res_temp_comp_ohm: nilai sementara atau temperatur sesuai konteks modul.
    volatile float res_temp_comp_ohm;
    // Variabel current_ki_temp_comp: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float current_ki_temp_comp;
    // Variabel current_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_scale;
    // Variabel dc_current_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    float dc_current_scale;
    // Variabel current_offset_u: offset kalibrasi untuk mengoreksi bias pengukuran.
    volatile float current_offset_u;
    // Variabel current_offset_v: offset kalibrasi untuk mengoreksi bias pengukuran.
    volatile float current_offset_v;
    // Variabel dc_current_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    volatile float dc_current_offset;
    // Variabel vd_int: tegangan sumbu-d keluaran regulator FOC.
    float vd_int;
    // Variabel vq_int: tegangan sumbu-q keluaran regulator FOC.
    float vq_int;

    // Variabel current_scale_q16: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t current_scale_q16;
    // Variabel dc_current_scale_q16: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t dc_current_scale_q16;
    /* PI coefficients are Q16.16; current/voltage samples remain Q15.
       Q15 error * Q16 gain >> 16 gives a Q15 voltage contribution. */
    // Variabel current_kp_q16: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t current_kp_q16;
    // Variabel current_ki_dt_q16: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t current_ki_dt_q16;
    /* Fixed-point dq feed-forward coefficients. Current is Q15 on
       FOC_CURRENT_Q_BASE_A, voltage Q15 on FOC_VOLTAGE_Q_BASE_V and electrical
       speed Q16.16 ERPM. These are precomputed task-side from MCCONF. */
    // Variabel decouple_ld_coeff_q30: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t decouple_ld_coeff_q30;
    // Variabel decouple_lq_coeff_q30: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t decouple_lq_coeff_q30;
    // Variabel bemf_flux_coeff_q30: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t bemf_flux_coeff_q30;
    /* VESC observer phase compensation factor (0.5 + foc_observer_offset)
       in signed Q15. It is consumed only by the fixed-point phase selector. */
    // Variabel observer_offset_factor_q15: offset kalibrasi untuk mengoreksi bias pengukuran.
    int32_t observer_offset_factor_q15;
    // Variabel foc_mag_vd_max_q15: tegangan sumbu-d keluaran regulator FOC.
    int32_t foc_mag_vd_max_q15;
    // Variabel current_offset_u_counts: offset kalibrasi untuk mengoreksi bias pengukuran.
    volatile int32_t current_offset_u_counts;
    // Variabel current_offset_v_counts: offset kalibrasi untuk mengoreksi bias pengukuran.
    volatile int32_t current_offset_v_counts;
    // Variabel dc_current_offset_counts: offset kalibrasi untuk mengoreksi bias pengukuran.
    volatile int32_t dc_current_offset_counts;
    // Variabel current_raw_u: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile uint16_t current_raw_u;
    // Variabel current_raw_v: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile uint16_t current_raw_v;
    /* Integral state keeps 16 fractional bits below Q15 (Q31 accumulator).
       This avoids losing small Ki updates at 16 kHz on Cortex-M3. */
    // Variabel vd_int_q31: tegangan sumbu-d keluaran regulator FOC.
    volatile int64_t vd_int_q31;
    // Variabel vq_int_q31: tegangan sumbu-q keluaran regulator FOC.
    volatile int64_t vq_int_q31;
    // Variabel vd_int_q15: tegangan sumbu-d keluaran regulator FOC.
    volatile int32_t vd_int_q15; /* telemetry/debug mirror */
    // Variabel vq_int_q15: tegangan sumbu-q keluaran regulator FOC.
    volatile int32_t vq_int_q15;

    // Variabel hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
    hall_state_t hall;
    // Variabel hall_table: data sensor Hall untuk menentukan sektor atau posisi rotor.
    int8_t hall_table[8];
    // Variabel foc_hall_table: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint8_t foc_hall_table[8]; /* VESC style: 0..200, invalid=255 */
    // Variabel hall_angle_u16: nilai sudut untuk posisi atau transformasi koordinat.
    uint16_t hall_angle_u16[8];
    // Variabel hall_offset_u16: offset kalibrasi untuk mengoreksi bias pengukuran.
    uint16_t hall_offset_u16;
    // Variabel encoder: data encoder untuk pengukuran posisi atau kecepatan rotor.
    encoder_state_t encoder;

    /* VESC-style FOC motor model and sensorless observer. HFI is deliberately
       not implemented on this STM32F103 port. Values mirror mc_configuration
       fields and are persisted through the canonical MCCONF wire image. */
    // Variabel foc_motor_r: state atau parameter motor yang sedang diproses.
    float foc_motor_r;
    // Variabel foc_motor_l: state atau parameter motor yang sedang diproses.
    float foc_motor_l;
    // Variabel foc_motor_ld_lq_diff: state atau parameter motor yang sedang diproses.
    float foc_motor_ld_lq_diff;
    // Variabel foc_motor_flux_linkage: state atau parameter motor yang sedang diproses.
    float foc_motor_flux_linkage;
    /* Online VESC-style motor-resistance estimate. It is diagnostic/adaptive
       state only; the observer keeps configured R unless a separate validated
       compensation policy is enabled. */
    // Variabel res_est_ohm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float res_est_ohm;
    // Variabel res_est_state_ohm: state mesin keadaan yang menentukan tahap operasi.
    volatile float res_est_state_ohm;
    // Variabel res_est_valid: penanda validitas hasil pengukuran atau konfigurasi.
    volatile bool res_est_valid;
    /* Configured VESC dead-time compensation in microseconds. The derived
       Q15 fraction is foc_dt_us * FOC ISR frequency and is used only by the
       fixed-point applied-voltage model; hardware timer dead-time is unchanged. */
    // Variabel foc_dt_us: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_dt_us;
    // Variabel deadtime_comp_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t deadtime_comp_q15;
    // Variabel foc_observer_gain: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    float foc_observer_gain;
    // Variabel foc_observer_gain_slow: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    float foc_observer_gain_slow;
    // Variabel foc_observer_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    float foc_observer_offset;
    // Variabel foc_sat_comp_mode: mode operasi yang menentukan jalur algoritma aktif.
    SAT_COMP_MODE foc_sat_comp_mode;
    // Variabel foc_sat_comp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_sat_comp;
    // Variabel foc_observer_type: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    mc_foc_observer_type foc_observer_type;
    // Variabel foc_current_filter_const: nilai arus untuk pengukuran, kendali, atau proteksi.
    float foc_current_filter_const;
    // Variabel foc_current_filter_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t foc_current_filter_q15;
    // Variabel foc_duty_dowmramp_kp: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float foc_duty_dowmramp_kp;
    // Variabel foc_duty_dowmramp_ki: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float foc_duty_dowmramp_ki;
    // Variabel foc_pll_kp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_pll_kp;
    // Variabel foc_pll_ki: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_pll_ki;
    // Variabel foc_cc_decoupling: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    mc_foc_cc_decoupling_mode foc_cc_decoupling;
    // Variabel foc_mtpa_mode: mode operasi yang menentukan jalur algoritma aktif.
    MTPA_MODE foc_mtpa_mode;
    // Variabel foc_fw_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    float foc_fw_current_max;
    // Variabel foc_fw_duty_start: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float foc_fw_duty_start;
    // Variabel foc_fw_ramp_time: nilai waktu untuk penjadwalan atau pengawasan.
    float foc_fw_ramp_time;
    // Variabel foc_fw_q_current_factor: nilai arus untuk pengukuran, kendali, atau proteksi.
    float foc_fw_q_current_factor;
    // Variabel foc_fw_backoff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_fw_backoff;
    // Variabel foc_mag_vd_max: tegangan sumbu-d keluaran regulator FOC.
    float foc_mag_vd_max;
    // Variabel foc_overmod_factor: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_overmod_factor;
    // Variabel foc_fw_current_now: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile float foc_fw_current_now;
    // Variabel mtpa_id_target: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    volatile float mtpa_id_target;
    /* VESC 7.x-style fast field weakening backend. Float configuration is
       converted task-side; the hard FOC path only uses integer/Q-format math. */
    // Variabel foc_fw_current_acc_q31: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t foc_fw_current_acc_q31;
    // Variabel foc_fw_current_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t foc_fw_current_q15;
    // Variabel foc_fw_duty_filter_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile int32_t foc_fw_duty_filter_q15;
    // Variabel fw_override_current_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t fw_override_current_q15;
    // Variabel foc_fw_max_q15: batas atau nilai maksimum untuk validasi dan proteksi.
    int32_t foc_fw_max_q15;
    // Variabel foc_fw_duty_start_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    int32_t foc_fw_duty_start_q15;
    // Variabel foc_fw_duty_end_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    int32_t foc_fw_duty_end_q15;
    // Variabel foc_fw_duty_span_inv_q30: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    int32_t foc_fw_duty_span_inv_q30;
    // Variabel foc_fw_duty_norm_scale_q16: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    int32_t foc_fw_duty_norm_scale_q16;
    // Variabel foc_fw_backoff_per_current_q16: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t foc_fw_backoff_per_current_q16;
    // Variabel foc_fw_q_factor_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t foc_fw_q_factor_q15;
    // Variabel foc_fw_ramp_step_q31: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t foc_fw_ramp_step_q31;
    // Variabel foc_fw_ramp_direct: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool foc_fw_ramp_direct;
    // Variabel foc_current_limit_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
    volatile int32_t foc_current_limit_q15;
    // Variabel foc_fw_fast_active: penanda bahwa state atau fitur sedang aktif.
    volatile bool foc_fw_fast_active;
    // Variabel foc_fw_hold_request: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool foc_fw_hold_request;
    // Variabel foc_hall_interp_erpm: kecepatan listrik motor dalam electrical RPM.
    float foc_hall_interp_erpm;
    // Variabel foc_sl_erpm_start: kecepatan listrik motor dalam electrical RPM.
    float foc_sl_erpm_start;
    // Variabel foc_sl_erpm: kecepatan listrik motor dalam electrical RPM.
    float foc_sl_erpm;
    /* Precomputed thresholds used by the 16-kHz ISR. Never convert float
       configuration values inside the hard current-control path. */
    // Variabel foc_sl_erpm_start_q16: kecepatan listrik motor dalam electrical RPM.
    int32_t foc_sl_erpm_start_q16;
    // Variabel foc_sl_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    int32_t foc_sl_erpm_q16;
    // Variabel foc_hall_interp_erpm_u32: kecepatan listrik motor dalam electrical RPM.
    uint32_t foc_hall_interp_erpm_u32; /* ISR threshold; no float in Hall fast path */
    // Variabel foc_openloop_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float foc_openloop_rpm;
    // Variabel foc_openloop_rpm_low: kecepatan putar yang digunakan oleh logika kendali.
    float foc_openloop_rpm_low;
    // Variabel foc_sl_openloop_time_lock: nilai waktu untuk penjadwalan atau pengawasan.
    float foc_sl_openloop_time_lock;
    // Variabel foc_sl_openloop_time_ramp: nilai waktu untuk penjadwalan atau pengawasan.
    float foc_sl_openloop_time_ramp;
    // Variabel foc_sl_openloop_time: nilai waktu untuk penjadwalan atau pengawasan.
    float foc_sl_openloop_time;
    // Variabel foc_sl_openloop_hyst: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_sl_openloop_hyst;
    // Variabel foc_sl_openloop_boost_q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float foc_sl_openloop_boost_q;
    // Variabel foc_sl_openloop_max_q: batas atau nilai maksimum untuk validasi dan proteksi.
    float foc_sl_openloop_max_q;

    // Variabel observer_flux_alpha: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_flux_alpha;
    // Variabel observer_flux_beta: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_flux_beta;
    // Variabel observer_phase_rad: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile float observer_phase_rad;
    // Variabel observer_speed_rad_s: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_speed_rad_s;
    // Variabel pll_phase_rad: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile float pll_phase_rad;
    // Variabel pll_speed_rad_s: nilai kecepatan untuk target atau pengukuran.
    volatile float pll_speed_rad_s;
    // Variabel observer_phase_deg: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile float observer_phase_deg;
    // Variabel observer_erpm: kecepatan listrik motor dalam electrical RPM.
    volatile float observer_erpm;
    // Variabel observer_quality: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_quality;
    // Variabel observer_v_alpha_prev: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_v_alpha_prev;
    // Variabel observer_v_beta_prev: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_v_beta_prev;
    // Variabel observer_i_alpha_prev: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_i_alpha_prev;
    // Variabel observer_i_beta_prev: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile float observer_i_beta_prev;

    /* Fixed-point voltage-model flux observer. The hard ADC/FOC loop owns
       these states; slow task-side floats above are telemetry mirrors only. */
    // Variabel observer_stator_flux_alpha_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int64_t observer_stator_flux_alpha_q30;
    // Variabel observer_stator_flux_beta_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int64_t observer_stator_flux_beta_q30;
    // Variabel observer_rotor_flux_alpha_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_rotor_flux_alpha_q30;
    // Variabel observer_rotor_flux_beta_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_rotor_flux_beta_q30;
    // Variabel observer_v_alpha_q15_prev: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_v_alpha_q15_prev;
    // Variabel observer_v_beta_q15_prev: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_v_beta_q15_prev;
    // Variabel observer_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    volatile int32_t observer_erpm_q16;
    // Variabel observer_phase_last_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t observer_phase_last_u16;
    // Variabel observer_r_i_to_v_q15: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    int32_t observer_r_i_to_v_q15;
    // Variabel observer_l_i_to_flux_q15: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    int32_t observer_l_i_to_flux_q15;
    // Variabel observer_vdt_to_flux_q15: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    int32_t observer_vdt_to_flux_q15;
    // Variabel observer_flux_target_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    int32_t observer_flux_target_q30;
    /* Adaptive flux estimate used by the VESC lambda-compensated Ortega,
       MXLEMMING and MXV observers. Q2.30 on FOC_FLUX_Q_BASE_WB. */
    // Variabel observer_lambda_est_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_lambda_est_q30;
    /* Previous alpha/beta currents for the MXLEMMING current-difference term.
       Keeping them in Q15 avoids any floating point in the hard FOC ISR. */
    // Variabel observer_i_alpha_last_q15: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_i_alpha_last_q15;
    // Variabel observer_i_beta_last_q15: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile int32_t observer_i_beta_last_q15;
    /* Observer correction coefficient in Q2.30. It already includes dt and
       the normalization by FOC_FLUX_Q_BASE_WB, so the 16-kHz ISR only performs
       fixed-point multiplies. Updated by the 1-kHz observer service. */
    // Variabel observer_gamma_coeff_q30: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    int32_t observer_gamma_coeff_q30;

    /* Fixed-point phase-locked loop. Phase is u16/revolution; speed is Q16.16
       electrical RPM. Kp*dt and Ki*dt*60 are precomputed task-side. */
    // Variabel pll_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t pll_phase_u16;
    // Variabel pll_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    volatile int32_t pll_erpm_q16;
    // Variabel pll_kp_dt_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t pll_kp_dt_q16;
    // Variabel pll_ki_dt60_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t pll_ki_dt60_q16;

    // Variabel observer_phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t observer_phase_u16;
    // Variabel observer_update_cycle: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile uint32_t observer_update_cycle;
    // Variabel observer_valid: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    volatile bool observer_valid;
    // Variabel using_encoder: data encoder untuk pengukuran posisi atau kecepatan rotor.
    volatile bool using_encoder;
    /* Optional VESC 7 additional-fault backend for LEFT ABI. It compares the
       physically referenced encoder electrical phase with the compensated
       observer phase only above foc_openloop_rpm. */
    // Variabel encoder_slip_bad_ticks: data encoder untuk pengukuran posisi atau kecepatan rotor.
    volatile uint16_t encoder_slip_bad_ticks;
    // Variabel encoder_slip_error_phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile int16_t encoder_slip_error_phase;
    // Variabel encoder_slip_check_active: data encoder untuk pengukuran posisi atau kecepatan rotor.
    volatile bool encoder_slip_check_active;
    // Variabel phase_observer_override: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile bool phase_observer_override;
    // Variabel phase_observer_override_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    volatile uint16_t phase_observer_override_u16;
    // Variabel openloop_start_tick: nilai tick scheduler untuk pengukuran waktu.
    volatile uint32_t openloop_start_tick;
    // Variabel openloop_started: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool openloop_started;
    // Variabel sensorless_start_failures: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t sensorless_start_failures;
    // Variabel openloop_erpm_now: kecepatan listrik motor dalam electrical RPM.
    volatile float openloop_erpm_now;

    /* Detection results are kept separate until explicitly applied/stored. */
    // Variabel detect_resistance_ohm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float detect_resistance_ohm;
    // Variabel detect_inductance_h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float detect_inductance_h;
    // Variabel detect_ld_lq_diff_h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float detect_ld_lq_diff_h;
    // Variabel detect_flux_linkage_wb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile float detect_flux_linkage_wb;
    // Variabel detect_rl_valid: penanda validitas hasil pengukuran atau konfigurasi.
    volatile bool detect_rl_valid;
    // Variabel detect_flux_valid: penanda validitas hasil pengukuran atau konfigurasi.
    volatile bool detect_flux_valid;

    // Variabel speed_pid: nilai kecepatan untuk target atau pengukuran.
    pid_state_t speed_pid;
    // Variabel position_pid: nilai posisi rotor atau aktuator.
    pid_state_t position_pid;
    // Variabel duty_pid: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    pid_state_t duty_pid;
    /* VESC duty/brake transition state. duty_pid.integrator is normalized
       (-1..1) while the dedicated duty down-ramp PI is active. */
    // Variabel duty_was_pi: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile bool duty_was_pi;
    // Variabel duty_pi_duty_last: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    volatile float duty_pi_duty_last;
    // Variabel force_zero_modulation: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile bool force_zero_modulation;
    /* Brake zero-cross guard. A 1-kHz hold tick spans 16 FOC samples on this
       board, exceeding upstream's minimum 10-control-cycle zero-duty hold. */
    // Variabel brake_speed_before_q16: nilai kecepatan untuk target atau pengukuran.
    volatile int32_t brake_speed_before_q16;
    // Variabel brake_vq_before_q15: tegangan sumbu-q keluaran regulator FOC.
    volatile int32_t brake_vq_before_q15;
    // Variabel brake_zero_hold_ticks: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t brake_zero_hold_ticks;
    // Variabel brake_zero_active: penanda bahwa state atau fitur sedang aktif.
    volatile bool brake_zero_active;
    // Variabel speed_kd_filter: nilai kecepatan untuk target atau pengukuran.
    float speed_kd_filter;
    // Variabel speed_derivative_filtered: nilai kecepatan untuk target atau pengukuran.
    float speed_derivative_filtered;
    // Variabel speed_pid_min_erpm: kecepatan listrik motor dalam electrical RPM.
    float speed_pid_min_erpm;
    // Variabel speed_pid_ramp_erpms_s: nilai kecepatan untuk target atau pengukuran.
    float speed_pid_ramp_erpms_s;
    // Variabel speed_pid_allow_braking: nilai kecepatan untuk target atau pengukuran.
    bool speed_pid_allow_braking;
    // Variabel speed_pid_source: nilai kecepatan untuk target atau pengukuran.
    S_PID_SPEED_SRC speed_pid_source;
    // Variabel position_kd_filter: nilai posisi rotor atau aktuator.
    float position_kd_filter;
    // Variabel position_kd_proc: nilai posisi rotor atau aktuator.
    float position_kd_proc;
    // Variabel position_ang_div: nilai posisi rotor atau aktuator.
    float position_ang_div;
    // Variabel position_gain_dec_angle: nilai sudut untuk posisi atau transformasi koordinat.
    float position_gain_dec_angle;
    // Variabel position_offset_deg: offset kalibrasi untuk mengoreksi bias pengukuran.
    float position_offset_deg;
    // Variabel duty_kp: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty_kp;
    // Variabel duty_ki: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty_ki;
    // Variabel position_derivative_filtered: nilai posisi rotor atau aktuator.
    float position_derivative_filtered;
    // Variabel position_derivative_proc_filtered: nilai posisi rotor atau aktuator.
    float position_derivative_proc_filtered;
    // Variabel position_prev_process_deg: nilai posisi rotor atau aktuator.
    float position_prev_process_deg;
    // Variabel position_dt_integrator: nilai posisi rotor atau aktuator.
    float position_dt_integrator;
    // Variabel position_dt_process_integrator: nilai posisi rotor atau aktuator.
    float position_dt_process_integrator;
    // Variabel cc_min_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float cc_min_current;
    // Variabel detect: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_detect_state_t detect;
    // Variabel stats: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_stats_t stats;

    // Variabel last_command_tick: nilai tick scheduler untuk pengukuran waktu.
    volatile uint32_t last_command_tick;
    // Variabel isr_max_cycles: batas atau nilai maksimum untuk validasi dan proteksi.
    volatile uint32_t isr_max_cycles;
    // Variabel isr_overruns: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t isr_overruns;

    /* Seqlock-protected ISR snapshot. Odd sequence = writer active; even =
       stable. Telemetry can copy a coherent 16-kHz frame without masking the
       ADC/DMA interrupt. */
    // Variabel rt_snapshot_seq: nomor urut untuk konsistensi snapshot atau record.
    volatile uint32_t rt_snapshot_seq;
    // Variabel rt_snapshot: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile foc_rt_snapshot_t rt_snapshot;
} MotorRuntime;

typedef struct {
    // Variabel phase_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    float phase_current_a;
    // Variabel phase_current_b: nilai arus untuk pengukuran, kendali, atau proteksi.
    float phase_current_b;
    // Variabel phase_current_c: nilai arus untuk pengukuran, kendali, atau proteksi.
    float phase_current_c;
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    float id;
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq;
    // Variabel id_filter: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    float id_filter;
    // Variabel iq_filter: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq_filter;
    // Variabel id_target: referensi arus sumbu-d efektif dari frame FOC yang sama dengan Id/Iq/Vd/Vq.
    float id_target;
    // Variabel iq_target: referensi arus sumbu-q efektif dari frame FOC yang sama dengan Id/Iq/Vd/Vq.
    float iq_target;
    // Variabel vd: tegangan sumbu-d keluaran regulator FOC.
    float vd;
    // Variabel vq: tegangan sumbu-q keluaran regulator FOC.
    float vq;
    // Variabel current_motor: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_motor;
    // Variabel current_in: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_in;
    // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty;
    // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
    float erpm;
    // Variabel mech_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float mech_rpm;
    // Variabel vbus: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    float vbus;
    // Variabel position_deg: nilai posisi rotor atau aktuator.
    float position_deg;
    // Variabel rotor_elec_deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float rotor_elec_deg;
    // Variabel current_offset_u: offset kalibrasi untuk mengoreksi bias pengukuran.
    float current_offset_u;
    // Variabel current_offset_v: offset kalibrasi untuk mengoreksi bias pengukuran.
    float current_offset_v;
    // Variabel dc_current_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    float dc_current_offset;
    // Variabel amp_hours: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float amp_hours;
    // Variabel amp_hours_charged: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float amp_hours_charged;
    // Variabel watt_hours: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float watt_hours;
    // Variabel watt_hours_charged: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float watt_hours_charged;
    // Variabel tachometer: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t tachometer;
    // Variabel tachometer_abs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t tachometer_abs;
    // Variabel fault: status atau data gangguan untuk sistem proteksi.
    uint8_t fault;
    // Variabel controller_id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    uint8_t controller_id;
    // Variabel sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t sensor_mode; /* physical GPIO/timer input block */
    // Variabel foc_sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t foc_sensor_mode; /* logical FOC phase source */
    // Variabel sensor_detect_state: state mesin keadaan yang menentukan tahap operasi.
    uint8_t sensor_detect_state;
    // Variabel calibration_done: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t calibration_done;
    // Variabel calibration_valid: penanda validitas hasil pengukuran atau konfigurasi.
    uint8_t calibration_valid;
    // Variabel timeout_active: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    uint8_t timeout_active;
    // Variabel observer_valid: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    uint8_t observer_valid;
    // Variabel using_encoder: data encoder untuk pengukuran posisi atau kecepatan rotor.
    uint8_t using_encoder;
    // Variabel encoder_synced: data encoder untuk pengukuran posisi atau kecepatan rotor.
    uint8_t encoder_synced;
    // Variabel observer_phase_deg: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    float observer_phase_deg;
    // Variabel observer_erpm: kecepatan listrik motor dalam electrical RPM.
    float observer_erpm;
    // Variabel observer_quality: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    float observer_quality;
    // Variabel foc_motor_r: state atau parameter motor yang sedang diproses.
    float foc_motor_r;
    // Variabel foc_motor_l: state atau parameter motor yang sedang diproses.
    float foc_motor_l;
    // Variabel foc_motor_ld_lq_diff: state atau parameter motor yang sedang diproses.
    float foc_motor_ld_lq_diff;
    // Variabel foc_motor_flux_linkage: state atau parameter motor yang sedang diproses.
    float foc_motor_flux_linkage;
    // Variabel foc_sl_erpm_start: kecepatan listrik motor dalam electrical RPM.
    float foc_sl_erpm_start;
    // Variabel foc_sl_erpm: kecepatan listrik motor dalam electrical RPM.
    float foc_sl_erpm;
    /* Precomputed thresholds used by the 16-kHz ISR. Never convert float
       configuration values inside the hard current-control path. */
    // Variabel foc_sl_erpm_start_q16: kecepatan listrik motor dalam electrical RPM.
    int32_t foc_sl_erpm_start_q16;
    // Variabel foc_sl_erpm_q16: kecepatan listrik motor dalam electrical RPM.
    int32_t foc_sl_erpm_q16;
    // Variabel foc_hall_interp_erpm_u32: kecepatan listrik motor dalam electrical RPM.
    uint32_t foc_hall_interp_erpm_u32; /* ISR threshold; no float in Hall fast path */
    // Variabel foc_openloop_rpm: kecepatan putar yang digunakan oleh logika kendali.
    float foc_openloop_rpm;
    // Variabel foc_openloop_rpm_low: kecepatan putar yang digunakan oleh logika kendali.
    float foc_openloop_rpm_low;
    // Variabel current_loop_hz: nilai arus untuk pengukuran, kendali, atau proteksi.
    uint32_t current_loop_hz;
    // Variabel telemetry_snapshot_hz: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t telemetry_snapshot_hz;
    // Variabel isr_max_cycles: batas atau nilai maksimum untuk validasi dan proteksi.
    uint32_t isr_max_cycles;
    // Variabel isr_overruns: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t isr_overruns;
    // Variabel abs_current_filtered: nilai arus untuk pengukuran, kendali, atau proteksi.
    float abs_current_filtered;
    // Variabel abs_current_peak: nilai arus untuk pengukuran, kendali, atau proteksi.
    float abs_current_peak;
    // Variabel over_voltage_fault_count: status atau data gangguan untuk sistem proteksi.
    uint8_t over_voltage_fault_count;
    // Variabel under_voltage_fault_count: status atau data gangguan untuk sistem proteksi.
    uint8_t under_voltage_fault_count;
} motor_telemetry_t;

typedef struct {
    // Variabel ia_cA: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int16_t ia_cA;
    // Variabel ib_cA: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int16_t ib_cA;
    // Variabel id_cA: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    int16_t id_cA;
    // Variabel iq_cA: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int16_t iq_cA;
    // Variabel vd_cV: tegangan sumbu-d keluaran regulator FOC.
    int16_t vd_cV;
    // Variabel vq_cV: tegangan sumbu-q keluaran regulator FOC.
    int16_t vq_cV;
    // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
    int16_t erpm;
    // Variabel phase_u16: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint16_t phase_u16;
    // Variabel duty_u_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    uint16_t duty_u_q15;
    // Variabel duty_v_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    uint16_t duty_v_q15;
    // Variabel duty_w_q15: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    uint16_t duty_w_q15;
    // Variabel current_raw_u: nilai arus untuk pengukuran, kendali, atau proteksi.
    uint16_t current_raw_u;
    // Variabel current_raw_v: nilai arus untuk pengukuran, kendali, atau proteksi.
    uint16_t current_raw_v;
    // Variabel vbus_dV: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    uint16_t vbus_dV;
    // Variabel motor: state atau parameter motor yang sedang diproses.
    uint8_t motor;
    // Variabel hall_raw: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint8_t hall_raw;
} debug_sample_t;
