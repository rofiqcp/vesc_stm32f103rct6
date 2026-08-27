#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_id_t;


/* Runtime sensor mux used by the STM32F103 board port. Keep this separate
 * from VESC mc_foc_sensor_mode / sensor_port_mode wire enums: this value
 * describes which physical input block is currently connected to the FOC
 * phase source. Defining it in datatypes.h (instead of appconf_default.h)
 * makes encoder/hw/motor modules share one canonical type. */
typedef enum {
    SENSOR_MODE_AUTO = 0,
    SENSOR_MODE_HALL = 1,
    SENSOR_MODE_ENCODER = 2,
    SENSOR_MODE_FORCED_OPENLOOP = 3
} motor_sensor_runtime_mode_t;

/* Canonical VESC enum values used on MCCONF / logic comparisons. HFI entries
 * are intentionally retained as wire-schema values so following fields keep
 * the upstream layout, but the STM32F103 runtime never executes HFI. */
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

typedef enum { COMM_MODE_INTEGRATE = 0, COMM_MODE_DELAY = 1 } mc_comm_mode;
typedef enum { SENSOR_MODE_SENSORLESS = 0, SENSOR_MODE_SENSORED = 1, SENSOR_MODE_HYBRID = 2 } mc_sensor_mode;
typedef enum { MOTOR_TYPE_BLDC = 0, MOTOR_TYPE_DC = 1, MOTOR_TYPE_FOC = 2 } mc_motor_type;
typedef enum { MC_STATE_OFF = 0, MC_STATE_DETECTING, MC_STATE_RUNNING, MC_STATE_FULL_BRAKE } mc_state;
typedef enum {
    CONTROL_MODE_DUTY = 0, CONTROL_MODE_SPEED, CONTROL_MODE_CURRENT,
    CONTROL_MODE_CURRENT_BRAKE, CONTROL_MODE_POS, CONTROL_MODE_HANDBRAKE,
    CONTROL_MODE_OPENLOOP, CONTROL_MODE_OPENLOOP_PHASE,
    CONTROL_MODE_OPENLOOP_DUTY, CONTROL_MODE_OPENLOOP_DUTY_PHASE, CONTROL_MODE_NONE
} mc_control_mode;

typedef struct {
    float cycle_int_limit;
    float cycle_int_limit_running;
    float cycle_int_limit_max;
    float comm_time_sum;
    float comm_time_sum_min_rpm;
    int32_t comms;
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
    float l_current_max, l_current_min, l_in_current_max, l_in_current_min;
    float l_in_current_map_start, l_in_current_map_filter;
    float l_abs_current_max, l_min_erpm, l_max_erpm, l_erpm_start, l_min_vin, l_max_vin;
    float l_battery_cut_start, l_battery_cut_end;
    bool l_slow_abs_current;
    float l_temp_fet_start, l_temp_fet_end;
    float l_temp_motor_start, l_temp_motor_end;
    float l_temp_accel_dec;
    /* Not present in the pinned VESC-6.00 wire image. Runtime-only until a
       dedicated protocol migration is performed. */
    uint8_t l_additional_faults;
    /* VESC-6 wire already contains watt/current scaling. Regen-cut fields were
       added later; they remain runtime-only defaults while FW reports 6.00. */
    float l_battery_regen_cut_start, l_battery_regen_cut_end;
    float l_min_duty, l_max_duty;
    float l_watt_max, l_watt_min;
    float l_current_max_scale, l_current_min_scale, l_duty_start;
    float lo_current_max, lo_current_min, lo_in_current_max, lo_in_current_min;
    mc_pwm_mode pwm_mode;
    mc_comm_mode comm_mode;
    mc_motor_type motor_type;
    mc_sensor_mode sensor_mode;
    float sl_min_erpm, sl_min_erpm_cycle_int_limit, sl_max_fullbreak_current_dir_change;
    float sl_cycle_int_limit, sl_phase_advance_at_br, sl_cycle_int_rpm_br, sl_bemf_coupling_k;
    int8_t hall_table[8];
    float hall_sl_erpm;
    float foc_current_kp, foc_current_ki, foc_f_zv, foc_dt_us;
    float foc_encoder_offset; bool foc_encoder_inverted; float foc_encoder_ratio;
    float foc_motor_l, foc_motor_ld_lq_diff, foc_motor_r, foc_motor_flux_linkage;
    float foc_observer_gain, foc_observer_gain_slow, foc_observer_offset;
    float foc_duty_dowmramp_kp, foc_duty_dowmramp_ki;
    float foc_start_curr_dec, foc_start_curr_dec_rpm;
    /* Newer VESC field with a real backend in Batch 9.3. It is runtime-only
       while the wire ABI remains VESC 6.00; serialization rejects true. */
    bool foc_short_ls_on_zero_duty;
    float foc_pll_kp, foc_pll_ki;
    float foc_openloop_rpm, foc_openloop_rpm_low, foc_sl_openloop_hyst;
    float foc_sl_openloop_time, foc_sl_openloop_time_lock, foc_sl_openloop_time_ramp;
    float foc_sl_openloop_boost_q, foc_sl_openloop_max_q;
    mc_foc_sensor_mode foc_sensor_mode;
    uint8_t foc_hall_table[8];
    float foc_hall_interp_erpm, foc_sl_erpm_start, foc_sl_erpm;
    bool foc_sample_v0_v7, foc_sample_high_current;
    FOC_SPEED_SRC foc_speed_source;
    SAT_COMP_MODE foc_sat_comp_mode;
    float foc_sat_comp, foc_current_filter_const;
    mc_foc_cc_decoupling_mode foc_cc_decoupling;
    mc_foc_observer_type foc_observer_type;
    MTPA_MODE foc_mtpa_mode;
    float foc_fw_current_max, foc_fw_duty_start, foc_fw_ramp_time, foc_fw_q_current_factor, foc_fw_backoff;
    float foc_mag_vd_max, foc_overmod_factor;
    /* VESC-style motor-temperature (resistance) compensation. The F103 port has
       no motor NTC, so the STM32 board-temperature proxy (board_temp_filter_c)
       is used as the thermal input. comp_factor = 1 + 0.00386*(T - base_temp). */
    bool foc_temp_comp;
    float foc_temp_comp_base_temp;
    /* VESC offset-calibration mode bits: bit0 = driven, bit1 = undriven,
       bit2 = periodic re-calibration when motor stopped (state OFF). */
    uint8_t foc_offsets_cal_mode;
    /* When false, the entire boot current-offset calibration pipeline is
       skipped and the motor runs with stored/gross-default offsets instead.
       VESC Tool can read/write this flag. Default is true. */
    bool foc_calibrate_on_boot;
    float s_pid_kp, s_pid_ki, s_pid_kd, s_pid_kd_filter, s_pid_min_erpm;
    bool s_pid_allow_braking;
    float s_pid_ramp_erpms_s;
    S_PID_SPEED_SRC s_pid_speed_source;
    float p_pid_kp, p_pid_ki, p_pid_kd, p_pid_kd_proc, p_pid_kd_filter, p_pid_ang_div, p_pid_gain_dec_angle, p_pid_offset;
    float cc_startup_boost_duty, cc_min_current, cc_gain, cc_ramp_step_max;
    uint32_t m_encoder_counts;
    sensor_port_mode m_sensor_port_mode;
    bool m_invert_direction;
    uint8_t si_motor_poles;
    float si_gear_ratio;
    float si_wheel_diameter;
    uint8_t si_battery_type;
    uint8_t si_battery_cells;
    float si_battery_ah;
    float si_motor_nl_current;
} mc_configuration;

/* Canonical VESC setup aggregate. In this dual-bridge port these values are
 * aggregated over both local motors, analogous to VESC setup values including
 * local/CAN controllers. */
typedef struct {
    float ah_tot;
    float ah_charge_tot;
    float wh_tot;
    float wh_charge_tot;
    float current_tot;
    float current_in_tot;
    uint8_t num_vescs;
} setup_values;

/* Canonical VESC statistics state. FreeRTOS tick time replaces ChibiOS
 * systime_t, while the numeric fields retain upstream semantics. */
typedef struct {
    uint32_t time_start;
    double samples;
    double speed_sum;
    float max_speed;
    double power_sum;
    float max_power;
    double temp_motor_sum;
    float max_temp_motor;
    double temp_mos_sum;
    float max_temp_mos;
    double current_sum;
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
    adc_control_type ctrl_type;
    float hyst;
    float voltage_start;
    float voltage_end;
    float voltage_min;
    float voltage_max;
    float voltage_center;
    float voltage2_start;
    float voltage2_end;
    bool use_filter;
    SAFE_START_MODE safe_start;
    uint8_t buttons;
    bool voltage_inverted;
    bool voltage2_inverted;
    float throttle_exp;
    float throttle_exp_brake;
    thr_exp_mode throttle_exp_mode;
    float ramp_time_pos;
    float ramp_time_neg;
    bool multi_esc;
    bool tc;
    float tc_max_diff;
    uint16_t update_rate_hz;
} adc_config;

/* Reduced canonical app_configuration view. The full VESC-6 493-byte wire
 * image remains the source of truth in confgenerator.c. APP ADC fields are now
 * represented because PA2/PA3 have a real backend; unsupported PPM/NRF/etc.
 * fields remain byte-preserved and immutable. */
typedef struct {
    uint8_t controller_id;
    uint32_t timeout_msec;
    float timeout_brake_current;
    bool permanent_uart_enabled;
    app_use app_to_use;
    adc_config app_adc_conf;
    uint32_t app_uart_baudrate;
    uint16_t crc;
} app_configuration;

/* Canonical VESC command IDs. Keeping the enum complete does not enable
 * unsupported subsystems; commands.c explicitly implements only commands
 * with a real backend on this hardware. */
typedef enum {
    COMM_FW_VERSION=0, COMM_JUMP_TO_BOOTLOADER=1, COMM_ERASE_NEW_APP=2,
    COMM_WRITE_NEW_APP_DATA=3, COMM_GET_VALUES=4, COMM_SET_DUTY=5,
    COMM_SET_CURRENT=6, COMM_SET_CURRENT_BRAKE=7, COMM_SET_RPM=8,
    COMM_SET_POS=9, COMM_SET_HANDBRAKE=10, COMM_SET_DETECT=11,
    COMM_SET_SERVO_POS=12, COMM_SET_MCCONF=13, COMM_GET_MCCONF=14,
    COMM_GET_MCCONF_DEFAULT=15, COMM_SET_APPCONF=16, COMM_GET_APPCONF=17,
    COMM_GET_APPCONF_DEFAULT=18, COMM_SAMPLE_PRINT=19, COMM_TERMINAL_CMD=20,
    COMM_PRINT=21, COMM_ROTOR_POSITION=22, COMM_EXPERIMENT_SAMPLE=23,
    COMM_DETECT_MOTOR_PARAM=24, COMM_DETECT_MOTOR_R_L=25,
    COMM_DETECT_MOTOR_FLUX_LINKAGE=26, COMM_DETECT_ENCODER=27,
    COMM_DETECT_HALL_FOC=28, COMM_REBOOT=29, COMM_ALIVE=30,
    COMM_GET_DECODED_PPM=31, COMM_GET_DECODED_ADC=32, COMM_GET_DECODED_CHUK=33,
    COMM_FORWARD_CAN=34, COMM_SET_CHUCK_DATA=35, COMM_CUSTOM_APP_DATA=36,
    COMM_NRF_START_PAIRING=37, COMM_GPD_SET_FSW=38, COMM_GPD_BUFFER_NOTIFY=39,
    COMM_GPD_BUFFER_SIZE_LEFT=40, COMM_GPD_FILL_BUFFER=41, COMM_GPD_OUTPUT_SAMPLE=42,
    COMM_GPD_SET_MODE=43, COMM_GPD_FILL_BUFFER_INT8=44, COMM_GPD_FILL_BUFFER_INT16=45,
    COMM_GPD_SET_BUFFER_INT_SCALE=46, COMM_GET_VALUES_SETUP=47,
    COMM_SET_MCCONF_TEMP=48, COMM_SET_MCCONF_TEMP_SETUP=49,
    COMM_GET_VALUES_SELECTIVE=50, COMM_GET_VALUES_SETUP_SELECTIVE=51,
    COMM_EXT_NRF_PRESENT=52, COMM_EXT_NRF_ESB_SET_CH_ADDR=53,
    COMM_EXT_NRF_ESB_SEND_DATA=54, COMM_EXT_NRF_ESB_RX_DATA=55,
    COMM_EXT_NRF_SET_ENABLED=56, COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP=57,
    COMM_DETECT_APPLY_ALL_FOC=58, COMM_JUMP_TO_BOOTLOADER_ALL_CAN=59,
    COMM_ERASE_NEW_APP_ALL_CAN=60, COMM_WRITE_NEW_APP_DATA_ALL_CAN=61,
    COMM_PING_CAN=62, COMM_APP_DISABLE_OUTPUT=63, COMM_TERMINAL_CMD_SYNC=64,
    COMM_GET_IMU_DATA=65, COMM_BM_CONNECT=66, COMM_BM_ERASE_FLASH_ALL=67,
    COMM_BM_WRITE_FLASH=68, COMM_BM_REBOOT=69, COMM_BM_DISCONNECT=70,
    COMM_BM_MAP_PINS_DEFAULT=71, COMM_BM_MAP_PINS_NRF5X=72, COMM_ERASE_BOOTLOADER=73,
    COMM_ERASE_BOOTLOADER_ALL_CAN=74, COMM_PLOT_INIT=75, COMM_PLOT_DATA=76,
    COMM_PLOT_ADD_GRAPH=77, COMM_PLOT_SET_GRAPH=78, COMM_GET_DECODED_BALANCE=79,
    COMM_BM_MEM_READ=80, COMM_WRITE_NEW_APP_DATA_LZO=81,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO=82, COMM_BM_WRITE_FLASH_LZO=83,
    COMM_SET_CURRENT_REL=84, COMM_CAN_FWD_FRAME=85, COMM_SET_BATTERY_CUT=86,
    COMM_SET_BLE_NAME=87, COMM_SET_BLE_PIN=88, COMM_SET_CAN_MODE=89,
    COMM_GET_IMU_CALIBRATION=90, COMM_GET_MCCONF_TEMP=91,
    COMM_GET_CUSTOM_CONFIG_XML=92, COMM_GET_CUSTOM_CONFIG=93,
    COMM_GET_CUSTOM_CONFIG_DEFAULT=94, COMM_SET_CUSTOM_CONFIG=95,
    COMM_BMS_GET_VALUES=96, COMM_BMS_SET_CHARGE_ALLOWED=97,
    COMM_BMS_SET_BALANCE_OVERRIDE=98, COMM_BMS_RESET_COUNTERS=99,
    COMM_BMS_FORCE_BALANCE=100, COMM_BMS_ZERO_CURRENT_OFFSET=101,
    COMM_JUMP_TO_BOOTLOADER_HW=102, COMM_ERASE_NEW_APP_HW=103,
    COMM_WRITE_NEW_APP_DATA_HW=104, COMM_ERASE_BOOTLOADER_HW=105,
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW=106, COMM_ERASE_NEW_APP_ALL_CAN_HW=107,
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW=108, COMM_ERASE_BOOTLOADER_ALL_CAN_HW=109,
    COMM_SET_ODOMETER=110, COMM_PSW_GET_STATUS=111, COMM_PSW_SWITCH=112,
    COMM_BMS_FWD_CAN_RX=113, COMM_BMS_HW_DATA=114, COMM_GET_BATTERY_CUT=115,
    COMM_BM_HALT_REQ=116, COMM_GET_QML_UI_HW=117, COMM_GET_QML_UI_APP=118,
    COMM_CUSTOM_HW_DATA=119, COMM_QMLUI_ERASE=120, COMM_QMLUI_WRITE=121,
    COMM_IO_BOARD_GET_ALL=122, COMM_IO_BOARD_SET_PWM=123, COMM_IO_BOARD_SET_DIGITAL=124,
    COMM_BM_MEM_WRITE=125, COMM_BMS_BLNC_SELFTEST=126, COMM_GET_EXT_HUM_TMP=127,
    COMM_GET_STATS=128, COMM_RESET_STATS=129, COMM_LISP_READ_CODE=130,
    COMM_LISP_WRITE_CODE=131, COMM_LISP_ERASE_CODE=132, COMM_LISP_SET_RUNNING=133,
    COMM_LISP_GET_STATS=134, COMM_LISP_PRINT=135, COMM_BMS_SET_BATT_TYPE=136,
    COMM_BMS_GET_BATT_TYPE=137, COMM_LISP_REPL_CMD=138, COMM_LISP_STREAM_CODE=139,
    COMM_FILE_LIST=140, COMM_FILE_READ=141, COMM_FILE_WRITE=142, COMM_FILE_MKDIR=143,
    COMM_FILE_REMOVE=144, COMM_LOG_START=145, COMM_LOG_STOP=146,
    COMM_LOG_CONFIG_FIELD=147, COMM_LOG_DATA_F32=148, COMM_SET_APPCONF_NO_STORE=149,
    COMM_GET_GNSS=150, COMM_LOG_DATA_F64=151, COMM_LISP_RMSG=152,
    COMM_SHUTDOWN=156, COMM_FW_INFO=157, COMM_CAN_UPDATE_BAUD_ALL=158,
    COMM_MOTOR_ESTOP=159
} COMM_PACKET_ID;

typedef struct {
    double lat, lon, height, speed, hdop;
    uint32_t ms_today;
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
    volatile uint16_t base_phase_u16;
    volatile int32_t phase_per_cycle_q16;
    volatile uint32_t edge_cycle;
    volatile uint32_t period_cycles;
    volatile int8_t direction;
    volatile int8_t sector;
    volatile uint8_t raw_state;
    volatile bool valid;
    volatile uint16_t invalid_count;
    volatile uint16_t sequence_error_count;
    volatile uint16_t recovery_valid_ticks;
    volatile int32_t edge_count;
    /* Explicit Hall electrical-angle slew limiter, matching current VESC's
       rate-limited Hall phase while remaining integer-only in the 16-kHz path. */
    volatile uint16_t rate_limited_phase_u16;
    volatile bool rate_limited_valid;
    volatile uint32_t rate_limit_frame;
} hall_state_t;

typedef struct {
    volatile int32_t turns;
    volatile uint16_t last_cnt;
    volatile int32_t extended_count;
    volatile int32_t prev_extended_count;
    volatile bool speed_sample_valid;
    /* Incremental AB has no absolute phase after reset. session_zero_count is
       rebuilt from the sensorless observer via encoder_set_deg() after the
       VESC-style open-loop startup reaches a trustworthy observer speed. */
    volatile int32_t session_zero_count;
    volatile int32_t mechanical_zero_count;
    volatile bool synced;
    volatile bool motion_proved;
    volatile bool sync_active;
    volatile uint32_t sync_start_tick;
    uint32_t cpr;
    /* Electrical revolutions per one mechanical encoder revolution.
       Kept separate from physical motor pole_pairs as in VESC mcconf.
       The float is task/config-side; Q16.16 and phase-per-count are the ISR
       representations, so non-integer VESC encoder ratios remain supported. */
    float electrical_ratio;
    uint32_t electrical_ratio_q16;
    uint32_t phase_per_count_q16;
    uint16_t elec_offset_u16;
    bool inverted;
} encoder_state_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
} pid_state_t;

#define FOC_DETECT_L_CAPTURE_MAX 32U

typedef struct {
    volatile sensor_detect_state_t state;
    volatile bool requested;
    volatile bool busy;
    volatile bool success;
    volatile uint8_t result_mode;
    volatile bool apply_result;
    volatile float drive_current_a;
    volatile uint32_t step_tick;
    volatile uint32_t sweep_index;
    volatile uint32_t sweep_pass;
    volatile uint16_t forced_phase_u16;
    volatile int32_t encoder_start_count;
    volatile int32_t encoder_end_count;
    volatile uint8_t hall_valid_states;

    /* Detection is a temporary motor-control transaction. Preserve everything
       it changes so standard VESC COMM_DETECT_* can restore the previous
       configuration after returning its result. */
    uint8_t saved_sensor_mode;
    uint8_t saved_sensor_request_mode;
    float saved_current_kp;
    float saved_current_ki;
    uint32_t saved_timeout_ms;
    float saved_timeout_brake_a;

    /* LEFT Hall and A/B encoder share PB6/PB7. A temporary Hall detect must
       restore the incremental coordinate when TIM4 ownership returns to the
       encoder. Hall detection performs equal forward/reverse electrical sweeps,
       so the expected final shaft coordinate is the transaction entry point. */
    uint16_t saved_encoder_hw_count;
    int32_t saved_encoder_turns;
    uint16_t saved_encoder_last_cnt;
    int32_t saved_encoder_extended_count;
    int32_t saved_encoder_prev_extended_count;
    int32_t saved_encoder_session_zero_count;
    int32_t saved_encoder_mechanical_zero_count;
    bool saved_encoder_speed_sample_valid;
    bool saved_encoder_synced;
    bool saved_encoder_motion_proved;
    bool saved_encoder_sync_active;
    uint32_t saved_encoder_sync_start_tick;
    bool saved_encoder_coordinate_valid;

    /* Result lives separately from active runtime config. This is required by
       VESC semantics: detect returns parameters; the Tool decides whether to
       apply/write them afterwards. */
    uint8_t result_hall_table[8];
    float result_encoder_offset_deg;
    float result_encoder_ratio;
    bool result_encoder_inverted;

    int64_t hall_sin_sum[8];
    int64_t hall_cos_sum[8];
    uint32_t hall_samples[8];

    /* PWM-rate inductance step capture. The task selects d or q before arming;
       the hard ISR stores only that axis current and the causally preceding
       applied axis voltage. The blocking detect task estimates L afterwards. */
    volatile bool l_capture_active;
    volatile bool l_capture_done;
    volatile uint8_t l_capture_axis; /* 0=d, 1=q */
    volatile uint16_t l_capture_count;
    int32_t l_capture_i_q15[FOC_DETECT_L_CAPTURE_MAX];
    int32_t l_capture_v_prev_q15[FOC_DETECT_L_CAPTURE_MAX];
} foc_detect_state_t;

typedef struct {
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int32_t tachometer;
    int32_t tachometer_abs;
    int32_t tachometer_last;
    bool tachometer_source_valid;
    float max_current;
    float max_input_current;
    float max_erpm;
    uint32_t runtime_ms;
} motor_stats_t;

typedef struct {
    /* Coherent data published by one FOC ISR pass. All values use the same
       fixed-point bases as MotorRuntime fast fields. */
    int32_t ia_q15, ib_q15, ic_q15;
    int32_t id_q15, iq_q15;
    int32_t id_filter_q15, iq_filter_q15;
    int32_t id_target_q15, iq_target_q15;
    int32_t vd_q15, vq_q15;
    int32_t vbus_q15, dc_current_q15;
    int32_t erpm_fast_q16;
    uint16_t duty_u_q15, duty_v_q15, duty_w_q15;
    uint16_t phase_control_u16, phase_observer_u16;
    uint16_t phase_encoder_u16, phase_hall_u16;
    uint32_t adc_frame;
    uint32_t cycle_counter;
} foc_rt_snapshot_t;

typedef struct MotorRuntime {
    motor_id_t id;
    TIM_TypeDef *pwm_tim;
    uint8_t pole_pairs;

    /* VESC backend selection from MCCONF. */
    mc_motor_type motor_type;
    mc_pwm_mode pwm_mode;
    mc_comm_mode comm_mode;
    /* VESC FOC angle-source strategy. Keep this distinct from sensor_mode,
       which only describes the physical Hall/ABI peripheral currently
       configured on this STM32F103 board. */
    mc_foc_sensor_mode foc_sensor_mode;
    volatile mc_state state;


    /* Runtime-selected sensor mode. 0=AUTO, 1=Hall, 2=Encoder. */
    volatile uint8_t sensor_mode;
    volatile uint8_t sensor_request_mode;

    volatile motor_control_mode_t control_mode;
    volatile motor_fault_t fault;
    volatile bool pwm_enabled;
    /* Short zero-vector blanking window after MOE is asserted. During this
       window the ISR holds 50%/50%/50% and only gross DC-current safety is
       active; phase-current PI/ABS fault starts after the analog front-end
       has settled. */
    volatile uint16_t pwm_enable_blank_cycles;
    /* MOE is asserted only from the ADC/DMA ISR after the 50% preload has
       crossed at least two hardware update boundaries. This prevents a stale
       CCR triplet from being exposed for a partial PWM cycle on re-enable. */
    volatile uint8_t pwm_enable_pending_events;
    /* VESC-style full low-side brake state used only for an exact zero vector.
       Default is disabled until the hoverboard gate-driver bootstrap behavior
       has been verified on real hardware. */
    volatile bool foc_short_ls_on_zero_duty;
    volatile bool full_brake_active;
    volatile bool command_active;
    volatile bool timeout_active;
    volatile bool detect_force_angle;
    volatile uint16_t detect_phase_u16;

    volatile float id_target;
    volatile float iq_target;
    volatile int32_t id_target_q15;
    volatile int32_t iq_target_q15;
    /* Slow-loop torque/MTPA request before fast field weakening. The 16-kHz
       FOC ISR derives the effective id/iq targets from these values. Direct
       detection/open-loop helpers still initialize both base and effective
       targets atomically through motor_set_foc_targets(). */
    volatile int32_t id_target_base_q15;
    volatile int32_t iq_target_base_q15;
    volatile float current_command_a;
    volatile float brake_current_a;
    volatile float handbrake_current_a;
    volatile float fw_override_current_a;
    volatile float current_off_delay_s;
    volatile float duty_command;
    volatile int32_t duty_command_q15;
    volatile float speed_target_erpm;
    volatile float speed_pid_set_erpm;
    volatile float position_target_deg;
    /* Explicit VESC FOC open-loop controls (not sensorless-startup state). */
    volatile float openloop_command_erpm;
    volatile uint16_t openloop_command_phase_u16;

    /* Runtime limits/configuration applied from VESC MCCONF. */
    volatile float current_max_a;
    volatile float current_min_a;
    volatile float input_current_max_a;
    volatile float input_current_min_a;
    volatile float current_max_scale;
    volatile float current_min_scale;
    volatile float watt_max;
    volatile float watt_min;
    volatile float duty_start;
    volatile float lo_current_max_a;
    volatile float lo_current_min_a;
    volatile float lo_input_current_max_a;
    volatile float lo_input_current_min_a;
    volatile float abs_current_max_a;
    volatile bool slow_abs_current;
    /* The stock hoverboard has no validated MOSFET NTC. l_temp_fet_* is
       therefore applied to the STM32 internal-temperature board proxy. */
    volatile float temp_fet_start;
    volatile float temp_fet_end;
    volatile float temp_motor_start;
    volatile float temp_motor_end;
    volatile float temp_accel_dec;
    volatile uint8_t additional_faults;
    volatile float min_vin;
    volatile float max_vin;
    volatile float battery_cut_start;
    volatile float battery_cut_end;
    /* Runtime-only high-voltage regenerative-current taper. These fields are
     * intentionally not serialized while the protocol ABI is VESC 6.00. */
    volatile float battery_regen_cut_start;
    volatile float battery_regen_cut_end;
    volatile int32_t min_vin_q15;
    volatile int32_t max_vin_q15;
    volatile int32_t hard_min_vin_q15;
    volatile int32_t hard_max_vin_q15;
    volatile uint8_t under_voltage_fault_count;
    volatile uint8_t over_voltage_fault_count;
    volatile float max_erpm;
    volatile float min_erpm;
    volatile float erpm_start;
    volatile float erpm_fault_filter;
    volatile float foc_start_curr_dec;
    volatile float foc_start_curr_dec_rpm;
    volatile float max_duty;
    volatile float min_duty;
    /* Dynamic modulation ceiling. In ordinary current/speed/position modes it
       equals max_duty. VESC duty mode lowers it to the requested duty only
       after the down-ramp PI no longer has to reduce an already larger duty. */
    volatile float duty_limit_now;
    /* Precomputed voltage-circle coefficient:
       1/sqrt(3) * duty_limit_now * foc_overmod_factor, additionally capped by
       the physical 10..90% current-sampling duty window. */
    volatile int32_t vmax_coeff_q15;
    volatile bool invert_direction;

    /* VESC 6.00 SI configuration used by GET_VALUES_SETUP. */
    volatile float si_gear_ratio;
    volatile float si_wheel_diameter;
    volatile uint8_t si_battery_type;
    volatile uint8_t si_battery_cells;
    volatile float si_battery_ah;
    volatile float si_motor_nl_current;

    volatile float ia;
    volatile float ib;
    volatile float ic;
    volatile float id_meas;
    volatile float iq_meas;
    volatile float id_filter;
    volatile float iq_filter;
    volatile float vd;
    volatile float vq;
    volatile float vd_filter;
    volatile float vq_filter;
    volatile float duty_u;
    volatile float duty_v;
    volatile float duty_w;
    volatile float duty_now;
    volatile float dc_current_a;
    volatile float dc_current_filter;
    volatile float vbus;
    volatile float vbus_filter;
    volatile float motor_current;
    volatile float input_current;
    /* VESC-style pre-emptive input-current map. This uses the hoverboard's
       physical per-motor DC-current sensor, which is preferable to estimating
       battery current from Iq*duty. */
    volatile float input_current_map_start;
    volatile float input_current_map_filter;
    volatile float input_current_map_filtered_a;
    volatile float input_current_map_limit_a;
    volatile float board_temp_c;
    volatile float board_temp_filter_c;
    volatile bool board_temp_valid;

    /* Hard real-time representations. */
    volatile int32_t ia_q15;
    volatile int32_t ib_q15;
    volatile int32_t ic_q15;
    volatile int32_t id_q15;
    volatile int32_t iq_q15;
    volatile int32_t id_filter_q15;
    volatile int32_t iq_filter_q15;
    volatile int32_t vd_q15;
    volatile int32_t vq_q15;
    volatile uint16_t duty_u_q15;
    volatile uint16_t duty_v_q15;
    volatile uint16_t duty_w_q15;
    /* Shared ADC1/2 current sampling has one hardware-valid 10..90% window.
       These counters make modulation saturation visible during commissioning
       instead of pretending unsupported V0/V7/high-current sample modes work. */
    volatile uint32_t sampling_window_clamp_count;
    volatile uint16_t sampling_margin_min_q15;
    volatile int32_t vbus_q15;
    volatile int32_t abs_current_trip_q15;
    volatile int32_t abs_phase_current_filter_q15;
    volatile int32_t abs_current_peak_q15;
    volatile uint8_t abs_current_fault_count;
    volatile uint16_t dc_current_raw;
    volatile int32_t dc_current_q15;

    volatile float erpm;
    volatile int32_t erpm_int; /* slow-loop integer mirror for ISR debug capture */
    /* VESC-style low-latency electrical-speed estimators. They are updated
       directly from electrical phase deltas in the hard FOC ISR using Q16.16
       ERPM, so mcpwm_foc_get_rpm_fast/faster do not alias the slow 1-kHz RPM. */
    volatile int32_t speed_est_fast_erpm_q16;
    volatile int32_t speed_est_faster_erpm_q16;
    volatile uint16_t phase_before_speed_est_u16;
    volatile bool speed_est_phase_valid;
    volatile int32_t speed_est_fast_corrected_erpm_q16;
    volatile uint16_t phase_before_speed_est_corrected_u16;
    volatile bool speed_est_corrected_valid;
    FOC_SPEED_SRC foc_speed_source;
    volatile float mech_rpm;
    volatile float position_deg;
    volatile float rotor_elec_deg;

    float current_kp;
    float current_ki;
    /* Cached temp-comp / offset-cal mode flags (mirrored from mc_configuration
       on every SET_MCCONF). The 1-kHz loop and 16-kHz ISR read these directly
       instead of dereferencing the configuration mirror each tick. */
    bool foc_temp_comp;
    float foc_temp_comp_base_temp;
    uint8_t foc_offsets_cal_mode;
    /* Cached mirror of mc_configuration.foc_calibrate_on_boot. The 1-kHz loop
       and boot skip logic read this directly instead of dereferencing the
       configuration mirror each tick. */
    bool foc_calibrate_on_boot;
    /* VESC-style temperature-compensated R and Ki. The F103 port has no motor
       NTC, so the STM32 board-temperature proxy (board_temp_filter_c) is the
       thermal input. comp_factor = 1 + 0.00386*(T - base_temp); these hold
       foc_motor_r*comp and current_ki*comp, consumed by the observer/PI. */
    volatile float res_temp_comp_ohm;
    volatile float current_ki_temp_comp;
    float current_scale;
    float dc_current_scale;
    volatile float current_offset_u;
    volatile float current_offset_v;
    volatile float dc_current_offset;
    float vd_int;
    float vq_int;

    int32_t current_scale_q16;
    int32_t dc_current_scale_q16;
    /* PI coefficients are Q16.16; current/voltage samples remain Q15.
       Q15 error * Q16 gain >> 16 gives a Q15 voltage contribution. */
    int32_t current_kp_q16;
    int32_t current_ki_dt_q16;
    /* Fixed-point dq feed-forward coefficients. Current is Q15 on
       FOC_CURRENT_Q_BASE_A, voltage Q15 on FOC_VOLTAGE_Q_BASE_V and electrical
       speed Q16.16 ERPM. These are precomputed task-side from MCCONF. */
    int32_t decouple_ld_coeff_q30;
    int32_t decouple_lq_coeff_q30;
    int32_t bemf_flux_coeff_q30;
    /* VESC observer phase compensation factor (0.5 + foc_observer_offset)
       in signed Q15. It is consumed only by the fixed-point phase selector. */
    int32_t observer_offset_factor_q15;
    int32_t foc_mag_vd_max_q15;
    volatile int32_t current_offset_u_counts;
    volatile int32_t current_offset_v_counts;
    volatile int32_t dc_current_offset_counts;
    volatile uint16_t current_raw_u;
    volatile uint16_t current_raw_v;
    /* Integral state keeps 16 fractional bits below Q15 (Q31 accumulator).
       This avoids losing small Ki updates at 16 kHz on Cortex-M3. */
    volatile int64_t vd_int_q31;
    volatile int64_t vq_int_q31;
    volatile int32_t vd_int_q15; /* telemetry/debug mirror */
    volatile int32_t vq_int_q15;

    hall_state_t hall;
    int8_t hall_table[8];
    uint8_t foc_hall_table[8]; /* VESC style: 0..200, invalid=255 */
    uint16_t hall_angle_u16[8];
    uint16_t hall_offset_u16;
    encoder_state_t encoder;

    /* VESC-style FOC motor model and sensorless observer. HFI is deliberately
       not implemented on this STM32F103 port. Values mirror mc_configuration
       fields and are persisted through the canonical MCCONF wire image. */
    float foc_motor_r;
    float foc_motor_l;
    float foc_motor_ld_lq_diff;
    float foc_motor_flux_linkage;
    /* Online VESC-style motor-resistance estimate. It is diagnostic/adaptive
       state only; the observer keeps configured R unless a separate validated
       compensation policy is enabled. */
    volatile float res_est_ohm;
    volatile float res_est_state_ohm;
    volatile bool res_est_valid;
    /* Configured VESC dead-time compensation in microseconds. The derived
       Q15 fraction is foc_dt_us * FOC ISR frequency and is used only by the
       fixed-point applied-voltage model; hardware timer dead-time is unchanged. */
    float foc_dt_us;
    int32_t deadtime_comp_q15;
    float foc_observer_gain;
    float foc_observer_gain_slow;
    float foc_observer_offset;
    SAT_COMP_MODE foc_sat_comp_mode;
    float foc_sat_comp;
    mc_foc_observer_type foc_observer_type;
    float foc_current_filter_const;
    int32_t foc_current_filter_q15;
    float foc_duty_dowmramp_kp;
    float foc_duty_dowmramp_ki;
    float foc_pll_kp;
    float foc_pll_ki;
    mc_foc_cc_decoupling_mode foc_cc_decoupling;
    MTPA_MODE foc_mtpa_mode;
    float foc_fw_current_max;
    float foc_fw_duty_start;
    float foc_fw_ramp_time;
    float foc_fw_q_current_factor;
    float foc_fw_backoff;
    float foc_mag_vd_max;
    float foc_overmod_factor;
    volatile float foc_fw_current_now;
    volatile float mtpa_id_target;
    /* VESC 7.x-style fast field weakening backend. Float configuration is
       converted task-side; the hard FOC path only uses integer/Q-format math. */
    volatile int32_t foc_fw_current_acc_q31;
    volatile int32_t foc_fw_current_q15;
    volatile int32_t foc_fw_duty_filter_q15;
    volatile int32_t fw_override_current_q15;
    int32_t foc_fw_max_q15;
    int32_t foc_fw_duty_start_q15;
    int32_t foc_fw_duty_end_q15;
    int32_t foc_fw_duty_span_inv_q30;
    int32_t foc_fw_duty_norm_scale_q16;
    int32_t foc_fw_backoff_per_current_q16;
    int32_t foc_fw_q_factor_q15;
    int32_t foc_fw_ramp_step_q31;
    bool foc_fw_ramp_direct;
    volatile int32_t foc_current_limit_q15;
    volatile bool foc_fw_fast_active;
    volatile bool foc_fw_hold_request;
    float foc_hall_interp_erpm;
    float foc_sl_erpm_start;
    float foc_sl_erpm;
    /* Precomputed thresholds used by the 16-kHz ISR. Never convert float
       configuration values inside the hard current-control path. */
    int32_t foc_sl_erpm_start_q16;
    int32_t foc_sl_erpm_q16;
    uint32_t foc_hall_interp_erpm_u32; /* ISR threshold; no float in Hall fast path */
    float foc_openloop_rpm;
    float foc_openloop_rpm_low;
    float foc_sl_openloop_time_lock;
    float foc_sl_openloop_time_ramp;
    float foc_sl_openloop_time;
    float foc_sl_openloop_hyst;
    float foc_sl_openloop_boost_q;
    float foc_sl_openloop_max_q;

    volatile float observer_flux_alpha;
    volatile float observer_flux_beta;
    volatile float observer_phase_rad;
    volatile float observer_speed_rad_s;
    volatile float pll_phase_rad;
    volatile float pll_speed_rad_s;
    volatile float observer_phase_deg;
    volatile float observer_erpm;
    volatile float observer_quality;
    volatile float observer_v_alpha_prev;
    volatile float observer_v_beta_prev;
    volatile float observer_i_alpha_prev;
    volatile float observer_i_beta_prev;

    /* Fixed-point voltage-model flux observer. The hard ADC/FOC loop owns
       these states; slow task-side floats above are telemetry mirrors only. */
    volatile int64_t observer_stator_flux_alpha_q30;
    volatile int64_t observer_stator_flux_beta_q30;
    volatile int32_t observer_rotor_flux_alpha_q30;
    volatile int32_t observer_rotor_flux_beta_q30;
    volatile int32_t observer_v_alpha_q15_prev;
    volatile int32_t observer_v_beta_q15_prev;
    volatile int32_t observer_erpm_q16;
    volatile uint16_t observer_phase_last_u16;
    int32_t observer_r_i_to_v_q15;
    int32_t observer_l_i_to_flux_q15;
    int32_t observer_vdt_to_flux_q15;
    int32_t observer_flux_target_q30;
    /* Adaptive flux estimate used by the VESC lambda-compensated Ortega,
       MXLEMMING and MXV observers. Q2.30 on FOC_FLUX_Q_BASE_WB. */
    volatile int32_t observer_lambda_est_q30;
    /* Previous alpha/beta currents for the MXLEMMING current-difference term.
       Keeping them in Q15 avoids any floating point in the hard FOC ISR. */
    volatile int32_t observer_i_alpha_last_q15;
    volatile int32_t observer_i_beta_last_q15;
    /* Observer correction coefficient in Q2.30. It already includes dt and
       the normalization by FOC_FLUX_Q_BASE_WB, so the 16-kHz ISR only performs
       fixed-point multiplies. Updated by the 1-kHz observer service. */
    int32_t observer_gamma_coeff_q30;

    /* Fixed-point phase-locked loop. Phase is u16/revolution; speed is Q16.16
       electrical RPM. Kp*dt and Ki*dt*60 are precomputed task-side. */
    volatile uint16_t pll_phase_u16;
    volatile int32_t pll_erpm_q16;
    int32_t pll_kp_dt_q16;
    int32_t pll_ki_dt60_q16;

    volatile uint16_t observer_phase_u16;
    volatile uint32_t observer_update_cycle;
    volatile bool observer_valid;
    volatile bool using_encoder;
    /* Optional VESC 7 additional-fault backend for LEFT ABI. It compares the
       physically referenced encoder electrical phase with the compensated
       observer phase only above foc_openloop_rpm. */
    volatile uint16_t encoder_slip_bad_ticks;
    volatile int16_t encoder_slip_error_phase;
    volatile bool encoder_slip_check_active;
    volatile bool phase_observer_override;
    volatile uint16_t phase_observer_override_u16;
    volatile uint32_t openloop_start_tick;
    volatile bool openloop_started;
    volatile uint8_t sensorless_start_failures;
    volatile float openloop_erpm_now;

    /* Detection results are kept separate until explicitly applied/stored. */
    volatile float detect_resistance_ohm;
    volatile float detect_inductance_h;
    volatile float detect_ld_lq_diff_h;
    volatile float detect_flux_linkage_wb;
    volatile bool detect_rl_valid;
    volatile bool detect_flux_valid;

    pid_state_t speed_pid;
    pid_state_t position_pid;
    pid_state_t duty_pid;
    /* VESC duty/brake transition state. duty_pid.integrator is normalized
       (-1..1) while the dedicated duty down-ramp PI is active. */
    volatile bool duty_was_pi;
    volatile float duty_pi_duty_last;
    volatile bool force_zero_modulation;
    /* Brake zero-cross guard. A 1-kHz hold tick spans 16 FOC samples on this
       board, exceeding upstream's minimum 10-control-cycle zero-duty hold. */
    volatile int32_t brake_speed_before_q16;
    volatile int32_t brake_vq_before_q15;
    volatile uint8_t brake_zero_hold_ticks;
    volatile bool brake_zero_active;
    float speed_kd_filter;
    float speed_derivative_filtered;
    float speed_pid_min_erpm;
    float speed_pid_ramp_erpms_s;
    bool speed_pid_allow_braking;
    S_PID_SPEED_SRC speed_pid_source;
    float position_kd_filter;
    float position_kd_proc;
    float position_ang_div;
    float position_gain_dec_angle;
    float position_offset_deg;
    float duty_kp;
    float duty_ki;
    float position_derivative_filtered;
    float position_derivative_proc_filtered;
    float position_prev_process_deg;
    float position_dt_integrator;
    float position_dt_process_integrator;
    float cc_min_current;
    foc_detect_state_t detect;
    motor_stats_t stats;

    volatile uint32_t last_command_tick;
    volatile uint32_t isr_max_cycles;
    volatile uint32_t isr_overruns;

    /* Seqlock-protected ISR snapshot. Odd sequence = writer active; even =
       stable. Telemetry can copy a coherent 16-kHz frame without masking the
       ADC/DMA interrupt. */
    volatile uint32_t rt_snapshot_seq;
    volatile foc_rt_snapshot_t rt_snapshot;
} MotorRuntime;

typedef struct {
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float id;
    float iq;
    float id_filter;
    float iq_filter;
    float vd;
    float vq;
    float current_motor;
    float current_in;
    float duty;
    float erpm;
    float mech_rpm;
    float vbus;
    float position_deg;
    float rotor_elec_deg;
    float current_offset_u;
    float current_offset_v;
    float dc_current_offset;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t fault;
    uint8_t controller_id;
    uint8_t sensor_mode;          /* physical GPIO/timer input block */
    uint8_t foc_sensor_mode;      /* logical FOC phase source */
    uint8_t sensor_detect_state;
    uint8_t calibration_done;
    uint8_t calibration_valid;
    uint8_t timeout_active;
    uint8_t observer_valid;
    uint8_t using_encoder;
    uint8_t encoder_synced;
    float observer_phase_deg;
    float observer_erpm;
    float observer_quality;
    float foc_motor_r;
    float foc_motor_l;
    float foc_motor_ld_lq_diff;
    float foc_motor_flux_linkage;
    float foc_sl_erpm_start;
    float foc_sl_erpm;
    /* Precomputed thresholds used by the 16-kHz ISR. Never convert float
       configuration values inside the hard current-control path. */
    int32_t foc_sl_erpm_start_q16;
    int32_t foc_sl_erpm_q16;
    uint32_t foc_hall_interp_erpm_u32; /* ISR threshold; no float in Hall fast path */
    float foc_openloop_rpm;
    float foc_openloop_rpm_low;
    uint32_t current_loop_hz;
    uint32_t telemetry_snapshot_hz;
    uint32_t isr_max_cycles;
    uint32_t isr_overruns;
    float abs_current_filtered;
    float abs_current_peak;
    uint8_t over_voltage_fault_count;
    uint8_t under_voltage_fault_count;
} motor_telemetry_t;

typedef struct {
    int16_t ia_cA;
    int16_t ib_cA;
    int16_t id_cA;
    int16_t iq_cA;
    int16_t vd_cV;
    int16_t vq_cV;
    int16_t erpm;
    uint16_t phase_u16;
    uint16_t duty_u_q15;
    uint16_t duty_v_q15;
    uint16_t duty_w_q15;
    uint16_t current_raw_u;
    uint16_t current_raw_v;
    uint16_t vbus_dV;
    uint8_t motor;
    uint8_t hall_raw;
} debug_sample_t;
