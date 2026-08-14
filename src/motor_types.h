#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_id_t;

typedef enum {
    MOTOR_CTRL_OFF = 0,
    MOTOR_CTRL_CURRENT,
    MOTOR_CTRL_BRAKE_CURRENT,
    MOTOR_CTRL_SPEED,
    MOTOR_CTRL_POSITION,
    MOTOR_CTRL_DUTY_APPROX,
    MOTOR_CTRL_HANDBRAKE,
    MOTOR_CTRL_DETECT
} motor_control_mode_t;

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
    MOTOR_FAULT_SENSOR_DETECT
} motor_fault_t;

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
    volatile int32_t edge_count;
} hall_state_t;

typedef struct {
    volatile int32_t turns;
    volatile uint16_t last_cnt;
    volatile int32_t extended_count;
    uint32_t cpr;
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

typedef struct {
    volatile sensor_detect_state_t state;
    volatile bool requested;
    volatile bool busy;
    volatile bool success;
    volatile uint8_t result_mode;
    volatile float drive_current_a;
    volatile uint32_t step_tick;
    volatile uint32_t sweep_index;
    volatile uint32_t sweep_pass;
    volatile uint16_t forced_phase_u16;
    volatile int32_t encoder_start_count;
    volatile int32_t encoder_end_count;
    volatile uint8_t hall_valid_states;
    int64_t hall_sin_sum[8];
    int64_t hall_cos_sum[8];
    uint32_t hall_samples[8];
} sensor_detect_t;

typedef struct {
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    int32_t tachometer;
    int32_t tachometer_abs;
    int32_t tachometer_last;
    float max_current;
    float max_input_current;
    float max_erpm;
    uint32_t runtime_ms;
} motor_stats_t;

typedef struct MotorRuntime {
    motor_id_t id;
    TIM_TypeDef *pwm_tim;
    uint8_t pole_pairs;

    /* Runtime-selected sensor mode. 0=AUTO, 1=Hall, 2=Encoder. */
    volatile uint8_t sensor_mode;
    volatile uint8_t sensor_request_mode;

    volatile motor_control_mode_t control_mode;
    volatile motor_fault_t fault;
    volatile bool pwm_enabled;
    volatile bool command_active;
    volatile bool timeout_active;
    volatile bool detect_force_angle;
    volatile uint16_t detect_phase_u16;

    volatile float id_target;
    volatile float iq_target;
    volatile int32_t id_target_q15;
    volatile int32_t iq_target_q15;
    volatile float current_command_a;
    volatile float brake_current_a;
    volatile float handbrake_current_a;
    volatile float duty_command;
    volatile float speed_target_erpm;
    volatile float position_target_deg;

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
    volatile int32_t vbus_q15;
    volatile uint16_t dc_current_raw;

    volatile float erpm;
    volatile int32_t erpm_int; /* slow-loop integer mirror for ISR debug capture */
    volatile float mech_rpm;
    volatile float position_deg;
    volatile float rotor_elec_deg;

    float current_kp;
    float current_ki;
    float current_scale;
    float dc_current_scale;
    volatile float current_offset_u;
    volatile float current_offset_v;
    volatile float dc_current_offset;
    float vd_int;
    float vq_int;

    int32_t current_scale_q16;
    int32_t current_kp_q15;
    int32_t current_ki_dt_q15;
    volatile int32_t current_offset_u_counts;
    volatile int32_t current_offset_v_counts;
    volatile int32_t dc_current_offset_counts;
    volatile int64_t current_offset_u_acc_q16;
    volatile int64_t current_offset_v_acc_q16;
    volatile int64_t dc_current_offset_acc_q16;
    volatile uint16_t current_raw_u;
    volatile uint16_t current_raw_v;
    volatile int32_t vd_int_q15;
    volatile int32_t vq_int_q15;

    hall_state_t hall;
    int8_t hall_table[8];
    uint8_t foc_hall_table[8]; /* VESC style: 0..200, invalid=255 */
    uint16_t hall_angle_u16[8];
    uint16_t hall_offset_u16;
    encoder_state_t encoder;

    pid_state_t speed_pid;
    sensor_detect_t detect;
    motor_stats_t stats;

    volatile uint32_t last_command_tick;
    volatile uint32_t isr_max_cycles;
    volatile uint32_t isr_overruns;
} MotorRuntime;

typedef struct {
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
    uint8_t sensor_mode;
    uint8_t sensor_detect_state;
    uint8_t calibration_done;
    uint8_t calibration_valid;
    uint8_t timeout_active;
    uint32_t isr_max_cycles;
    uint32_t isr_overruns;
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
