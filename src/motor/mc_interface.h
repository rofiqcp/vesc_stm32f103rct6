#pragma once
#include "datatypes.h"

extern MotorRuntime g_motor_left;
extern MotorRuntime g_motor_right;

void motor_control_init(void);
MotorRuntime *motor_get(motor_id_t id);
void motor_set_current(MotorRuntime *m, float amp);
void motor_set_brake_current(MotorRuntime *m, float amp);
void motor_set_handbrake(MotorRuntime *m, float amp);
void motor_set_current_rel(MotorRuntime *m, float rel);
void motor_set_speed(MotorRuntime *m, float erpm);
void motor_set_position(MotorRuntime *m, float deg);
void motor_set_duty(MotorRuntime *m, float duty);
void motor_stop(MotorRuntime *m);
void motor_clear_fault(MotorRuntime *m);
/* Force-clear a fault for a safe stopped recalibration. Hardware-latched
 * power-stage faults (PVD/BKIN/break) still refuse reset; a config-flash fault
 * is cleared so offset calibration can arm the 50% zero-vector MOE handshake. */
void motor_clear_fault_for_cal(MotorRuntime *m);
void motor_touch_command(MotorRuntime *m);
void motor_keepalive(MotorRuntime *m);
void motor_slow_update_1khz(MotorRuntime *m, uint32_t now_ms);
void motor_rpm_update_1khz(MotorRuntime *m);
void motor_pid_update_1khz(MotorRuntime *m);
void motor_raise_fault_from_task(MotorRuntime *m, motor_fault_t fault);
void motor_request_fault_from_isr(MotorRuntime *m, motor_fault_t fault);
uint32_t motor_take_pending_fault_mask(void);

/* Board-internal fault <-> VESC wire/API mapping. Never expose motor_fault_t
 * numerically on the VESC protocol; its values are intentionally local. */
mc_fault_code motor_fault_to_vesc(motor_fault_t fault);
motor_fault_t motor_fault_from_vesc(mc_fault_code fault);

bool motor_select_sensor_mode(MotorRuntime *m, uint8_t mode);
void motor_set_foc_targets(MotorRuntime *m, float id_a, float iq_a);
void motor_set_current_pi_gains(MotorRuntime *m, float kp, float ki);
int32_t motor_encoder_extended_count(MotorRuntime *m);

/* ========================================================================
 * VESC 6.00 FOC-subset command/status mc_interface API.
 * The upstream-shaped mc_interface_* wrappers provide VESC Tool compatibility.
 * The explicit MotorRuntime helpers above and the F103-only mc_interface_*
 * additions (resource stats, per-motor odometer, input-gate helpers,
 * mc_interface_motor_runtime_now bridge) are port extensions, not VESC public
 * API guarantees. mc_interface_adc_inj_int_handler() is intentionally absent;
 * this target uses the platform ADC/DMA ISR instead.
 * ======================================================================== */
void mc_interface_init(bool reset_conf);
bool mc_interface_start_threads(void);

typedef struct {
	uint32_t heap_free_bytes;
	uint32_t heap_min_ever_bytes;
	uint32_t motor_service_stack_free_bytes;
	uint32_t sample_sender_stack_free_bytes;
	uint32_t fault_stack_free_bytes;
	uint32_t status_stack_free_bytes;
} mc_interface_resource_stats_t;

uint32_t mc_interface_free_heap_bytes(void);
uint32_t mc_interface_min_ever_free_heap_bytes(void);
void mc_interface_get_resource_stats(mc_interface_resource_stats_t *stats);

int mc_interface_motor_now(void);
void mc_interface_select_motor_thread(int motor);
int mc_interface_get_motor_thread(void);
MotorRuntime *mc_interface_motor_runtime_now(void); /* F103 internal bridge */
const volatile mc_configuration* mc_interface_get_configuration(void);
void mc_interface_set_configuration(mc_configuration *configuration);
unsigned mc_interface_calc_crc(mc_configuration* conf, bool is_motor_2);
bool mc_interface_dccal_done(void);
void mc_interface_set_pwm_callback(void (*p_func)(void));
void mc_interface_lock(void);
void mc_interface_unlock(void);
void mc_interface_lock_override_once(void);
mc_fault_code mc_interface_get_fault(void);
const char* mc_interface_fault_to_string(mc_fault_code fault);
mc_state mc_interface_get_state(void);
mc_control_mode mc_interface_get_control_mode(void);
void mc_interface_set_duty(float dutyCycle);
void mc_interface_set_duty_noramp(float dutyCycle);
void mc_interface_set_pid_speed(float rpm);
void mc_interface_set_pid_pos(float pos);
void mc_interface_set_current(float current);
void mc_interface_set_brake_current(float current);
void mc_interface_set_current_rel(float val);
void mc_interface_set_brake_current_rel(float val);
void mc_interface_set_handbrake(float current);
void mc_interface_set_handbrake_rel(float val);
void mc_interface_set_openloop_current(float current, float rpm);
void mc_interface_set_openloop_phase(float current, float phase);
void mc_interface_set_openloop_duty(float dutyCycle, float rpm);
void mc_interface_set_openloop_duty_phase(float dutyCycle, float phase);
int mc_interface_set_tachometer_value(int steps);
void mc_interface_brake_now(void);
void mc_interface_release_motor(void);
void mc_interface_release_motor_override(void);
bool mc_interface_wait_for_motor_release(float timeout);
float mc_interface_get_duty_cycle_set(void);
float mc_interface_get_duty_cycle_now(void);
float mc_interface_get_sampling_frequency_now(void);
float mc_interface_get_rpm(void);
float mc_interface_get_amp_hours(bool reset);
float mc_interface_get_amp_hours_charged(bool reset);
float mc_interface_get_watt_hours(bool reset);
float mc_interface_get_watt_hours_charged(bool reset);
float mc_interface_get_tot_current(void);
float mc_interface_get_tot_current_filtered(void);
float mc_interface_get_tot_current_directional(void);
float mc_interface_get_tot_current_directional_filtered(void);
float mc_interface_get_tot_current_in(void);
float mc_interface_get_tot_current_in_filtered(void);
float mc_interface_get_input_voltage_filtered(void);
float mc_interface_get_abs_motor_current_unbalance(void);
int mc_interface_get_tachometer_value(bool reset);
int mc_interface_get_tachometer_abs_value(bool reset);
float mc_interface_get_last_inj_adc_isr_duration(void);
float mc_interface_read_reset_avg_motor_current(void);
float mc_interface_read_reset_avg_input_current(void);
float mc_interface_read_reset_avg_id(void);
float mc_interface_read_reset_avg_iq(void);
float mc_interface_read_reset_avg_vd(void);
float mc_interface_read_reset_avg_vq(void);
float mc_interface_get_pid_pos_set(void);
float mc_interface_get_pid_pos_now(void);
void mc_interface_update_pid_pos_offset(float angle_now, bool store);
float mc_interface_get_last_sample_adc_isr_duration(void);
void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len, uint8_t decimation,
		bool raw, void (*reply_func)(unsigned char *data, unsigned int len));
/* The initiating VESC transport for the asynchronous debug-sample sender.
 * This preserves upstream reply_func routing instead of silently falling back
 * to the board's default UART queue. */
void (*mc_interface_sample_reply_func(void))(unsigned char *data, unsigned int len);

/* --- Debug sample buffer (VESC-standard mc_interface ownership) --- */
void mc_interface_sample_init(void);
bool mc_interface_sample_control(debug_sampling_mode mode, motor_id_t motor,
		uint16_t len, uint16_t decimation, bool raw);
void mc_interface_sample_start(motor_id_t motor, uint16_t len,
		uint16_t decimation);
void mc_interface_sample_start_ex(motor_id_t motor, uint16_t len,
		uint16_t decimation, bool raw);
void mc_interface_sample_capture_isr(MotorRuntime *active);
bool mc_interface_sample_ready(void);
bool mc_interface_sample_has_capture(void);
uint16_t mc_interface_sample_count(void);
const debug_sample_t *mc_interface_sample_data(void);
const debug_sample_t *mc_interface_sample_at(uint16_t logical_index);
void mc_interface_sample_mark_sent(void);
bool mc_interface_sample_active(void);
bool mc_interface_sample_raw(void);
debug_sampling_mode mc_interface_sample_mode(void);
float mc_interface_temp_fet_filtered(void);
float mc_interface_temp_motor_filtered(void);
float mc_interface_get_battery_level(float *wh_left);
float mc_interface_get_speed(void);
float mc_interface_get_distance(void);
float mc_interface_get_distance_abs(void);
void mc_interface_odometer_add_tach_delta(motor_id_t id, uint32_t abs_tach_steps);
void mc_interface_override_wheel_speed(bool ovr, float speed);
setup_values mc_interface_get_setup_values(void);
volatile gnss_data *mc_interface_gnss(void);
uint64_t mc_interface_get_odometer(void);
void mc_interface_set_odometer(uint64_t new_odometer_meters);
uint64_t mc_interface_get_odometer_motor(motor_id_t id);
void mc_interface_set_odometer_motor(motor_id_t id, uint64_t new_odometer_meters);
void mc_interface_ignore_input(int time_ms);
void mc_interface_set_current_off_delay(float delay_sec);
void mc_interface_override_temp_motor(float temp);
void mc_interface_ignore_input_both(int time_ms);
void mc_interface_release_motor_override_both(void);
bool mc_interface_wait_for_motor_release_both(float timeout);
float mc_interface_stat_speed_avg(void);
float mc_interface_stat_speed_max(void);
float mc_interface_stat_power_avg(void);
float mc_interface_stat_power_max(void);
float mc_interface_stat_current_avg(void);
float mc_interface_stat_current_max(void);
float mc_interface_stat_temp_mosfet_avg(void);
float mc_interface_stat_temp_mosfet_max(void);
float mc_interface_stat_temp_motor_avg(void);
float mc_interface_stat_temp_motor_max(void);
float mc_interface_stat_count_time(void);
void mc_interface_stat_reset(void);
void mc_interface_set_fault_info(const char *str, int argn, float arg0, float arg1);
void mc_interface_fault_stop(mc_fault_code fault, bool is_second_motor, bool is_isr);
int mc_interface_try_input(void);
int mc_interface_try_input_motor(motor_id_t id);
void mc_interface_mc_timer_isr(bool is_second_motor, float dt);
