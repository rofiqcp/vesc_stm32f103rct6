#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

/* ========================================================================
 * VESC 6.00 FOC-subset public mcpwm_foc API.
 * The normal control/status signatures retain the upstream names. This F103
 * target intentionally does not provide HFI types/runtime, motor-audio APIs,
 * or mcpwm_foc_tim_sample_int_handler(); ADC/DMA sampling is owned by the
 * platform-specific mcpwm_foc_adc_words_isr() path. Callers must feature-gate
 * those capabilities rather than treating this header as full master parity.
 * ======================================================================== */
void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2);
void mcpwm_foc_deinit(void);
bool mcpwm_foc_init_done(void);
void mcpwm_foc_set_configuration(mc_configuration *configuration);
mc_state mcpwm_foc_get_state(void);
mc_control_mode mcpwm_foc_control_mode(void);
bool mcpwm_foc_is_dccal_done(void);
int mcpwm_foc_isr_motor(void);
void mcpwm_foc_stop_pwm(bool is_second_motor);
void mcpwm_foc_set_duty(float dutyCycle);
void mcpwm_foc_set_duty_noramp(float dutyCycle);
void mcpwm_foc_set_pid_speed(float rpm);
void mcpwm_foc_set_pid_pos(float pos);
void mcpwm_foc_set_current(float current);
void mcpwm_foc_release_motor(void);
void mcpwm_foc_set_brake_current(float current);
void mcpwm_foc_set_handbrake(float current);
void mcpwm_foc_set_openloop_current(float current, float rpm);
void mcpwm_foc_set_openloop_phase(float current, float phase);
void mcpwm_foc_set_openloop_duty(float dutyCycle, float rpm);
void mcpwm_foc_set_openloop_duty_phase(float dutyCycle, float phase);
void mcpwm_foc_set_fw_override(float current);
int mcpwm_foc_set_tachometer_value(int steps);
float mcpwm_foc_get_duty_cycle_set(void);
float mcpwm_foc_get_duty_cycle_now(void);
float mcpwm_foc_get_duty_cycle_abs_filter(void);
float mcpwm_foc_get_pid_speed_set(void);
float mcpwm_foc_get_pid_pos_set(void);
float mcpwm_foc_get_pid_pos_now(void);
float mcpwm_foc_get_switching_frequency_now(void);
float mcpwm_foc_get_sampling_frequency_now(void);
float mcpwm_foc_get_rpm(void);
float mcpwm_foc_get_rpm_fast(void);
float mcpwm_foc_get_rpm_faster(void);
float mcpwm_foc_get_tot_current(void);
float mcpwm_foc_get_tot_current_filtered(void);
float mcpwm_foc_get_abs_motor_current(void);
float mcpwm_foc_get_abs_motor_current_unbalance(void);
float mcpwm_foc_get_abs_motor_voltage(void);
float mcpwm_foc_get_abs_motor_current_filtered(void);
float mcpwm_foc_get_tot_current_directional(void);
float mcpwm_foc_get_tot_current_directional_filtered(void);
float mcpwm_foc_get_id(void);
float mcpwm_foc_get_iq(void);
float mcpwm_foc_get_id_set(void);
float mcpwm_foc_get_iq_set(void);
float mcpwm_foc_get_id_target(void);
float mcpwm_foc_get_iq_target(void);
float mcpwm_foc_get_id_filter(void);
float mcpwm_foc_get_iq_filter(void);
float mcpwm_foc_get_tot_current_in(void);
float mcpwm_foc_get_tot_current_in_filtered(void);
int mcpwm_foc_get_tachometer_value(bool reset);
int mcpwm_foc_get_tachometer_abs_value(bool reset);
float mcpwm_foc_get_phase(void);
float mcpwm_foc_get_phase_observer(void);
float mcpwm_foc_get_phase_bemf(void);
float mcpwm_foc_get_phase_encoder(void);
float mcpwm_foc_get_phase_hall(void);
float mcpwm_foc_get_vd(void);
float mcpwm_foc_get_vq(void);
/* Modulation and voltage getters below expose commanded/reconstructed values
 * derived from duty and Vbus. This board has no independent phase-voltage ADC
 * channels; these are not phase-voltage measurements. */
float mcpwm_foc_get_mod_alpha_raw(void);
float mcpwm_foc_get_mod_beta_raw(void);
float mcpwm_foc_get_mod_alpha_measured(void);
float mcpwm_foc_get_mod_beta_measured(void);
float mcpwm_foc_get_v_alpha(void);
float mcpwm_foc_get_v_beta(void);
float mcpwm_foc_get_est_lambda(void);
float mcpwm_foc_get_est_res(void);
float mcpwm_foc_get_est_ind(void);
int mcpwm_foc_encoder_detect(float current, bool print, float *offset, float *ratio, bool *inverted);
int mcpwm_foc_measure_resistance(float current, int samples, bool stop_after, float *resistance);
int mcpwm_foc_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff, float *inductance);
int mcpwm_foc_measure_inductance_current(float curr_goal, int samples, float *curr, float *ld_lq_diff, float *inductance);
int mcpwm_foc_measure_res_ind(float *res, float *ind, float *ld_lq_diff);
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result);
int mcpwm_foc_dc_cal(bool cal_undriven);
void mcpwm_foc_print_state(void);
void mcpwm_foc_get_current_offsets(volatile float *curr0_offset, volatile float *curr1_offset,
                                   volatile float *curr2_offset, bool is_second_motor);
void mcpwm_foc_set_current_offsets(volatile float curr0_offset, volatile float curr1_offset,
                                   volatile float curr2_offset);
void mcpwm_foc_get_voltage_offsets(float *v0_offset, float *v1_offset, float *v2_offset, bool is_second_motor);
void mcpwm_foc_get_voltage_offsets_undriven(float *v0_offset, float *v1_offset, float *v2_offset, bool is_second_motor);
void mcpwm_foc_get_currents_adc(float *ph0, float *ph1, float *ph2, bool is_second_motor);
float mcpwm_foc_get_ts(void);
bool mcpwm_foc_is_using_encoder(void);
void mcpwm_foc_get_observer_state(float *x1, float *x2);
void mcpwm_foc_set_current_off_delay(float delay_sec);
float mcpwm_foc_get_tot_current_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_filtered_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_filtered_motor(bool is_second_motor);
float mcpwm_foc_get_abs_motor_current_motor(bool is_second_motor);
float mcpwm_foc_get_abs_motor_current_filtered_motor(bool is_second_motor);
mc_state mcpwm_foc_get_state_motor(bool is_second_motor);
void mcpwm_foc_adc_int_handler(void *p, uint32_t flags);

/* ========================================================================
 * STM32F103 fixed-point/internal hooks.
 * Board/dual-motor code uses these explicit-runtime helpers; the VESC public
 * API above is a thin selected-motor wrapper around them.
 * ======================================================================== */
void mcpwm_foc_init_hw(void);
void mcpwm_foc_adc_words_isr(const volatile uint32_t adc_words[6]);
static inline void foc_adc_dma_isr(const volatile uint32_t adc_words[6]) { mcpwm_foc_adc_words_isr(adc_words); }
void mcpwm_foc_set_duty_motor(MotorRuntime *m, float duty);
void mcpwm_foc_set_duty_noramp_motor(MotorRuntime *m, float duty);
void mcpwm_foc_set_pid_speed_motor(MotorRuntime *m, float erpm);
void mcpwm_foc_set_pid_pos_motor(MotorRuntime *m, float pos_deg);
void mcpwm_foc_set_current_motor(MotorRuntime *m, float current);
void mcpwm_foc_set_brake_current_motor(MotorRuntime *m, float current);
void mcpwm_foc_set_handbrake_motor(MotorRuntime *m, float current);
void mcpwm_foc_set_openloop_current_motor(MotorRuntime *m, float current, float erpm);
void mcpwm_foc_set_openloop_phase_motor(MotorRuntime *m, float current, float phase_deg);
void mcpwm_foc_set_openloop_duty_motor(MotorRuntime *m, float duty, float erpm);
void mcpwm_foc_set_openloop_duty_phase_motor(MotorRuntime *m, float duty, float phase_deg);
void mcpwm_foc_release_motor_motor(MotorRuntime *m);
void mcpwm_foc_set_current_off_delay_motor(MotorRuntime *m, float delay_s);
float mcpwm_foc_get_tot_current_rt(const MotorRuntime *m);
float mcpwm_foc_get_phase_observer_rt(const MotorRuntime *m);
float mcpwm_foc_get_phase_encoder_rt(const MotorRuntime *m);
float mcpwm_foc_get_phase_bemf_rt(const MotorRuntime *m);
float mcpwm_foc_get_phase_hall_rt(const MotorRuntime *m);
int mcpwm_foc_encoder_detect_motor(MotorRuntime *m, float current, bool print, float *offset, float *ratio, bool *inverted);
int mcpwm_foc_hall_detect_motor(MotorRuntime *m, float current, uint8_t *hall_table, bool *result);
int mcpwm_foc_measure_resistance_motor(MotorRuntime *m, float current, int samples, bool stop_after, float *resistance);
int mcpwm_foc_measure_inductance_motor(MotorRuntime *m, float duty, int samples, float *curr, float *ld_lq_diff, float *inductance);
int mcpwm_foc_measure_inductance_current_motor(MotorRuntime *m, float curr_goal, int samples, float *curr, float *ld_lq_diff, float *inductance);
int mcpwm_foc_measure_res_ind_motor(MotorRuntime *m, float *res, float *ind, float *ld_lq_diff);
int mcpwm_foc_measure_flux_linkage_motor(MotorRuntime *m, float current_a, float target_erpm, float erpm_per_sec, float *flux_wb);
int mcpwm_foc_measure_flux_linkage_motor_bounded(MotorRuntime *m, float current_a, float target_erpm, float erpm_per_sec, float max_duty, float resistance_ohm, float inductance_h, float *flux_wb);
int16_t mcpwm_foc_detect_apply_all_motor(MotorRuntime *m, float current_a);

/* Current calibration/debug API owned by this F103 port. */
typedef enum {
    FOC_CAL_STAGE_UNDRIVEN = 0, FOC_CAL_STAGE_WAIT_LEFT_DRIVEN,
    FOC_CAL_STAGE_LEFT_WARMUP, FOC_CAL_STAGE_LEFT_DRIVEN,
    FOC_CAL_STAGE_WAIT_RIGHT_DRIVEN, FOC_CAL_STAGE_RIGHT_WARMUP,
    FOC_CAL_STAGE_RIGHT_DRIVEN, FOC_CAL_STAGE_WAIT_FINALIZE,
    FOC_CAL_STAGE_DONE, FOC_CAL_STAGE_FAILED
} foc_cal_stage_t;
typedef struct {
    uint8_t valid,motor,fault,cal_stage; uint16_t raw_u,raw_v,raw_dc;
    int32_t offset_u,offset_v,offset_dc,ia_q15,ib_q15,ic_q15,trip_q15,id_target_q15,iq_target_q15;
    uint16_t ccr1,ccr2,ccr3,tim_cnt,dma_cndtr; uint32_t adc_isr_count; uint16_t blank_cycles;
    uint8_t pwm_enabled,moe,pending_events,reserved;
} foc_fault_snapshot_t;
typedef struct {
    int32_t mean;
    uint16_t min, max;
    uint32_t variance_x100;
} foc_cal_channel_diag_t;
typedef struct {
    uint8_t stage, reserved;
    uint16_t shift_warn_mask, warn_mask, fail_range_mask, fail_noise_mask;
    foc_cal_channel_diag_t ch[6];
    int32_t undriven_mean[6], driven_mean[6];
    uint16_t outlier_count[6];
    uint8_t moe_fail_mask, moe_confirmed_mask;
    uint32_t moe_request_adc[2], moe_confirm_adc[2], first_sample_adc[2];
    uint32_t moe_drop_adc[2];
    uint8_t moe_drop_bdtr[2], moe_drop_pending[2], moe_drop_pwm_enabled[2];
} foc_cal_diag_t;
bool foc_calibration_done(void);
bool foc_calibration_valid(void);
bool foc_calibration_in_progress(void);
foc_cal_stage_t foc_calibration_stage(void);
void foc_calibration_service_task(void);
void foc_request_recalibration(void);
uint32_t foc_adc_isr_count(void);
uint32_t foc_isr_total_max_cycles(void);
float foc_last_isr_duration_s(void);
uint32_t foc_isr_near_deadline_count(void);
uint32_t foc_isr_period_min_cycles(void);
uint32_t foc_isr_period_max_cycles(void);
uint32_t foc_vbus_dma_stale_events(void);
uint8_t foc_vbus_dma_stale_count(void);
void foc_get_calibration_progress(uint32_t *count, uint32_t *target);
void foc_get_calibration_diag(foc_cal_diag_t *out);
void foc_get_fault_snapshot(foc_fault_snapshot_t *out);
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m);
void motor_hall_edge_isr(MotorRuntime *m);
