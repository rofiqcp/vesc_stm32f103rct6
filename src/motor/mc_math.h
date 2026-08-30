#ifndef MC_MATH_H
#define MC_MATH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Task-side motor limit helpers. These functions are intentionally floating
 * point because they are never called from the 16-kHz hard FOC ISR. */
float mc_math_battery_cut_input_max(float configured_input_max,
                                    float vbus,
                                    float cut_start,
                                    float cut_end);
float mc_math_battery_regen_cut_input_min(float configured_input_min,
                                          float vbus,
                                          float cut_start,
                                          float cut_end);
float mc_math_limit_input_current(float iq,
                                  float erpm,
                                  float duty_now,
                                  float measured_input_current,
                                  float input_current_max,
                                  float input_current_min);

/* VESC-style 1-kHz thermal/startup limit helpers. These stay outside the
 * hard FOC ISR so the Cortex-M3 fast path remains fixed-point. */
float mc_math_thermal_current_limit(float current_abs_max,
                                    float temperature_c,
                                    float temp_start_c,
                                    float temp_end_c);
float mc_math_thermal_accel_limit(float current_max,
                                  float temperature_c,
                                  float temp_start_c,
                                  float temp_end_c,
                                  float temp_accel_dec);
float mc_math_start_current_limit(float current_max,
                                  float erpm_abs,
                                  float start_current_fraction,
                                  float start_current_erpm);

/* Estimate one dq-axis inductance from a PWM-rate current step.
 * v_axis_prev_q15[k] is the selected-axis voltage applied during the interval
 * that produced i_axis_q15[k]-i_axis_q15[k-1]. This runs only in the blocking
 * detect task and is shared by the B8 Ld and Lq measurements. */
bool mc_math_estimate_inductance_q15(const int32_t *i_axis_q15,
                                     const int32_t *v_axis_prev_q15,
                                     uint16_t count,
                                     float current_q_base_a,
                                     float voltage_q_base_v,
                                     float sample_hz,
                                     float resistance_ohm,
                                     float *inductance_h,
                                     float *current_a,
                                     uint16_t *valid_samples);

#ifdef __cplusplus
}
#endif

#endif
