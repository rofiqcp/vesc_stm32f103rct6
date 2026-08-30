#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct MotorRuntime MotorRuntime;
typedef struct mc_configuration mc_configuration;

#define FOC_Q15_ONE               32768
#define FOC_Q15_HALF              16384
#define FOC_Q15_INV_SQRT3         18919   /* 1/sqrt(3) */
#define FOC_Q15_SQRT3_BY_2        28378   /* sqrt(3)/2 */

void foc_math_init(void);
void foc_fast_sincos_u16_q15(uint16_t phase, int32_t *s, int32_t *c);
int32_t foc_q15_mul(int32_t a, int32_t b);
int32_t foc_q15_clamp(int32_t x, int32_t lo, int32_t hi);
void foc_svm_q15(int32_t v_alpha_q15, int32_t v_beta_q15,
                 int32_t inv_vbus_q30,
                 uint16_t *d_u_q15, uint16_t *d_v_q15, uint16_t *d_w_q15);
/* Reconstruct the alpha/beta voltage that the centered PWM duties actually
 * apply. Common-mode offset cancels in Clarke, so this also captures any
 * vector-preserving SVM scaling used to keep the hoverboard sampling window. */
void foc_pwm_applied_voltage_q15(uint16_t d_u_q15, uint16_t d_v_q15,
                                 uint16_t d_w_q15, int32_t vbus_q15,
                                 int32_t *v_alpha_q15, int32_t *v_beta_q15);
/* Correct an applied alpha/beta voltage model for PWM dead-time using the
 * signs of the three phase currents. deadtime_comp_q15 is foc_dt_us*foc_f_zv. */
void foc_deadtime_compensate_voltage_q15(int32_t ia_q15, int32_t ib_q15,
                                         int32_t ic_q15, int32_t vbus_q15,
                                         int32_t deadtime_comp_q15,
                                         int32_t *v_alpha_q15,
                                         int32_t *v_beta_q15);

float foc_clampf(float x, float lo, float hi);
uint16_t foc_deg_to_u16(float deg);
float foc_wrap_deg(float deg);


/* VESC-style observer/PLL and FOC_SENSOR_MODE_ENCODER_AB runtime support.
 * Implemented in foc_math.c so the file ownership follows upstream VESC. */
void foc_observer_precalc(MotorRuntime *m);
void foc_observer_reset(MotorRuntime *m, uint16_t phase_u16);
void foc_observer_update_fixed(MotorRuntime *m, int32_t v_alpha_q15, int32_t v_beta_q15,
                         int32_t i_alpha_q15, int32_t i_beta_q15);
void foc_pll_run_fixed(MotorRuntime *m, uint16_t phase_u16);
void foc_observer_update_1khz(MotorRuntime *m);
bool foc_encoder_ab_startup_1khz(MotorRuntime *m, uint32_t now_ms);
bool foc_sensorless_startup_1khz(MotorRuntime *m, uint32_t now_ms,
                                  float direction_hint, float iq_hint_a);
void foc_sensorless_startup_abort(MotorRuntime *m);
void foc_encoder_ab_sync_from_observer(MotorRuntime *m);


/* Current VESC foc_math public surface. motor_all_state_t maps to the compact
 * F103 MotorRuntime; hard-loop arithmetic remains fixed-point. HFI types and
 * HFI angle-adjust runtime is intentionally omitted by build requirement. */
typedef MotorRuntime motor_all_state_t;
typedef struct {
    float x1;
    float x2;
    float lambda_est;
    float i_alpha_last;
    float i_beta_last;
} observer_state;

void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
                         float dt, observer_state *state, float *phase,
                         motor_all_state_t *motor);
void foc_pll_run(float phase, float dt, float *phase_var, float *speed_var,
                 mc_configuration *conf);
void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
             uint32_t *tAout, uint32_t *tBout, uint32_t *tCout,
             uint32_t *svm_sector);
void foc_run_pid_control_pos(bool index_found, float dt, motor_all_state_t *motor);
void foc_run_pid_control_speed(bool index_found, float dt, motor_all_state_t *motor);
float foc_correct_encoder(float obs_angle, float enc_angle, float speed,
                          float sl_erpm, motor_all_state_t *motor);
float foc_correct_hall(float angle, float dt, motor_all_state_t *motor, int hall_val);
void foc_run_fw(motor_all_state_t *motor, float dt);
void foc_precalc_values(motor_all_state_t *motor);
void foc_update_modulation_limit(motor_all_state_t *motor);
