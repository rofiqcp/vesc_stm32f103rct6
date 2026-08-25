#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"
#include "motor/foc_math.h"
#include "applications/appconf_default.h"

/* Host symbols required by the STM32 stubs. */
TIM_TypeDef _stub_tim1, _stub_tim8;
DMA_Channel_TypeDef _stub_dma2_ch5;
DWT_Type _stub_dwt;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int32_t volt_to_q15(float v) {
    float q = v / FOC_VOLTAGE_Q_BASE_V * 32768.0f;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int32_t)lrintf(q);
}

static int32_t amp_to_q15(float a) {
    float q = a / FOC_CURRENT_Q_BASE_A * 32768.0f;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int32_t)lrintf(q);
}

static float phase_error_deg(uint16_t estimate, float theta) {
    float wrapped = fmodf(theta, 2.0f * (float)M_PI);
    if (wrapped < 0.0f) wrapped += 2.0f * (float)M_PI;
    const uint16_t truth = (uint16_t)lrintf(wrapped * (65536.0f / (2.0f * (float)M_PI)));
    const int16_t delta = (int16_t)(truth - estimate);
    return fabsf((float)delta * (360.0f / 65536.0f));
}

static bool lambda_type(int type) {
    return type == FOC_OBSERVER_ORTEGA_LAMBDA_COMP ||
           type == FOC_OBSERVER_MXLEMMING_LAMBDA_COMP ||
           type == FOC_OBSERVER_MXV_LAMBDA_COMP ||
           type == FOC_OBSERVER_MXV_LAMBDA_COMP_LIN;
}

static int run_case(int type, float erpm, float iq_a) {
    const float flux = 0.012f;
    const float r = 0.10f;
    const float l = 0.00020f;
    const float dt = FOC_DT_S;
    const float omega = erpm * (2.0f * (float)M_PI / 60.0f);
    const float dtheta = omega * dt;

    MotorRuntime m;
    memset(&m, 0, sizeof(m));
    m.foc_motor_r = r;
    m.foc_motor_l = l;
    m.foc_motor_ld_lq_diff = 0.0f;
    m.foc_motor_flux_linkage = flux;
    m.foc_observer_gain = 2000.0f;
    m.foc_observer_gain_slow = 0.10f;
    m.foc_pll_kp = 1000.0f;
    m.foc_pll_ki = 50000.0f;
    m.foc_sl_erpm_start = 100.0f;
    m.foc_sl_erpm = 800.0f;
    m.current_max_a = 30.0f;
    m.foc_observer_type = (mc_foc_observer_type)type;
    foc_observer_reset(&m, 0U);

    float theta_prev = 0.0f;
    float ia_prev = 0.0f;
    float ib_prev = iq_a;
    float psi_a_prev = flux + l * ia_prev;
    float psi_b_prev = l * ib_prev;

    /* Reset assumes zero current. Give the observer one coherent initial sample
       at theta=0 before entering the trajectory so L*i state has a causal start. */
    foc_observer_update_fixed(&m, volt_to_q15(r * ia_prev), volt_to_q15(r * ib_prev),
                              amp_to_q15(ia_prev), amp_to_q15(ib_prev));

    for (int k = 1; k <= 4000; k++) {
        const float theta = dtheta * (float)k;
        const float c = cosf(theta);
        const float s = sinf(theta);
        /* q-axis current rotating with the rotor. */
        const float ia = -iq_a * s;
        const float ib =  iq_a * c;
        const float psi_a = flux * c + l * ia;
        const float psi_b = flux * s + l * ib;
        /* PMSM alpha/beta voltage equation: v = R*i + d(psi_s)/dt. */
        const float va = r * ia + (psi_a - psi_a_prev) / dt;
        const float vb = r * ib + (psi_b - psi_b_prev) / dt;
        foc_observer_update_fixed(&m, volt_to_q15(va), volt_to_q15(vb),
                                  amp_to_q15(ia), amp_to_q15(ib));
        ia_prev = ia;
        ib_prev = ib;
        psi_a_prev = psi_a;
        psi_b_prev = psi_b;
        theta_prev = theta;
    }

    const float err = phase_error_deg(m.observer_phase_u16, theta_prev);
    const float lam = (float)m.observer_lambda_est_q30 *
                      (FOC_FLUX_Q_BASE_WB / 1073741824.0f);
    const float fa = (float)m.observer_rotor_flux_alpha_q30 *
                     (FOC_FLUX_Q_BASE_WB / 1073741824.0f);
    const float fb = (float)m.observer_rotor_flux_beta_q30 *
                     (FOC_FLUX_Q_BASE_WB / 1073741824.0f);
    const float mag = sqrtf(fa * fa + fb * fb);

    printf("observer=%d erpm=%+.0f iq=%+.1f err_deg=%.3f flux=%.6f lambda=%.6f valid=%d\n",
           type, (double)erpm, (double)iq_a, (double)err,
           (double)mag, (double)lam, m.observer_valid ? 1 : 0);

    if (!isfinite(err) || err > 5.0f) return 1;
    if (!isfinite(mag) || mag < flux * 0.25f || mag > flux * 3.0f) return 1;
    if (lambda_type(type) &&
        (!isfinite(lam) || lam < flux * 0.30f || lam > flux * 2.50f)) return 1;
    return 0;
}

int main(void) {
    int failures = 0;
    for (int type = FOC_OBSERVER_ORTEGA_ORIGINAL;
         type <= FOC_OBSERVER_MXV_LAMBDA_COMP_LIN; type++) {
        failures += run_case(type, 3000.0f, 0.0f);
        failures += run_case(type, 3000.0f, 4.0f);
        failures += run_case(type, -2500.0f, -3.0f);
    }
    if (failures != 0) {
        fprintf(stderr, "observer runtime failures=%d\n", failures);
        return 1;
    }
    puts("ALL OBSERVER RUNTIME TRAJECTORIES: PASS");
    return 0;
}
