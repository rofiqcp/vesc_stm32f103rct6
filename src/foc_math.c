#include "foc_math.h"
#include "app_config.h"
#include <math.h>

#define LUT_BITS 10U
#define LUT_SIZE (1U << LUT_BITS)
static int16_t s_sin_lut[LUT_SIZE];

void foc_math_init(void) {
    /* Boot-time only. No sinf/cosf is used from the current-control ISR. */
    for (uint32_t i = 0; i < LUT_SIZE; i++) {
        float v = sinf((2.0f * 3.14159265358979323846f * (float)i) / (float)LUT_SIZE);
        int32_t q = (int32_t)(v * 32767.0f);
        if (q > 32767) q = 32767;
        if (q < -32767) q = -32767;
        s_sin_lut[i] = (int16_t)q;
    }
}

void foc_fast_sincos_u16_q15(uint16_t phase, int32_t *s, int32_t *c) {
    uint32_t idx_s = ((uint32_t)phase * LUT_SIZE) >> 16;
    uint32_t idx_c = (idx_s + (LUT_SIZE / 4U)) & (LUT_SIZE - 1U);
    *s = (int32_t)s_sin_lut[idx_s & (LUT_SIZE - 1U)];
    *c = (int32_t)s_sin_lut[idx_c];
}

int32_t foc_q15_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * (int64_t)b) >> 15);
}

int32_t foc_q15_clamp(int32_t x, int32_t lo, int32_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t duty_from_phase(int32_t phase_q15, int32_t inv_vbus_q30) {
    /* ratio_q15 = phase / vbus * 2^15. inv_vbus_q30 = 2^30/vbus_q15. */
    int32_t ratio_q15 = (int32_t)(((int64_t)phase_q15 * inv_vbus_q30) >> 15);
    return foc_q15_clamp(FOC_Q15_HALF + ratio_q15,
                         (int32_t)PWM_MIN_DUTY_Q15, (int32_t)PWM_MAX_DUTY_Q15);
}

void foc_svm_q15(int32_t v_alpha_q15, int32_t v_beta_q15,
                 int32_t inv_vbus_q30,
                 uint16_t *d_u_q15, uint16_t *d_v_q15, uint16_t *d_w_q15) {
    int32_t beta_term = foc_q15_mul(FOC_Q15_SQRT3_BY_2, v_beta_q15);
    int32_t vu = v_alpha_q15;
    int32_t vv = -(v_alpha_q15 >> 1) + beta_term;
    int32_t vw = -(v_alpha_q15 >> 1) - beta_term;

    int32_t vmax = vu;
    if (vv > vmax) vmax = vv;
    if (vw > vmax) vmax = vw;
    int32_t vmin = vu;
    if (vv < vmin) vmin = vv;
    if (vw < vmin) vmin = vw;

    int32_t common = (vmax + vmin) >> 1;
    *d_u_q15 = (uint16_t)duty_from_phase(vu - common, inv_vbus_q30);
    *d_v_q15 = (uint16_t)duty_from_phase(vv - common, inv_vbus_q30);
    *d_w_q15 = (uint16_t)duty_from_phase(vw - common, inv_vbus_q30);
}

float foc_clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

uint16_t foc_deg_to_u16(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return (uint16_t)(deg * (65536.0f / 360.0f));
}

float foc_wrap_deg(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}
