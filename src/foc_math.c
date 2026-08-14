#include "foc_math.h"
#include "app_config.h"
#include "foc_sin_lut_q15.h"

#define LUT_BITS 10U
#define LUT_SIZE (1U << LUT_BITS)

void foc_math_init(void) {
    /* LUT is compile-time const in flash. Kept as an API hook so the motor
       startup sequence does not need to change. */
}

static inline int32_t sin_lut_interp_q15(uint16_t phase) {
    /* 16-bit electrical phase + 1024-entry LUT leaves 6 fractional bits.
       Linear interpolation costs one small multiply but reduces the phase
       quantization from 0.3516 deg to roughly the 16-bit phase resolution. */
    const uint32_t frac_bits = 16U - LUT_BITS;
    const uint32_t frac_mask = (1U << frac_bits) - 1U;
    uint32_t idx = ((uint32_t)phase >> frac_bits) & (LUT_SIZE - 1U);
    uint32_t next = (idx + 1U) & (LUT_SIZE - 1U);
    int32_t y0 = foc_sin_lut_q15[idx];
    int32_t dy = (int32_t)foc_sin_lut_q15[next] - y0;
    uint32_t frac = (uint32_t)phase & frac_mask;
    return y0 + (int32_t)((dy * (int32_t)frac) >> frac_bits);
}

void foc_fast_sincos_u16_q15(uint16_t phase, int32_t *s, int32_t *c) {
    *s = sin_lut_interp_q15(phase);
    *c = sin_lut_interp_q15((uint16_t)(phase + 16384U));
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
