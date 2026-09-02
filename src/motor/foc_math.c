#include "motor/foc_math.h"
#include "motor/mcconf_default.h"
#include "foc_sin_lut_q15.h"
#include <math.h>
#include <stddef.h>
#include <limits.h>
#ifndef FOC_MATH_UNIT_TEST
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "encoder/encoder.h"
#include "hwconf/hw.h"
#endif

#define LUT_BITS 10U
#define LUT_SIZE (1U << LUT_BITS)

// Fungsi foc_math_init: menginisialisasi foc math init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void foc_math_init(void) {
    /* LUT is compile-time const in flash. Kept as an API hook so the motor
       startup sequence does not need to change. */
}

// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi sin_lut_interp_q15: menjalankan operasi sin lut interp q15 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static inline int32_t sin_lut_interp_q15(uint16_t phase) {
    /* 16-bit electrical phase + 1024-entry LUT leaves 6 fractional bits.
       Linear interpolation costs one small multiply but reduces the phase
       quantization from 0.3516 deg to roughly the 16-bit phase resolution. */
    // Variabel frac_bits: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t frac_bits = 16U - LUT_BITS;
    // Variabel frac_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t frac_mask = (1U << frac_bits) - 1U;
    // Variabel idx: indeks elemen yang sedang diproses.
    uint32_t idx = ((uint32_t)phase >> frac_bits) & (LUT_SIZE - 1U);
    // Variabel next: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t next = (idx + 1U) & (LUT_SIZE - 1U);
    // Variabel y0: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t y0 = foc_sin_lut_q15[idx];
    // Variabel dy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dy = (int32_t)foc_sin_lut_q15[next] - y0;
    // Variabel frac: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t frac = (uint32_t)phase & frac_mask;
    return y0 + (int32_t)((dy * (int32_t)frac) >> frac_bits);
}

// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter s: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_fast_sincos_u16_q15: menjalankan bagian foc fast sincos u16 q15 pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_fast_sincos_u16_q15(uint16_t phase, int32_t *s, int32_t *c) {
    *s = sin_lut_interp_q15(phase);
    *c = sin_lut_interp_q15((uint16_t)(phase + 16384U));
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_q15_mul: menjalankan bagian foc q15 mul pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
int32_t foc_q15_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * (int64_t)b) >> 15);
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_q15_clamp: menjalankan bagian foc q15 clamp pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
int32_t foc_q15_clamp(int32_t x, int32_t lo, int32_t hi) {
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inv_vbus_q30: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter d_u_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_v_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_w_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_svm_q15: menjalankan bagian foc svm q15 pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
void foc_svm_q15(int32_t v_alpha_q15, int32_t v_beta_q15,
                 int32_t inv_vbus_q30,
                 uint16_t *d_u_q15, uint16_t *d_v_q15, uint16_t *d_w_q15) {
    /* Centered SVPWM with vector-preserving enforcement of the hoverboard
       current-sampling window. U/V/W are never clipped independently: if the
       requested vector is too large, all three phase excursions are scaled by
       the same factor so the alpha/beta angle is preserved. */
    // Variabel beta_term: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t beta_term = foc_q15_mul(FOC_Q15_SQRT3_BY_2, v_beta_q15);
    // Variabel vu: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vu = v_alpha_q15;
    // Variabel vv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vv = -(v_alpha_q15 / 2) + beta_term;
    // Variabel vw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vw = -(v_alpha_q15 / 2) - beta_term;

    // Variabel vmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vmax = vu;
    if (vv > vmax)
        vmax = vv;
    if (vw > vmax)
        vmax = vw;
    // Variabel vmin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vmin = vu;
    if (vv < vmin)
        vmin = vv;
    if (vw < vmin)
        vmin = vw;
    // Variabel common: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t common = (vmax + vmin) / 2;

    // Variabel ru: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ru = (int32_t)(((int64_t)(vu - common) * inv_vbus_q30) >> 15);
    // Variabel rv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t rv = (int32_t)(((int64_t)(vv - common) * inv_vbus_q30) >> 15);
    // Variabel rw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t rw = (int32_t)(((int64_t)(vw - common) * inv_vbus_q30) >> 15);

    // Variabel absmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t absmax = ru < 0 ? -ru : ru;
    // Variabel av: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t av = rv < 0 ? -rv : rv;
    // Variabel aw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t aw = rw < 0 ? -rw : rw;
    if (av > absmax)
        absmax = av;
    if (aw > absmax)
        absmax = aw;

    // Variabel window_half: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t window_half =
        ((int32_t)PWM_MAX_DUTY_Q15 - (int32_t)PWM_MIN_DUTY_Q15) / 2;
    if (absmax > window_half && absmax > 0) {
        // Variabel scale_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t scale_q15 =
            (int32_t)(((int64_t)window_half * 32768LL) / absmax);
        ru = foc_q15_mul(ru, scale_q15);
        rv = foc_q15_mul(rv, scale_q15);
        rw = foc_q15_mul(rw, scale_q15);
    }

    // Variabel du: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t du = FOC_Q15_HALF + ru;
    // Variabel dv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dv = FOC_Q15_HALF + rv;
    // Variabel dw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dw = FOC_Q15_HALF + rw;
    /* Rounding-only safety clamps. Normal limiting happened above as one
       vector operation, so these must not distort the commanded vector. */
    du = foc_q15_clamp(du, (int32_t)PWM_MIN_DUTY_Q15, (int32_t)PWM_MAX_DUTY_Q15);
    dv = foc_q15_clamp(dv, (int32_t)PWM_MIN_DUTY_Q15, (int32_t)PWM_MAX_DUTY_Q15);
    dw = foc_q15_clamp(dw, (int32_t)PWM_MIN_DUTY_Q15, (int32_t)PWM_MAX_DUTY_Q15);
    *d_u_q15 = (uint16_t)du;
    *d_v_q15 = (uint16_t)dv;
    *d_w_q15 = (uint16_t)dw;
}

// Parameter d_u_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_v_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_w_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter vbus_q15: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_pwm_applied_voltage_q15: menjalankan bagian foc pwm applied voltage q15 pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_pwm_applied_voltage_q15(uint16_t d_u_q15, uint16_t d_v_q15,
                                 uint16_t d_w_q15, int32_t vbus_q15,
                                 int32_t *v_alpha_q15, int32_t *v_beta_q15) {
    if (!v_alpha_q15 || !v_beta_q15)
        return;
    /* Duty is Q15 in [0,1]. The common 0.5/common-mode term cancels from
       Clarke, so no explicit midpoint subtraction is required here. */
    // Variabel du: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t du = (int32_t)d_u_q15;
    // Variabel dv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t dv = (int32_t)d_v_q15;
    // Variabel dw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t dw = (int32_t)d_w_q15;
    // Variabel mod_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t mod_alpha = (2 * du - dv - dw) / 3;
    // Variabel mod_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t mod_beta = foc_q15_mul(FOC_Q15_INV_SQRT3, dv - dw);
    *v_alpha_q15 = foc_q15_mul(vbus_q15, mod_alpha);
    *v_beta_q15 = foc_q15_mul(vbus_q15, mod_beta);
}

// Parameter ia_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ib_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ic_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter vbus_q15: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter deadtime_comp_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_deadtime_compensate_voltage_q15: menjalankan bagian foc deadtime compensate voltage q15 pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_deadtime_compensate_voltage_q15(int32_t ia_q15, int32_t ib_q15,
                                         int32_t ic_q15, int32_t vbus_q15,
                                         int32_t deadtime_comp_q15,
                                         int32_t *v_alpha_q15,
                                         int32_t *v_beta_q15) {
    if (!v_alpha_q15 || !v_beta_q15 || deadtime_comp_q15 <= 0)
        return;
    // Variabel sa: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t sa = (ia_q15 > 0) - (ia_q15 < 0);
    // Variabel sb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t sb = (ib_q15 > 0) - (ib_q15 < 0);
    // Variabel sc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t sc = (ic_q15 > 0) - (ic_q15 < 0);
    // Variabel base_v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t base_v = foc_q15_mul(vbus_q15, deadtime_comp_q15);
    // Variabel alpha_comp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t alpha_comp = (base_v * (2 * sa - sb - sc)) / 3;
    // Variabel beta_comp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t beta_comp = foc_q15_mul(base_v, FOC_Q15_INV_SQRT3) * (sb - sc);
    *v_alpha_q15 -= alpha_comp;
    *v_beta_q15 -= beta_comp;
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_clampf: menjalankan bagian foc clampf pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
float foc_clampf(float x, float lo, float hi) {
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_deg_to_u16: menjalankan bagian foc deg to u16 pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
uint16_t foc_deg_to_u16(float deg) {
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return (uint16_t)(deg * (65536.0f / 360.0f));
}

// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_wrap_deg: menjalankan bagian foc wrap deg pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
float foc_wrap_deg(float deg) {
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return deg;
}


#ifndef FOC_MATH_UNIT_TEST
/* ================= VESC observer / ENCODER_AB startup ================= */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Fixed-point CORDIC vectoring. Output is one electrical revolution mapped
 * to uint16_t 0..65535, matching VESC's phase representation in this port. */
// Parameter y: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi atan2_u16_q30: menjalankan operasi atan2 u16 q30 sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
static uint16_t atan2_u16_q30(int32_t y, int32_t x) {
    // Variabel atan_u16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static const int16_t atan_u16[] = {
        8192, 4836, 2555, 1297, 651, 326, 163, 81,
        41, 20, 10, 5, 3, 1, 1
    };
    if (x == 0 && y == 0)
        return 0U;

    // Variabel z: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t z = 0;
    /* CORDIC gain is ~1.647. Scale large vectors first so x +/- (y>>i)
       cannot overflow signed 32-bit during vectoring. */
    // Variabel ax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ax = x < 0 ? -x : x;
    // Variabel ay: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ay = y < 0 ? -y : y;
    while (ax > 0x20000000 || ay > 0x20000000) {
        x >>= 1;
        y >>= 1;
        ax >>= 1;
        ay >>= 1;
    }
    /* Bring the vector into the right half-plane and account for pi. */
    if (x < 0) {
        x = -x;
        y = -y;
        z = 32768;
    }
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0; i < sizeof(atan_u16) / sizeof(atan_u16[0]); i++) {
        // Variabel xo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t xo = x;
        // Variabel yo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t yo = y;
        if (yo > 0) {
            x = xo + (yo >> i);
            y = yo - (xo >> i);
            z += atan_u16[i];
        }
        else {
            x = xo - (yo >> i);
            y = yo + (xo >> i);
            z -= atan_u16[i];
        }
    }
    return (uint16_t)z;
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi sat_i32_from_i64: menjalankan operasi sat i32 from i64 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int32_t sat_i32_from_i64(int64_t v) {
    if (v > 2147483647LL)
        return 2147483647;
    if (v < -2147483647LL - 1LL)
        return (-2147483647 - 1);
    return (int32_t)v;
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi abs_i32_sat: menjalankan operasi abs i32 sat sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int32_t abs_i32_sat(int32_t v) {
    if (v >= 0)
        return v;
    if (v == (-2147483647 - 1))
        return 2147483647;
    return -v;
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mag_approx_q30: menjalankan operasi mag approx q30 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int32_t mag_approx_q30(int32_t a, int32_t b) {
    // Variabel aa: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t aa = abs_i32_sat(a);
    // Variabel bb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t bb = abs_i32_sat(b);
    // Variabel hi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t hi = aa > bb ? aa : bb;
    // Variabel lo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t lo = aa > bb ? bb : aa;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t m = (int64_t)hi + (((int64_t)lo * 3) >> 3);
    return sat_i32_from_i64(m);
}

/* Exact-enough vector magnitude for VESC clamp semantics without libm or a
 * 64-bit divide. Inputs are Q2.30; shifting to Q14 guarantees that a^2+b^2
 * fits a signed 32-bit word even at the defensive INT32 state limit. The
 * restoring integer square-root is 16 cheap shift/subtract iterations and is
 * only used when MXV clipping/adaptive-linear magnitude actually needs it. */
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi isqrt_u32: menjalankan operasi isqrt u32 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static uint32_t isqrt_u32(uint32_t x) {
    // Variabel res: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t res = 0U;
    // Variabel bit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t bit = 1UL << 30;
    while (bit > x)
        bit >>= 2;
    while (bit != 0U) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        }
        else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mag_sqrt_q30_fast: menjalankan operasi mag sqrt q30 fast sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int32_t mag_sqrt_q30_fast(int32_t a, int32_t b) {
    // Variabel aq14: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t aq14 = a >> 16;
    // Variabel bq14: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t bq14 = b >> 16;
    // Variabel sq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t sq = (uint32_t)((int64_t)aq14 * aq14 + (int64_t)bq14 * bq14);
    // Variabel mag_q14: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mag_q14 = isqrt_u32(sq);
    if (mag_q14 > 32767U)
        mag_q14 = 32767U;
    return (int32_t)(mag_q14 << 16);
}

// Parameter type: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi observer_uses_lambda_comp: memperbarui observer uses lambda comp untuk memperkirakan fase atau
// kecepatan rotor pada operasi sensorless.
static bool observer_uses_lambda_comp(mc_foc_observer_type type) {
    return type == FOC_OBSERVER_ORTEGA_LAMBDA_COMP ||
           type == FOC_OBSERVER_MXLEMMING_LAMBDA_COMP ||
           type == FOC_OBSERVER_MXV_LAMBDA_COMP ||
           type == FOC_OBSERVER_MXV_LAMBDA_COMP_LIN;
}

/* Square of a Q2.30 vector, returned as Q2.30. Inputs are shifted once before
 * squaring so even the defensive INT32 flux bounds cannot overflow int64. */
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi flux_mag_sq_q30: menjalankan operasi flux mag sq q30 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int32_t flux_mag_sq_q30(int32_t a, int32_t b) {
    // Variabel ah: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t ah = (int64_t)(a >> 1);
    // Variabel bh: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t bh = (int64_t)(b >> 1);
    return sat_i32_from_i64(((ah * ah) + (bh * bh)) >> 28);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi lambda_bounds_q30: menjalankan operasi lambda bounds q30 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void lambda_bounds_q30(const MotorRuntime *m, int32_t *lo, int32_t *hi) {
    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t target = m->observer_flux_target_q30 > 0 ? m->observer_flux_target_q30 : 1024;
    *lo = (int32_t)(((int64_t)target * 3LL) / 10LL);
    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t h = ((int64_t)target * 5LL) / 2LL;
    if (h > INT32_MAX)
        h = INT32_MAX;
    *hi = (int32_t)h;
    if (*lo < 1024)
        *lo = 1024;
    if (*hi < *lo)
        *hi = *lo;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi lambda_clamp_q30: membatasi lambda clamp q30 ke rentang yang diizinkan agar pengendali dan perangkat
// keras tetap aman.
static int32_t lambda_clamp_q30(const MotorRuntime *m, int32_t v) {
    // Variabel hi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel lo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t lo, hi;
    lambda_bounds_q30(m, &lo, &hi);
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* VESC lambda estimator:
 *   lambda += gain * gamma_half * lambda * (-err) * dt
 * observer_gamma_coeff_q30 already represents gamma_half*dt*flux_base^2 in
 * normalized Q30. gain_q15 is 0.2 for Ortega/MXV and 0.1 for MXLEMMING. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lambda_q30: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter err_q30: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter gain_q15: penguatan regulator/observer yang menentukan respons algoritma.
// Fungsi lambda_adapt_nonlinear_q30: menjalankan operasi lambda adapt nonlinear q30 sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static int32_t lambda_adapt_nonlinear_q30(const MotorRuntime *m, int32_t lambda_q30,
                                           int32_t err_q30, int32_t gain_q15) {
    // Variabel gl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t gl = sat_i32_from_i64(((int64_t)m->observer_gamma_coeff_q30 * lambda_q30) >> 30);
    // Variabel neg_err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t neg_err = -(int64_t)err_q30;
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t d = sat_i32_from_i64(((int64_t)gl * neg_err) >> 30);
    d = sat_i32_from_i64(((int64_t)d * gain_q15) >> 15);
    return lambda_clamp_q30(m, sat_i32_from_i64((int64_t)lambda_q30 + d));
}

/* Return numerator/denominator in Q15 for 0 <= numerator <= denominator.
 * Shift both operands together until the 32-bit numerator shift is safe; this
 * avoids a costly 64-bit divide in the 16-kHz path. */
// Parameter numerator: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter denominator: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi ratio_le_one_q15: menjalankan operasi ratio le one q15 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int32_t ratio_le_one_q15(int32_t numerator, int32_t denominator) {
    if (numerator <= 0 || denominator <= 0)
        return 0;
    if (numerator >= denominator)
        return 32768;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint32_t n = (uint32_t)numerator;
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t d = (uint32_t)denominator;
    while (d > 32767U) {
        n >>= 1;
        d >>= 1;
        if (d == 0U)
            return 32768;
    }
    return (int32_t)((n << 15) / d);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter rotor_a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter rotor_b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter limit_q30: batas aman yang diterapkan pada nilai kendali atau pengukuran.
// Fungsi clamp_observer_state_vector: memperbarui clamp observer state vector untuk memperkirakan fase atau
// kecepatan rotor pada operasi sensorless.
static void clamp_observer_state_vector(MotorRuntime *m, int32_t rotor_a, int32_t rotor_b,
                                         int32_t limit_q30) {
    /* Match VESC's `if (NORM2(rotor_flux) > lambda)` decision using a squared
       Q30 comparison first. The cheap alpha-max magnitude is deliberately NOT
       used for the decision: it can over-estimate by several percent at some
       vector angles and would then clamp a perfectly valid lambda-radius
       trajectory every electrical revolution, creating observer phase lag. */
    // Variabel mag_sq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t mag_sq = flux_mag_sq_q30(rotor_a, rotor_b);
    // Variabel lim_sq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t lim_sq = flux_mag_sq_q30(limit_q30, 0);
    if (mag_sq <= lim_sq || mag_sq <= 0)
        return;

    /* Once clipping is required, use the small restoring integer square-root
       for the scale denominator. This matches VESC NORM2 closely enough to
       avoid the repeated over-clamp/phase-lag that alpha-max approximations
       create on a rotating flux vector. */
    // Variabel mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t mag = mag_sqrt_q30_fast(rotor_a, rotor_b);
    if (mag <= 0)
        return;
    // Variabel scale_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t scale_q15 = ratio_le_one_q15(limit_q30, mag);
    // Variabel sx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
    // Variabel sy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
    m->observer_stator_flux_alpha_q30 = ((int64_t)sx * scale_q15) >> 15;
    m->observer_stator_flux_beta_q30 = ((int64_t)sy * scale_q15) >> 15;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_observer_precalc: menjalankan bagian foc observer precalc pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_observer_precalc(MotorRuntime *m) {
    if (!m)
        return;
    /* VESC-style temperature compensation: use the compensated resistance when
       enabled and valid, otherwise the configured R. This keeps the voltage
       model consistent with the copper drift tracked in the 1-kHz loop. */
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = fmaxf((m->foc_temp_comp && m->board_temp_valid)
                         ? m->res_temp_comp_ohm : m->foc_motor_r, 1.0e-6f);
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l = fmaxf(m->foc_motor_l, 1.0e-8f);
    // Variabel flux: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float flux = fmaxf(m->foc_motor_flux_linkage, 1.0e-6f);

    // Variabel rv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float rv = r * (FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V);
    // Variabel lf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lf = l * (FOC_CURRENT_Q_BASE_A / FOC_FLUX_Q_BASE_WB);
    // Variabel vf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float vf = FOC_VOLTAGE_Q_BASE_V * FOC_DT_S / FOC_FLUX_Q_BASE_WB;
    // Variabel ft: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ft = flux / FOC_FLUX_Q_BASE_WB;
    if (rv > 1.999f)
        rv = 1.999f;
    if (lf > 1.999f)
        lf = 1.999f;
    if (vf > 1.999f)
        vf = 1.999f;
    if (ft > 1.999f)
        ft = 1.999f;
    m->observer_r_i_to_v_q15 = (int32_t)lrintf(rv * 32768.0f);
    m->observer_l_i_to_flux_q15 = (int32_t)lrintf(lf * 32768.0f);
    m->observer_vdt_to_flux_q15 = (int32_t)lrintf(vf * 32768.0f);
    m->observer_flux_target_q30 = (int32_t)lrintf(ft * 1073741824.0f);
    if (m->observer_flux_target_q30 < 1024)
        m->observer_flux_target_q30 = 1024;

    /* Fixed-point PLL coefficients. In u16/revolution phase units the angle
       scale cancels out:
         dphase_counts = Kp * error_counts * dt
         d(ERPM_Q16)    = Ki * error_counts * dt * 60
       Keeping both coefficients Q16.16 avoids float in the 16-kHz path. */
    // Variabel pll_kp_q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float pll_kp_q = m->foc_pll_kp * (65536.0f / (float)FOC_ISR_EVENT_HZ);
    // Variabel pll_ki_q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float pll_ki_q = m->foc_pll_ki * (60.0f * 65536.0f / (float)FOC_ISR_EVENT_HZ);
    pll_kp_q = foc_clampf(pll_kp_q, 0.0f, 2147483000.0f);
    pll_ki_q = foc_clampf(pll_ki_q, 0.0f, 2147483000.0f);
    m->pll_kp_dt_q16 = (int32_t)lrintf(pll_kp_q);
    m->pll_ki_dt60_q16 = (int32_t)lrintf(pll_ki_q);

    /* Until the 1-kHz service has current/duty information, start with VESC's
       slow-gain floor. The normalized Ortega coefficient is
       gamma*dt/2*flux_base^2 and is consumed as Q2.30 in the ISR. */
    // Variabel gamma: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float gamma = fmaxf(0.0f, m->foc_observer_gain) *
                  foc_clampf(m->foc_observer_gain_slow, 0.0f, 1.0f) * 4.0f;
    // Variabel gamma_coeff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float gamma_coeff = gamma * (0.5f / (float)FOC_ISR_EVENT_HZ) *
                        FOC_FLUX_Q_BASE_WB * FOC_FLUX_Q_BASE_WB;
    gamma_coeff = foc_clampf(gamma_coeff, 0.0f, 0.75f);
    m->observer_gamma_coeff_q30 = (int32_t)lrintf(gamma_coeff * 1073741824.0f);
    /* Q16.16 signed ERPM is intentionally bounded to this board's 30 kERPM
       envelope so no configuration can wrap the fixed-point speed state. */
    // Variabel erpm_q16_max: kecepatan listrik motor dalam electrical RPM.
    const float erpm_q16_max = fminf(MOTOR_DEFAULT_MAX_ERPM, 32760.0f);
    m->foc_sl_erpm_start_q16 = (int32_t)lrintf(
        foc_clampf(m->foc_sl_erpm_start, 0.0f, erpm_q16_max) * 65536.0f);
    m->foc_sl_erpm_q16 = (int32_t)lrintf(
        foc_clampf(m->foc_sl_erpm, 0.0f, erpm_q16_max) * 65536.0f);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter phase_u16: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi foc_observer_reset: menjalankan bagian foc observer reset pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_observer_reset(MotorRuntime *m, uint16_t phase_u16) {
    if (!m)
        return;
    foc_observer_precalc(m);
    // Variabel cs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel sn: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sn, cs;
    foc_fast_sincos_u16_q15(phase_u16, &sn, &cs);
    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t target = m->observer_flux_target_q30;
    // Variabel fa: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fa = (int32_t)(((int64_t)cs * target) >> 15);
    // Variabel fb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fb = (int32_t)(((int64_t)sn * target) >> 15);

    /* stator flux = rotor flux + L*i; reset assumes zero current. */
    m->observer_stator_flux_alpha_q30 = fa;
    m->observer_stator_flux_beta_q30 = fb;
    m->observer_rotor_flux_alpha_q30 = fa;
    m->observer_rotor_flux_beta_q30 = fb;
    m->observer_lambda_est_q30 = target;
    m->observer_i_alpha_last_q15 = 0;
    m->observer_i_beta_last_q15 = 0;
    m->observer_v_alpha_q15_prev = 0;
    m->observer_v_beta_q15_prev = 0;
    m->observer_phase_u16 = phase_u16;
    m->observer_phase_last_u16 = phase_u16;
    m->observer_erpm_q16 = 0;
    m->pll_phase_u16 = phase_u16;
    m->pll_erpm_q16 = 0;
    m->speed_est_fast_erpm_q16 = 0;
    m->speed_est_faster_erpm_q16 = 0;
    m->speed_est_phase_valid = false;
    m->phase_before_speed_est_u16 = phase_u16;
    m->observer_update_cycle = DWT->CYCCNT;
    m->observer_erpm = 0.0f;
    m->observer_quality = 0.0f;
    m->observer_valid = false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter i_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_observer_update_fixed: menjalankan bagian foc observer update fixed pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_observer_update_fixed(MotorRuntime *m, int32_t v_alpha_q15, int32_t v_beta_q15,
                         int32_t i_alpha_q15, int32_t i_beta_q15) {
    if (!m)
        return;

    /* All observer variants below follow the current VESC observer equations,
       but the STM32F103 hard path remains integer-only. VESC/MESC attribution
       for MXLEMMING is retained below as required by the upstream source. */
    // Variabel ri_a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t ri_a = foc_q15_mul(i_alpha_q15, m->observer_r_i_to_v_q15);
    // Variabel ri_b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t ri_b = foc_q15_mul(i_beta_q15, m->observer_r_i_to_v_q15);
    // Variabel emf_a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t emf_a = v_alpha_q15 - ri_a;
    // Variabel emf_b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t emf_b = v_beta_q15 - ri_b;
    // Variabel li_a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t li_a = (int32_t)((int64_t)i_alpha_q15 * m->observer_l_i_to_flux_q15);
    // Variabel li_b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t li_b = (int32_t)((int64_t)i_beta_q15 * m->observer_l_i_to_flux_q15);
    // Variabel fa: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel fb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fa = 0, fb = 0;

    switch (m->foc_observer_type) {
    case FOC_OBSERVER_MXLEMMING:
    case FOC_OBSERVER_MXLEMMING_LAMBDA_COMP:
        {
        /* LICENCE/ATTRIBUTION NOTE from upstream VESC/MESC implementation:
         * This observer work is original to the MESC FOC project. Preserve
         * credit to David Molony as the original author when modifying it. */
        // Variabel dia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t dia = i_alpha_q15 - m->observer_i_alpha_last_q15;
        // Variabel dib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t dib = i_beta_q15 - m->observer_i_beta_last_q15;
        // Variabel dli_a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t dli_a = (int32_t)((int64_t)dia * m->observer_l_i_to_flux_q15);
        // Variabel dli_b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t dli_b = (int32_t)((int64_t)dib * m->observer_l_i_to_flux_q15);
        m->observer_stator_flux_alpha_q30 += (int64_t)emf_a * m->observer_vdt_to_flux_q15 - dli_a;
        m->observer_stator_flux_beta_q30 += (int64_t)emf_b * m->observer_vdt_to_flux_q15 - dli_b;
        // Variabel sx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
        // Variabel sy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
        // Variabel limit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t limit = m->observer_flux_target_q30;
        if (m->foc_observer_type == FOC_OBSERVER_MXLEMMING_LAMBDA_COMP) {
            // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t err = sat_i32_from_i64((int64_t)flux_mag_sq_q30(m->observer_lambda_est_q30, 0) -
                                                  flux_mag_sq_q30(sx, sy));
            m->observer_lambda_est_q30 = lambda_adapt_nonlinear_q30(m, m->observer_lambda_est_q30,
                                                                     err, 3277); /* 0.1 */
            limit = m->observer_lambda_est_q30;
        }
        if (sx > limit)
            sx = limit;
        else if (sx < -limit) {
            sx = -limit;
        }
        if (sy > limit)
            sy = limit;
        else if (sy < -limit) {
            sy = -limit;
        }
        m->observer_stator_flux_alpha_q30 = sx;
        m->observer_stator_flux_beta_q30 = sy;
        /* MXLEMMING state is already rotor flux, so L*i must not be subtracted
           again before atan2. */
        fa = sx;
        fb = sy;
    } break;

    case FOC_OBSERVER_MXV:
    case FOC_OBSERVER_MXV_LAMBDA_COMP:
    case FOC_OBSERVER_MXV_LAMBDA_COMP_LIN:
        {
        m->observer_stator_flux_alpha_q30 += (int64_t)emf_a * m->observer_vdt_to_flux_q15;
        m->observer_stator_flux_beta_q30 += (int64_t)emf_b * m->observer_vdt_to_flux_q15;
        // Variabel sx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
        // Variabel sy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
        fa = sat_i32_from_i64((int64_t)sx - li_a);
        fb = sat_i32_from_i64((int64_t)sy - li_b);
        // Variabel limit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t limit = m->observer_flux_target_q30;

        if (m->foc_observer_type == FOC_OBSERVER_MXV_LAMBDA_COMP) {
            // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t err = sat_i32_from_i64((int64_t)flux_mag_sq_q30(m->observer_lambda_est_q30, 0) -
                                                  flux_mag_sq_q30(fa, fb));
            m->observer_lambda_est_q30 = lambda_adapt_nonlinear_q30(m, m->observer_lambda_est_q30,
                                                                     err, 6554); /* 0.2 */
            limit = m->observer_lambda_est_q30;
        }
        else if (m->foc_observer_type == FOC_OBSERVER_MXV_LAMBDA_COMP_LIN) {
            // Variabel mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t mag = mag_sqrt_q30_fast(fa, fb);
            // Variabel lam: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t lam = m->observer_lambda_est_q30;
            // Variabel lam_sq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t lam_sq = flux_mag_sq_q30(lam, 0);
            // Variabel alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int32_t alpha = sat_i32_from_i64(((int64_t)m->observer_gamma_coeff_q30 * lam_sq) >> 30);
            alpha = sat_i32_from_i64(((int64_t)alpha * 3277) >> 15); /* 0.1 */
            if (alpha > 1073741824)
                alpha = 1073741824; /* alpha <= 1.0 in Q30 */
            // Variabel dl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t dl = sat_i32_from_i64(((int64_t)(mag - lam) * alpha) >> 30);
            m->observer_lambda_est_q30 = lambda_clamp_q30(m, sat_i32_from_i64((int64_t)lam + dl));
            limit = m->observer_lambda_est_q30;
        }

        clamp_observer_state_vector(m, fa, fb, limit);
        sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
        sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
        fa = sat_i32_from_i64((int64_t)sx - li_a);
        fb = sat_i32_from_i64((int64_t)sy - li_b);
    } break;

    case FOC_OBSERVER_ORTEGA_LAMBDA_COMP:
    case FOC_OBSERVER_ORTEGA_ORIGINAL:
    default:
        {
        m->observer_stator_flux_alpha_q30 += (int64_t)emf_a * m->observer_vdt_to_flux_q15;
        m->observer_stator_flux_beta_q30 += (int64_t)emf_b * m->observer_vdt_to_flux_q15;
        // Variabel sx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
        // Variabel sy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
        fa = sat_i32_from_i64((int64_t)sx - li_a);
        fb = sat_i32_from_i64((int64_t)sy - li_b);
        // Variabel lambda: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t lambda = m->observer_flux_target_q30;
        if (m->foc_observer_type == FOC_OBSERVER_ORTEGA_LAMBDA_COMP)
            lambda = m->observer_lambda_est_q30;
        // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t err = sat_i32_from_i64((int64_t)flux_mag_sq_q30(lambda, 0) - flux_mag_sq_q30(fa, fb));
        if (m->foc_observer_type == FOC_OBSERVER_ORTEGA_LAMBDA_COMP) {
            m->observer_lambda_est_q30 = lambda_adapt_nonlinear_q30(m, m->observer_lambda_est_q30,
                                                                     err, 6554); /* 0.2 */
        }
        /* VESC/Ortega convergence is improved by keeping the correction error
           non-positive. */
        if (err > 0)
            err = 0;
        // Variabel gamma: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t gamma = m->observer_gamma_coeff_q30;
        if (gamma > 0 && err < 0) {
            // Variabel ca: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t ca = sat_i32_from_i64(((int64_t)err * fa) >> 30);
            // Variabel cb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t cb = sat_i32_from_i64(((int64_t)err * fb) >> 30);
            m->observer_stator_flux_alpha_q30 += ((int64_t)ca * gamma) >> 30;
            m->observer_stator_flux_beta_q30 += ((int64_t)cb * gamma) >> 30;
            sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
            sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
            fa = sat_i32_from_i64((int64_t)sx - li_a);
            fb = sat_i32_from_i64((int64_t)sy - li_b);
        }
    } break;
    }

    /* Defensive state bounds and the same anti-collapse principle as VESC.
       The bound protects a corrupt configuration from numeric runaway. */
    // Variabel sx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sx = sat_i32_from_i64(m->observer_stator_flux_alpha_q30);
    // Variabel sy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sy = sat_i32_from_i64(m->observer_stator_flux_beta_q30);
    m->observer_stator_flux_alpha_q30 = sx;
    m->observer_stator_flux_beta_q30 = sy;
    if (mag_approx_q30(sx, sy) < (m->observer_flux_target_q30 / 2)) {
        // Variabel grow_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t grow_q15 = 36045; /* round(1.1 * 32768) */
        sx = sat_i32_from_i64(((int64_t)sx * grow_q15) >> 15);
        sy = sat_i32_from_i64(((int64_t)sy * grow_q15) >> 15);
        m->observer_stator_flux_alpha_q30 = sx;
        m->observer_stator_flux_beta_q30 = sy;
        if (m->foc_observer_type == FOC_OBSERVER_MXLEMMING ||
            m->foc_observer_type == FOC_OBSERVER_MXLEMMING_LAMBDA_COMP) {
            fa = sx;
            fb = sy;
        }
        else {
            fa = sat_i32_from_i64((int64_t)sx - li_a);
            fb = sat_i32_from_i64((int64_t)sy - li_b);
        }
    }

    m->observer_i_alpha_last_q15 = i_alpha_q15;
    m->observer_i_beta_last_q15 = i_beta_q15;
    m->observer_rotor_flux_alpha_q30 = fa;
    m->observer_rotor_flux_beta_q30 = fb;

    // Variabel phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    const uint16_t phase = atan2_u16_q30(fb, fa);
    // Variabel dphase: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int16_t dphase = (int16_t)(phase - m->observer_phase_last_u16);
    m->observer_phase_last_u16 = phase;
    m->observer_phase_u16 = phase;

    // Variabel erpm_inst64: kecepatan listrik motor dalam electrical RPM.
    int64_t erpm_inst64 = (int64_t)dphase * (int64_t)(FOC_ISR_EVENT_HZ * 60UL);
    if (erpm_inst64 > INT32_MAX)
        erpm_inst64 = INT32_MAX;
    if (erpm_inst64 < INT32_MIN)
        erpm_inst64 = INT32_MIN;
    // Variabel erpm_inst_q16: kecepatan listrik motor dalam electrical RPM.
    const int32_t erpm_inst_q16 = (int32_t)erpm_inst64;
    // Variabel de64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t de64 = (int64_t)erpm_inst_q16 - (int64_t)m->observer_erpm_q16;
    if (de64 > INT32_MAX)
        de64 = INT32_MAX;
    if (de64 < INT32_MIN)
        de64 = INT32_MIN;
    m->observer_erpm_q16 += (int32_t)(((int64_t)(int32_t)de64 * FOC_OBSERVER_SPEED_ALPHA_Q15) >> 15);
    m->observer_update_cycle = DWT->CYCCNT;

    // Variabel mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t mag = mag_approx_q30(fa, fb);
    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t target = observer_uses_lambda_comp(m->foc_observer_type) ?
                           m->observer_lambda_est_q30 : m->observer_flux_target_q30;
    // Variabel lo: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t lo = (int32_t)((int64_t)target * 45 / 100);
    // Variabel hi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t hi = sat_i32_from_i64((int64_t)target * 180 / 100);
    // Variabel erpm_abs_q16: kecepatan listrik motor dalam electrical RPM.
    const int32_t erpm_abs_q16 = abs_i32_sat(m->observer_erpm_q16);
    /* foc_sl_erpm_start adalah ambang perpindahan Hall/encoder ke observer,
       bukan syarat kesehatan observer pada mode SENSORLESS murni. Dengan
       default open-loop rendah, memakai foc_sl_erpm_start di sini membuat
       observer tidak mungkin dinyatakan valid saat forced-openloop masih
       bekerja. Gunakan lantai kecil hanya untuk menolak estimasi diam/noise;
       perpindahan sensorless tetap memiliki gate kecepatan dan koherensi fase
       tersendiri di foc_sensorless_startup_1khz(). */
    const int32_t observer_min_erpm_q16 = 50 * 65536;
    m->observer_valid = (mag >= lo && mag <= hi &&
                         erpm_abs_q16 >= observer_min_erpm_q16);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter phase_u16: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi foc_pll_run_fixed: menjalankan bagian foc pll run fixed pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_pll_run_fixed(MotorRuntime *m, uint16_t phase_u16) {
    if (!m)
        return;

    // Variabel err_counts: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t err_counts = (int16_t)(phase_u16 - m->pll_phase_u16);
    // Variabel speed_advance: nilai kecepatan untuk target atau pengukuran.
    int64_t speed_advance = (int64_t)m->pll_erpm_q16 /
                            (60LL * (int64_t)FOC_ISR_EVENT_HZ);
    // Variabel prop_advance: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t prop_advance = ((int64_t)err_counts * m->pll_kp_dt_q16) >> 16;
    // Variabel phase_advance: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    int64_t phase_advance = speed_advance + prop_advance;
    if (phase_advance > INT32_MAX)
        phase_advance = INT32_MAX;
    if (phase_advance < INT32_MIN)
        phase_advance = INT32_MIN;
    m->pll_phase_u16 = (uint16_t)((uint32_t)m->pll_phase_u16 +
                                  (uint32_t)(int32_t)phase_advance);

    // Variabel dspeed: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t dspeed = ((int64_t)err_counts * m->pll_ki_dt60_q16) >> 16;
    // Variabel next_speed: nilai kecepatan untuk target atau pengukuran.
    int64_t next_speed = (int64_t)m->pll_erpm_q16 + dspeed;
    // Variabel max_speed_q16: batas atau nilai maksimum untuk validasi dan proteksi.
    const int64_t max_speed_q16 = (int64_t)((int32_t)MOTOR_DEFAULT_MAX_ERPM) * 65536LL;
    if (next_speed > max_speed_q16)
        next_speed = max_speed_q16;
    if (next_speed < -max_speed_q16)
        next_speed = -max_speed_q16;
    m->pll_erpm_q16 = (int32_t)next_speed;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_observer_update_1khz: menjalankan bagian foc observer update 1khz pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_observer_update_1khz(MotorRuntime *m) {
    if (!m)
        return;

    /* VESC-style task-side observer parameter adaptation. The hard observer
       stays fixed-point; only these slowly changing coefficients are computed
       with floats and published atomically as 32-bit values. */
    // Variabel l_eff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l_eff = fmaxf(m->foc_motor_l, 1.0e-8f);
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    const float id = m->id_filter;
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq = m->iq_filter;
    // Variabel i2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i2 = id * id + iq * iq;
    if (fabsf(m->foc_motor_ld_lq_diff) > 1.0e-12f && i2 > 0.01f) {
        l_eff = l_eff - 0.5f * m->foc_motor_ld_lq_diff +
                m->foc_motor_ld_lq_diff * (iq * iq / i2);
        l_eff = fmaxf(l_eff, 1.0e-8f);
    }

    // Variabel flux_eff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float flux_eff = fmaxf(m->foc_motor_flux_linkage, 1.0e-6f);
    // Variabel lambda_observer: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    const bool lambda_observer = observer_uses_lambda_comp(m->foc_observer_type);
    // Variabel lambda_est: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float lambda_est = fmaxf((float)m->observer_lambda_est_q30 *
                                   (FOC_FLUX_Q_BASE_WB / 1073741824.0f), 1.0e-6f);
    // Variabel i_abs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_abs = sqrtf(i2);
    // Variabel i_lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_lim = fmaxf(m->current_max_a, 0.1f);
    // Variabel comp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float comp = foc_clampf(m->foc_sat_comp * i_abs / i_lim, 0.0f, 0.95f);
    switch (m->foc_sat_comp_mode) {
    case SAT_COMP_LAMBDA:
        if (lambda_observer)
            l_eff *= lambda_est / fmaxf(m->foc_motor_flux_linkage, 1.0e-6f);
        break;
    case SAT_COMP_FACTOR:
        l_eff *= (1.0f - comp);
        flux_eff *= (1.0f - comp);
        break;
    case SAT_COMP_LAMBDA_AND_FACTOR:
        if (lambda_observer)
            l_eff *= lambda_est / fmaxf(m->foc_motor_flux_linkage, 1.0e-6f);
        l_eff *= (1.0f - comp);
        break;
    case SAT_COMP_DISABLED:
    default:
        break;
    }
    l_eff = fmaxf(l_eff, 1.0e-8f);

    // Variabel lf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lf = l_eff * (FOC_CURRENT_Q_BASE_A / FOC_FLUX_Q_BASE_WB);
    // Variabel ft: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ft = flux_eff / FOC_FLUX_Q_BASE_WB;
    lf = foc_clampf(lf, 0.0f, 1.999f);
    ft = foc_clampf(ft, 1.0e-6f, 1.999f);
    // Variabel l_coeff_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t l_coeff_q15 = (int32_t)lrintf(lf * 32768.0f);
    // Variabel flux_target_q30: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t flux_target_q30 = (int32_t)lrintf(ft * 1073741824.0f);
    if (flux_target_q30 < 1024)
        flux_target_q30 = 1024;

    // Variabel vbus: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    const float vbus = fmaxf(m->vbus_filter, 1.0f);
    // Variabel gamma: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float gamma = fmaxf(m->foc_observer_gain, 0.0f) *
                  (fabsf(m->duty_now) * vbus / 40.0f);
    // Variabel gamma_floor: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float gamma_floor = fmaxf(m->foc_observer_gain, 0.0f) *
                              foc_clampf(m->foc_observer_gain_slow, 0.0f, 1.0f);
    if (gamma < gamma_floor)
        gamma = gamma_floor;
    gamma *= 4.0f;
    // Variabel gamma_coeff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float gamma_coeff = gamma * (0.5f / (float)FOC_ISR_EVENT_HZ) *
                        FOC_FLUX_Q_BASE_WB * FOC_FLUX_Q_BASE_WB;
    gamma_coeff = foc_clampf(gamma_coeff, 0.0f, 0.75f);
    // Variabel gamma_coeff_q30: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t gamma_coeff_q30 = (int32_t)lrintf(gamma_coeff * 1073741824.0f);

    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // Variabel fa: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fa = m->observer_rotor_flux_alpha_q30;
    // Variabel fb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fb = m->observer_rotor_flux_beta_q30;
    m->observer_l_i_to_flux_q15 = l_coeff_q15;
    m->observer_flux_target_q30 = flux_target_q30;
    m->observer_gamma_coeff_q30 = gamma_coeff_q30;
    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t target = observer_uses_lambda_comp(m->foc_observer_type) ?
                     m->observer_lambda_est_q30 : flux_target_q30;
    if (!primask)
        __enable_irq();
    // Variabel mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t mag = mag_approx_q30(fa, fb);

    m->observer_flux_alpha = (float)fa * (FOC_FLUX_Q_BASE_WB / 1073741824.0f);
    m->observer_flux_beta = (float)fb * (FOC_FLUX_Q_BASE_WB / 1073741824.0f);
    m->observer_phase_deg = (float)m->observer_phase_u16 * (360.0f / 65536.0f);
    m->observer_phase_rad = (float)m->observer_phase_u16 * (2.0f * (float)M_PI / 65536.0f);
    m->observer_erpm = (float)m->observer_erpm_q16 / 65536.0f;
    m->observer_speed_rad_s = m->observer_erpm * (2.0f * (float)M_PI / 60.0f);
    m->pll_phase_rad = (float)m->pll_phase_u16 * (2.0f * (float)M_PI / 65536.0f);
    // Variabel pll_erpm: kecepatan listrik motor dalam electrical RPM.
    const float pll_erpm = (float)m->pll_erpm_q16 / 65536.0f;
    m->pll_speed_rad_s = pll_erpm * (2.0f * (float)M_PI / 60.0f);
    m->observer_quality = target > 0 ? foc_clampf((float)mag / (float)target, 0.0f, 2.0f) : 0.0f;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_encoder_ab_sync_from_observer: menjalankan bagian foc encoder ab sync from observer pada algoritma
// FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_encoder_ab_sync_from_observer(MotorRuntime *m) {
    if (!m || m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER ||
        m->encoder.phase_per_count_q16 == 0U)
        return;
    /* Invert the normal VESC mapping:
       electrical = signed(mechanical * ratio) - offset. */
    // Variabel obs_deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float obs_deg = (float)m->observer_phase_u16 * (360.0f / 65536.0f);
    // Variabel off_deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float off_deg = (float)m->encoder.elec_offset_u16 * (360.0f / 65536.0f);
    // Variabel mech_deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float mech_deg = (obs_deg + off_deg) / fmaxf(m->encoder.electrical_ratio, 1.0e-6f);
    if (m->encoder.inverted)
        mech_deg = 360.0f - mech_deg;
    // Variabel first: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool first = !encoder_index_found(m);
    encoder_set_deg(m, mech_deg);
    if (first)
        m->encoder.mechanical_zero_count = motor_encoder_extended_count(m);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_encoder_ab_startup_1khz: menjalankan bagian foc encoder ab startup 1khz pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
bool foc_encoder_ab_startup_1khz(MotorRuntime *m, uint32_t now_ms) {
    if (!m || m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER || m->encoder.synced)
        return true;
    if (!m->openloop_started) {
        m->openloop_started = true;
        m->openloop_start_tick = now_ms;
        m->phase_observer_override = true;
        m->phase_observer_override_u16 = m->observer_phase_u16;
        m->openloop_erpm_now = 0.0f;
        foc_observer_reset(m, m->phase_observer_override_u16);
        return false;
    }

    /* timer_thread requests/preloads the bridge from openloop_started. Do not
       consume lock/ramp time until MOE is really active and the current-sense
       blanking window has completed. This mirrors the VESC principle that the
       forced phase sequence is a real energized startup, not a software timer. */
    if (!m->pwm_enabled || m->pwm_enable_pending_events != 0U ||
        m->pwm_enable_blank_cycles != 0U) {
        m->openloop_start_tick = now_ms;
        m->openloop_erpm_now = 0.0f;
        /* Startup encoder memakai jalur sinkronisasi observer sendiri. Jangan
           menyisipkan hint sensorless di sini karena fungsi ini tidak menerima parameter arah/torsi sensorless. Tahan target
           nol sampai MOE siap. */
        motor_set_foc_targets(m, 0.0f, 0.0f);
        return false;
    }

    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float t = (float)(now_ms - m->openloop_start_tick) * 0.001f;
    // Variabel lock: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lock = fmaxf(m->foc_sl_openloop_time_lock, 0.0f);
    // Variabel ramp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ramp = fmaxf(m->foc_sl_openloop_time_ramp, 0.05f);
    // Variabel constant: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float constant = fmaxf(m->foc_sl_openloop_time, 0.05f);
    // Variabel dir: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dir = 1.0f;
    if (m->iq_target < 0.0f || m->current_command_a < 0.0f || m->speed_target_erpm < 0.0f)
        dir = -1.0f;
    // Variabel rpm: kecepatan putar yang digunakan oleh logika kendali.
    float rpm = 0.0f;
    /* ENCODER_AB cannot establish its software absolute reference unless the
       forced startup actually crosses the observer handover threshold. Some
       older F103 defaults had openloop_rpm < foc_sl_erpm, an impossible sync.
       Treat configured openloop RPM as a minimum and guarantee 10% headroom. */
    // Variabel rpm_hi: kecepatan putar yang digunakan oleh logika kendali.
    float rpm_hi = fmaxf(m->foc_openloop_rpm, m->foc_sl_erpm * 1.10f);
    // Variabel rpm_low_fraction: kecepatan putar yang digunakan oleh logika kendali.
    const float rpm_low_fraction = foc_clampf(m->foc_openloop_rpm_low, 0.0f, 1.0f);
    // Variabel rpm_lo: kecepatan putar yang digunakan oleh logika kendali.
    float rpm_lo = rpm_hi * rpm_low_fraction;
    if (t > lock) {
        // Variabel tr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float tr = t - lock;
        if (tr < ramp)
            rpm = rpm_lo + (rpm_hi - rpm_lo) * (tr / ramp);
        else rpm = rpm_hi;
    }
    rpm *= dir;
    m->openloop_erpm_now = rpm;
    // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t step = (int32_t)lrintf(rpm * (65536.0f / 60.0f) * 0.001f);
    m->phase_observer_override_u16 = (uint16_t)(m->phase_observer_override_u16 + step);

    // Variabel boost: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float boost = fminf(fabsf(m->foc_sl_openloop_boost_q), fabsf(m->foc_sl_openloop_max_q));
    // Variabel iq_cmd: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq_cmd = m->iq_target;
    if (fabsf(iq_cmd) < boost)
        iq_cmd = copysignf(boost, dir);
    motor_set_foc_targets(m, 0.0f, iq_cmd);

    if (m->observer_valid &&
        abs_i32_sat(m->speed_est_fast_erpm_q16) >= m->foc_sl_erpm_q16) {
        foc_encoder_ab_sync_from_observer(m);
        m->phase_observer_override = false;
        m->openloop_started = false;
        return true;
    }
    if (t > lock + ramp + constant + 2.0f) {
        /* Do not claim a fake sync. Leave the command alive but restart the
           VESC-style open-loop sequence from current phase. */
        m->openloop_started = false;
    }
    return false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_sensorless_startup_abort: menjalankan bagian foc sensorless startup abort pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_sensorless_startup_abort(MotorRuntime *m) {
    if (!m)
        return;
    /* Remove torque before releasing a forced phase. Otherwise an unsuccessful
       lock could leave the previous Iq target active for up to one 1-kHz task
       period while the hard loop has already returned to an invalid observer
       phase. Callers that are handing over to a valid observer publish their
       normal target later in the same service tick. */
    motor_set_foc_targets(m, 0.0f, 0.0f);
    m->phase_observer_override = false;
    m->openloop_started = false;
    m->openloop_erpm_now = 0.0f;
}

/* VESC-style low-speed startup for pure sensorless FOC, adapted to this
 * phase-voltage-sensorless hoverboard target. The 1-kHz task owns the float
 * timing/ramp math; the 16-kHz FOC ISR still consumes only fixed-point phase
 * and current targets. HFI is intentionally not used.
 *
 * Safety rules:
 * - no forced startup when direction is unknowable (brake/position/hold),
 * - bridge ramp time only advances after MOE + current-sense blanking are live,
 * - observer handover requires both valid BEMF speed and phase coherence,
 * - failed lock restarts instead of claiming a false observer lock. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter direction_hint: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter iq_hint_a: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi foc_sensorless_startup_1khz: menjalankan bagian foc sensorless startup 1khz pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
bool foc_sensorless_startup_1khz(MotorRuntime *m, uint32_t now_ms,
                                  float direction_hint, float iq_hint_a) {
    if (!m || m->foc_sensor_mode != FOC_SENSOR_MODE_SENSORLESS)
        return true;

    // Variabel speed_abs_q16: nilai kecepatan untuk target atau pengukuran.
    const int32_t speed_abs_q16 = abs_i32_sat(m->speed_est_fast_erpm_q16);

    /* Upstream VESC semantics: foc_openloop_rpm_low is a 0..1 fraction of
       foc_openloop_rpm at minimum motor current. Scale toward full openloop
       ERPM with measured |Iq| instead of treating the field as an ERPM value. */
    // Variabel openloop_base_erpm: kecepatan listrik motor dalam electrical RPM.
    const float openloop_base_erpm = fmaxf(fabsf(m->foc_openloop_rpm), 1.0f);
    // Variabel low_fraction: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float low_fraction = foc_clampf(m->foc_openloop_rpm_low, 0.0f, 1.0f);
    // Variabel current_scale_a: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_scale_a = fmaxf(fabsf(m->current_max_a), fabsf(m->current_min_a));
    if (!isfinite(current_scale_a) || current_scale_a < 0.1f)
        current_scale_a = FOC_MAX_CURRENT_A;

    /* Ikuti mcpwm_foc VESC: kecepatan open-loop dipetakan dari arus aktual
       ditambah foc_sl_openloop_boost_q. Boost harus ikut ke pemetaan RPM;
       jika hanya Iq_filter yang dipakai, start dari diam dengan
       foc_openloop_rpm_low=0 menghasilkan 0 ERPM dan fase forced-openloop
       tidak pernah berputar. */
    // Variabel openloop_current_a: arus efektif untuk menentukan kecepatan forced-openloop.
    float openloop_current_a = fabsf(m->iq_filter) + fabsf(m->foc_sl_openloop_boost_q);
    if (m->foc_sl_openloop_max_q > 0.0f)
        openloop_current_a = fminf(openloop_current_a, fabsf(m->foc_sl_openloop_max_q));
    // Variabel openloop_current_ratio: rasio arus open-loop terhadap envelope arus motor.
    const float openloop_current_ratio = foc_clampf(openloop_current_a / current_scale_a, 0.0f, 1.0f);
    // Variabel openloop_target_erpm: kecepatan listrik motor dalam electrical RPM.
    const float openloop_target_erpm = openloop_base_erpm *
                                       (low_fraction +
                                        (1.0f - low_fraction) * openloop_current_ratio);
    // Variabel handover_erpm: ambang minimum kecepatan untuk melepas forced-openloop ke observer.
    const float handover_erpm = fmaxf(openloop_target_erpm, 50.0f);
    // Variabel handover_q16: ambang handover dalam format ERPM Q16.16.
    const int32_t handover_q16 = (int32_t)lrintf(
        fminf(handover_erpm, 32760.0f) * 65536.0f);

    /* Once a previous startup has completed, a trustworthy observer can be
       used directly. During active forced startup keep the override enabled
       and blend it toward the observer below instead of performing a hard
       electrical-angle switch. */
    if (!m->openloop_started && m->observer_valid && speed_abs_q16 >= handover_q16) {
        m->phase_observer_override = false;
        m->openloop_erpm_now = 0.0f;
        m->sensorless_start_failures = 0U;
        return true;
    }

    // Variabel dir: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int dir = (direction_hint > 1.0e-6f) ? 1 :
                    (direction_hint < -1.0e-6f ? -1 : 0);
    if (dir == 0) {
        foc_sensorless_startup_abort(m);
        return false;
    }

    /* A command reversal during forced startup must establish a new alignment
       instead of continuing a phase trajectory seeded for the opposite torque
       direction. Zero-speed lock has openloop_erpm_now == 0, so it is left
       undisturbed until the ramp has actually acquired a direction. */
    if (m->openloop_started && m->openloop_erpm_now != 0.0f &&
        ((m->openloop_erpm_now > 0.0f) != (dir > 0))) {
        foc_sensorless_startup_abort(m);
    }

    if (!m->openloop_started) {
        m->openloop_started = true;
        m->openloop_start_tick = now_ms;
        m->openloop_erpm_now = 0.0f;
        m->phase_observer_override = true;

        /* Current VESC adds roughly 60 electrical degrees when a motor is
           stuck before beginning the forced sequence. Seed this reduced port
           the same way, and initialize the observer another 45 degrees in the
           torque direction so it starts from a physically useful flux state. */
        // Variabel phase_60: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
        const int16_t phase_60 = (int16_t)10923; /* 65536 * 60 / 360 */
        // Variabel phase_45: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
        const int16_t phase_45 = (int16_t)8192; /* 65536 * 45 / 360 */
        m->phase_observer_override_u16 =
            (uint16_t)(m->observer_phase_u16 + (dir > 0 ? phase_60 : -phase_60));
        foc_observer_reset(m, (uint16_t)(m->phase_observer_override_u16 +
                           (dir > 0 ? phase_45 : -phase_45)));

        /* Startup sensorless harus langsung menerbitkan torsi alignment. Pada
           Run26 target 0 A membuat bridge tidak pernah memperoleh permintaan
           arus yang berguna dan startup dapat berhenti pada 0 ERPM. Gunakan
           boost-Q VESC sebagai minimum alignment dan tetap hormati limit arus. */
        float start_limit = dir > 0 ? m->lo_current_max_a : -m->lo_current_min_a;
        if (!isfinite(start_limit) || start_limit <= 0.0f)
            start_limit = dir > 0 ? m->current_max_a : -m->current_min_a;
        if (!isfinite(start_limit) || start_limit <= 0.0f)
            start_limit = FOC_MAX_CURRENT_A;
        start_limit = fminf(start_limit, FOC_MAX_CURRENT_A);
        float start_iq = fabsf(iq_hint_a) + fabsf(m->foc_sl_openloop_boost_q);
        if (m->foc_sl_openloop_max_q > 0.0f)
            start_iq = fminf(start_iq, fabsf(m->foc_sl_openloop_max_q));
        start_iq = foc_clampf(start_iq, fminf(fabsf(m->foc_sl_openloop_boost_q), start_limit), start_limit);
        motor_set_foc_targets(m, 0.0f, copysignf(start_iq, (float)dir));
        return false;
    }

    /* Do not consume lock/ramp time until hardware PWM is really active. */
    if (!m->pwm_enabled || m->pwm_enable_pending_events != 0U ||
        m->pwm_enable_blank_cycles != 0U) {
        m->openloop_start_tick = now_ms;
        m->openloop_erpm_now = 0.0f;
        /* Pertahankan alignment torque selama MOE/blanking handshake. Jangan
           kembali ke target nol karena itu membuat startup terlihat idle. */
        float wait_limit = dir > 0 ? m->lo_current_max_a : -m->lo_current_min_a;
        if (!isfinite(wait_limit) || wait_limit <= 0.0f)
            wait_limit = dir > 0 ? m->current_max_a : -m->current_min_a;
        if (!isfinite(wait_limit) || wait_limit <= 0.0f)
            wait_limit = FOC_MAX_CURRENT_A;
        wait_limit = fminf(wait_limit, FOC_MAX_CURRENT_A);
        float wait_iq = fabsf(iq_hint_a) + fabsf(m->foc_sl_openloop_boost_q);
        if (m->foc_sl_openloop_max_q > 0.0f)
            wait_iq = fminf(wait_iq, fabsf(m->foc_sl_openloop_max_q));
        wait_iq = foc_clampf(wait_iq, fminf(fabsf(m->foc_sl_openloop_boost_q), wait_limit), wait_limit);
        motor_set_foc_targets(m, 0.0f, copysignf(wait_iq, (float)dir));
        return false;
    }

    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float t = (float)(now_ms - m->openloop_start_tick) * 0.001f;
    // Variabel lock: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float lock = fmaxf(m->foc_sl_openloop_time_lock, 0.0f);
    // Variabel ramp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ramp = fmaxf(m->foc_sl_openloop_time_ramp, 0.05f);
    // Variabel constant: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float constant = fmaxf(m->foc_sl_openloop_time, 0.05f);

    /* Pure sensorless follows the configured VESC open-loop target. The
       current-dependent target above already implements foc_openloop_rpm_low.
       Do not force foc_sl_erpm here: that field belongs to hybrid sensored
       handover and previously made a 900-ERPM setting accelerate to 2750 ERPM. */
    // Variabel rpm_hi: kecepatan putar yang digunakan oleh logika kendali.
    const float rpm_hi = openloop_target_erpm;
    // Variabel rpm_lo: kecepatan putar yang digunakan oleh logika kendali.
    const float rpm_lo = 0.0f;
    // Variabel rpm: kecepatan putar yang digunakan oleh logika kendali.
    float rpm = 0.0f;
    if (t > lock) {
        // Variabel tr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float tr = t - lock;
        rpm = tr < ramp ? rpm_lo + (rpm_hi - rpm_lo) * (tr / ramp) : rpm_hi;
    }
    rpm *= (float)dir;
    m->openloop_erpm_now = rpm;
    // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t step = (int32_t)lrintf(rpm * (65536.0f / 60.0f) * 0.001f);
    m->phase_observer_override_u16 =
        (uint16_t)(m->phase_observer_override_u16 + step);

    /* Upstream adds open-loop Q boost and caps it. On this hoverboard port the
       user MCCONF/open-loop value must never bypass the currently valid motor
       current limit. Use the direction-specific live limit (including runtime
       derating) and the configured 15 A board envelope as an upper bound. */
    // Variabel iq_abs: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq_abs = fabsf(iq_hint_a) + fabsf(m->foc_sl_openloop_boost_q);
    // Variabel cfg_max_q: batas atau nilai maksimum untuk validasi dan proteksi.
    const float cfg_max_q = fmaxf(fabsf(m->foc_sl_openloop_max_q),
                                  fmaxf(m->cc_min_current, 0.1f));
    // Variabel dir_limit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dir_limit = dir > 0 ? m->lo_current_max_a : -m->lo_current_min_a;
    if (!isfinite(dir_limit) || dir_limit <= 0.0f) {
        dir_limit = dir > 0 ? m->current_max_a : -m->current_min_a;
    }
    if (!isfinite(dir_limit) || dir_limit <= 0.0f)
        dir_limit = FOC_MAX_CURRENT_A;
    dir_limit = fminf(dir_limit, FOC_MAX_CURRENT_A);
    // Variabel max_q: batas atau nilai maksimum untuk validasi dan proteksi.
    const float max_q = fmaxf(0.1f, fminf(cfg_max_q, dir_limit));
    // Variabel min_q: batas atau nilai minimum untuk validasi dan proteksi.
    const float min_q = fminf(fabsf(m->foc_sl_openloop_boost_q), max_q);
    iq_abs = foc_clampf(iq_abs, min_q, max_q);
    motor_set_foc_targets(m, 0.0f, copysignf(iq_abs, (float)dir));

    if (m->observer_valid && speed_abs_q16 >= handover_q16) {
        /* True handover blend. The open-loop angle already advances at the
           commanded electrical speed. Add only a bounded correction toward
           the observer each 1-kHz tick, keeping the 16-kHz ISR fixed-point and
           preventing the previous <=60-degree instantaneous phase jump. */
        // Variabel signed_err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t signed_err = (int16_t)(m->observer_phase_u16 -
                                       m->phase_observer_override_u16);
        // Variabel abs_err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t abs_err = signed_err < 0 ? -signed_err : signed_err;
        if (abs_err <= 10923) { /* enter blend inside 60 electrical degrees */
            // Variabel corr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int32_t corr = signed_err / 8;
            // Variabel max_corr: batas atau nilai maksimum untuk validasi dan proteksi.
            const int32_t max_corr = 546; /* about 3 electrical degrees/tick */
            if (corr > max_corr)
                corr = max_corr;
            if (corr < -max_corr)
                corr = -max_corr;
            if (corr == 0 && signed_err != 0)
                corr = signed_err > 0 ? 1 : -1;
            m->phase_observer_override_u16 =
                (uint16_t)(m->phase_observer_override_u16 + corr);

            signed_err = (int16_t)(m->observer_phase_u16 -
                                    m->phase_observer_override_u16);
            abs_err = signed_err < 0 ? -signed_err : signed_err;
            if (abs_err <= 182) { /* <= 1 electrical degree */
                m->phase_observer_override = false;
                m->openloop_started = false;
                m->openloop_erpm_now = 0.0f;
                m->sensorless_start_failures = 0U;
                return true;
            }
        }
    }

    if (t > lock + ramp + constant + 2.0f) {
        /* Do not restart forever without visibility. Repeated real lock
           timeouts are promoted to MOTOR_FAULT_SENSORLESS_OBSERVER by the
           1-kHz motor service. */
        if (m->sensorless_start_failures < 255U)
            m->sensorless_start_failures++;
        foc_sensorless_startup_abort(m);
    }
    return false;
}

#endif /* FOC_MATH_UNIT_TEST */


#ifndef FOC_MATH_UNIT_TEST
/* -------------------------------------------------------------------------
 * VESC foc_math API compatibility layer (non-HFI)
 * ------------------------------------------------------------------------- */
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi wrap_rad_local: menjalankan operasi wrap rad local sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float wrap_rad_local(float a) {
    while (a > (float)M_PI)
        a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI)
        a += 2.0f * (float)M_PI;
    return a;
}

// Parameter v_alpha: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter v_beta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i_alpha: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i_beta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_observer_update: menjalankan bagian foc observer update pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
                         float dt, observer_state *state, float *phase,
                         motor_all_state_t *motor) {
    (void)dt;
    if (!motor)
        return;
    // Variabel va: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t va = (int32_t)lrintf(foc_clampf(v_alpha/FOC_VOLTAGE_Q_BASE_V, -0.99997f, 0.99997f)*32768.0f);
    // Variabel vb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vb = (int32_t)lrintf(foc_clampf(v_beta /FOC_VOLTAGE_Q_BASE_V, -0.99997f, 0.99997f)*32768.0f);
    // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ia = (int32_t)lrintf(foc_clampf(i_alpha/FOC_CURRENT_Q_BASE_A, -0.99997f, 0.99997f)*32768.0f);
    // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ib = (int32_t)lrintf(foc_clampf(i_beta /FOC_CURRENT_Q_BASE_A, -0.99997f, 0.99997f)*32768.0f);
    foc_observer_update_fixed(motor, va, vb, ia, ib);
    if (phase)
        *phase = (float)motor->observer_phase_u16*(2.0f*(float)M_PI/65536.0f);
    if (state) {
        state->x1 = (float)motor->observer_rotor_flux_alpha_q30*(FOC_FLUX_Q_BASE_WB/1073741824.0f);
        state->x2 = (float)motor->observer_rotor_flux_beta_q30 *(FOC_FLUX_Q_BASE_WB/1073741824.0f);
        state->lambda_est = sqrtf(state->x1*state->x1+state->x2*state->x2);
        state->i_alpha_last = i_alpha;
        state->i_beta_last = i_beta;
    }
}

// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter phase_var: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter speed_var: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi foc_pll_run: menjalankan bagian foc pll run pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
void foc_pll_run(float phase, float dt, float *phase_var, float *speed_var,
                 mc_configuration *conf) {
    if (!phase_var || !speed_var || !conf || dt <= 0.0f)
        return;
    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float err = wrap_rad_local(phase-*phase_var);
    *phase_var = wrap_rad_local(*phase_var + (*speed_var + conf->foc_pll_kp*err)*dt);
    *speed_var += conf->foc_pll_ki*err*dt;
}

// Parameter alpha: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter beta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter max_mod: batas atau nilai maksimum untuk validasi dan proteksi.
// Parameter top: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter tb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter tc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sector: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_svm: menjalankan bagian foc svm pada algoritma FOC dengan skala, konvensi tanda, dan batas numerik
// yang konsisten.
void foc_svm(float alpha, float beta, float max_mod, uint32_t top,
             uint32_t *ta, uint32_t *tb, uint32_t *tc, uint32_t *sector) {
    if (!ta || !tb || !tc || top == 0U)
        return;
    max_mod = foc_clampf(fabsf(max_mod), 0.0f, 0.999f);
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float a = alpha;
    // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float b = -0.5f*alpha + 0.866025403784f*beta;
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float c = -0.5f*alpha - 0.866025403784f*beta;
    // Variabel mn: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel mx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float mx = fmaxf(a, fmaxf(b, c)), mn = fminf(a, fminf(b, c));
    // Variabel span: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float span = mx-mn;
    if (span > 2.0f*max_mod && span > 1.0e-9f) {
        // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        float k = (2.0f*max_mod)/span;
        a *= k;
        b *= k;
        c *= k;
        mx = fmaxf(a, fmaxf(b, c));
        mn = fminf(a, fminf(b, c));
    }
    // Variabel z: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float z = 0.5f*(mx+mn);
    a -= z;
    b -= z;
    c -= z;
    // Variabel da: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float da = foc_clampf(0.5f+0.5f*a, 0.0f, 1.0f);
    // Variabel db: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float db = foc_clampf(0.5f+0.5f*b, 0.0f, 1.0f);
    // Variabel dc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dc = foc_clampf(0.5f+0.5f*c, 0.0f, 1.0f);
    *ta = (uint32_t)lrintf(da*(float)top);
    *tb = (uint32_t)lrintf(db*(float)top);
    *tc = (uint32_t)lrintf(dc*(float)top);
    if (sector) {
        // Variabel ang: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float ang = atan2f(beta, alpha);
        if (ang < 0.0f)
            ang += 2.0f*(float)M_PI;
        *sector = (uint32_t)(ang/((float)M_PI/3.0f))+1U;
        if (*sector > 6U)
            *sector = 6U;
    }
}

// Parameter index_found: indeks elemen yang sedang diproses.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_run_pid_control_speed: menjalankan bagian foc run pid control speed pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_run_pid_control_speed(bool index_found, float dt, motor_all_state_t *m) {
    (void)index_found;
    if (m == NULL || dt <= 0.0f)
        return;

    if (m->speed_pid_ramp_erpms_s > 0.0f) {
        // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float step = m->speed_pid_ramp_erpms_s * dt;
        if (m->speed_pid_set_erpm < m->speed_target_erpm)
            m->speed_pid_set_erpm = fminf(m->speed_target_erpm, m->speed_pid_set_erpm + step);
        else if (m->speed_pid_set_erpm > m->speed_target_erpm)
            m->speed_pid_set_erpm = fmaxf(m->speed_target_erpm, m->speed_pid_set_erpm - step);
    }
    else {
        m->speed_pid_set_erpm = m->speed_target_erpm;
    }
    m->speed_pid_set_erpm = foc_clampf(m->speed_pid_set_erpm, m->min_erpm, m->max_erpm);

    // Variabel rpm: kecepatan putar yang digunakan oleh logika kendali.
    float rpm;
    switch (m->speed_pid_source) {
    case S_PID_SPEED_SRC_FAST:
        rpm = (float)m->speed_est_fast_erpm_q16/65536.0f;
    break;
    case S_PID_SPEED_SRC_FASTER:
        rpm = (float)m->speed_est_faster_erpm_q16/65536.0f;
    break;
    case S_PID_SPEED_SRC_PLL:
    default:
        rpm = (float)m->pll_erpm_q16/65536.0f;
    break;
    }
    if (m->invert_direction)
        rpm = -rpm;
    // Variabel error: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float error = m->speed_pid_set_erpm - rpm;
    if (fabsf(m->speed_pid_set_erpm) < m->speed_pid_min_erpm) {
        m->speed_pid.integrator = 0.0f;
        m->speed_pid.prev_error = error;
        m->speed_derivative_filtered = 0.0f;
        motor_set_foc_targets(m, -fabsf(m->fw_override_current_a), 0.0f);
        return;
    }

    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float p = error*m->speed_pid.kp*(1.0f/20.0f);
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float d = (error-m->speed_pid.prev_error)*(m->speed_pid.kd/dt)*(1.0f/20.0f);
    m->speed_pid.prev_error = error;
    m->speed_derivative_filtered += foc_clampf(m->speed_kd_filter, 0.0f, 1.0f) *
                                    (d-m->speed_derivative_filtered);
    // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float out = foc_clampf(p+m->speed_pid.integrator+m->speed_derivative_filtered, -1.0f, 1.0f);
    m->speed_pid.integrator += error*m->speed_pid.ki*dt*(1.0f/20.0f);
    m->speed_pid.integrator = foc_clampf(m->speed_pid.integrator, -1.0f, 1.0f);
    if (m->speed_pid.ki < 1.0e-9f)
        m->speed_pid.integrator = 0.0f;
    if (!m->speed_pid_allow_braking) {
        if (rpm > 20.0f && out < 0.0f)
            out = 0.0f;
        if (rpm < -20.0f && out > 0.0f)
            out = 0.0f;
    }
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq = out >= 0.0f ? out*fmaxf(m->lo_current_max_a, 0.0f)
                          : (-out)*fminf(m->lo_current_min_a, 0.0f);
    if (m->invert_direction)
        iq = -iq;
    motor_set_foc_targets(m, -fabsf(m->fw_override_current_a), iq);
}

// Parameter index_found: indeks elemen yang sedang diproses.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_run_pid_control_pos: menjalankan bagian foc run pid control pos pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_run_pid_control_pos(bool index_found, float dt, motor_all_state_t *m) {
    if (m == NULL || dt <= 0.0f || !index_found)
        return;

    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float now = foc_wrap_deg(m->position_deg+m->position_offset_deg);
    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float err = m->position_target_deg-now;
    while (err > 180.0f)
        err -= 360.0f;
    while (err < -180.0f)
        err += 360.0f;
    // Variabel err_sign: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float err_sign = (m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.inverted) ? -1.0f : 1.0f;
    err *= err_sign;




    // Variabel kd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel kdp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ki: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel kp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float kp = m->position_pid.kp, ki = m->position_pid.ki, kd = m->position_pid.kd, kdp = m->position_kd_proc;
    if (m->position_gain_dec_angle > 0.1f && m->position_ang_div > 0.001f) {
        // Variabel min_err: batas atau nilai minimum untuk validasi dan proteksi.
        float min_err = m->position_gain_dec_angle/m->position_ang_div;
        if (min_err > 1e-6f && fabsf(err) < min_err) {
            // Variabel sc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float sc = fabsf(err)/min_err;
            kp *= sc;
            ki *= sc;
            kd *= sc;
            kdp *= sc;
        }
    }
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float p = err*kp;
    m->position_pid.integrator += err*ki*dt;
    m->position_dt_integrator += dt;
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float d = 0.0f;
    if (err != m->position_pid.prev_error && m->position_dt_integrator > 0.0f) {
        d = (err-m->position_pid.prev_error)*(kd/m->position_dt_integrator);
        m->position_dt_integrator = 0.0f;
    }
    // Variabel f: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float f = foc_clampf(m->position_kd_filter, 0.0f, 1.0f);
    m->position_derivative_filtered += f*(d-m->position_derivative_filtered);

    m->position_dt_process_integrator += dt;
    // Variabel dp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dp = 0.0f;
    if (now != m->position_prev_process_deg && m->position_dt_process_integrator > 0.0f) {
        // Variabel pd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float pd = now-m->position_prev_process_deg;
        while (pd > 180.0f)
            pd -= 360.0f;
        while (pd < -180.0f)
            pd += 360.0f;
        dp = -pd*err_sign*(kdp/m->position_dt_process_integrator);
        m->position_dt_process_integrator = 0.0f;
    }
    m->position_derivative_proc_filtered += f*(dp-m->position_derivative_proc_filtered);
    // Variabel pclip: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float pclip = foc_clampf(p, -1.0f, 1.0f);
    // Variabel ilim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ilim = fmaxf(1.0f-fabsf(pclip), 0.0f);
    m->position_pid.integrator = foc_clampf(m->position_pid.integrator, -ilim, ilim);
    m->position_pid.prev_error = err;
    m->position_prev_process_deg = now;
    // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float out = foc_clampf(p+m->position_pid.integrator+
                         m->position_derivative_filtered+
                         m->position_derivative_proc_filtered, -1.0f, 1.0f);
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq = out >= 0.0f ? out*fmaxf(m->lo_current_max_a, 0.0f)
                       : (-out)*fminf(m->lo_current_min_a, 0.0f);
    if (m->invert_direction)
        iq = -iq;
    motor_set_foc_targets(m, -fabsf(m->fw_override_current_a), iq);
}

// Parameter obs_angle: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter enc_angle: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter speed: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Parameter sl_erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_correct_encoder: menjalankan bagian foc correct encoder pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
float foc_correct_encoder(float obs_angle, float enc_angle, float speed, float sl_erpm,
                          motor_all_state_t *m) {
    if (m == NULL)
        return enc_angle;
    // Variabel rpm_abs: kecepatan putar yang digunakan oleh logika kendali.
    const float rpm_abs = fabsf(speed*(60.0f/(2.0f*(float)M_PI)));
    // Variabel hyst: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float hyst = fabsf(sl_erpm)*0.05f;
    if (m->using_encoder) {
        if (rpm_abs > (fabsf(sl_erpm)+hyst))
            m->using_encoder = false;
    }
    else{
        if (rpm_abs < fmaxf(fabsf(sl_erpm)-hyst, 0.0f))
            m->using_encoder = true;
    }
    return m->using_encoder ? enc_angle : obs_angle;
}

// Parameter angle: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hall_val: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Fungsi foc_correct_hall: menjalankan bagian foc correct hall pada algoritma FOC dengan skala, konvensi tanda,
// dan batas numerik yang konsisten.
float foc_correct_hall(float angle, float dt, motor_all_state_t *m, int hall_val) {
    (void)dt;
    if (m == NULL || hall_val < 0 || hall_val > 7 || !m->hall.valid ||
       m->foc_hall_table[hall_val] >= 201U)
       return angle;

    // Variabel hp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t hp = (uint16_t)((int32_t)m->hall.base_phase_u16+(int32_t)m->hall_offset_u16);
    if (m->foc_hall_interp_erpm_u32 > 0U && m->hall.period_cycles > 0U) {
        // Variabel erpm_num: kecepatan listrik motor dalam electrical RPM.
        const uint64_t erpm_num = (uint64_t)CPU_CLOCK_HZ*10ULL;
        // Variabel threshold_num: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint64_t threshold_num = (uint64_t)m->foc_hall_interp_erpm_u32*
                                     (uint64_t)m->hall.period_cycles;
        if (erpm_num >= threshold_num) {
            // Variabel elapsed: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t elapsed = DWT->CYCCNT-m->hall.edge_cycle;
            // Variabel adv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int64_t adv = ((int64_t)(int32_t)elapsed*(int64_t)m->hall.phase_per_cycle_q16)>>16;
            // Variabel sector: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const int32_t sector = 65536/6;
            if (adv > sector)
                adv = sector;
            if (adv < -sector)
                adv = -sector;
            hp = (uint16_t)((int32_t)m->hall.base_phase_u16+(int32_t)adv+
                          (int32_t)m->hall_offset_u16);
        }
    }
    // Variabel hall_angle: nilai sudut untuk posisi atau transformasi koordinat.
    float hall_angle = (float)hp*(2.0f*(float)M_PI/65536.0f);
    // Variabel rpm_abs: kecepatan putar yang digunakan oleh logika kendali.
    float rpm_abs = fabsf((float)m->speed_est_fast_erpm_q16/65536.0f);
    if (!m->observer_valid || m->foc_sl_erpm <= m->foc_sl_erpm_start)
        return hall_angle;
    if (rpm_abs <= m->foc_sl_erpm_start)
        return hall_angle;
    if (rpm_abs >= m->foc_sl_erpm)
        return angle;
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float t = (rpm_abs-m->foc_sl_erpm_start)/(m->foc_sl_erpm-m->foc_sl_erpm_start);
    // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float diff = wrap_rad_local(angle-hall_angle);
    return wrap_rad_local(hall_angle+diff*foc_clampf(t, 0.0f, 1.0f));
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Fungsi foc_run_fw: menjalankan bagian foc run fw pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
void foc_run_fw(motor_all_state_t *m, float dt) {
    if (!m || dt <= 0.0f)
        return;

    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float target = 0.0f;
    // Variabel fw_max_cfg: batas atau nilai maksimum untuk validasi dan proteksi.
    const float fw_max_cfg = fmaxf(m->foc_fw_current_max, 0.0f);
    if (fw_max_cfg < fmaxf(m->cc_min_current, 0.001f)) {
        m->foc_fw_current_now = 0.0f;
        return;
    }
    // Variabel duty_abs: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_abs = fabsf(m->duty_now);
    // Variabel duty_end: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_end = fmaxf(m->max_duty, 0.001f);
    // Variabel duty_start: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_start = foc_clampf(m->foc_fw_duty_start, 0.0f, 1.0f) * duty_end;

    if (fw_max_cfg > 0.001f && duty_abs > duty_start) {
        // Variabel i_fw_max: batas atau nilai maksimum untuk validasi dan proteksi.
        float i_fw_max = fw_max_cfg;
        if (m->foc_fw_backoff > 0.001f && i_fw_max > 0.001f) {
            // Variabel speed_sign: nilai kecepatan untuk target atau pengukuran.
            const float speed_sign = (m->erpm > 0.0f) ? 1.0f : ((m->erpm < 0.0f) ? -1.0f : 0.0f);
            // Variabel backoff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float backoff = speed_sign * (m->iq_filter - m->iq_target) / i_fw_max;
            backoff *= m->foc_fw_backoff;
            backoff = foc_clampf(backoff, 0.0f, 1.0f);
            i_fw_max *= (1.0f - backoff);
        }
        // Variabel span: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float span = fmaxf(duty_end - duty_start, 1.0e-4f);
        target = foc_clampf((duty_abs - duty_start) / span, 0.0f, 1.0f) * i_fw_max;
        /* Keep modulation alive after leaving FW, matching the VESC safeguard
           against abrupt body-diode braking while current-loop oscillations
           decay around the FW threshold. */
        if (target > m->cc_min_current) {
            mcpwm_foc_set_current_off_delay_motor(m, 1.0f);
        }
    }

    /* Manual FW override is additive in intent but bounded by the same physical
       current envelope. Use the larger request so diagnostics can force FW
       without fighting the automatic duty-based controller. */
    target = fmaxf(target, fabsf(m->fw_override_current_a));
    target = fminf(target, fmaxf(fabsf(m->current_min_a), m->current_max_a));

    // Variabel ramp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ramp = fmaxf(m->foc_fw_ramp_time, dt);
    // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float step = (dt / ramp) * fmaxf(fw_max_cfg, target);
    if (m->foc_fw_current_now < target)
        m->foc_fw_current_now = fminf(target, m->foc_fw_current_now + step);
    else if (m->foc_fw_current_now > target) {
        m->foc_fw_current_now = fmaxf(target, m->foc_fw_current_now - step);
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_update_modulation_limit: menjalankan bagian foc update modulation limit pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_update_modulation_limit(motor_all_state_t *m) {
    if (!m)
        return;
    /* VESC semantics: current-controller voltage circle follows l_max_duty and
       foc_overmod_factor. This hoverboard also requires a 10..90% sampling
       window, so never allow a larger modulation than the hardware can apply
       without losing the two-shunt sampling window. */
    // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float duty = foc_clampf(m->duty_limit_now, 0.0f, m->max_duty);
    // Variabel over: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float over = foc_clampf(m->foc_overmod_factor, 0.0f, 1.5f);
    // Variabel effective: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float effective = duty * over;
    // Variabel hw_window: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float hw_window = PWM_MAX_DUTY - PWM_MIN_DUTY;
    if (effective > hw_window)
        effective = hw_window;
    if (effective < 0.01f)
        effective = 0.01f;
    // Variabel coeff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float coeff = effective * 0.5773502691896258f;
    m->vmax_coeff_q15 = (int32_t)lrintf(coeff * 32768.0f);
    if (m->vmax_coeff_q15 < 1)
        m->vmax_coeff_q15 = 1;
    if (m->vmax_coeff_q15 > 32767)
        m->vmax_coeff_q15 = 32767;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_precalc_values: menjalankan bagian foc precalc values pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_precalc_values(motor_all_state_t *m) {
    if (!m)
        return;
    foc_observer_precalc(m);

    // Variabel lq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float lq = fmaxf(m->foc_motor_l + 0.5f * m->foc_motor_ld_lq_diff, 1.0e-8f);
    // Variabel ld: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ld = fmaxf(m->foc_motor_l - 0.5f * m->foc_motor_ld_lq_diff, 1.0e-8f);
    // Variabel omega_per_erpm: kecepatan listrik motor dalam electrical RPM.
    const float omega_per_erpm = (2.0f * (float)M_PI) / 60.0f;
    // Variabel cross_scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float cross_scale = FOC_CURRENT_Q_BASE_A * omega_per_erpm / FOC_VOLTAGE_Q_BASE_V;
    // Variabel q30: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float q30 = 1073741824.0f;
    m->decouple_lq_coeff_q30 = (int32_t)lrintf(foc_clampf(cross_scale * lq, -1.9f, 1.9f) * q30);
    m->decouple_ld_coeff_q30 = (int32_t)lrintf(foc_clampf(cross_scale * ld, -1.9f, 1.9f) * q30);
    /* For BEMF the 32768 voltage-Q15 scaling is applied in the ISR by a
       >>15 shift, so this coefficient stays small and cannot overflow int32. */
    // Variabel bemf_norm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float bemf_norm = omega_per_erpm * m->foc_motor_flux_linkage / FOC_VOLTAGE_Q_BASE_V;
    m->bemf_flux_coeff_q30 = (int32_t)lrintf(foc_clampf(bemf_norm, -1.9f, 1.9f) * q30);
    /* VESC compensates the observer angle for the PWM/ADC sample delay by
       advancing it by speed * dt * (0.5 + foc_observer_offset). Keep the
       configurable factor task-side; the ISR applies it with integer math. */
    m->observer_offset_factor_q15 = (int32_t)lrintf(
        foc_clampf(0.5f + m->foc_observer_offset, -16.0f, 16.0f) * 32768.0f);
    m->foc_current_filter_q15 = (int32_t)lrintf(foc_clampf(m->foc_current_filter_const, 0.0f, 1.0f) * 32768.0f);
    if (m->foc_current_filter_q15 > 32767)
        m->foc_current_filter_q15 = 32767;
    m->foc_mag_vd_max_q15 = (int32_t)lrintf(foc_clampf(m->foc_mag_vd_max, 0.0f, 1.0f) * 32768.0f);
    if (m->foc_mag_vd_max_q15 > 32767)
        m->foc_mag_vd_max_q15 = 32767;

    /* Batch-10 Part-1: precompute every field needed by VESC 7.x-style fast
       field weakening. The ISR never performs floating-point configuration
       math or a division. This board's physical 10..90% sampling window is
       normalized back to l_max_duty for FW threshold semantics, so a 0.90 FW
       start still means 90% of the usable modulation range. */
    {
        // Variabel fw_max_a: batas atau nilai maksimum untuk validasi dan proteksi.
        float fw_max_a = foc_clampf(m->foc_fw_current_max, 0.0f, FOC_CURRENT_Q_BASE_A);
        /* Match upstream: disable FW when its configured ceiling is below the
           minimum useful current. */
        if (fw_max_a < fmaxf(m->cc_min_current, 0.001f))
            fw_max_a = 0.0f;
        // Variabel max_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
        const float max_duty = foc_clampf(m->max_duty, 0.001f, 0.999f);
        // Variabel fw_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float fw_start = foc_clampf(m->foc_fw_duty_start, 0.0f, 1.0f) * max_duty;
        // Variabel hw_window: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float hw_window = fmaxf(PWM_MAX_DUTY - PWM_MIN_DUTY, 0.01f);
        m->foc_fw_max_q15 = (int32_t)lrintf((fw_max_a / FOC_CURRENT_Q_BASE_A) * 32768.0f);
        if (m->foc_fw_max_q15 < 0)
            m->foc_fw_max_q15 = 0;
        if (m->foc_fw_max_q15 > 32767)
            m->foc_fw_max_q15 = 32767;
        m->foc_fw_duty_start_q15 = (int32_t)lrintf(fw_start * 32768.0f);
        m->foc_fw_duty_end_q15 = (int32_t)lrintf(max_duty * 32768.0f);
        if (m->foc_fw_duty_end_q15 > 32767)
            m->foc_fw_duty_end_q15 = 32767;
        // Variabel span: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t span = m->foc_fw_duty_end_q15 - m->foc_fw_duty_start_q15;
        if (span < 1)
            span = 1;
        m->foc_fw_duty_span_inv_q30 = (int32_t)((1LL << 30) / span);
        // Variabel hw_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t hw_q15 = (int32_t)lrintf(hw_window * 32768.0f);
        m->foc_fw_duty_norm_scale_q16 = hw_q15 > 0 ?
            (int32_t)(((int64_t)m->foc_fw_duty_end_q15 << 16) / hw_q15) : 65536;
        m->foc_fw_q_factor_q15 = (int32_t)lrintf(foc_clampf(m->foc_fw_q_current_factor, 0.0f, 1.0f) * 32768.0f);
        if (m->foc_fw_q_factor_q15 > 32767)
            m->foc_fw_q_factor_q15 = 32767;
        // Variabel backoff_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t backoff_q15 = (int32_t)lrintf(foc_clampf(m->foc_fw_backoff, 0.0f, 32.0f) * 32768.0f);
        if (m->foc_fw_max_q15 > 0) {
            // Variabel bc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int64_t bc = ((int64_t)backoff_q15 << 16) / m->foc_fw_max_q15;
            if (bc > INT32_MAX)
                bc = INT32_MAX;
            m->foc_fw_backoff_per_current_q16 = (int32_t)bc;
        }
        else {
            m->foc_fw_backoff_per_current_q16 = 0;
        }
        // Variabel fast_dt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float fast_dt = 1.0f / (float)FOC_ISR_EVENT_HZ;
        m->foc_fw_ramp_direct = m->foc_fw_ramp_time <= fast_dt;
        // Variabel ramp_s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float ramp_s = fmaxf(m->foc_fw_ramp_time, fast_dt);
        {
            // Variabel ramp_step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int64_t ramp_step = (int64_t)llrint(
                ((double)m->foc_fw_max_q15 * 65536.0) /
                ((double)ramp_s * (double)FOC_ISR_EVENT_HZ));
            if (ramp_step > INT32_MAX)
                ramp_step = INT32_MAX;
            if (m->foc_fw_max_q15 > 0 && ramp_step < 1)
                ramp_step = 1;
            m->foc_fw_ramp_step_q31 = (int32_t)ramp_step;
        }
    }

    /* Upstream VESC dead-time model uses foc_dt_us * foc_f_zv as a
       normalized modulation error. Precompute it here so the 16-kHz ISR only
       performs fixed-point sign transforms and multiplications. */
    // Variabel dt_frac: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dt_frac = foc_clampf(m->foc_dt_us * 1.0e-6f * (float)VESC_FOC_F_ZV_HZ,
                               0.0f, 0.25f);
    m->deadtime_comp_q15 = (int32_t)lrintf(dt_frac * 32768.0f);
    if (m->deadtime_comp_q15 < 0)
        m->deadtime_comp_q15 = 0;
    if (m->deadtime_comp_q15 > 8192)
        m->deadtime_comp_q15 = 8192;
    foc_update_modulation_limit(m);
}

#endif /* FOC_MATH_UNIT_TEST */
