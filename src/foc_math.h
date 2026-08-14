#pragma once
#include <stdint.h>

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

float foc_clampf(float x, float lo, float hi);
uint16_t foc_deg_to_u16(float deg);
float foc_wrap_deg(float deg);
