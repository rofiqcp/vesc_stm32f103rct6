#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int32_t abs_i32(int32_t x) { return x < 0 ? -x : x; }
static int32_t max_abs(int32_t a, int32_t b) { return abs_i32(a) >= abs_i32(b) ? a : b; }
static uint16_t phase_err(uint16_t a, uint16_t b) {
    int16_t d = (int16_t)(a - b);
    return (uint16_t)(d < 0 ? -(int32_t)d : d);
}

int main(void) {
    /* VESC 7 composition: do not sum MTPA and FW Id. */
    if (max_abs(-4000, -6000) != -6000) return 1;
    if (max_abs(-7000, -3000) != -7000) return 2;

    /* Current-circle headroom improves compared with old additive Id. */
    double iq_new = sqrt(20.0 * 20.0 - 6.0 * 6.0);
    double iq_old = sqrt(20.0 * 20.0 - 10.0 * 10.0);
    if (!(iq_new > iq_old && fabs(iq_new - 19.078784) < 0.001)) return 3;

    /* Hardware-window normalization: 90% of the 0.80 physical span maps to
       90% of l_max_duty=0.95, i.e. 0.855 in VESC duty semantics. */
    double physical = 0.80 * 0.90;
    double normalized = physical * (0.95 / 0.80);
    if (fabs(normalized - 0.855) > 1e-9) return 4;

    /* 16-kHz fractional accumulator: 10 A / 1 s reaches target in 16000 frames
       without needing a >=1 Q15-LSB step per frame. */
    const int32_t target_q15 = (int32_t)llround((10.0 / 50.0) * 32768.0);
    const int64_t target_q31 = (int64_t)target_q15 << 16;
    const int64_t step = (target_q31 + 15999) / 16000;
    int64_t acc = 0;
    for (int i = 0; i < 16000; i++) {
        acc += step;
        if (acc > target_q31) acc = target_q31;
    }
    if (llabs(acc - target_q31) > step) return 5;

    /* Wrapped phase difference is safe across 0/65535. 15 degrees is ~2731. */
    if (phase_err(100, 65500) > 200) return 6;
    if (!(phase_err(10000, (uint16_t)(10000 + 2732)) > 2731)) return 7;
    if (!(phase_err(10000, (uint16_t)(10000 + 2730)) < 2731)) return 8;

    puts("ALL BATCH 10 PART-1 NUMERIC TESTS: PASS");
    return 0;
}
