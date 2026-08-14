#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "foc_math.h"
#include "app_config.h"

static int iabs_i(int x) { return x < 0 ? -x : x; }

int main(void) {
    foc_math_init();
    int32_t s,c;
    foc_fast_sincos_u16_q15(0U,&s,&c);
    assert(iabs_i((int)s) < 100);
    assert(c > 32600);
    foc_fast_sincos_u16_q15(16384U,&s,&c);
    assert(s > 32600);
    assert(iabs_i((int)c) < 100);

    uint16_t u,v,w;
    /* 48 V represented in Q15 base. */
    int32_t vbus_q15=(int32_t)((48.0f/FOC_VOLTAGE_Q_BASE_V)*32768.0f);
    int32_t inv=(int32_t)(((int64_t)1<<30)/vbus_q15);
    foc_svm_q15(0,0,inv,&u,&v,&w);
    assert(iabs_i((int)u-(int)FOC_Q15_HALF) <= 1);
    assert(iabs_i((int)v-(int)FOC_Q15_HALF) <= 1);
    assert(iabs_i((int)w-(int)FOC_Q15_HALF) <= 1);

    foc_svm_q15(4096,0,inv,&u,&v,&w);
    assert(u >= PWM_MIN_DUTY_Q15 && u <= PWM_MAX_DUTY_Q15);
    assert(v >= PWM_MIN_DUTY_Q15 && v <= PWM_MAX_DUTY_Q15);
    assert(w >= PWM_MIN_DUTY_Q15 && w <= PWM_MAX_DUTY_Q15);
    puts("test_foc_math: PASS");
    return 0;
}
