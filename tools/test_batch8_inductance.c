#include "motor/mc_math.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int32_t q15_i(float a) { return (int32_t)lrintf(a / 64.0f * 32768.0f); }
static int32_t q15_v(float v) { return (int32_t)lrintf(v / 64.0f * 32768.0f); }
static int run_axis(float L) {
    enum { N=24 };
    int32_t iq[N], vq[N];
    const float R=0.18f, V=3.0f, fs=16000.0f, dt=1.0f/fs;
    const float a=R*dt/(2.0f*L), b=V*dt/L;
    float i=0.0f;
    iq[0]=q15_i(i); vq[0]=q15_v(V);
    for(int k=1;k<N;k++) {
        i=((1.0f-a)*i+b)/(1.0f+a);
        iq[k]=q15_i(i); vq[k]=q15_v(V);
    }
    float out=0.0f, cur=0.0f; uint16_t valid=0;
    if(!mc_math_estimate_inductance_q15(iq,vq,N,64.0f,64.0f,fs,R,&out,&cur,&valid)) return 1;
    if(valid < 6U || fabsf(out-L)/L > 0.03f) return 2;
    printf("L target=%g measured=%g valid=%u\n",(double)L,(double)out,(unsigned)valid);
    return 0;
}
int main(void) {
    if(run_axis(240e-6f)) return 1;
    if(run_axis(360e-6f)) return 2;
    return 0;
}
