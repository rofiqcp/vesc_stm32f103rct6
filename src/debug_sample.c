#include "debug_sample.h"
#include "app_config.h"
#include "foc_control.h"
#include <string.h>

static debug_sample_t s_samples[SAMPLE_BUFFER_LEN];
static volatile uint16_t s_target_len;
static volatile uint16_t s_wr;
static volatile uint16_t s_decimation;
static volatile uint16_t s_decim_count;
static volatile motor_id_t s_motor;
static volatile bool s_active;
static volatile bool s_ready;
static volatile bool s_raw;

static int16_t sat_i16(int32_t v){if(v>32767)return 32767;if(v<-32768)return -32768;return (int16_t)v;}

void debug_sample_init(void) {
    memset(s_samples, 0, sizeof(s_samples));
    s_target_len = SAMPLE_BUFFER_LEN;
    s_wr = 0U;
    s_decimation = SAMPLE_DEFAULT_DECIMATION;
    s_decim_count = 0U;
    s_motor = MOTOR_LEFT;
    s_active = false;
    s_ready = false;
    s_raw = false;
}

void debug_sample_start_ex(motor_id_t motor, uint16_t len, uint16_t decimation, bool raw) {
    if (len == 0U || len > SAMPLE_BUFFER_LEN) {
        len = SAMPLE_BUFFER_LEN;
    }
    if (decimation == 0U) {
        decimation = 1U;
    }
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_motor = motor;
    s_target_len = len;
    s_decimation = decimation;
    s_decim_count = 0U;
    s_wr = 0U;
    s_ready = false;
    s_raw = raw;
    s_active = true;
    if (pm == 0U) {
        __enable_irq();
    }
}

void debug_sample_start(motor_id_t motor, uint16_t len, uint16_t decimation) {
    debug_sample_start_ex(motor, len, decimation, false);
}

void debug_sample_capture_isr(MotorRuntime *active) {
    if (!s_active || s_ready) {
        return;
    }
    if (++s_decim_count < s_decimation) {
        return;
    }
    s_decim_count = 0U;
    if (active == NULL || active->id != s_motor) return;
    MotorRuntime *m = active;
    uint16_t i = s_wr;
    if (i >= s_target_len) {
        s_active = false;
        s_ready = true;
        return;
    }
    debug_sample_t *d=&s_samples[i];
    d->ia_cA=sat_i16((m->ia_q15*6400L)/32768L);d->ib_cA=sat_i16((m->ib_q15*6400L)/32768L);
    d->id_cA=sat_i16((m->id_q15*6400L)/32768L);d->iq_cA=sat_i16((m->iq_q15*6400L)/32768L);
    d->vd_cV=sat_i16((m->vd_q15*6400L)/32768L);d->vq_cV=sat_i16((m->vq_q15*6400L)/32768L);
    d->erpm=sat_i16(m->erpm_int);d->phase_u16=motor_sensor_electrical_phase_u16(m);
    d->duty_u_q15=m->duty_u_q15; d->duty_v_q15=m->duty_v_q15; d->duty_w_q15=m->duty_w_q15;
    d->current_raw_u=m->current_raw_u; d->current_raw_v=m->current_raw_v;
    int32_t vdV=(m->vbus_q15*640L)/32768L; if(vdV<0)vdV=0; if(vdV>65535)vdV=65535; d->vbus_dV=(uint16_t)vdV;
    d->motor=(uint8_t)m->id; d->hall_raw=m->hall.raw_state;
    s_wr=(uint16_t)(i+1U);if(s_wr>=s_target_len){s_active=false;s_ready=true;}
}

bool debug_sample_ready(void){return s_ready;}
uint16_t debug_sample_count(void){return s_wr;}
const debug_sample_t *debug_sample_data(void){return s_samples;}
void debug_sample_mark_sent(void){uint32_t pm=__get_PRIMASK();__disable_irq();s_ready=false;s_wr=0;if(!pm)__enable_irq();}
bool debug_sample_active(void){return s_active;}
bool debug_sample_raw(void){return s_raw;}
