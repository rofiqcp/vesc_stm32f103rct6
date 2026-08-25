#include "debug_sample.h"
#include "applications/appconf_default.h"
#include "motor/mcpwm_foc.h"
#include <string.h>

static debug_sample_t s_samples[SAMPLE_BUFFER_LEN];
static volatile uint16_t s_target_len;
static volatile uint16_t s_wr;             /* physical next-write index */
static volatile uint16_t s_count;          /* valid samples in buffer */
static volatile uint16_t s_read_start;     /* oldest physical sample */
static volatile uint16_t s_decimation;
static volatile uint16_t s_decim_count;
static volatile uint16_t s_post_remaining;
static volatile motor_id_t s_motor;
static volatile debug_sampling_mode s_mode;
static volatile bool s_active;
static volatile bool s_armed;
static volatile bool s_triggered;
static volatile bool s_capture_valid;
static volatile bool s_send_pending;
static volatile bool s_auto_send;
static volatile bool s_raw;
static volatile bool s_prev_running;
static volatile bool s_prev_fault;

static int16_t sat_i16(int32_t v){if(v>32767)return 32767;if(v<-32768)return -32768;return (int16_t)v;}

static void sample_fill(debug_sample_t *d, MotorRuntime *m) {
    d->ia_cA=sat_i16((m->ia_q15*6400L)/32768L);d->ib_cA=sat_i16((m->ib_q15*6400L)/32768L);
    d->id_cA=sat_i16((m->id_q15*6400L)/32768L);d->iq_cA=sat_i16((m->iq_q15*6400L)/32768L);
    d->vd_cV=sat_i16((m->vd_q15*6400L)/32768L);d->vq_cV=sat_i16((m->vq_q15*6400L)/32768L);
    d->erpm=sat_i16(m->erpm_int);d->phase_u16=motor_sensor_electrical_phase_u16(m);
    d->duty_u_q15=m->duty_u_q15; d->duty_v_q15=m->duty_v_q15; d->duty_w_q15=m->duty_w_q15;
    d->current_raw_u=m->current_raw_u; d->current_raw_v=m->current_raw_v;
    int32_t vdV=(m->vbus_q15*640L)/32768L; if(vdV<0)vdV=0; if(vdV>65535)vdV=65535; d->vbus_dV=(uint16_t)vdV;
    d->motor=(uint8_t)m->id; d->hall_raw=m->hall.raw_state;
}

static void finish_capture_isr(void) {
    s_active = false;
    s_armed = false;
    s_capture_valid = (s_count != 0U);
    s_read_start = (s_count >= s_target_len) ? s_wr : 0U;
    s_send_pending = s_capture_valid && s_auto_send;
}

void debug_sample_init(void) {
    memset(s_samples, 0, sizeof(s_samples));
    s_target_len = SAMPLE_BUFFER_LEN;
    s_wr = 0U; s_count = 0U; s_read_start = 0U;
    s_decimation = SAMPLE_DEFAULT_DECIMATION;
    s_decim_count = 0U; s_post_remaining = 0U;
    s_motor = MOTOR_LEFT; s_mode = DEBUG_SAMPLING_OFF;
    s_active = false; s_armed = false; s_triggered = false;
    s_capture_valid = false; s_send_pending = false; s_auto_send = true;
    s_raw = false; s_prev_running = false; s_prev_fault = false;
}

bool debug_sample_control(debug_sampling_mode mode, motor_id_t motor,
                          uint16_t len, uint16_t decimation, bool raw) {
    /* debug_sampling_mode is an enum with DEBUG_SAMPLING_OFF as its lowest
       valid value; checking mode < 0 is both redundant and triggers
       -Wtype-limits on ARM GCC when the enum is represented unsigned. */
    if (mode > DEBUG_SAMPLING_SEND_SINGLE_SAMPLE) return false;
    if (len == 0U || len > SAMPLE_BUFFER_LEN) len = SAMPLE_BUFFER_LEN;
    if (decimation == 0U) decimation = 1U;

    uint32_t pm = __get_PRIMASK();
    __disable_irq();

    if (mode == DEBUG_SAMPLING_OFF) {
        s_active = false; s_armed = false; s_triggered = false;
        s_send_pending = false; s_mode = mode;
        if (pm == 0U) __enable_irq();
        return true;
    }

    if (mode == DEBUG_SAMPLING_SEND_LAST_SAMPLES) {
        if (s_capture_valid) s_send_pending = true;
        if (pm == 0U) __enable_irq();
        return s_capture_valid;
    }

    /* Never overwrite a buffer while the UART sample worker is serializing it. */
    if (s_send_pending) {
        if (pm == 0U) __enable_irq();
        return false;
    }

    s_motor = motor;
    s_mode = mode;
    s_target_len = (mode == DEBUG_SAMPLING_SEND_SINGLE_SAMPLE) ? 1U : len;
    s_decimation = decimation;
    s_decim_count = 0U;
    s_wr = 0U; s_count = 0U; s_read_start = 0U;
    s_post_remaining = 0U;
    s_raw = raw;
    s_capture_valid = false;
    s_send_pending = false;
    s_triggered = false;
    s_prev_running = false;
    s_prev_fault = false;
    s_auto_send = (mode != DEBUG_SAMPLING_TRIGGER_START_NOSEND &&
                   mode != DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND);

    switch (mode) {
    case DEBUG_SAMPLING_NOW:
    case DEBUG_SAMPLING_SEND_SINGLE_SAMPLE:
        s_active = true;
        s_armed = false;
        break;
    case DEBUG_SAMPLING_START:
    case DEBUG_SAMPLING_TRIGGER_START:
    case DEBUG_SAMPLING_TRIGGER_FAULT:
    case DEBUG_SAMPLING_TRIGGER_START_NOSEND:
    case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND:
        s_active = true;
        s_armed = true;
        break;
    default:
        s_active = false;
        s_armed = false;
        break;
    }

    if (pm == 0U) __enable_irq();
    return true;
}

void debug_sample_start_ex(motor_id_t motor, uint16_t len, uint16_t decimation, bool raw) {
    (void)debug_sample_control(DEBUG_SAMPLING_NOW, motor, len, decimation, raw);
}

void debug_sample_start(motor_id_t motor, uint16_t len, uint16_t decimation) {
    debug_sample_start_ex(motor, len, decimation, false);
}

void debug_sample_capture_isr(MotorRuntime *active) {
    if (!s_active || active == NULL || active->id != s_motor) return;

    const bool running = active->pwm_enabled && active->fault == MOTOR_FAULT_NONE;
    const bool fault_now = active->fault != MOTOR_FAULT_NONE;
    const bool start_edge = running && !s_prev_running;
    const bool fault_edge = fault_now && !s_prev_fault;
    s_prev_running = running;
    s_prev_fault = fault_now;

    if (s_armed && !s_triggered) {
        bool trigger = false;
        switch (s_mode) {
        case DEBUG_SAMPLING_START:
            trigger = start_edge;
            break;
        case DEBUG_SAMPLING_TRIGGER_START:
        case DEBUG_SAMPLING_TRIGGER_START_NOSEND:
            trigger = start_edge;
            break;
        case DEBUG_SAMPLING_TRIGGER_FAULT:
        case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND:
            trigger = fault_edge;
            break;
        default:
            trigger = true;
            break;
        }
        if (trigger) {
            s_triggered = true;
            s_armed = false;
            if (s_mode == DEBUG_SAMPLING_START) {
                /* START begins a fresh capture on the run edge. */
                s_wr = 0U; s_count = 0U; s_read_start = 0U;
                s_post_remaining = s_target_len;
            } else {
                /* Trigger modes keep a circular pre-trigger history. Continuing
                   for half the requested samples naturally leaves roughly
                   50/50 pre/post data without copying in the ISR. */
                s_post_remaining = (uint16_t)((s_target_len + 1U) / 2U);
            }
        }
    }

    if (++s_decim_count < s_decimation) return;
    s_decim_count = 0U;

    /* START does not record before its start edge. Trigger modes do, as a
       circular pre-trigger history. */
    const bool trigger_mode = (s_mode == DEBUG_SAMPLING_TRIGGER_START ||
                               s_mode == DEBUG_SAMPLING_TRIGGER_FAULT ||
                               s_mode == DEBUG_SAMPLING_TRIGGER_START_NOSEND ||
                               s_mode == DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND);
    if (s_mode == DEBUG_SAMPLING_START && !s_triggered) return;

    sample_fill(&s_samples[s_wr], active);
    s_wr++;
    if (s_wr >= s_target_len) s_wr = 0U;
    if (s_count < s_target_len) s_count++;

    if (trigger_mode) {
        if (s_triggered && s_post_remaining > 0U) {
            s_post_remaining--;
            if (s_post_remaining == 0U) finish_capture_isr();
        }
        return;
    }

    if (s_mode == DEBUG_SAMPLING_START) {
        if (s_triggered && s_post_remaining > 0U) {
            s_post_remaining--;
            if (s_post_remaining == 0U) finish_capture_isr();
        }
        return;
    }

    if (s_count >= s_target_len) finish_capture_isr();
}

bool debug_sample_ready(void){return s_capture_valid && s_send_pending;}
bool debug_sample_has_capture(void){return s_capture_valid;}
uint16_t debug_sample_count(void){return s_count;}
const debug_sample_t *debug_sample_data(void){return s_samples;}
const debug_sample_t *debug_sample_at(uint16_t logical_index){
    if (!s_capture_valid || logical_index >= s_count || s_target_len == 0U) return NULL;
    uint16_t p=(uint16_t)(s_read_start + logical_index);
    while (p >= s_target_len) p=(uint16_t)(p - s_target_len);
    return &s_samples[p];
}
void debug_sample_mark_sent(void){uint32_t pm=__get_PRIMASK();__disable_irq();s_send_pending=false;if(!pm)__enable_irq();}
bool debug_sample_active(void){return s_active;}
bool debug_sample_raw(void){return s_raw;}
debug_sampling_mode debug_sample_mode(void){return s_mode;}
