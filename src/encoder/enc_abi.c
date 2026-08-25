#include "encoder/enc_abi.h"
#include "motor/mc_interface.h"
#include "hwconf/hw.h"
#include <math.h>

static float wrap_360(float x) {
    while (x < 0.0f) x += 360.0f;
    while (x >= 360.0f) x -= 360.0f;
    return x;
}

bool enc_abi_init(MotorRuntime *m, const encoder_cfg_ABI_t *cfg) {
    if (!m || !cfg || m->id != MOTOR_LEFT || cfg->counts < 4U || cfg->counts > 65535U) return false;
    m->encoder.cpr = cfg->counts;
    m->encoder.synced = false;
    m->encoder.motion_proved = false;
    m->encoder.speed_sample_valid = false;
    motor_hw_configure_sensor(m, SENSOR_MODE_ENCODER);
    return true;
}

void enc_abi_deinit(MotorRuntime *m) {
    if (!m) return;
    m->encoder.synced = false;
    m->encoder.motion_proved = false;
}

float enc_abi_read_deg(MotorRuntime *m) {
    if (!m || m->encoder.cpr == 0U) return 0.0f;
    int32_t rel = motor_encoder_extended_count(m) - m->encoder.session_zero_count;
    float deg = ((float)rel * 360.0f) / (float)m->encoder.cpr;
    return wrap_360(deg);
}

void enc_abi_set_deg(MotorRuntime *m, float deg) {
    if (!m || m->encoder.cpr == 0U) return;
    deg = wrap_360(deg);
    int32_t old_ext = motor_encoder_extended_count(m);
    int32_t old_pos_delta = old_ext - m->encoder.mechanical_zero_count;
    uint32_t target = (uint32_t)lrintf((deg / 360.0f) * (float)m->encoder.cpr);
    if (target >= m->encoder.cpr) target = 0U;

    /* Match upstream enc_abi_set_deg(): the hardware quadrature counter itself
       is rebased. The extra mechanical_zero_count compensation is only for the
       F103 application's multi-turn position coordinate and does not affect
       the electrical ABI angle. */
    motor_hw_encoder_set_count(m, (uint16_t)target);
    m->encoder.session_zero_count = 0;
    m->encoder.mechanical_zero_count = (int32_t)target - old_pos_delta;
    m->encoder.prev_extended_count = (int32_t)target;
    m->encoder.extended_count = (int32_t)target;
    m->encoder.speed_sample_valid = false;
    m->encoder.synced = true;
    m->encoder.motion_proved = true;
}

bool enc_abi_index_found(const MotorRuntime *m) {
    return m && m->encoder.synced;
}
