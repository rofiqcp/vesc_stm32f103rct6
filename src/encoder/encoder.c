#include "encoder/encoder.h"
#include "encoder/enc_abi.h"

bool encoder_init(MotorRuntime *m) {
    if (!m || m->id != MOTOR_LEFT) return false;
    encoder_cfg_ABI.counts = m->encoder.cpr;
    return enc_abi_init(m, &encoder_cfg_ABI);
}
void encoder_deinit(MotorRuntime *m) { enc_abi_deinit(m); }
float encoder_read_deg(MotorRuntime *m) { return enc_abi_read_deg(m); }
void encoder_set_deg(MotorRuntime *m, float deg) { enc_abi_set_deg(m, deg); }
bool encoder_index_found(const MotorRuntime *m) { return enc_abi_index_found(m); }

void encoder_update_config(MotorRuntime *m) {
    if (!m || m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER) return;
    if (encoder_cfg_ABI.counts != m->encoder.cpr) {
        encoder_cfg_ABI.counts = m->encoder.cpr;
        (void)enc_abi_init(m, &encoder_cfg_ABI);
    }
}

bool encoder_is_configured(const MotorRuntime *m) {
    return m && m->id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.cpr >= 4U;
}
