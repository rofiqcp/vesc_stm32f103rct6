#include "sensor_detect.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_control.h"
#include "foc_math.h"
#include "app_config.h"
#include "vesc_config.h"
#include "vesc_timeout.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Exactly one detection transaction at a time, matching the VESC blocking
 * command model. This also protects the controller-wide timeout settings. */
static MotorRuntime *s_detect_owner = NULL;

static uint16_t deg_to_u16_int(uint32_t deg) {
    return (uint16_t)(((deg % 360U) * 65536UL) / 360UL);
}

static void detect_clear_acc(MotorRuntime *m) {
    memset(m->detect.hall_sin_sum, 0, sizeof(m->detect.hall_sin_sum));
    memset(m->detect.hall_cos_sum, 0, sizeof(m->detect.hall_cos_sum));
    memset(m->detect.hall_samples, 0, sizeof(m->detect.hall_samples));
    memset(m->detect.result_hall_table, 255, sizeof(m->detect.result_hall_table));
    m->detect.hall_valid_states = 0U;
}

static void detect_restore_timeout_and_gains(MotorRuntime *m) {
    motor_set_current_pi_gains(m, m->detect.saved_current_kp, m->detect.saved_current_ki);
    vesc_timeout_configure(m->detect.saved_timeout_ms, m->detect.saved_timeout_brake_a);
    vesc_timeout_reset();
}

static void hall_apply_result(MotorRuntime *m) {
    typedef struct { uint8_t raw; uint16_t ang; } item_t;
    item_t items[6];
    uint8_t n = 0U;

    for (uint8_t raw = 0U; raw < 8U; raw++) {
        uint8_t v = m->detect.result_hall_table[raw];
        if (v == 255U) {
            m->foc_hall_table[raw] = 255U;
            m->hall_angle_u16[raw] = 0U;
            m->hall_table[raw] = -1;
            continue;
        }
        uint16_t ang = (uint16_t)(((uint32_t)v * 65536U) / 200U);
        m->foc_hall_table[raw] = v;
        m->hall_angle_u16[raw] = ang;
        if (n < 6U) {
            items[n].raw = raw;
            items[n].ang = ang;
            n++;
        }
    }

    /* hall_table[] is only the fast neighbor/sector ordering used by this
       F103 port for direction interpolation. The canonical VESC FOC table is
       foc_hall_table[] above and remains the configuration source of truth. */
    for (uint8_t i = 0U; i < n; i++) {
        for (uint8_t j = (uint8_t)(i + 1U); j < n; j++) {
            if (items[j].ang < items[i].ang) {
                item_t t = items[i]; items[i] = items[j]; items[j] = t;
            }
        }
    }
    for (uint8_t i = 0U; i < n; i++) m->hall_table[items[i].raw] = (int8_t)i;

    m->hall.valid = false;
    m->hall.sector = -1;
    m->hall.phase_per_cycle_q16 = 0;
    m->hall_offset_u16 = 0U;
    motor_hall_edge_isr(m);
}

bool sensor_detect_request_current_ex(MotorRuntime *m, uint8_t requested_mode,
                                      float current_a, bool apply_result) {
    if (m == NULL || requested_mode > SENSOR_MODE_ENCODER) return false;
    if (m->id == MOTOR_RIGHT && requested_mode == SENSOR_MODE_ENCODER) return false;

    /* The standard COMM_DETECT_* path is serialized by blocking_thread with
       queue depth 1, exactly as upstream commands.c. Keep the same single
       detection owner for the optional custom diagnostic path as well. */
    if (s_detect_owner != NULL || m->detect.busy || m->detect.requested) return false;
    s_detect_owner = m;

    /* Do not erase an electrical fault just to start a measurement. VESC
       detection aborts when mc_interface_get_fault() becomes non-NONE. */
    if (m->fault != MOTOR_FAULT_NONE) {
        if (s_detect_owner == m) s_detect_owner = NULL;
        return false;
    }

    motor_stop(m);
    sensor_detect_t *d = &m->detect;
    memset((void *)d->result_hall_table, 255, sizeof(d->result_hall_table));
    d->requested = true;
    d->busy = false;
    d->success = false;
    d->result_mode = SENSOR_MODE_AUTO;
    d->state = SENSOR_DETECT_PREPARE;
    d->apply_result = apply_result;

    current_a = fabsf(current_a);
    if (current_a < 0.2f) current_a = 0.2f;
    if (current_a > SENSOR_DETECT_MAX_CURRENT_A) current_a = SENSOR_DETECT_MAX_CURRENT_A;
    if (m->current_max_a > 0.0f && current_a > m->current_max_a) current_a = m->current_max_a;
    d->drive_current_a = current_a;

    d->saved_sensor_mode = m->sensor_mode;
    d->saved_sensor_request_mode = m->sensor_request_mode;
    d->saved_current_kp = m->current_kp;
    d->saved_current_ki = m->current_ki;
    d->saved_timeout_ms = vesc_timeout_get_timeout_ms();
    d->saved_timeout_brake_a = vesc_timeout_get_brake_current();
    d->result_encoder_offset_deg = 0.0f;
    d->result_encoder_ratio = 0.0f;
    d->result_encoder_inverted = false;

    m->sensor_request_mode = requested_mode;
    return true;
}

bool sensor_detect_request_current(MotorRuntime *m, uint8_t requested_mode, float current_a) {
    /* Port-specific UI request applies a successful result immediately. The
       standard VESC COMM_DETECT_* path calls the _ex() form with false. */
    return sensor_detect_request_current_ex(m, requested_mode, current_a, true);
}

bool sensor_detect_request(MotorRuntime *m, uint8_t requested_mode) {
    return sensor_detect_request_current_ex(m, requested_mode, SENSOR_DETECT_CURRENT_A, true);
}

static void detect_restore_sensor(MotorRuntime *m) {
    uint8_t mode = m->detect.saved_sensor_mode;
    if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER) mode = SENSOR_MODE_HALL;
    motor_hw_configure_sensor(m, mode);
    m->sensor_request_mode = m->detect.saved_sensor_request_mode;
    if (mode == SENSOR_MODE_HALL) motor_hall_edge_isr(m);
}

static void detect_common_stop(MotorRuntime *m) {
    motor_set_foc_targets(m, 0.0f, 0.0f);
    m->detect_force_angle = false;
    m->control_mode = MOTOR_CTRL_OFF;
    m->command_active = false;
    motor_hw_set_pwm_enabled(m, false);
    detect_restore_timeout_and_gains(m);
}

static void detect_fail(MotorRuntime *m) {
    motor_fault_t original_fault = m->fault;
    if (m->id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER) m->encoder.synced = false;
    detect_common_stop(m);
    detect_restore_sensor(m);
    m->detect.busy = false;
    m->detect.requested = false;
    m->detect.success = false;
    m->detect.state = SENSOR_DETECT_FAILED;
    if (s_detect_owner == m) s_detect_owner = NULL;

    /* VESC detection failure is a measurement result, not automatically a
       latched electrical motor fault. Preserve any real electrical fault that
       caused the abort, but a clean algorithmic Hall/encoder failure only
       returns failure to the caller. */
    (void)original_fault;
}

static void detect_done(MotorRuntime *m, uint8_t mode) {
    detect_common_stop(m);

    if (m->detect.apply_result) {
        if (mode == SENSOR_MODE_HALL) {
            motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
            hall_apply_result(m);
            m->sensor_mode = SENSOR_MODE_HALL;
            m->sensor_request_mode = SENSOR_MODE_HALL;
        } else if (mode == SENSOR_MODE_ENCODER && m->id == MOTOR_LEFT) {
            motor_hw_configure_sensor(m, SENSOR_MODE_ENCODER);
            m->pole_pairs = (uint8_t)lroundf(m->detect.result_encoder_ratio);
            if (m->pole_pairs < 1U) m->pole_pairs = 1U;
            m->encoder.electrical_ratio = m->detect.result_encoder_ratio;
            m->encoder.electrical_ratio_q16 = (uint32_t)lroundf(m->encoder.electrical_ratio * 65536.0f);
            m->encoder.inverted = m->detect.result_encoder_inverted;
            m->encoder.elec_offset_u16 = foc_deg_to_u16(m->detect.result_encoder_offset_deg);
            m->encoder.phase_per_count_q16 = (uint32_t)(((uint64_t)m->encoder.electrical_ratio_q16 << 16) / m->encoder.cpr);
            int32_t zero = motor_encoder_extended_count(m);
            m->encoder.session_zero_count = zero;
            m->encoder.mechanical_zero_count = zero;
            m->encoder.synced = true;
            m->encoder.motion_proved = true;
            m->sensor_mode = SENSOR_MODE_ENCODER;
            m->sensor_request_mode = SENSOR_MODE_ENCODER;
        }
        vesc_config_sync_motor_runtime(m->id);
    } else {
        /* Standard VESC detection returns parameters and then restores the
           old configuration. VESC Tool decides whether to apply/write them. */
        detect_restore_sensor(m);
    }

    m->detect.busy = false;
    m->detect.requested = false;
    m->detect.success = true;
    m->detect.result_mode = mode;
    m->detect.state = SENSOR_DETECT_DONE;
    if (s_detect_owner == m) s_detect_owner = NULL;
}

static void hall_sample(MotorRuntime *m) {
    /* Upstream counts all 8 raw states. Normal 120-degree Hall should leave
       exactly two states unobserved (normally 0 and 7). Do not silently drop
       invalid states here; they are part of detect-quality validation. */
    uint8_t raw = motor_hw_read_hall_raw(m->id) & 7U;
    int32_t sn, cs;
    foc_fast_sincos_u16_q15(m->detect_phase_u16, &sn, &cs);
    m->detect.hall_sin_sum[raw] += sn;
    m->detect.hall_cos_sum[raw] += cs;
    m->detect.hall_samples[raw]++;
}

static bool hall_evaluate(MotorRuntime *m) {
    uint8_t fails = 0U;
    uint8_t valid = 0U;
    for (uint8_t raw = 0U; raw < 8U; raw++) {
        if (m->detect.hall_samples[raw] > SENSOR_DETECT_HALL_MIN_SAMPLES) {
            float a = atan2f((float)m->detect.hall_sin_sum[raw],
                             (float)m->detect.hall_cos_sum[raw]);
            if (a < 0.0f) a += 2.0f * 3.14159265358979323846f;
            uint32_t v = (uint32_t)lroundf(a * (200.0f / (2.0f * 3.14159265358979323846f)));
            if (v >= 200U) v -= 200U;
            m->detect.result_hall_table[raw] = (uint8_t)v;
            valid++;
        } else {
            m->detect.result_hall_table[raw] = 255U;
            fails++;
        }
    }
    m->detect.hall_valid_states = valid;
    /* Same core criterion as VESC (fails==2), plus the normal Hall invariant
       from this board architecture: all-low/all-high must be the missing two. */
    return fails == 2U &&
           m->detect.result_hall_table[0] == 255U &&
           m->detect.result_hall_table[7] == 255U;
}

void sensor_detect_update_1khz(MotorRuntime *m, uint32_t now_ms) {
    if (m == NULL || (!m->detect.requested && !m->detect.busy)) return;
    sensor_detect_t *d = &m->detect;
    const uint32_t hall_fwd_steps = 360U * SENSOR_DETECT_SWEEPS;
    const uint32_t hall_rev_steps = 361U * SENSOR_DETECT_SWEEPS;
    const uint32_t encoder_steps = 360U * SENSOR_DETECT_SWEEPS;

    if (d->busy) {
        /* Upstream disables the command timeout during detect. Keep the global
           timer alive while the blocking detection transaction owns it. */
        vesc_timeout_reset();
        if (m->fault != MOTOR_FAULT_NONE) {
            detect_fail(m);
            return;
        }
    }

    switch (d->state) {
    case SENSOR_DETECT_PREPARE:
        if (!foc_calibration_done()) return;
        if (!foc_calibration_valid() || m->fault != MOTOR_FAULT_NONE) { detect_fail(m); return; }

        d->requested = false;
        d->busy = true;
        d->success = false;
        d->sweep_index = 0U;
        d->sweep_pass = 0U;
        m->control_mode = MOTOR_CTRL_DETECT;
        m->command_active = false;
        m->detect_force_angle=true;
        m->detect_phase_u16 = 0U;
        motor_set_foc_targets(m, 0.0f, 0.0f);
        detect_clear_acc(m);

        /* Match the VESC Hall-detect temporary controller values. Keeping the
           fixed 16-kHz F103 PWM is intentional; changing f_zv at runtime would
           also require rebuilding the ADC/TIM2 synchronization. */
        if (m->sensor_request_mode != SENSOR_MODE_ENCODER) {
            motor_set_current_pi_gains(m, 0.01f, 10.0f);
        }

        vesc_timeout_configure(60000U, 0.0f);
        vesc_timeout_reset();

        if (m->sensor_request_mode == SENSOR_MODE_ENCODER && m->id == MOTOR_LEFT) {
            motor_hw_configure_sensor(m, SENSOR_MODE_ENCODER);
            motor_hw_encoder_reset();
            d->state = SENSOR_DETECT_ENCODER_LOCK0;
        } else {
            motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
            motor_hall_edge_isr(m);
            d->state = SENSOR_DETECT_HALL_LOCK;
        }
        d->step_tick = now_ms;
        break;

    case SENSOR_DETECT_HALL_LOCK: {
        /* VESC: 1000 iterations, 1 ms each, Id ramp 0 -> requested current. */
        uint32_t elapsed = (uint32_t)(now_ms - d->step_tick);
        if (elapsed > SENSOR_DETECT_CURRENT_RAMP_MS) elapsed = SENSOR_DETECT_CURRENT_RAMP_MS;
        float id = d->drive_current_a * ((float)elapsed / (float)SENSOR_DETECT_CURRENT_RAMP_MS);
        m->detect_phase_u16 = 0U;
        motor_set_foc_targets(m, id, 0.0f);
        if (elapsed >= SENSOR_DETECT_CURRENT_RAMP_MS) {
            motor_set_foc_targets(m,d->drive_current_a,0.0f);
            detect_clear_acc(m);
            d->sweep_index = 0U;
            m->detect_phase_u16 = 0U;
            d->step_tick = now_ms;
            d->state = SENSOR_DETECT_HALL_FWD;
        }
    } break;

    case SENSOR_DETECT_HALL_FWD:
        motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if ((uint32_t)(now_ms - d->step_tick) >= SENSOR_DETECT_STEP_MS) {
            /* Phase was held for 5 ms; now sample it, exactly like upstream. */
            hall_sample(m);
            d->sweep_index++;
            if (d->sweep_index >= hall_fwd_steps) {
                d->sweep_index = 0U;
                m->detect_phase_u16 = 0U; /* 360 degrees */
                d->state = SENSOR_DETECT_HALL_REV;
            } else {
                m->detect_phase_u16 = deg_to_u16_int(d->sweep_index % 360U);
            }
            d->step_tick = now_ms;
        }
        break;

    case SENSOR_DETECT_HALL_REV:
        motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if ((uint32_t)(now_ms - d->step_tick) >= SENSOR_DETECT_STEP_MS) {
            hall_sample(m);
            d->sweep_index++;
            if (d->sweep_index >= hall_rev_steps) {
                d->state = SENSOR_DETECT_HALL_EVAL;
            } else {
                uint32_t j = 360U - (d->sweep_index % 361U);
                m->detect_phase_u16 = deg_to_u16_int(j);
            }
            d->step_tick = now_ms;
        }
        break;

    case SENSOR_DETECT_HALL_EVAL:
        if (hall_evaluate(m)) {
            detect_done(m, SENSOR_MODE_HALL);
        } else if (m->id == MOTOR_LEFT && m->sensor_request_mode == SENSOR_MODE_AUTO) {
            /* Port-specific AUTO fallback only. Standard COMM_DETECT_HALL_FOC
               never guesses encoder mode. */
            motor_set_foc_targets(m, 0.0f, 0.0f);
            motor_hw_set_pwm_enabled(m, false);
            motor_hw_configure_sensor(m, SENSOR_MODE_ENCODER);
            motor_hw_encoder_reset();
            d->busy = true;
            m->detect_force_angle=true;
            m->detect_phase_u16 = 0U;
            d->step_tick = now_ms;
            d->state = SENSOR_DETECT_ENCODER_LOCK0;
        } else {
            detect_fail(m);
        }
        break;

    case SENSOR_DETECT_ENCODER_LOCK0: {
        /* AB has no index. Use the same gentle current ramp before sweeping a
           known electrical phase; this establishes a safe session reference. */
        uint32_t elapsed = (uint32_t)(now_ms - d->step_tick);
        if (elapsed > SENSOR_DETECT_CURRENT_RAMP_MS) elapsed = SENSOR_DETECT_CURRENT_RAMP_MS;
        float id = d->drive_current_a * ((float)elapsed / (float)SENSOR_DETECT_CURRENT_RAMP_MS);
        m->detect_phase_u16 = 0U;
        motor_set_foc_targets(m, id, 0.0f);
        if (elapsed >= SENSOR_DETECT_CURRENT_RAMP_MS) {
            motor_hw_encoder_reset();
            d->encoder_start_count = motor_encoder_extended_count(m);
            d->sweep_index = 0U;
            m->detect_phase_u16 = 0U;
            d->step_tick = now_ms;
            d->state = SENSOR_DETECT_ENCODER_SWEEP;
        }
    } break;

    case SENSOR_DETECT_ENCODER_SWEEP:
        motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if ((uint32_t)(now_ms - d->step_tick) >= SENSOR_DETECT_STEP_MS) {
            d->sweep_index++;
            if (d->sweep_index >= encoder_steps) {
                d->encoder_end_count = motor_encoder_extended_count(m);
                d->state = SENSOR_DETECT_ENCODER_EVAL;
            } else {
                m->detect_phase_u16 = deg_to_u16_int(d->sweep_index % 360U);
            }
            d->step_tick = now_ms;
        }
        break;

    case SENSOR_DETECT_ENCODER_EVAL: {
        int32_t delta = d->encoder_end_count - d->encoder_start_count;
        int32_t ad = abs(delta);
        if (ad < SENSOR_DETECT_MIN_ENCODER_COUNTS) { detect_fail(m); break; }
        float counts_per_elec = (float)ad / (float)SENSOR_DETECT_SWEEPS;
        float ratio = (float)m->encoder.cpr / counts_per_elec;
        if (!isfinite(ratio) || ratio < 1.0f || ratio > (float)SENSOR_DETECT_MAX_POLE_PAIRS) {
            detect_fail(m); break;
        }
        d->result_encoder_ratio = ratio;
        d->result_encoder_inverted = (delta < 0);
        /* With AB-no-index the absolute electrical offset is session-specific.
           Return phase zero after controlled alignment and establish the boot
           zero only when the caller explicitly applies this result. */
        d->result_encoder_offset_deg = 0.0f;
        m->detect_phase_u16 = 0U;
        motor_set_foc_targets(m,d->drive_current_a,0.0f);
        d->step_tick = now_ms;
        d->state = SENSOR_DETECT_ENCODER_RETURN0;
    } break;

    case SENSOR_DETECT_ENCODER_RETURN0:
        m->detect_phase_u16 = 0U;
        motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if ((uint32_t)(now_ms - d->step_tick) >= SENSOR_DETECT_SETTLE_MS) {
            d->step_tick = now_ms;
            d->state = SENSOR_DETECT_ENCODER_ALIGN;
        }
        break;

    case SENSOR_DETECT_ENCODER_ALIGN:
        detect_done(m, SENSOR_MODE_ENCODER);
        break;

    default:
        break;
    }
}
