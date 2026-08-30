#include "motor/mc_math.h"
#include <math.h>
#include <stdbool.h>

static float clampf_local(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

float mc_math_battery_cut_input_max(float configured_input_max,
                                    float vbus,
                                    float cut_start,
                                    float cut_end) {
    float lim = fmaxf(configured_input_max, 0.0f);
    if (cut_start > cut_end && vbus < cut_start) {
        float scale = clampf_local((vbus - cut_end) / (cut_start - cut_end), 0.0f, 1.0f);
        lim *= scale;
    }
    return lim;
}

float mc_math_battery_regen_cut_input_min(float configured_input_min,
                                          float vbus,
                                          float cut_start,
                                          float cut_end) {
    float lim = fminf(configured_input_min, 0.0f);
    if (cut_end > cut_start) {
        if (vbus >= cut_end) return 0.0f;
        if (vbus > cut_start) {
            const float scale = clampf_local((cut_end - vbus) / (cut_end - cut_start),
                                             0.0f, 1.0f);
            lim *= scale;
        }
    }
    return lim;
}

static float map_clamped_local(float x, float in0, float in1, float out0, float out1) {
    if (fabsf(in1 - in0) < 1.0e-9f) return out1;
    const float t = clampf_local((x - in0) / (in1 - in0), 0.0f, 1.0f);
    return out0 + (out1 - out0) * t;
}

float mc_math_thermal_current_limit(float current_abs_max,
                                    float temperature_c,
                                    float temp_start_c,
                                    float temp_end_c) {
    const float base = fmaxf(current_abs_max, 0.0f);
    if (temperature_c < (temp_start_c + 0.1f)) return base;
    if (temperature_c > (temp_end_c - 0.1f)) return 0.0f;
    return map_clamped_local(temperature_c, temp_start_c, temp_end_c, base, 0.0f);
}

float mc_math_thermal_accel_limit(float current_max,
                                  float temperature_c,
                                  float temp_start_c,
                                  float temp_end_c,
                                  float temp_accel_dec) {
    const float base = fmaxf(current_max, 0.0f);
    const float dec = clampf_local(temp_accel_dec, 0.0f, 1.0f);
    const float start = temp_start_c + (25.0f - temp_start_c) * dec;
    const float end = temp_end_c + (25.0f - temp_end_c) * dec;
    if (temperature_c < (start + 0.1f)) return base;
    if (temperature_c > (end - 0.1f)) return 0.0f;
    return map_clamped_local(temperature_c, start, end, base, 0.0f);
}

float mc_math_start_current_limit(float current_max,
                                  float erpm_abs,
                                  float start_current_fraction,
                                  float start_current_erpm) {
    const float base = fmaxf(current_max, 0.0f);
    if (start_current_erpm <= 0.1f || erpm_abs >= start_current_erpm) return base;
    const float frac = clampf_local(start_current_fraction, 0.0f, 1.0f);
    return map_clamped_local(fabsf(erpm_abs), 0.0f, start_current_erpm, frac * base, base);
}

float mc_math_limit_input_current(float iq,
                                  float erpm,
                                  float duty_now,
                                  float measured_input_current,
                                  float input_current_max,
                                  float input_current_min) {
    if (iq == 0.0f) return 0.0f;

    const float in_max = fmaxf(input_current_max, 0.0f);
    const float in_min = fminf(input_current_min, 0.0f);
    const bool drawing = (iq * erpm) >= 0.0f;
    const float duty_abs = fabsf(duty_now);

    /* First-order inverter power relation: Iin ~= |duty| * |Imotor|.
       The Iq*ERPM sign distinguishes motoring from regenerative operation in
       both forward and reverse directions. */
    if (duty_abs > 0.02f) {
        const float in_mag = drawing ? in_max : fabsf(in_min);
        const float phase_mag = in_mag / duty_abs;
        if (fabsf(iq) > phase_mag) iq = copysignf(phase_mag, iq);
    }

    /* The DC shunt is the final source of truth. If the measured battery
       current already exceeds its configured side of the envelope, reduce the
       requested Iq proportionally without ever changing torque direction. */
    if (drawing) {
        if (in_max <= 0.0f) return 0.0f;
        if (measured_input_current > in_max && measured_input_current > 0.1f) {
            iq *= clampf_local(in_max / measured_input_current, 0.0f, 1.0f);
        }
    } else {
        if (in_min >= 0.0f) return 0.0f;
        if (measured_input_current < in_min && measured_input_current < -0.1f) {
            iq *= clampf_local(fabsf(in_min / measured_input_current), 0.0f, 1.0f);
        }
    }
    return iq;
}


bool mc_math_estimate_inductance_q15(const int32_t *i_axis_q15,
                                     const int32_t *v_axis_prev_q15,
                                     uint16_t count,
                                     float current_q_base_a,
                                     float voltage_q_base_v,
                                     float sample_hz,
                                     float resistance_ohm,
                                     float *inductance_h,
                                     float *current_a,
                                     uint16_t *valid_samples) {
    if (inductance_h) *inductance_h = 0.0f;
    if (current_a) *current_a = 0.0f;
    if (valid_samples) *valid_samples = 0U;
    if (!i_axis_q15 || !v_axis_prev_q15 || count < 3U ||
        !isfinite(current_q_base_a) || current_q_base_a <= 0.0f ||
        !isfinite(voltage_q_base_v) || voltage_q_base_v <= 0.0f ||
        !isfinite(sample_hz) || sample_hz <= 0.0f ||
        !isfinite(resistance_ohm) || resistance_ohm < 0.0f) {
        return false;
    }

    const float i_scale = current_q_base_a / 32768.0f;
    const float v_scale = voltage_q_base_v / 32768.0f;
    float l_sum = 0.0f;
    float i_sum = 0.0f;
    uint16_t n = 0U;

    for (uint16_t k = 1U; k < count; k++) {
        const float i0 = (float)i_axis_q15[k - 1U] * i_scale;
        const float i1 = (float)i_axis_q15[k] * i_scale;
        const float di_dt = (i1 - i0) * sample_hz;
        if (fabsf(di_dt) < 20.0f) continue;

        const float v = (float)v_axis_prev_q15[k] * v_scale;
        const float i_mid = 0.5f * (i0 + i1);
        const float l = (v - resistance_ohm * i_mid) / di_dt;
        if (!isfinite(l) || l < 0.2e-6f || l > 0.02f) continue;

        l_sum += l;
        i_sum += fabsf(i_mid);
        n++;
    }

    if (valid_samples) *valid_samples = n;
    if (n < 3U) return false;
    if (inductance_h) *inductance_h = l_sum / (float)n;
    if (current_a) *current_a = i_sum / (float)n;
    return true;
}
