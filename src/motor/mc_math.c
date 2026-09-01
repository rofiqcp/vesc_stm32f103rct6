#include "motor/mc_math.h"
#include <math.h>
#include <stdbool.h>

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi clampf_local: menjalankan operasi clampf local sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
static float clampf_local(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Parameter configured_input_max: batas atau nilai maksimum untuk validasi dan proteksi.
// Parameter vbus: tegangan DC bus yang digunakan untuk modulasi dan proteksi.
// Parameter cut_start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter cut_end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_math_battery_cut_input_max: menjalankan operasi mc math battery cut input max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_math_battery_cut_input_max(float configured_input_max,
                                    float vbus,
                                    float cut_start,
                                    float cut_end) {
    // Variabel lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lim = fmaxf(configured_input_max, 0.0f);
    if (cut_start > cut_end && vbus < cut_start) {
        // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float scale = clampf_local((vbus - cut_end) / (cut_start - cut_end), 0.0f, 1.0f);
        lim *= scale;
    }
    return lim;
}

// Parameter configured_input_min: batas atau nilai minimum untuk validasi dan proteksi.
// Parameter vbus: tegangan DC bus yang digunakan untuk modulasi dan proteksi.
// Parameter cut_start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter cut_end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_math_battery_regen_cut_input_min: menjalankan operasi mc math battery regen cut input min sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_math_battery_regen_cut_input_min(float configured_input_min,
                                          float vbus,
                                          float cut_start,
                                          float cut_end) {
    // Variabel lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lim = fminf(configured_input_min, 0.0f);
    if (cut_end > cut_start) {
        if (vbus >= cut_end)
            return 0.0f;
        if (vbus > cut_start) {
            // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float scale = clampf_local((cut_end - vbus) / (cut_end - cut_start),
                                             0.0f, 1.0f);
            lim *= scale;
        }
    }
    return lim;
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter in0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter in1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi map_clamped_local: menjalankan operasi map clamped local sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float map_clamped_local(float x, float in0, float in1, float out0, float out1) {
    if (fabsf(in1 - in0) < 1.0e-9f)
        return out1;
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float t = clampf_local((x - in0) / (in1 - in0), 0.0f, 1.0f);
    return out0 + (out1 - out0) * t;
}

// Parameter current_abs_max: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter temperature_c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter temp_start_c: temperatur atau nilai sementara sesuai konteks modul.
// Parameter temp_end_c: temperatur atau nilai sementara sesuai konteks modul.
// Fungsi mc_math_thermal_current_limit: membatasi mc math thermal current limit ke rentang yang diizinkan agar
// pengendali dan perangkat keras tetap aman.
float mc_math_thermal_current_limit(float current_abs_max,
                                    float temperature_c,
                                    float temp_start_c,
                                    float temp_end_c) {
    // Variabel base: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float base = fmaxf(current_abs_max, 0.0f);
    if (temperature_c < (temp_start_c + 0.1f))
        return base;
    if (temperature_c > (temp_end_c - 0.1f))
        return 0.0f;
    return map_clamped_local(temperature_c, temp_start_c, temp_end_c, base, 0.0f);
}

// Parameter current_max: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter temperature_c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter temp_start_c: temperatur atau nilai sementara sesuai konteks modul.
// Parameter temp_end_c: temperatur atau nilai sementara sesuai konteks modul.
// Parameter temp_accel_dec: temperatur atau nilai sementara sesuai konteks modul.
// Fungsi mc_math_thermal_accel_limit: membatasi mc math thermal accel limit ke rentang yang diizinkan agar
// pengendali dan perangkat keras tetap aman.
float mc_math_thermal_accel_limit(float current_max,
                                  float temperature_c,
                                  float temp_start_c,
                                  float temp_end_c,
                                  float temp_accel_dec) {
    // Variabel base: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float base = fmaxf(current_max, 0.0f);
    // Variabel dec: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float dec = clampf_local(temp_accel_dec, 0.0f, 1.0f);
    // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float start = temp_start_c + (25.0f - temp_start_c) * dec;
    // Variabel end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float end = temp_end_c + (25.0f - temp_end_c) * dec;
    if (temperature_c < (start + 0.1f))
        return base;
    if (temperature_c > (end - 0.1f))
        return 0.0f;
    return map_clamped_local(temperature_c, start, end, base, 0.0f);
}

// Parameter current_max: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter erpm_abs: kecepatan listrik rotor dalam electrical RPM.
// Parameter start_current_fraction: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter start_current_erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mc_math_start_current_limit: memulai mc math start current limit setelah prasyarat hardware,
// konfigurasi, dan state keselamatan terpenuhi.
float mc_math_start_current_limit(float current_max,
                                  float erpm_abs,
                                  float start_current_fraction,
                                  float start_current_erpm) {
    // Variabel base: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float base = fmaxf(current_max, 0.0f);
    if (start_current_erpm <= 0.1f || erpm_abs >= start_current_erpm)
        return base;
    // Variabel frac: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float frac = clampf_local(start_current_fraction, 0.0f, 1.0f);
    return map_clamped_local(fabsf(erpm_abs), 0.0f, start_current_erpm, frac * base, base);
}

// Parameter iq: arus sumbu-q FOC yang terutama menghasilkan torsi motor.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter duty_now: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter measured_input_current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter input_current_max: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter input_current_min: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mc_math_limit_input_current: membatasi mc math limit input current ke rentang yang diizinkan agar
// pengendali dan perangkat keras tetap aman.
float mc_math_limit_input_current(float iq,
                                  float erpm,
                                  float duty_now,
                                  float measured_input_current,
                                  float input_current_max,
                                  float input_current_min) {
    if (iq == 0.0f)
        return 0.0f;

    // Variabel in_max: batas atau nilai maksimum untuk validasi dan proteksi.
    const float in_max = fmaxf(input_current_max, 0.0f);
    // Variabel in_min: batas atau nilai minimum untuk validasi dan proteksi.
    const float in_min = fminf(input_current_min, 0.0f);
    // Variabel drawing: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool drawing = (iq * erpm) >= 0.0f;
    // Variabel duty_abs: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_abs = fabsf(duty_now);

    /* First-order inverter power relation: Iin ~= |duty| * |Imotor|.
       The Iq*ERPM sign distinguishes motoring from regenerative operation in
       both forward and reverse directions. */
    if (duty_abs > 0.02f) {
        // Variabel in_mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float in_mag = drawing ? in_max : fabsf(in_min);
        // Variabel phase_mag: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
        const float phase_mag = in_mag / duty_abs;
        if (fabsf(iq) > phase_mag)
            iq = copysignf(phase_mag, iq);
    }

    /* The DC shunt is the final source of truth. If the measured battery
       current already exceeds its configured side of the envelope, reduce the
       requested Iq proportionally without ever changing torque direction. */
    if (drawing) {
        if (in_max <= 0.0f)
            return 0.0f;
        if (measured_input_current > in_max && measured_input_current > 0.1f) {
            iq *= clampf_local(in_max / measured_input_current, 0.0f, 1.0f);
        }
    }
    else {
        if (in_min >= 0.0f)
            return 0.0f;
        if (measured_input_current < in_min && measured_input_current < -0.1f) {
            iq *= clampf_local(fabsf(in_min / measured_input_current), 0.0f, 1.0f);
        }
    }
    return iq;
}


// Parameter i_axis_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter v_axis_prev_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Parameter current_q_base_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter voltage_q_base_v: nilai tegangan untuk pengukuran, kendali, atau proteksi.
// Parameter sample_hz: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter resistance_ohm: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter inductance_h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter valid_samples: penanda bahwa data atau hasil pemeriksaan dinyatakan valid.
// Fungsi mc_math_estimate_inductance_q15: menjalankan operasi mc math estimate inductance q15 sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
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
    if (inductance_h)
        *inductance_h = 0.0f;
    if (current_a)
        *current_a = 0.0f;
    if (valid_samples)
        *valid_samples = 0U;
    if (!i_axis_q15 || !v_axis_prev_q15 || count < 3U ||
        !isfinite(current_q_base_a) || current_q_base_a <= 0.0f ||
        !isfinite(voltage_q_base_v) || voltage_q_base_v <= 0.0f ||
        !isfinite(sample_hz) || sample_hz <= 0.0f ||
        !isfinite(resistance_ohm) || resistance_ohm < 0.0f) {
        return false;
    }

    // Variabel i_scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_scale = current_q_base_a / 32768.0f;
    // Variabel v_scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v_scale = voltage_q_base_v / 32768.0f;
    // Variabel l_sum: akumulator penjumlahan untuk averaging atau statistik.
    float l_sum = 0.0f;
    // Variabel i_sum: akumulator penjumlahan untuk averaging atau statistik.
    float i_sum = 0.0f;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t n = 0U;

    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint16_t k = 1U; k < count; k++) {
        // Variabel i0: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float i0 = (float)i_axis_q15[k - 1U] * i_scale;
        // Variabel i1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float i1 = (float)i_axis_q15[k] * i_scale;
        // Variabel di_dt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float di_dt = (i1 - i0) * sample_hz;
        if (fabsf(di_dt) < 20.0f)
            continue;

        // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float v = (float)v_axis_prev_q15[k] * v_scale;
        // Variabel i_mid: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float i_mid = 0.5f * (i0 + i1);
        // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float l = (v - resistance_ohm * i_mid) / di_dt;
        if (!isfinite(l) || l < 0.2e-6f || l > 0.02f)
            continue;

        l_sum += l;
        i_sum += fabsf(i_mid);
        n++;
    }

    if (valid_samples)
        *valid_samples = n;
    if (n < 3U)
        return false;
    if (inductance_h)
        *inductance_h = l_sum / (float)n;
    if (current_a)
        *current_a = i_sum / (float)n;
    return true;
}
