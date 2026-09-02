#ifndef MC_MATH_H
#define MC_MATH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Task-side motor limit helpers. These functions are intentionally floating
 * point because they are never called from the 16-kHz hard FOC ISR. */
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
                                    float cut_end);
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
                                          float cut_end);
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
                                  float input_current_min);

/* VESC-style 1-kHz thermal/startup limit helpers. These stay outside the
 * hard FOC ISR so the Cortex-M3 fast path remains fixed-point. */
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
                                    float temp_end_c);
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
                                  float temp_accel_dec);
// Parameter current_max: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter erpm_abs: kecepatan listrik rotor dalam electrical RPM.
// Parameter start_current_fraction: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter start_current_erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mc_math_start_current_limit: memulai mc math start current limit setelah prasyarat hardware,
// konfigurasi, dan state keselamatan terpenuhi.
float mc_math_start_current_limit(float current_max,
                                  float erpm_abs,
                                  float start_current_fraction,
                                  float start_current_erpm);

/* Estimate one dq-axis inductance from a PWM-rate current step.
 * v_axis_prev_q15[k] is the selected-axis voltage applied during the interval
 * that produced i_axis_q15[k]-i_axis_q15[k-1]. This runs only in the blocking
 * detect task and is shared by the B8 Ld and Lq measurements. */
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
                                     uint16_t *valid_samples);

#ifdef __cplusplus
}
#endif

#endif
