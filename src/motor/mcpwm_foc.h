#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

/* ========================================================================
 * VESC 6.00 FOC-subset public mcpwm_foc API.
 * The normal control/status signatures retain the upstream names. This F103
 * target intentionally does not provide HFI types/runtime, motor-audio APIs,
 * or mcpwm_foc_tim_sample_int_handler(); ADC/DMA sampling is owned by the
 * platform-specific mcpwm_foc_adc_words_isr() path. Callers must feature-gate
 * those capabilities rather than treating this header as full master parity.
 * ======================================================================== */
// Parameter conf_m1: data konfigurasi yang menentukan perilaku firmware.
// Parameter conf_m2: data konfigurasi yang menentukan perilaku firmware.
// Fungsi mcpwm_foc_init: menginisialisasi mcpwm foc init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2);
// Fungsi mcpwm_foc_deinit: melepas atau menonaktifkan resource mcpwm foc deinit dengan urutan yang aman.
void mcpwm_foc_deinit(void);
// Fungsi mcpwm_foc_init_done: menginisialisasi mcpwm foc init done sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
bool mcpwm_foc_init_done(void);
// Parameter configuration: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mcpwm_foc_set_configuration: mengatur mcpwm foc set configuration setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_configuration(mc_configuration *configuration);
// Fungsi mcpwm_foc_get_state: membaca mcpwm foc get state tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
mc_state mcpwm_foc_get_state(void);
// Fungsi mcpwm_foc_control_mode: menjalankan bagian mcpwm foc control mode pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
mc_control_mode mcpwm_foc_control_mode(void);
// Fungsi mcpwm_foc_is_dccal_done: menjalankan bagian mcpwm foc is dccal done pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
bool mcpwm_foc_is_dccal_done(void);
// Fungsi mcpwm_foc_isr_motor: menangani mcpwm foc isr motor pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
int mcpwm_foc_isr_motor(void);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_stop_pwm: menjalankan bagian mcpwm foc stop pwm pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void mcpwm_foc_stop_pwm(bool is_second_motor);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mcpwm_foc_set_duty: mengatur mcpwm foc set duty setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void mcpwm_foc_set_duty(float dutyCycle);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mcpwm_foc_set_duty_noramp: mengatur mcpwm foc set duty noramp setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_duty_noramp(float dutyCycle);
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mcpwm_foc_set_pid_speed: mengatur mcpwm foc set pid speed setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_speed(float rpm);
// Parameter pos: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi mcpwm_foc_set_pid_pos: mengatur mcpwm foc set pid pos setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_pos(float pos);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_current: mengatur mcpwm foc set current setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void mcpwm_foc_set_current(float current);
// Fungsi mcpwm_foc_release_motor: menjalankan bagian mcpwm foc release motor pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void mcpwm_foc_release_motor(void);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_brake_current: mengatur mcpwm foc set brake current setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_brake_current(float current);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_handbrake: mengatur mcpwm foc set handbrake setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_handbrake(float current);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mcpwm_foc_set_openloop_current: mengatur mcpwm foc set openloop current setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_current(float current, float rpm);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_phase: mengatur mcpwm foc set openloop phase setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_phase(float current, float phase);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mcpwm_foc_set_openloop_duty: mengatur mcpwm foc set openloop duty setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty(float dutyCycle, float rpm);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_duty_phase: mengatur mcpwm foc set openloop duty phase setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty_phase(float dutyCycle, float phase);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_fw_override: mengatur mcpwm foc set fw override setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_fw_override(float current);
// Parameter steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_tachometer_value: mengatur mcpwm foc set tachometer value setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
int mcpwm_foc_set_tachometer_value(int steps);
// Fungsi mcpwm_foc_get_duty_cycle_set: membaca mcpwm foc get duty cycle set tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_set(void);
// Fungsi mcpwm_foc_get_duty_cycle_now: membaca mcpwm foc get duty cycle now tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_now(void);
// Fungsi mcpwm_foc_get_duty_cycle_abs_filter: membaca mcpwm foc get duty cycle abs filter tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_abs_filter(void);
// Fungsi mcpwm_foc_get_pid_speed_set: membaca mcpwm foc get pid speed set tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_speed_set(void);
// Fungsi mcpwm_foc_get_pid_pos_set: membaca mcpwm foc get pid pos set tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_pos_set(void);
// Fungsi mcpwm_foc_get_pid_pos_now: membaca mcpwm foc get pid pos now tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_pos_now(void);
// Fungsi mcpwm_foc_get_switching_frequency_now: membaca mcpwm foc get switching frequency now tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_switching_frequency_now(void);
// Fungsi mcpwm_foc_get_sampling_frequency_now: membaca mcpwm foc get sampling frequency now tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_sampling_frequency_now(void);
// Fungsi mcpwm_foc_get_rpm: membaca mcpwm foc get rpm tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_rpm(void);
// Fungsi mcpwm_foc_get_rpm_fast: membaca mcpwm foc get rpm fast tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_fast(void);
// Fungsi mcpwm_foc_get_rpm_faster: membaca mcpwm foc get rpm faster tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_faster(void);
// Fungsi mcpwm_foc_get_tot_current: membaca mcpwm foc get tot current tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current(void);
// Fungsi mcpwm_foc_get_tot_current_filtered: membaca mcpwm foc get tot current filtered tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_filtered(void);
// Fungsi mcpwm_foc_get_abs_motor_current: membaca mcpwm foc get abs motor current tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current(void);
// Fungsi mcpwm_foc_get_abs_motor_current_unbalance: membaca mcpwm foc get abs motor current unbalance tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_unbalance(void);
// Fungsi mcpwm_foc_get_abs_motor_voltage: membaca mcpwm foc get abs motor voltage tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_voltage(void);
// Fungsi mcpwm_foc_get_abs_motor_current_filtered: membaca mcpwm foc get abs motor current filtered tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_filtered(void);
// Fungsi mcpwm_foc_get_tot_current_directional: membaca mcpwm foc get tot current directional tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_directional(void);
// Fungsi mcpwm_foc_get_tot_current_directional_filtered: membaca mcpwm foc get tot current directional filtered
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_directional_filtered(void);
// Fungsi mcpwm_foc_get_id: membaca mcpwm foc get id tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_id(void);
// Fungsi mcpwm_foc_get_iq: membaca mcpwm foc get iq tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_iq(void);
// Fungsi mcpwm_foc_get_id_set: membaca mcpwm foc get id set tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_set(void);
// Fungsi mcpwm_foc_get_iq_set: membaca mcpwm foc get iq set tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_set(void);
// Fungsi mcpwm_foc_get_id_target: membaca mcpwm foc get id target tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_target(void);
// Fungsi mcpwm_foc_get_iq_target: membaca mcpwm foc get iq target tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_target(void);
// Fungsi mcpwm_foc_get_id_filter: membaca mcpwm foc get id filter tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_filter(void);
// Fungsi mcpwm_foc_get_iq_filter: membaca mcpwm foc get iq filter tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_filter(void);
// Fungsi mcpwm_foc_get_tot_current_in: membaca mcpwm foc get tot current in tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in(void);
// Fungsi mcpwm_foc_get_tot_current_in_filtered: membaca mcpwm foc get tot current in filtered tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_filtered(void);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tachometer_value: membaca mcpwm foc get tachometer value tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
int mcpwm_foc_get_tachometer_value(bool reset);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tachometer_abs_value: membaca mcpwm foc get tachometer abs value tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
int mcpwm_foc_get_tachometer_abs_value(bool reset);
// Fungsi mcpwm_foc_get_phase: membaca mcpwm foc get phase tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float mcpwm_foc_get_phase(void);
// Fungsi mcpwm_foc_get_phase_observer: membaca mcpwm foc get phase observer tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_observer(void);
// Fungsi mcpwm_foc_get_phase_bemf: membaca mcpwm foc get phase bemf tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_bemf(void);
// Fungsi mcpwm_foc_get_phase_encoder: membaca mcpwm foc get phase encoder tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_encoder(void);
// Fungsi mcpwm_foc_get_phase_hall: membaca mcpwm foc get phase hall tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_hall(void);
// Fungsi mcpwm_foc_get_vd: membaca mcpwm foc get vd tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_vd(void);
// Fungsi mcpwm_foc_get_vq: membaca mcpwm foc get vq tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_vq(void);
/* Modulation and voltage getters below expose commanded/reconstructed values
 * derived from duty and Vbus. This board has no independent phase-voltage ADC
 * channels; these are not phase-voltage measurements. */
// Fungsi mcpwm_foc_get_mod_alpha_raw: membaca mcpwm foc get mod alpha raw tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_alpha_raw(void);
// Fungsi mcpwm_foc_get_mod_beta_raw: membaca mcpwm foc get mod beta raw tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_beta_raw(void);
// Fungsi mcpwm_foc_get_mod_alpha_measured: membaca mcpwm foc get mod alpha measured tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_alpha_measured(void);
// Fungsi mcpwm_foc_get_mod_beta_measured: membaca mcpwm foc get mod beta measured tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_beta_measured(void);
// Fungsi mcpwm_foc_get_v_alpha: membaca mcpwm foc get v alpha tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_v_alpha(void);
// Fungsi mcpwm_foc_get_v_beta: membaca mcpwm foc get v beta tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_v_beta(void);
// Fungsi mcpwm_foc_get_est_lambda: membaca mcpwm foc get est lambda tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_lambda(void);
// Fungsi mcpwm_foc_get_est_res: membaca mcpwm foc get est res tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_res(void);
// Fungsi mcpwm_foc_get_est_ind: membaca mcpwm foc get est ind tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_ind(void);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter print: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_encoder_detect: menjalankan deteksi mcpwm foc encoder detect dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
int mcpwm_foc_encoder_detect(float current, bool print, float *offset, float *ratio, bool *inverted);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter stop_after: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter resistance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_resistance: menjalankan bagian mcpwm foc measure resistance pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_resistance(float current, int samples, bool stop_after, float *resistance);
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance: menjalankan bagian mcpwm foc measure inductance pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff, float *inductance);
// Parameter curr_goal: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance_current: menjalankan bagian mcpwm foc measure inductance current pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance_current(float curr_goal, int samples, float *curr, float *ld_lq_diff, float *inductance);
// Parameter res: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ind: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_res_ind: menjalankan bagian mcpwm foc measure res ind pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_res_ind(float *res, float *ind, float *ld_lq_diff);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter hall_table: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Parameter result: hasil sementara atau akhir dari operasi yang sedang dijalankan.
// Fungsi mcpwm_foc_hall_detect: menjalankan deteksi mcpwm foc hall detect dengan proteksi motor dan memvalidasi
// hasil sebelum parameter diterapkan.
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result);
// Parameter cal_undriven: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mcpwm_foc_dc_cal: menangani kalibrasi mcpwm foc dc cal agar offset atau parameter hasil ukur valid
// sebelum dipakai kendali.
int mcpwm_foc_dc_cal(bool cal_undriven);
// Fungsi mcpwm_foc_print_state: menjalankan bagian mcpwm foc print state pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void mcpwm_foc_print_state(void);
// Parameter curr0_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter curr1_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter curr2_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_current_offsets: membaca mcpwm foc get current offsets tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_current_offsets(volatile float *curr0_offset, volatile float *curr1_offset,
                                   volatile float *curr2_offset, bool is_second_motor);
// Parameter curr0_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter curr1_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter curr2_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Fungsi mcpwm_foc_set_current_offsets: mengatur mcpwm foc set current offsets setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_offsets(volatile float curr0_offset, volatile float curr1_offset,
                                   volatile float curr2_offset);
// Parameter v0_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter v1_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter v2_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_voltage_offsets: membaca mcpwm foc get voltage offsets tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_voltage_offsets(float *v0_offset, float *v1_offset, float *v2_offset, bool is_second_motor);
// Parameter v0_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter v1_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter v2_offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_voltage_offsets_undriven: membaca mcpwm foc get voltage offsets undriven tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_voltage_offsets_undriven(float *v0_offset, float *v1_offset, float *v2_offset, bool is_second_motor);
// Parameter ph0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ph1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ph2: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_currents_adc: membaca mcpwm foc get currents adc tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void mcpwm_foc_get_currents_adc(float *ph0, float *ph1, float *ph2, bool is_second_motor);
// Fungsi mcpwm_foc_get_ts: membaca mcpwm foc get ts tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_ts(void);
// Fungsi mcpwm_foc_is_using_encoder: menjalankan bagian mcpwm foc is using encoder pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
bool mcpwm_foc_is_using_encoder(void);
// Parameter x1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter x2: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_observer_state: membaca mcpwm foc get observer state tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
void mcpwm_foc_get_observer_state(float *x1, float *x2);
// Parameter delay_sec: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_set_current_off_delay: mengatur mcpwm foc set current off delay setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_off_delay(float delay_sec);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_tot_current_motor: membaca mcpwm foc get tot current motor tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_motor(bool is_second_motor);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_tot_current_filtered_motor: membaca mcpwm foc get tot current filtered motor tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_filtered_motor(bool is_second_motor);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_tot_current_in_motor: membaca mcpwm foc get tot current in motor tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_tot_current_in_filtered_motor: membaca mcpwm foc get tot current in filtered motor tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_filtered_motor(bool is_second_motor);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_abs_motor_current_motor: membaca mcpwm foc get abs motor current motor tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_motor(bool is_second_motor);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_abs_motor_current_filtered_motor: membaca mcpwm foc get abs motor current filtered motor
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_filtered_motor(bool is_second_motor);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_get_state_motor: membaca mcpwm foc get state motor tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_state mcpwm_foc_get_state_motor(bool is_second_motor);
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter flags: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_adc_int_handler: menangani mcpwm foc adc int handler pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
void mcpwm_foc_adc_int_handler(void *p, uint32_t flags);

/* ========================================================================
 * STM32F103 fixed-point/internal hooks.
 * Board/dual-motor code uses these explicit-runtime helpers; the VESC public
 * API above is a thin selected-motor wrapper around them.
 * ======================================================================== */
// Fungsi mcpwm_foc_init_hw: menginisialisasi mcpwm foc init hw sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void mcpwm_foc_init_hw(void);
// Parameter start_cycle: cycle CPU yang ditangkap pada awal IRQ DMA sebelum pemeriksaan flag.
// Fungsi mcpwm_foc_finish_irq_timing: menutup statistik cycle pada jalur IRQ yang keluar sebelum FOC dijalankan.
void mcpwm_foc_finish_irq_timing(uint32_t start_cycle);
// Parameter adc_words: lima word frame dual-ADC terbaru yang memuat kanal arus sinkron kedua motor.
// Parameter start_cycle: cycle CPU yang ditangkap pada awal IRQ DMA sebelum pemeriksaan flag.
// Fungsi mcpwm_foc_adc_words_isr_timed: menjalankan FOC dengan timestamp IRQ asli agar budget 12000 cycle terukur end-to-end pada update 16k/3.
void mcpwm_foc_adc_words_isr_timed(const volatile uint32_t adc_words[5], uint32_t start_cycle);
// Parameter adc_words: lima word frame dual-ADC terbaru yang memuat kanal arus sinkron kedua motor.
// Fungsi mcpwm_foc_adc_words_isr: entry kompatibilitas untuk test/simulasi yang menangkap timestamp saat dipanggil.
void mcpwm_foc_adc_words_isr(const volatile uint32_t adc_words[5]);
// Parameter adc_words: lima word frame dual-ADC terbaru yang memuat kanal arus sinkron kedua motor.
// Parameter start_cycle: cycle CPU yang ditangkap pada awal IRQ DMA sebelum pemeriksaan flag.
// Fungsi foc_adc_dma_isr_timed: meneruskan sampel DMA beserta timestamp IRQ asli ke jalur FOC.
static inline void foc_adc_dma_isr_timed(const volatile uint32_t adc_words[5], uint32_t start_cycle) {
    mcpwm_foc_adc_words_isr_timed(adc_words, start_cycle);
}
// Parameter adc_words: lima word frame dual-ADC terbaru yang memuat kanal arus sinkron kedua motor.
// Fungsi foc_adc_dma_isr: entry kompatibilitas untuk pemanggil yang tidak menyediakan timestamp IRQ.
static inline void foc_adc_dma_isr(const volatile uint32_t adc_words[5]) {
    mcpwm_foc_adc_words_isr(adc_words);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mcpwm_foc_set_duty_motor: mengatur mcpwm foc set duty motor setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_duty_motor(MotorRuntime *m, float duty);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mcpwm_foc_set_duty_noramp_motor: mengatur mcpwm foc set duty noramp motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_duty_noramp_motor(MotorRuntime *m, float duty);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mcpwm_foc_set_pid_speed_motor: mengatur mcpwm foc set pid speed motor setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_speed_motor(MotorRuntime *m, float erpm);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter pos_deg: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi mcpwm_foc_set_pid_pos_motor: mengatur mcpwm foc set pid pos motor setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_pos_motor(MotorRuntime *m, float pos_deg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_current_motor: mengatur mcpwm foc set current motor setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_motor(MotorRuntime *m, float current);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_brake_current_motor: mengatur mcpwm foc set brake current motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_brake_current_motor(MotorRuntime *m, float current);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_handbrake_motor: mengatur mcpwm foc set handbrake motor setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_handbrake_motor(MotorRuntime *m, float current);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mcpwm_foc_set_openloop_current_motor: mengatur mcpwm foc set openloop current motor setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_current_motor(MotorRuntime *m, float current, float erpm);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter phase_deg: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_phase_motor: mengatur mcpwm foc set openloop phase motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_phase_motor(MotorRuntime *m, float current, float phase_deg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mcpwm_foc_set_openloop_duty_motor: mengatur mcpwm foc set openloop duty motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty_motor(MotorRuntime *m, float duty, float erpm);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter phase_deg: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_duty_phase_motor: mengatur mcpwm foc set openloop duty phase motor setelah
// nilai masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty_phase_motor(MotorRuntime *m, float duty, float phase_deg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_release_motor_motor: menjalankan bagian mcpwm foc release motor motor pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void mcpwm_foc_release_motor_motor(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter delay_s: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_set_current_off_delay_motor: mengatur mcpwm foc set current off delay motor setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_off_delay_motor(MotorRuntime *m, float delay_s);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_rt: membaca mcpwm foc get tot current rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_rt(const MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_observer_rt: membaca mcpwm foc get phase observer rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_observer_rt(const MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_encoder_rt: membaca mcpwm foc get phase encoder rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_encoder_rt(const MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_bemf_rt: membaca mcpwm foc get phase bemf rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_bemf_rt(const MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_hall_rt: membaca mcpwm foc get phase hall rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_hall_rt(const MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter print: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_encoder_detect_motor: menjalankan deteksi mcpwm foc encoder detect motor dengan proteksi
// motor dan memvalidasi hasil sebelum parameter diterapkan.
int mcpwm_foc_encoder_detect_motor(MotorRuntime *m, float current, bool print, float *offset, float *ratio, bool *inverted);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter hall_table: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Parameter result: hasil sementara atau akhir dari operasi yang sedang dijalankan.
// Fungsi mcpwm_foc_hall_detect_motor: menjalankan deteksi mcpwm foc hall detect motor dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
int mcpwm_foc_hall_detect_motor(MotorRuntime *m, float current, uint8_t *hall_table, bool *result);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter stop_after: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter resistance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_resistance_motor: menjalankan bagian mcpwm foc measure resistance motor pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_resistance_motor(MotorRuntime *m, float current, int samples, bool stop_after, float *resistance);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance_motor: menjalankan bagian mcpwm foc measure inductance motor pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance_motor(MotorRuntime *m, float duty, int samples, float *curr, float *ld_lq_diff, float *inductance);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter curr_goal: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance_current_motor: menjalankan bagian mcpwm foc measure inductance current
// motor pada algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance_current_motor(MotorRuntime *m, float curr_goal, int samples, float *curr, float *ld_lq_diff, float *inductance);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter res: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ind: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_res_ind_motor: menjalankan bagian mcpwm foc measure res ind motor pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_res_ind_motor(MotorRuntime *m, float *res, float *ind, float *ld_lq_diff);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter target_erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter erpm_per_sec: kecepatan listrik rotor dalam electrical RPM.
// Parameter flux_wb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_flux_linkage_motor: menjalankan bagian mcpwm foc measure flux linkage motor pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_flux_linkage_motor(MotorRuntime *m, float current_a, float target_erpm, float erpm_per_sec, float *flux_wb);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter target_erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter erpm_per_sec: kecepatan listrik rotor dalam electrical RPM.
// Parameter max_duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter resistance_ohm: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter inductance_h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter flux_wb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_flux_linkage_motor_bounded: menjalankan bagian mcpwm foc measure flux linkage motor
// bounded pada algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_flux_linkage_motor_bounded(MotorRuntime *m, float current_a, float target_erpm, float erpm_per_sec, float max_duty, float resistance_ohm, float inductance_h, float *flux_wb);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_detect_apply_all_motor: menjalankan deteksi mcpwm foc detect apply all motor dengan proteksi
// motor dan memvalidasi hasil sebelum parameter diterapkan.
int16_t mcpwm_foc_detect_apply_all_motor(MotorRuntime *m, float current_a);

/* Current calibration/debug API owned by this F103 port. */
typedef enum {
    FOC_CAL_STAGE_UNDRIVEN = 0, FOC_CAL_STAGE_WAIT_LEFT_DRIVEN,
    FOC_CAL_STAGE_LEFT_WARMUP, FOC_CAL_STAGE_LEFT_DRIVEN,
    FOC_CAL_STAGE_WAIT_RIGHT_DRIVEN, FOC_CAL_STAGE_RIGHT_WARMUP,
    FOC_CAL_STAGE_RIGHT_DRIVEN, FOC_CAL_STAGE_WAIT_FINALIZE,
    FOC_CAL_STAGE_DONE, FOC_CAL_STAGE_FAILED
} foc_cal_stage_t;
typedef struct {
    // Variabel cal_stage: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel fault: status atau data gangguan untuk sistem proteksi.
    // Variabel motor: state atau parameter motor yang sedang diproses.
    // Variabel valid: penanda validitas hasil pengukuran atau konfigurasi.
    uint8_t valid, motor, fault, cal_stage;
    // Variabel raw_dc: nilai mentah sebelum konversi ke satuan fisik.
    // Variabel raw_u: nilai mentah sebelum konversi ke satuan fisik.
    // Variabel raw_v: nilai mentah sebelum konversi ke satuan fisik.
    uint16_t raw_u, raw_v, raw_dc;
    // Variabel ia_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ib_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ic_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel id_target_q15: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    // Variabel iq_target_q15: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    // Variabel offset_dc: offset kalibrasi untuk mengoreksi bias pengukuran.
    // Variabel offset_u: offset kalibrasi untuk mengoreksi bias pengukuran.
    // Variabel offset_v: offset kalibrasi untuk mengoreksi bias pengukuran.
    // Variabel trip_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t offset_u, offset_v, offset_dc, ia_q15, ib_q15, ic_q15, trip_q15, id_target_q15, iq_target_q15;
    // Variabel ccr1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ccr2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ccr3: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel dma_cndtr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel tim_cnt: pencacah kejadian atau sampel.
    uint16_t ccr1, ccr2, ccr3, tim_cnt, dma_cndtr;
    // Variabel adc_isr_count: nilai atau state ADC pada jalur pengukuran.
    uint32_t adc_isr_count;
    // Variabel blank_cycles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t blank_cycles;
    // Variabel moe: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel pending_events: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel pwm_enabled: state atau nilai PWM untuk pengendalian inverter.
    // Variabel reserved: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t pwm_enabled, moe, pending_events, reserved;
} foc_fault_snapshot_t;
typedef struct {
    // Variabel mean: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t mean;
    // Variabel max: batas atau nilai maksimum untuk validasi dan proteksi.
    // Variabel min: batas atau nilai minimum untuk validasi dan proteksi.
    uint16_t min, max;
    // Variabel variance_x100: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t variance_x100;
} foc_cal_channel_diag_t;
typedef struct {
    // Variabel reserved: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel stage: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t stage, reserved;
    // Variabel fail_noise_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel fail_range_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel shift_warn_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel warn_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t shift_warn_mask, warn_mask, fail_range_mask, fail_noise_mask;
    // Variabel ch: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_cal_channel_diag_t ch[6];
    // Variabel driven_mean: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel undriven_mean: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t undriven_mean[6], driven_mean[6];
    // Variabel outlier_count: pencacah kejadian atau sampel.
    uint16_t outlier_count[6];
    // Variabel moe_confirmed_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel moe_fail_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t moe_fail_mask, moe_confirmed_mask;
    // Variabel first_sample_adc: nilai atau state ADC pada jalur pengukuran.
    // Variabel moe_confirm_adc: nilai atau state ADC pada jalur pengukuran.
    // Variabel moe_request_adc: nilai atau state ADC pada jalur pengukuran.
    uint32_t moe_request_adc[2], moe_confirm_adc[2], first_sample_adc[2];
    // Variabel moe_drop_adc: nilai atau state ADC pada jalur pengukuran.
    uint32_t moe_drop_adc[2];
    // Variabel moe_drop_bdtr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel moe_drop_pending: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel moe_drop_pwm_enabled: state atau nilai PWM untuk pengendalian inverter.
    uint8_t moe_drop_bdtr[2], moe_drop_pending[2], moe_drop_pwm_enabled[2];
} foc_cal_diag_t;
// Fungsi foc_calibration_done: menangani kalibrasi foc calibration done agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
bool foc_calibration_done(void);
// Fungsi foc_calibration_valid: menangani kalibrasi foc calibration valid agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
bool foc_calibration_valid(void);
// Fungsi foc_calibration_in_progress: menangani kalibrasi foc calibration in progress agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
bool foc_calibration_in_progress(void);
// Fungsi foc_calibration_stage: menangani kalibrasi foc calibration stage agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
foc_cal_stage_t foc_calibration_stage(void);
// Fungsi foc_calibration_service_task: menangani kalibrasi foc calibration service task agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
void foc_calibration_service_task(void);
/* When skip is true, the boot offset-calibration pipeline is bypassed and the
 * bridge is treated as already calibrated (stored/gross-default offsets). */
// Parameter skip: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_calibration_set_skip: mengatur foc calibration set skip setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void foc_calibration_set_skip(bool skip);
// Fungsi foc_request_recalibration: menjalankan bagian foc request recalibration pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_request_recalibration(void);
// Fungsi foc_adc_isr_count: menangani foc adc isr count pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
uint32_t foc_adc_isr_count(void);
// Fungsi foc_isr_total_max_cycles: menangani foc isr total max cycles pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_total_max_cycles(void);
// Fungsi foc_last_isr_duration_s: menangani foc last isr duration s pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
float foc_last_isr_duration_s(void);
// Fungsi foc_isr_near_deadline_count: menangani foc isr near deadline count pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_near_deadline_count(void);
// Fungsi foc_isr_period_min_cycles: menangani foc isr period min cycles pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_period_min_cycles(void);
// Fungsi foc_isr_period_max_cycles: menangani foc isr period max cycles pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_period_max_cycles(void);
// Fungsi foc_vbus_dma_stale_events: menjalankan bagian foc vbus dma stale events pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
uint32_t foc_vbus_dma_stale_events(void);
// Fungsi foc_vbus_dma_stale_count: menjalankan bagian foc vbus dma stale count pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
uint8_t foc_vbus_dma_stale_count(void);
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Parameter target: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_get_calibration_progress: membaca foc get calibration progress tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
void foc_get_calibration_progress(uint32_t *count, uint32_t *target);
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_get_calibration_diag: membaca foc get calibration diag tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void foc_get_calibration_diag(foc_cal_diag_t *out);
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_get_fault_snapshot: membaca foc get fault snapshot tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void foc_get_fault_snapshot(foc_fault_snapshot_t *out);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_sensor_electrical_phase_u16: menjalankan operasi motor sensor electrical phase u16 sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hall_edge_isr: menangani motor hall edge isr pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
void motor_hall_edge_isr(MotorRuntime *m);
