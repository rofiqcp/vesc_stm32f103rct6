#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct MotorRuntime MotorRuntime;
typedef struct mc_configuration mc_configuration;

#define FOC_Q15_ONE               32768
#define FOC_Q15_HALF              16384
#define FOC_Q15_INV_SQRT3         18919   /* 1/sqrt(3) */
#define FOC_Q15_SQRT3_BY_2        28378   /* sqrt(3)/2 */

// Fungsi foc_math_init: menginisialisasi foc math init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void foc_math_init(void);
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter s: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_fast_sincos_u16_q15: menjalankan bagian foc fast sincos u16 q15 pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_fast_sincos_u16_q15(uint16_t phase, int32_t *s, int32_t *c);
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_q15_mul: menjalankan bagian foc q15 mul pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
int32_t foc_q15_mul(int32_t a, int32_t b);
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_q15_clamp: menjalankan bagian foc q15 clamp pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
int32_t foc_q15_clamp(int32_t x, int32_t lo, int32_t hi);
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inv_vbus_q30: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter d_u_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_v_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_w_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_svm_q15: menjalankan bagian foc svm q15 pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
void foc_svm_q15(int32_t v_alpha_q15, int32_t v_beta_q15,
                 int32_t inv_vbus_q30,
                 uint16_t *d_u_q15, uint16_t *d_v_q15, uint16_t *d_w_q15);
/* Reconstruct the alpha/beta voltage that the centered PWM duties actually
 * apply. Common-mode offset cancels in Clarke, so this also captures any
 * vector-preserving SVM scaling used to keep the hoverboard sampling window. */
// Parameter d_u_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_v_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter d_w_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter vbus_q15: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_pwm_applied_voltage_q15: menjalankan bagian foc pwm applied voltage q15 pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_pwm_applied_voltage_q15(uint16_t d_u_q15, uint16_t d_v_q15,
                                 uint16_t d_w_q15, int32_t vbus_q15,
                                 int32_t *v_alpha_q15, int32_t *v_beta_q15);
/* Correct an applied alpha/beta voltage model for PWM dead-time using the
 * signs of the three phase currents. deadtime_comp_q15 is foc_dt_us*foc_f_zv. */
// Parameter ia_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ib_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ic_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter vbus_q15: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter deadtime_comp_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_deadtime_compensate_voltage_q15: menjalankan bagian foc deadtime compensate voltage q15 pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_deadtime_compensate_voltage_q15(int32_t ia_q15, int32_t ib_q15,
                                         int32_t ic_q15, int32_t vbus_q15,
                                         int32_t deadtime_comp_q15,
                                         int32_t *v_alpha_q15,
                                         int32_t *v_beta_q15);

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_clampf: menjalankan bagian foc clampf pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
float foc_clampf(float x, float lo, float hi);
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_deg_to_u16: menjalankan bagian foc deg to u16 pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
uint16_t foc_deg_to_u16(float deg);
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_wrap_deg: menjalankan bagian foc wrap deg pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
float foc_wrap_deg(float deg);


/* VESC-style observer/PLL and FOC_SENSOR_MODE_ENCODER_AB runtime support.
 * Implemented in foc_math.c so the file ownership follows upstream VESC. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_observer_precalc: menjalankan bagian foc observer precalc pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_observer_precalc(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter phase_u16: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi foc_observer_reset: menjalankan bagian foc observer reset pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_observer_reset(MotorRuntime *m, uint16_t phase_u16);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter i_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_observer_update_fixed: menjalankan bagian foc observer update fixed pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_observer_update_fixed(MotorRuntime *m, int32_t v_alpha_q15, int32_t v_beta_q15,
                         int32_t i_alpha_q15, int32_t i_beta_q15);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter phase_u16: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi foc_pll_run_fixed: menjalankan bagian foc pll run fixed pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_pll_run_fixed(MotorRuntime *m, uint16_t phase_u16);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_observer_update_1khz: menjalankan bagian foc observer update 1khz pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_observer_update_1khz(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_encoder_ab_startup_1khz: menjalankan bagian foc encoder ab startup 1khz pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
bool foc_encoder_ab_startup_1khz(MotorRuntime *m, uint32_t now_ms);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter direction_hint: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter iq_hint_a: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi foc_sensorless_startup_1khz: menjalankan bagian foc sensorless startup 1khz pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
bool foc_sensorless_startup_1khz(MotorRuntime *m, uint32_t now_ms,
                                  float direction_hint, float iq_hint_a);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_sensorless_startup_abort: menjalankan bagian foc sensorless startup abort pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_sensorless_startup_abort(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_encoder_ab_sync_from_observer: menjalankan bagian foc encoder ab sync from observer pada algoritma
// FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_encoder_ab_sync_from_observer(MotorRuntime *m);


/* Current VESC foc_math public surface. motor_all_state_t maps to the compact
 * F103 MotorRuntime; hard-loop arithmetic remains fixed-point. HFI types and
 * HFI angle-adjust runtime is intentionally omitted by build requirement. */
typedef MotorRuntime motor_all_state_t;
typedef struct {
    // Variabel x1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float x1;
    // Variabel x2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float x2;
    // Variabel lambda_est: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lambda_est;
    // Variabel i_alpha_last: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float i_alpha_last;
    // Variabel i_beta_last: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float i_beta_last;
} observer_state;

// Parameter v_alpha: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter v_beta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i_alpha: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i_beta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_observer_update: menjalankan bagian foc observer update pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
                         float dt, observer_state *state, float *phase,
                         motor_all_state_t *motor);
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter phase_var: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter speed_var: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi foc_pll_run: menjalankan bagian foc pll run pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
void foc_pll_run(float phase, float dt, float *phase_var, float *speed_var,
                 mc_configuration *conf);
// Parameter alpha: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter beta: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter max_mod: batas atau nilai maksimum untuk validasi dan proteksi.
// Parameter PWMFullDutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter tAout: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter tBout: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter tCout: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter svm_sector: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_svm: menjalankan bagian foc svm pada algoritma FOC dengan skala, konvensi tanda, dan batas numerik
// yang konsisten.
void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle,
             uint32_t *tAout, uint32_t *tBout, uint32_t *tCout,
             uint32_t *svm_sector);
// Parameter index_found: indeks elemen yang sedang diproses.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_run_pid_control_pos: menjalankan bagian foc run pid control pos pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void foc_run_pid_control_pos(bool index_found, float dt, motor_all_state_t *motor);
// Parameter index_found: indeks elemen yang sedang diproses.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_run_pid_control_speed: menjalankan bagian foc run pid control speed pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_run_pid_control_speed(bool index_found, float dt, motor_all_state_t *motor);
// Parameter obs_angle: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter enc_angle: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter speed: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Parameter sl_erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_correct_encoder: menjalankan bagian foc correct encoder pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
float foc_correct_encoder(float obs_angle, float enc_angle, float speed,
                          float sl_erpm, motor_all_state_t *motor);
// Parameter angle: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter hall_val: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Fungsi foc_correct_hall: menjalankan bagian foc correct hall pada algoritma FOC dengan skala, konvensi tanda,
// dan batas numerik yang konsisten.
float foc_correct_hall(float angle, float dt, motor_all_state_t *motor, int hall_val);
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Fungsi foc_run_fw: menjalankan bagian foc run fw pada algoritma FOC dengan skala, konvensi tanda, dan batas
// numerik yang konsisten.
void foc_run_fw(motor_all_state_t *motor, float dt);
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_precalc_values: menjalankan bagian foc precalc values pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void foc_precalc_values(motor_all_state_t *motor);
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi foc_update_modulation_limit: menjalankan bagian foc update modulation limit pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_update_modulation_limit(motor_all_state_t *motor);
