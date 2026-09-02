#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "datatypes.h"

// Variabel g_motor_left: state atau parameter motor yang sedang diproses.
extern MotorRuntime g_motor_left;
// Variabel g_motor_right: state atau parameter motor yang sedang diproses.
extern MotorRuntime g_motor_right;

// Fungsi motor_control_init: menginisialisasi motor control init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void motor_control_init(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi motor_get: membaca motor get tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
MotorRuntime *motor_get(motor_id_t id);
// Parameter m: motor yang menerima tabel Hall hasil konfigurasi atau deteksi.
// Parameter table: tabel Hall VESC 0..200, dengan 255 menandai state invalid.
// Fungsi motor_apply_foc_hall_table: menyatukan tabel Hall VESC dengan lookup sudut dan urutan sektor ISR.
bool motor_apply_foc_hall_table(MotorRuntime *m, const uint8_t table[8]);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter amp: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_current: mengatur motor set current setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void motor_set_current(MotorRuntime *m, float amp);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter amp: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_brake_current: mengatur motor set brake current setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_set_brake_current(MotorRuntime *m, float amp);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter amp: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_handbrake: mengatur motor set handbrake setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void motor_set_handbrake(MotorRuntime *m, float amp);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter rel: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_current_rel: mengatur motor set current rel setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_set_current_rel(MotorRuntime *m, float rel);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi motor_set_speed: mengatur motor set speed setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void motor_set_speed(MotorRuntime *m, float erpm);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_set_position: mengatur motor set position setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void motor_set_position(MotorRuntime *m, float deg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi motor_set_duty: mengatur motor set duty setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void motor_set_duty(MotorRuntime *m, float duty);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_stop: menghentikan motor stop dengan menonaktifkan output atau state terkait secara aman.
void motor_stop(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_clear_fault: menangani motor clear fault dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
void motor_clear_fault(MotorRuntime *m);
/* Force-clear a fault for a safe stopped recalibration. Hardware-latched
 * power-stage faults (PVD/BKIN/break) still refuse reset; a config-flash fault
 * is cleared so offset calibration can arm the 50% zero-vector MOE handshake. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_clear_fault_for_cal: menangani kalibrasi motor clear fault for cal agar offset atau parameter
// hasil ukur valid sebelum dipakai kendali.
void motor_clear_fault_for_cal(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_touch_command: menjalankan operasi motor touch command sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void motor_touch_command(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_keepalive: menjalankan operasi motor keepalive sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void motor_keepalive(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi motor_slow_update_1khz: memperbarui motor slow update 1khz menggunakan state terbaru dengan urutan
// yang konsisten dan aman.
void motor_slow_update_1khz(MotorRuntime *m, uint32_t now_ms);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_rpm_update_1khz: memperbarui motor rpm update 1khz menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void motor_rpm_update_1khz(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_pid_update_1khz: memperbarui motor pid update 1khz menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void motor_pid_update_1khz(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi motor_raise_fault_from_task: menangani motor raise fault from task dengan memprioritaskan pemadaman
// keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
void motor_raise_fault_from_task(MotorRuntime *m, motor_fault_t fault);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi motor_request_fault_from_isr: menangani motor request fault from isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void motor_request_fault_from_isr(MotorRuntime *m, motor_fault_t fault);
// Fungsi motor_take_pending_fault_mask: menangani motor take pending fault mask dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
uint32_t motor_take_pending_fault_mask(void);

/* Board-internal fault <-> VESC wire/API mapping. Never expose motor_fault_t
 * numerically on the VESC protocol; its values are intentionally local. */
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi motor_fault_to_vesc: menangani motor fault to vesc dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
mc_fault_code motor_fault_to_vesc(motor_fault_t fault);
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi motor_fault_from_vesc: menangani motor fault from vesc dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
motor_fault_t motor_fault_from_vesc(mc_fault_code fault);

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi motor_select_sensor_mode: menjalankan operasi motor select sensor mode sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool motor_select_sensor_mode(MotorRuntime *m, uint8_t mode);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id_a: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Parameter iq_a: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi motor_set_foc_targets: mengatur motor set foc targets setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_set_foc_targets(MotorRuntime *m, float id_a, float iq_a);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter kp: penguatan proporsional regulator.
// Parameter ki: penguatan integral regulator.
// Fungsi motor_set_current_pi_gains: mengatur motor set current pi gains setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_set_current_pi_gains(MotorRuntime *m, float kp, float ki);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_encoder_extended_count: menjalankan operasi motor encoder extended count sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
int32_t motor_encoder_extended_count(MotorRuntime *m);

/* ========================================================================
 * VESC 6.00 FOC-subset command/status mc_interface API.
 * The upstream-shaped mc_interface_* wrappers provide VESC Tool compatibility.
 * The explicit MotorRuntime helpers above and the F103-only mc_interface_*
 * additions (resource stats, per-motor odometer, input-gate helpers,
 * mc_interface_motor_runtime_now bridge) are port extensions, not VESC public
 * API guarantees. mc_interface_adc_inj_int_handler() is intentionally absent;
 * this target uses the platform ADC/DMA ISR instead.
 * ======================================================================== */
// Parameter reset_conf: data konfigurasi yang menentukan perilaku firmware.
// Fungsi mc_interface_init: menginisialisasi mc interface init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void mc_interface_init(bool reset_conf);
// Fungsi mc_interface_start_threads: memulai mc interface start threads setelah prasyarat hardware,
// konfigurasi, dan state keselamatan terpenuhi.
bool mc_interface_start_threads(void);
// Parameter timer: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sample: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi mc_interface_set_thread_ids: mengatur mc interface set thread ids setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_thread_ids(TaskHandle_t timer, TaskHandle_t sample, TaskHandle_t fault);

typedef struct {
    // Variabel heap_free_bytes: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t heap_free_bytes;
    // Variabel heap_min_ever_bytes: batas atau nilai minimum untuk validasi dan proteksi.
    uint32_t heap_min_ever_bytes;
    // Variabel motor_service_stack_free_bytes: ruang stack atau informasi pemakaian stack task.
    uint32_t motor_service_stack_free_bytes;
    // Variabel sample_sender_stack_free_bytes: ruang stack atau informasi pemakaian stack task.
    uint32_t sample_sender_stack_free_bytes;
    // Variabel fault_stack_free_bytes: status atau data gangguan untuk sistem proteksi.
    uint32_t fault_stack_free_bytes;
    // Variabel status_stack_free_bytes: status runtime untuk diagnostik atau keputusan kendali.
    uint32_t status_stack_free_bytes;
} mc_interface_resource_stats_t;

// Fungsi mc_interface_free_heap_bytes: menjalankan operasi mc interface free heap bytes sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
uint32_t mc_interface_free_heap_bytes(void);
// Fungsi mc_interface_min_ever_free_heap_bytes: menjalankan operasi mc interface min ever free heap bytes
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t mc_interface_min_ever_free_heap_bytes(void);
// Parameter stats: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_resource_stats: membaca mc interface get resource stats tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
void mc_interface_get_resource_stats(mc_interface_resource_stats_t *stats);

// Fungsi mc_interface_motor_now: menjalankan operasi mc interface motor now sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
int mc_interface_motor_now(void);
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Fungsi mc_interface_select_motor_thread: menjalankan operasi mc interface select motor thread sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_select_motor_thread(int motor);
// Fungsi mc_interface_get_motor_thread: membaca mc interface get motor thread tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
int mc_interface_get_motor_thread(void);
// Fungsi mc_interface_motor_runtime_now: menjalankan operasi mc interface motor runtime now sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
MotorRuntime *mc_interface_motor_runtime_now(void); /* F103 internal bridge */
// Fungsi mc_interface_get_configuration: membaca mc interface get configuration tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
const volatile mc_configuration* mc_interface_get_configuration(void);
// Parameter configuration: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mc_interface_set_configuration: mengatur mc interface set configuration setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_configuration(mc_configuration *configuration);
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter is_motor_2: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mc_interface_calc_crc: menangani kalibrasi mc interface calc crc agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
unsigned mc_interface_calc_crc(mc_configuration* conf, bool is_motor_2);
// Fungsi mc_interface_dccal_done: menjalankan operasi mc interface dccal done sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_dccal_done(void);
// Parameter p_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_set_pwm_callback: mengatur mc interface set pwm callback setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_pwm_callback(void (*p_func)(void));
// Fungsi mc_interface_lock: menjalankan operasi mc interface lock sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void mc_interface_lock(void);
// Fungsi mc_interface_unlock: menjalankan operasi mc interface unlock sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void mc_interface_unlock(void);
// Fungsi mc_interface_lock_override_once: menjalankan operasi mc interface lock override once sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_lock_override_once(void);
// Fungsi mc_interface_get_fault: membaca mc interface get fault tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_fault_code mc_interface_get_fault(void);
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi mc_interface_fault_to_string: menangani mc interface fault to string dengan memprioritaskan pemadaman
// keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
const char* mc_interface_fault_to_string(mc_fault_code fault);
// Fungsi mc_interface_get_state: membaca mc interface get state tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_state mc_interface_get_state(void);
// Fungsi mc_interface_get_control_mode: membaca mc interface get control mode tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
mc_control_mode mc_interface_get_control_mode(void);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mc_interface_set_duty: mengatur mc interface set duty setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void mc_interface_set_duty(float dutyCycle);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mc_interface_set_duty_noramp: mengatur mc interface set duty noramp setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_duty_noramp(float dutyCycle);
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mc_interface_set_pid_speed: mengatur mc interface set pid speed setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_pid_speed(float rpm);
// Parameter pos: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi mc_interface_set_pid_pos: mengatur mc interface set pid pos setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_pid_pos(float pos);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mc_interface_set_current: mengatur mc interface set current setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_current(float current);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mc_interface_set_brake_current: mengatur mc interface set brake current setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_brake_current(float current);
// Parameter val: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_current_rel: mengatur mc interface set current rel setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_current_rel(float val);
// Parameter val: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_brake_current_rel: mengatur mc interface set brake current rel setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_brake_current_rel(float val);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mc_interface_set_handbrake: mengatur mc interface set handbrake setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_handbrake(float current);
// Parameter val: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_handbrake_rel: mengatur mc interface set handbrake rel setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_handbrake_rel(float val);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mc_interface_set_openloop_current: mengatur mc interface set openloop current setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_current(float current, float rpm);
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mc_interface_set_openloop_phase: mengatur mc interface set openloop phase setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_phase(float current, float phase);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mc_interface_set_openloop_duty: mengatur mc interface set openloop duty setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_duty(float dutyCycle, float rpm);
// Parameter dutyCycle: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mc_interface_set_openloop_duty_phase: mengatur mc interface set openloop duty phase setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_openloop_duty_phase(float dutyCycle, float phase);
// Parameter steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_tachometer_value: mengatur mc interface set tachometer value setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
int mc_interface_set_tachometer_value(int steps);
// Fungsi mc_interface_brake_now: menjalankan operasi mc interface brake now sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void mc_interface_brake_now(void);
// Fungsi mc_interface_release_motor: menjalankan operasi mc interface release motor sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void mc_interface_release_motor(void);
// Fungsi mc_interface_release_motor_override: menjalankan operasi mc interface release motor override sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_release_motor_override(void);
// Parameter timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi mc_interface_wait_for_motor_release: menjalankan operasi mc interface wait for motor release sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_wait_for_motor_release(float timeout);
// Fungsi mc_interface_get_duty_cycle_set: membaca mc interface get duty cycle set tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_duty_cycle_set(void);
// Fungsi mc_interface_get_duty_cycle_now: membaca mc interface get duty cycle now tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_duty_cycle_now(void);
// Fungsi mc_interface_get_sampling_frequency_now: membaca mc interface get sampling frequency now tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_sampling_frequency_now(void);
// Fungsi mc_interface_get_rpm: membaca mc interface get rpm tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_rpm(void);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_amp_hours: membaca mc interface get amp hours tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_amp_hours(bool reset);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_amp_hours_charged: membaca mc interface get amp hours charged tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_amp_hours_charged(bool reset);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_watt_hours: membaca mc interface get watt hours tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_watt_hours(bool reset);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_watt_hours_charged: membaca mc interface get watt hours charged tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_watt_hours_charged(bool reset);
// Fungsi mc_interface_get_tot_current: membaca mc interface get tot current tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current(void);
// Fungsi mc_interface_get_tot_current_filtered: membaca mc interface get tot current filtered tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_filtered(void);
// Fungsi mc_interface_get_tot_current_directional: membaca mc interface get tot current directional tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_directional(void);
// Fungsi mc_interface_get_tot_current_directional_filtered: membaca mc interface get tot current directional
// filtered tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_directional_filtered(void);
// Fungsi mc_interface_get_tot_current_in: membaca mc interface get tot current in tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_in(void);
// Fungsi mc_interface_get_tot_current_in_filtered: membaca mc interface get tot current in filtered tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_tot_current_in_filtered(void);
// Fungsi mc_interface_get_input_voltage_filtered: membaca mc interface get input voltage filtered tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_input_voltage_filtered(void);
// Fungsi mc_interface_get_abs_motor_current_unbalance: membaca mc interface get abs motor current unbalance
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mc_interface_get_abs_motor_current_unbalance(void);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_tachometer_value: membaca mc interface get tachometer value tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
int mc_interface_get_tachometer_value(bool reset);
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_get_tachometer_abs_value: membaca mc interface get tachometer abs value tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
int mc_interface_get_tachometer_abs_value(bool reset);
// Fungsi mc_interface_get_last_inj_adc_isr_duration: menangani mc interface get last inj adc isr duration pada
// konteks interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
float mc_interface_get_last_inj_adc_isr_duration(void);
// Fungsi mc_interface_read_reset_avg_motor_current: mereset mc interface read reset avg motor current ke
// kondisi awal yang aman tanpa meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_motor_current(void);
// Fungsi mc_interface_read_reset_avg_input_current: mereset mc interface read reset avg input current ke
// kondisi awal yang aman tanpa meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_input_current(void);
// Fungsi mc_interface_read_reset_avg_id: mereset mc interface read reset avg id ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_id(void);
// Fungsi mc_interface_read_reset_avg_iq: mereset mc interface read reset avg iq ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_iq(void);
// Fungsi mc_interface_read_reset_avg_vd: mereset mc interface read reset avg vd ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_vd(void);
// Fungsi mc_interface_read_reset_avg_vq: mereset mc interface read reset avg vq ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
float mc_interface_read_reset_avg_vq(void);
// Fungsi mc_interface_get_pid_pos_set: membaca mc interface get pid pos set tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_pid_pos_set(void);
// Fungsi mc_interface_get_pid_pos_now: membaca mc interface get pid pos now tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mc_interface_get_pid_pos_now(void);
// Parameter angle_now: nilai sudut untuk posisi rotor atau transformasi koordinat.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_update_pid_pos_offset: memperbarui mc interface update pid pos offset menggunakan state
// terbaru dengan urutan yang konsisten dan aman.
void mc_interface_update_pid_pos_offset(float angle_now, bool store);
// Fungsi mc_interface_get_last_sample_adc_isr_duration: menangani mc interface get last sample adc isr duration
// pada konteks interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
float mc_interface_get_last_sample_adc_isr_duration(void);
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter reply_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_sample_print_data: menjalankan operasi mc interface sample print data sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len, uint8_t decimation,
        bool raw, void (*reply_func)(unsigned char *data, unsigned int len));
/* The initiating VESC transport for the asynchronous debug-sample sender.
 * This preserves upstream reply_func routing instead of silently falling back
 * to the board's default UART queue. */
// Fungsi mc_interface_sample_reply_func: menyusun atau mengirim mc interface sample reply func dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
void (*mc_interface_sample_reply_func(void))(unsigned char *data, unsigned int len);

/* --- Debug sample buffer (VESC-standard mc_interface ownership) --- */
// Fungsi mc_interface_sample_init: menginisialisasi mc interface sample init sehingga resource, konfigurasi
// awal, dan state modul siap digunakan dengan aman.
void mc_interface_sample_init(void);
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi mc_interface_sample_control: menjalankan operasi mc interface sample control sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_control(debug_sampling_mode mode, motor_id_t motor,
        uint16_t len, uint16_t decimation, bool raw);
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_sample_start: memulai mc interface sample start setelah prasyarat hardware, konfigurasi,
// dan state keselamatan terpenuhi.
void mc_interface_sample_start(motor_id_t motor, uint16_t len,
        uint16_t decimation);
// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter decimation: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi mc_interface_sample_start_ex: memulai mc interface sample start ex setelah prasyarat hardware,
// konfigurasi, dan state keselamatan terpenuhi.
void mc_interface_sample_start_ex(motor_id_t motor, uint16_t len,
        uint16_t decimation, bool raw);
// Parameter active: penanda bahwa state atau fitur sedang aktif.
// Fungsi mc_interface_sample_capture_isr: menangani mc interface sample capture isr pada konteks interrupt
// dengan pekerjaan minimum agar timing FOC tetap deterministik.
void mc_interface_sample_capture_isr(MotorRuntime *active);
// Fungsi mc_interface_sample_ready: menjalankan operasi mc interface sample ready sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_ready(void);
// Fungsi mc_interface_sample_has_capture: menjalankan operasi mc interface sample has capture sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_has_capture(void);
// Fungsi mc_interface_sample_count: menjalankan operasi mc interface sample count sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
uint16_t mc_interface_sample_count(void);
// Fungsi mc_interface_sample_data: menjalankan operasi mc interface sample data sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
const debug_sample_t *mc_interface_sample_data(void);
// Parameter logical_index: indeks elemen yang sedang diproses.
// Fungsi mc_interface_sample_at: menjalankan operasi mc interface sample at sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
const debug_sample_t *mc_interface_sample_at(uint16_t logical_index);
// Fungsi mc_interface_sample_mark_sent: menjalankan operasi mc interface sample mark sent sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_sample_mark_sent(void);
// Fungsi mc_interface_sample_active: menjalankan operasi mc interface sample active sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_active(void);
// Fungsi mc_interface_sample_raw: menjalankan operasi mc interface sample raw sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool mc_interface_sample_raw(void);
// Fungsi mc_interface_sample_mode: menjalankan operasi mc interface sample mode sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
debug_sampling_mode mc_interface_sample_mode(void);
// Fungsi mc_interface_temp_fet_filtered: menjalankan operasi mc interface temp fet filtered sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_temp_fet_filtered(void);
// Fungsi mc_interface_temp_motor_filtered: menjalankan operasi mc interface temp motor filtered sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_temp_motor_filtered(void);
// Parameter wh_left: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_get_battery_level: membaca mc interface get battery level tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_battery_level(float *wh_left);
// Fungsi mc_interface_get_speed: membaca mc interface get speed tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_speed(void);
// Fungsi mc_interface_get_distance: membaca mc interface get distance tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mc_interface_get_distance(void);
// Fungsi mc_interface_get_distance_abs: membaca mc interface get distance abs tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mc_interface_get_distance_abs(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter abs_tach_steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mc_interface_odometer_add_tach_delta: menjalankan operasi mc interface odometer add tach delta sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_odometer_add_tach_delta(motor_id_t id, uint32_t abs_tach_steps);
// Parameter ovr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter speed: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Fungsi mc_interface_override_wheel_speed: menjalankan operasi mc interface override wheel speed sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_override_wheel_speed(bool ovr, float speed);
// Fungsi mc_interface_get_setup_values: membaca mc interface get setup values tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
setup_values mc_interface_get_setup_values(void);
// Fungsi mc_interface_gnss: menjalankan operasi mc interface gnss sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
volatile gnss_data *mc_interface_gnss(void);
// Fungsi mc_interface_get_odometer: membaca mc interface get odometer tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
uint64_t mc_interface_get_odometer(void);
// Parameter new_odometer_meters: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mc_interface_set_odometer: mengatur mc interface set odometer setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_odometer(uint64_t new_odometer_meters);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi mc_interface_get_odometer_motor: membaca mc interface get odometer motor tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
uint64_t mc_interface_get_odometer_motor(motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter new_odometer_meters: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mc_interface_set_odometer_motor: mengatur mc interface set odometer motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_odometer_motor(motor_id_t id, uint64_t new_odometer_meters);
// Parameter time_ms: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi mc_interface_ignore_input: menjalankan operasi mc interface ignore input sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void mc_interface_ignore_input(int time_ms);
// Parameter delay_sec: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_set_current_off_delay: mengatur mc interface set current off delay setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_current_off_delay(float delay_sec);
// Parameter temp: temperatur atau nilai sementara sesuai konteks modul.
// Fungsi mc_interface_override_temp_motor: menjalankan operasi mc interface override temp motor sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_override_temp_motor(float temp);
// Parameter time_ms: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi mc_interface_ignore_input_both: menjalankan operasi mc interface ignore input both sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_ignore_input_both(int time_ms);
// Fungsi mc_interface_release_motor_override_both: menjalankan operasi mc interface release motor override both
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void mc_interface_release_motor_override_both(void);
// Parameter timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi mc_interface_wait_for_motor_release_both: menjalankan operasi mc interface wait for motor release both
// sesuai tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
bool mc_interface_wait_for_motor_release_both(float timeout);
// Fungsi mc_interface_stat_speed_avg: menjalankan operasi mc interface stat speed avg sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_speed_avg(void);
// Fungsi mc_interface_stat_speed_max: menjalankan operasi mc interface stat speed max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_speed_max(void);
// Fungsi mc_interface_stat_power_avg: menjalankan operasi mc interface stat power avg sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_power_avg(void);
// Fungsi mc_interface_stat_power_max: menjalankan operasi mc interface stat power max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_power_max(void);
// Fungsi mc_interface_stat_current_avg: menjalankan operasi mc interface stat current avg sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_current_avg(void);
// Fungsi mc_interface_stat_current_max: menjalankan operasi mc interface stat current max sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_current_max(void);
// Fungsi mc_interface_stat_temp_mosfet_avg: menjalankan operasi mc interface stat temp mosfet avg sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_mosfet_avg(void);
// Fungsi mc_interface_stat_temp_mosfet_max: menjalankan operasi mc interface stat temp mosfet max sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_mosfet_max(void);
// Fungsi mc_interface_stat_temp_motor_avg: menjalankan operasi mc interface stat temp motor avg sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_motor_avg(void);
// Fungsi mc_interface_stat_temp_motor_max: menjalankan operasi mc interface stat temp motor max sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_temp_motor_max(void);
// Fungsi mc_interface_stat_count_time: menjalankan operasi mc interface stat count time sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
float mc_interface_stat_count_time(void);
// Fungsi mc_interface_stat_reset: mereset mc interface stat reset ke kondisi awal yang aman tanpa meninggalkan
// state lama yang tidak konsisten.
void mc_interface_stat_reset(void);
// Parameter str: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter argn: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter arg0: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter arg1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mc_interface_set_fault_info: mengatur mc interface set fault info setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mc_interface_set_fault_info(const char *str, int argn, float arg0, float arg1);
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Parameter is_isr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mc_interface_fault_stop: menangani mc interface fault stop dengan memprioritaskan pemadaman keluaran
// daya, pencatatan penyebab, dan pemulihan yang aman.
void mc_interface_fault_stop(mc_fault_code fault, bool is_second_motor, bool is_isr);
// Fungsi mc_interface_try_input: menjalankan operasi mc interface try input sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
int mc_interface_try_input(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi mc_interface_try_input_motor: menjalankan operasi mc interface try input motor sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
int mc_interface_try_input_motor(motor_id_t id);
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Parameter dt: selang waktu antar pembaruan algoritma.
// Fungsi mc_interface_mc_timer_isr: menangani mc interface mc timer isr pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
void mc_interface_mc_timer_isr(bool is_second_motor, float dt);
