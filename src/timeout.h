#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TIMEOUT_HEARTBEAT_FOC = 0,
    TIMEOUT_HEARTBEAT_MOTOR_SERVICE = 1,
    TIMEOUT_HEARTBEAT_COMM = 2,
    TIMEOUT_HEARTBEAT_FAULT = 3,
    TIMEOUT_HEARTBEAT_COUNT = 4
} timeout_heartbeat_id_t;

/* Capture RCC reset flags early in main(), before they are cleared. */
// Fungsi timeout_capture_reset_reason: mereset timeout capture reset reason ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
void timeout_capture_reset_reason(void);
// Fungsi timeout_get_reset_flags: membaca timeout get reset flags tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
uint32_t timeout_get_reset_flags(void);
// Fungsi timeout_had_iwdg_reset: mereset timeout had iwdg reset ke kondisi awal yang aman tanpa meninggalkan
// state lama yang tidak konsisten.
bool timeout_had_iwdg_reset(void);

// Fungsi timeout_init: menginisialisasi timeout init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
bool timeout_init(void);
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi timeout_update_10ms: memperbarui timeout update 10ms menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void timeout_update_10ms(uint32_t now_ms);
// Fungsi timeout_reset: mereset timeout reset ke kondisi awal yang aman tanpa meninggalkan state lama yang
// tidak konsisten.
void timeout_reset(void);
// Parameter timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Parameter brake_current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi timeout_configure: menjalankan operasi timeout configure sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void timeout_configure(uint32_t timeout_ms, float brake_current_a);
// Fungsi timeout_has_timeout: menjalankan operasi timeout has timeout sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool timeout_has_timeout(void);
// Fungsi timeout_get_timeout_ms: membaca timeout get timeout ms tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
uint32_t timeout_get_timeout_ms(void);
// Fungsi timeout_get_brake_current: membaca timeout get brake current tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float timeout_get_brake_current(void);

/* Hardware watchdog / liveness gate. The watchdog is started only after the
 * service/communication threads exist. FOC becomes a required heartbeat only
 * after synchronized ADC sampling has actually started. */
// Fungsi timeout_watchdog_start: memulai timeout watchdog start setelah prasyarat hardware, konfigurasi, dan
// state keselamatan terpenuhi.
void timeout_watchdog_start(void);
// Parameter required: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi timeout_watchdog_require_foc: menjalankan bagian timeout watchdog require foc pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void timeout_watchdog_require_foc(bool required);
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi timeout_watchdog_update_10ms: memperbarui timeout watchdog update 10ms menggunakan state terbaru
// dengan urutan yang konsisten dan aman.
void timeout_watchdog_update_10ms(uint32_t now_ms);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_heartbeat: menjalankan operasi timeout heartbeat sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void timeout_heartbeat(timeout_heartbeat_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_heartbeat_from_isr: menangani timeout heartbeat from isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void timeout_heartbeat_from_isr(timeout_heartbeat_id_t id);
// Fungsi timeout_watchdog_started: menjalankan operasi timeout watchdog started sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool timeout_watchdog_started(void);
// Fungsi timeout_watchdog_healthy: menjalankan operasi timeout watchdog healthy sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool timeout_watchdog_healthy(void);
// Fungsi timeout_watchdog_required_mask: menjalankan operasi timeout watchdog required mask sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_watchdog_required_mask(void);
// Fungsi timeout_watchdog_unhealthy_mask: menjalankan operasi timeout watchdog unhealthy mask sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_watchdog_unhealthy_mask(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_watchdog_miss_count: menjalankan operasi timeout watchdog miss count sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_watchdog_miss_count(timeout_heartbeat_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_heartbeat_count: menjalankan operasi timeout heartbeat count sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_heartbeat_count(timeout_heartbeat_id_t id);
