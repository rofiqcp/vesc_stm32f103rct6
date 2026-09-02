#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    CONF_BOOT_VIRGIN = 0,
    CONF_BOOT_VALID = 1,
    CONF_BOOT_CORRUPT = 2
} conf_boot_status_t;

/* STM32F103RC has no data EEPROM. emulates it with four transactional
   2-KiB flash pages reserved at the end of flash. */
/* VESC-style configuration/persistence front end. Persistent payloads are
   exact VESC Tool wire images; runtime hardware clamping is intentionally
   separate so a write/readback does not produce false Parameters truncated. */
// Fungsi conf_general_init: menginisialisasi conf general init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
bool conf_general_init(void);
// Fungsi conf_general_load_apply: memuat conf general load apply dan memvalidasi integritas data sebelum
// digunakan oleh runtime.
bool conf_general_load_apply(void);
// Fungsi conf_general_store_all: menyimpan conf general store all secara transaksional dengan pemeriksaan
// integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
bool conf_general_store_all(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter wire: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi conf_general_store_mc_wire_persistent: menyimpan conf general store mc wire persistent secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_mc_wire_persistent(motor_id_t id, const uint8_t *wire);
// Parameter wire: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi conf_general_store_app_wire_persistent: menyimpan conf general store app wire persistent secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_app_wire_persistent(const uint8_t *wire);
// Fungsi conf_general_is_valid: menjalankan operasi conf general is valid sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool conf_general_is_valid(void);
// Fungsi conf_general_boot_status: menjalankan operasi conf general boot status sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
conf_boot_status_t conf_general_boot_status(void);
// Fungsi conf_general_get_save_count: membaca conf general get save count tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
uint32_t conf_general_get_save_count(void);
// Fungsi conf_general_integrity_ok: menjalankan operasi conf general integrity ok sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool conf_general_integrity_ok(void);
// Fungsi conf_general_get_integrity_checks: membaca conf general get integrity checks tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
uint32_t conf_general_get_integrity_checks(void);
// Fungsi conf_general_get_integrity_failures: membaca conf general get integrity failures tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
uint32_t conf_general_get_integrity_failures(void);
// Fungsi conf_general_request_aux_store: menyimpan conf general request aux store secara transaksional dengan
// pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
void conf_general_request_aux_store(void);
// Fungsi conf_general_service_100hz: menjalankan operasi conf general service 100hz sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void conf_general_service_100hz(void);

/* Canonical upstream-style typed configuration front end. Persistent storage
 * is still the transactional exact VESC-6 wire record used by this port. */
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi conf_general_read_app_configuration: menjalankan operasi conf general read app configuration sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void conf_general_read_app_configuration(app_configuration *conf);
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi conf_general_store_app_configuration: menyimpan conf general store app configuration secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_app_configuration(app_configuration *conf);
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter is_motor_2: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi conf_general_read_mc_configuration: menjalankan operasi conf general read mc configuration sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2);
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter is_motor_2: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi conf_general_store_mc_configuration: menyimpan conf general store mc configuration secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2);
