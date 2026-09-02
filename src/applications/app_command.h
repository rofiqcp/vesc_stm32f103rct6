#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    APP_CMD_SRC_NONE = 0,
    APP_CMD_SRC_ADC,
    APP_CMD_SRC_UART,
    APP_CMD_SRC_CALIBRATION,
    APP_CMD_SRC_DETECTION,
    APP_CMD_SRC_INTERNAL
} app_command_source_t;

// Alasan penolakan terakhir saat UART mencoba mengambil kendali motor.
typedef enum {
    APP_UART_REJECT_NONE = 0,
    APP_UART_REJECT_INVALID_ID,
    APP_UART_REJECT_DETECT_BUSY,
    APP_UART_REJECT_OUTPUT_DISABLED,
    APP_UART_REJECT_FAULT,
    APP_UART_REJECT_CAL_NOT_DONE,
    APP_UART_REJECT_CAL_INVALID
} app_uart_reject_reason_t;

// Fungsi app_command_init: menginisialisasi app command init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void app_command_init(void);
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi app_command_service_1khz: menjalankan operasi app command service 1khz sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_command_service_1khz(uint32_t now_ms);
// Fungsi app_command_configuration_changed: menjalankan operasi app command configuration changed sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void app_command_configuration_changed(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_uart_claim: menjalankan operasi app command uart claim sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool app_command_uart_claim(motor_id_t id);
// Parameter id: identitas motor yang ingin diperiksa alasan penolakan UART terakhirnya.
// Fungsi app_command_uart_reject_reason: membaca alasan penolakan UART tanpa mengubah state kendali.
app_uart_reject_reason_t app_command_uart_reject_reason(motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_uart_keepalive: menjalankan operasi app command uart keepalive sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_command_uart_keepalive(motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter neutral_stable: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi app_command_adc_claim: menjalankan operasi app command adc claim sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool app_command_adc_claim(motor_id_t id, bool neutral_stable);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_adc_block: menjalankan operasi app command adc block sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void app_command_adc_block(motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter stop_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi app_command_adc_release: menjalankan operasi app command adc release sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_command_adc_release(motor_id_t id, bool stop_motor);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_force_adc_rearm: menjalankan operasi app command force adc rearm sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
void app_command_force_adc_rearm(motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter stop_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi app_command_release: menjalankan operasi app command release sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void app_command_release(motor_id_t id, bool stop_motor);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_get_source: membaca app command get source tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
app_command_source_t app_command_get_source(motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_adc_rearm_required: menjalankan operasi app command adc rearm required sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
bool app_command_adc_rearm_required(motor_id_t id);
