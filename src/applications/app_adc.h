#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    APP_ADC_FAULT_NONE = 0,
    APP_ADC_FAULT_NOT_READY = 1U << 0,
    APP_ADC_FAULT_CONFIG = 1U << 1,
    APP_ADC_FAULT_THROTTLE_RANGE = 1U << 2,
    APP_ADC_FAULT_BRAKE_RANGE = 1U << 3,
    APP_ADC_FAULT_IMPLAUSIBLE = 1U << 4,
    APP_ADC_FAULT_START_ACTIVE = 1U << 5
} app_adc_fault_t;

typedef struct {
    // Variabel raw1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t raw1;
    // Variabel raw2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t raw2;
    // Variabel voltage1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float voltage1;
    // Variabel voltage2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float voltage2;
    // Variabel decoded1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float decoded1;
    // Variabel decoded2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float decoded2;
    // Variabel command: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float command;
    // Variabel armed_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool armed_left;
    // Variabel armed_right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool armed_right;
    // Variabel range_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool range_ok;
    // Variabel fault_flags: status atau data gangguan untuk sistem proteksi.
    uint8_t fault_flags;
} app_adc_status_t;

// Fungsi app_adc_init: menginisialisasi app adc init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
void app_adc_init(void);
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi app_adc_service_1khz: menjalankan operasi app adc service 1khz sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void app_adc_service_1khz(uint32_t now_ms);
// Fungsi app_adc_get_decoded_level: membaca app adc get decoded level tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float app_adc_get_decoded_level(void);
// Fungsi app_adc_get_voltage: membaca app adc get voltage tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float app_adc_get_voltage(void);
// Fungsi app_adc_get_decoded_level2: membaca app adc get decoded level2 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float app_adc_get_decoded_level2(void);
// Fungsi app_adc_get_voltage2: membaca app adc get voltage2 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float app_adc_get_voltage2(void);
// Fungsi app_adc_range_ok: menjalankan operasi app adc range ok sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool app_adc_range_ok(void);
// Fungsi app_adc_data_ready: menjalankan operasi app adc data ready sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool app_adc_data_ready(void);
// Fungsi app_adc_fault_flags: menangani app adc fault flags dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
uint8_t app_adc_fault_flags(void);
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi app_adc_get_status: membaca app adc get status tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
void app_adc_get_status(app_adc_status_t *out);
