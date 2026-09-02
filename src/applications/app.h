#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    UART_PORT_COMM_HEADER = 0,
    UART_PORT_BUILTIN,
    UART_PORT_EXTRA_HEADER
} UART_PORT;

// Fungsi app_get_configuration: membaca app get configuration tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
const app_configuration *app_get_configuration(void);
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi app_set_configuration: mengatur app set configuration setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void app_set_configuration(app_configuration *conf);
// Parameter time_ms: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi app_disable_output: menjalankan operasi app disable output sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void app_disable_output(int time_ms);
// Fungsi app_is_output_disabled: menjalankan operasi app is output disabled sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool app_is_output_disabled(void);
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi app_calc_crc: menangani kalibrasi app calc crc agar offset atau parameter hasil ukur valid sebelum
// dipakai kendali.
unsigned short app_calc_crc(app_configuration *conf);
// Fungsi app_notify_configuration_changed: menjalankan operasi app notify configuration changed sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void app_notify_configuration_changed(void);

// Fungsi app_uartcomm_initialize: menjalankan operasi app uartcomm initialize sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_uartcomm_initialize(void);
// Parameter port_number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi app_uartcomm_start: memulai app uartcomm start setelah prasyarat hardware, konfigurasi, dan state
// keselamatan terpenuhi.
void app_uartcomm_start(UART_PORT port_number);
// Parameter port_number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi app_uartcomm_stop: menghentikan app uartcomm stop dengan menonaktifkan output atau state terkait
// secara aman.
void app_uartcomm_stop(UART_PORT port_number);
// Parameter baudrate: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter permanent_enabled: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter port_number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi app_uartcomm_configure: menjalankan operasi app uartcomm configure sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void app_uartcomm_configure(uint32_t baudrate, bool permanent_enabled, UART_PORT port_number);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter port_number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi app_uartcomm_send_packet: menyusun atau mengirim app uartcomm send packet dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void app_uartcomm_send_packet(unsigned char *data, unsigned int len, UART_PORT port_number);
