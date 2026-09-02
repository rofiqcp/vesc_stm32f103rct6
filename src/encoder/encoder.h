#pragma once
#include "encoder_datatype.h"

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_init: menginisialisasi encoder init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
bool encoder_init(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_deinit: melepas atau menonaktifkan resource encoder deinit dengan urutan yang aman.
void encoder_deinit(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_read_deg: menjalankan operasi encoder read deg sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
float encoder_read_deg(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_set_deg: mengatur encoder set deg setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void encoder_set_deg(MotorRuntime *m, float deg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_index_found: menjalankan operasi encoder index found sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool encoder_index_found(const MotorRuntime *m);

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_update_config: memperbarui encoder update config menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void encoder_update_config(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_is_configured: menjalankan operasi encoder is configured sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool encoder_is_configured(const MotorRuntime *m);
