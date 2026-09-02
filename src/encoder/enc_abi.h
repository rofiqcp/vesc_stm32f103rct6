#pragma once
#include "encoder_datatype.h"
#include "encoder_cfg.h"

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter cfg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi enc_abi_init: menginisialisasi enc abi init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
bool enc_abi_init(MotorRuntime *m, const encoder_cfg_ABI_t *cfg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi enc_abi_deinit: melepas atau menonaktifkan resource enc abi deinit dengan urutan yang aman.
void enc_abi_deinit(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi enc_abi_read_deg: menjalankan operasi enc abi read deg sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
float enc_abi_read_deg(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi enc_abi_set_deg: mengatur enc abi set deg setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void enc_abi_set_deg(MotorRuntime *m, float deg);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi enc_abi_index_found: menjalankan operasi enc abi index found sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool enc_abi_index_found(const MotorRuntime *m);
