#include "encoder/encoder.h"
#include "encoder/enc_abi.h"

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_init: menginisialisasi encoder init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
bool encoder_init(MotorRuntime *m) {
    if (!m || m->id != MOTOR_LEFT)
        return false;
    encoder_cfg_ABI.counts = m->encoder.cpr;
    return enc_abi_init(m, &encoder_cfg_ABI);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_deinit: melepas atau menonaktifkan resource encoder deinit dengan urutan yang aman.
void encoder_deinit(MotorRuntime *m) {
    enc_abi_deinit(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_read_deg: menjalankan operasi encoder read deg sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
float encoder_read_deg(MotorRuntime *m) {
    return enc_abi_read_deg(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_set_deg: mengatur encoder set deg setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
void encoder_set_deg(MotorRuntime *m, float deg) {
    enc_abi_set_deg(m, deg);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_index_found: menjalankan operasi encoder index found sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool encoder_index_found(const MotorRuntime *m) {
    return enc_abi_index_found(m);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_update_config: memperbarui encoder update config menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void encoder_update_config(MotorRuntime *m) {
    if (!m || m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER)
        return;
    if (encoder_cfg_ABI.counts != m->encoder.cpr) {
        encoder_cfg_ABI.counts = m->encoder.cpr;
        (void)enc_abi_init(m, &encoder_cfg_ABI);
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_is_configured: menjalankan operasi encoder is configured sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool encoder_is_configured(const MotorRuntime *m) {
    return m && m->id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.cpr >= 4U;
}
