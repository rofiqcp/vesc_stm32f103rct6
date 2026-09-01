#pragma once
#include <stdbool.h>
#include "datatypes.h"
#include <stdint.h>

// Fungsi telemetry_init: menginisialisasi telemetry init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
bool telemetry_init(void);
// Fungsi telemetry_update_100hz: menyiapkan telemetry update 100hz secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
void telemetry_update_100hz(void);
// Fungsi telemetry_stats_update_100hz: menyiapkan telemetry stats update 100hz secara koheren untuk telemetri
// tanpa memblokir jalur ISR FOC.
void telemetry_stats_update_100hz(void);
// Fungsi telemetry_snapshot_100hz: menyiapkan telemetry snapshot 100hz secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
void telemetry_snapshot_100hz(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi telemetry_get: membaca telemetry get tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
void telemetry_get(motor_id_t id, motor_telemetry_t *out);
/* Lock-free task-side snapshot for VESC command replies. Uses the ISR/cache
 * seqlock so forwarded GET_VALUES tidak pernah menunggu mutex telemetry. */
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi telemetry_get_realtime: membaca telemetry get realtime tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void telemetry_get_realtime(motor_id_t id, motor_telemetry_t *out);

/* VESC COMM_GET_VALUES uses read-reset averages for these six quantities.
 * The mask uses the standard GET_VALUES bit positions 2,3,4,5,19,20. */
typedef struct {
    // Variabel current_motor: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_motor;
    // Variabel current_in: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_in;
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    float id;
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq;
    // Variabel vd: tegangan sumbu-d keluaran regulator FOC.
    float vd;
    // Variabel vq: tegangan sumbu-q keluaran regulator FOC.
    float vq;
} motor_telemetry_avg_t;

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi telemetry_read_reset_avg: menyiapkan telemetry read reset avg secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
void telemetry_read_reset_avg(motor_id_t id, uint32_t mask, motor_telemetry_avg_t *out);
