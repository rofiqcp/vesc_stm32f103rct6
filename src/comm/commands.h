#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "datatypes.h"

typedef void (*vesc_appdata_handler_t)(const uint8_t *data, uint16_t len, motor_id_t motor);

/* Canonical VESC command API names used by upstream modules. The reduced
 * STM32F103 port still exposes board-specific vesc_comm_* helpers below for
 * transport readiness and the dual-motor USART3 integration. */
// Fungsi commands_init: menginisialisasi commands init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void commands_init(void);
// Fungsi commands_is_initialized: menjalankan operasi commands is initialized sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool commands_is_initialized(void);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi commands_send_packet: menyusun atau mengirim commands send packet dengan pemeriksaan panjang buffer
// dan jalur transport yang aman.
void commands_send_packet(unsigned char *data, unsigned int len);
#if defined(__GNUC__) || defined(__clang__)
// Parameter fmt: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter printf: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi commands_printf: menjalankan operasi commands printf sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
int commands_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#else
// Parameter fmt: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_printf: menjalankan operasi commands printf sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
int commands_printf(const char *fmt, ...);
#endif
// Parameter rotor_pos: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi commands_send_rotor_pos: menyusun atau mengirim commands send rotor pos dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void commands_send_rotor_pos(float rotor_pos);
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi commands_send_experiment_samples: menyusun atau mengirim commands send experiment samples dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
void commands_send_experiment_samples(float *samples, int len);
// Parameter namex: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter namey: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_init_plot: menginisialisasi commands init plot sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void commands_init_plot(const char *namex, const char *namey);
// Parameter name: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_plot_add_graph: menjalankan operasi commands plot add graph sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void commands_plot_add_graph(const char *name);
// Parameter graph: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_plot_set_graph: mengatur commands plot set graph setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void commands_plot_set_graph(int graph);
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter y: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_send_plot_points: menyusun atau mengirim commands send plot points dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void commands_send_plot_points(float x, float y);

// Fungsi vesc_comm_task_init: menginisialisasi vesc comm task init sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
bool vesc_comm_task_init(void);
// Parameter packet: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter blocking: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_comm_set_thread_ids: mengatur vesc comm set thread ids setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void vesc_comm_set_thread_ids(TaskHandle_t packet, TaskHandle_t blocking);
// Parameter ready: penanda bahwa resource atau state siap digunakan.
// Fungsi vesc_comm_set_config_ready: mengatur vesc comm set config ready setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void vesc_comm_set_config_ready(bool ready);
// Parameter ready: penanda bahwa resource atau state siap digunakan.
// Fungsi vesc_comm_set_motor_ready: mengatur vesc comm set motor ready setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void vesc_comm_set_motor_ready(bool ready);
// Fungsi vesc_comm_motor_ready: menjalankan operasi vesc comm motor ready sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool vesc_comm_motor_ready(void);
// Fungsi vesc_comm_try_recover_motor_ready: memulihkan gate motor hanya setelah sampling hard-critical dan kalibrasi terbukti sehat.
bool vesc_comm_try_recover_motor_ready(void);
// Fungsi vesc_comm_poll_once: menjalankan operasi vesc comm poll once sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool vesc_comm_poll_once(void);
// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi vesc_comm_send_payload: menyusun atau mengirim vesc comm send payload dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void vesc_comm_send_payload(const uint8_t *payload, uint16_t len);
// Fungsi vesc_comm_periodic_100hz: menjalankan operasi vesc comm periodic 100hz sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void vesc_comm_periodic_100hz(void);
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Fungsi vesc_comm_send_sample_buffer: menyusun atau mengirim vesc comm send sample buffer dengan pemeriksaan
// panjang buffer dan jalur transport yang aman.
void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count);
// Parameter reply: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Fungsi vesc_comm_send_sample_buffer_to: menyusun atau mengirim vesc comm send sample buffer to dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
void vesc_comm_send_sample_buffer_to(void (*reply)(unsigned char *data, unsigned int len),
                                     uint16_t count);

// Parameter handler: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_comm_register_appdata_handler: menangani vesc comm register appdata handler pada konteks
// interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
void vesc_comm_register_appdata_handler(vesc_appdata_handler_t handler);

/* VESC COMM_PRINT transport hook used by motor diagnostics. */
// Parameter msg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_send_print: menyusun atau mengirim commands send print dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
void commands_send_print(const char *msg);

typedef struct {
    // Variabel packet_stack_free_bytes: ruang stack atau informasi pemakaian stack task.
    uint32_t packet_stack_free_bytes;
    // Variabel blocking_stack_free_bytes: ruang stack atau informasi pemakaian stack task.
    uint32_t blocking_stack_free_bytes;
} vesc_comm_resource_stats_t;
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_comm_get_resource_stats: membaca vesc comm get resource stats tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
void vesc_comm_get_resource_stats(vesc_comm_resource_stats_t *out);

typedef struct {
    // Variabel last_outer_cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t last_outer_cmd;
    // Variabel last_forward_target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t last_forward_target;
    // Variabel last_forward_inner_cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t last_forward_inner_cmd;
    // Variabel last_motor_context: state atau parameter motor yang sedang diproses.
    volatile uint8_t last_motor_context;
    // Variabel last_reply_cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t last_reply_cmd;
    // Variabel last_tx_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint8_t last_tx_ok;
    // Variabel last_control_cmd: command kendali motor standar VESC terakhir yang diterima.
    volatile uint8_t last_control_cmd;
    // Variabel last_control_motor: motor terakhir yang menjadi target command kendali, 0=LEFT dan 1=RIGHT.
    volatile uint8_t last_control_motor;
    // Variabel last_control_result: hasil penerimaan command kendali untuk diagnostik silent-reject.
    volatile uint8_t last_control_result;
    // Variabel last_control_app_reject: alasan penolakan dari arbitration aplikasi UART.
    volatile uint8_t last_control_app_reject;
    // Variabel reserved: ruang alignment yang dipertahankan untuk struktur diagnostik.
    volatile uint8_t reserved;
    // Variabel last_control_value_scaled: nilai int32 wire asli command kendali terakhir.
    volatile int32_t last_control_value_scaled;
    // Variabel control_accept_count: jumlah command kendali yang benar-benar diterapkan.
    volatile uint32_t control_accept_count;
    // Variabel control_reject_count: jumlah command kendali yang ditolak sebelum diterapkan.
    volatile uint32_t control_reject_count;
    // Variabel get_values_m1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t get_values_m1;
    // Variabel get_values_m2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t get_values_m2;
    // Variabel forward_m2_count: pencacah kejadian atau sampel.
    volatile uint32_t forward_m2_count;
    // Variabel forward_m2_reply_count: pencacah kejadian atau sampel.
    volatile uint32_t forward_m2_reply_count;
} vesc_comm_trace_t;
// Variabel g_vesc_comm_trace: state global firmware yang dibagikan antarbagian modul.
extern vesc_comm_trace_t g_vesc_comm_trace;
