#include "comm/commands.h"
#include "applications/app_uartcomm.h"
#include "applications/app.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "util/buffer.h"
#include "comm/packet.h"
#include "motor/mc_interface.h"
#include "hwconf/hw.h"
#include "telemetry.h"
#include "motor/mcpwm_foc.h"
#include "motor/foc_math.h"
#include "applications/appconf_default.h"
#include "timeout.h"
#include "conf_general.h"
#include "confgenerator.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <limits.h>

// Variabel g_vesc_boot_stage: state global firmware yang dibagikan antarbagian modul.
extern volatile uint32_t g_vesc_boot_stage;
// Variabel g_vesc_boot_error: state global firmware yang dibagikan antarbagian modul.
extern volatile uint32_t g_vesc_boot_error;
// Variabel g_vesc_sampling_contract_flags: snapshot seluruh bit kontrak sampling saat boot.
extern volatile uint32_t g_vesc_sampling_contract_flags;

#define BLOCK_QUEUE_DEPTH 1U
#define BLOCK_DATA_MAX    VESC_PACKET_MAX_PAYLOAD
#define VESC_TEMP_UNAVAILABLE_DECIC (-3000)
#define CURRENT_CAL_REPLY_REV19_LEN 463U

#if VESC_PACKET_MAX_PAYLOAD < CURRENT_CAL_REPLY_REV19_LEN
#error "VESC packet payload is too small for current-cal revision 17"
#endif

/* Command numbers are locked to the VESC firmware 6.00 ABI for the subset
 * this reduced STM32F103 FOC port intentionally implements. */
/* COMM_PACKET_ID is canonical and shared from datatypes.h. */

enum {
    CUSTOM_SELECT_MOTOR = 0xA0,
    CUSTOM_DUAL_SUMMARY = 0xA1,
    CUSTOM_CLEAR_FAULT = 0xA2,
    CUSTOM_STOP = 0xA3,
    CUSTOM_SENSOR_SELECT = 0xA4,
    CUSTOM_SENSOR_DETECT = 0xA5,
    CUSTOM_CURRENT_CAL = 0xA6,
    CUSTOM_SAMPLE_START = 0xA7,
    CUSTOM_EXT_TELEMETRY = 0xA8,
    CUSTOM_SENSOR_INFO = 0xA9,
    CUSTOM_COMM_DIAG = 0xAA,
    CUSTOM_CONFIG_SAVE = 0xAB,
    CUSTOM_CONFIG_STATUS = 0xAC,
    CUSTOM_BUZZER_TEST = 0xAD,
    INTERNAL_CUSTOM_SENSOR_DETECT = 0xF0,
    /* Diagnostik SVPWM open-loop fixed-phase TANPA current PI. Digunakan untuk
     * menguji apakah sampling arus phase benar terhadap sudut tegangan tanpa
     * melibatkan observer/PI/forced-angle detect. Aman: duty dibatasi kecil. */
    CUSTOM_OPENLOOP_PHASE = 0xF1,
    CUSTOM_ADC_PHASE_OFFSET = 0xF2
};

typedef struct {
    // Variabel cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t cmd;
    // Variabel motor: state atau parameter motor yang sedang diproses.
    uint8_t motor;
    // Variabel len: panjang data yang sedang diproses atau dikirim.
    uint16_t len;
    // Variabel data: data kerja yang diproses atau dipertukarkan modul.
    uint8_t data[BLOCK_DATA_MAX];
} blocking_job_t;

typedef struct {
    // Variabel rx_frames_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t rx_frames_ok;
    // Variabel tx_frames: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t tx_frames;
    // Variabel blocking_busy_drops: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t blocking_busy_drops;
    // Variabel motor2_forwards: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t motor2_forwards;
    // Variabel unsupported_forward_ids: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    volatile uint32_t unsupported_forward_ids;
} comm_diag_t;

// Variabel s_parser: state internal modul yang dipertahankan antar pemanggilan fungsi.
static vesc_packet_parser_t s_parser;
// Variabel s_block_queue: handle atau state antrean FreeRTOS untuk pertukaran data antartask.
static QueueHandle_t s_block_queue;
// Variabel s_packet_tp: state internal modul yang dipertahankan antar pemanggilan fungsi.
static TaskHandle_t s_packet_tp;
// Variabel s_blocking_tp: state internal modul yang dipertahankan antar pemanggilan fungsi.
static TaskHandle_t s_blocking_tp;
// Variabel s_payload_mutex: handle sinkronisasi untuk melindungi resource bersama.
static SemaphoreHandle_t s_payload_mutex;
// Variabel s_send_mutex: handle sinkronisasi untuk melindungi resource bersama.
static SemaphoreHandle_t s_send_mutex;
// Variabel s_display_mode: mode operasi yang menentukan jalur algoritma aktif.
static volatile uint8_t s_display_mode[2];
// Variabel s_display_owner: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile int8_t s_display_owner = -1;
// Variabel s_tx_payload: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_tx_payload[VESC_PACKET_MAX_PAYLOAD];
// Variabel s_tx_frame: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_tx_frame[VESC_PACKET_BUFFER_SIZE];
/* Blocking-worker-owned config scratch. Static BSS avoids placing ~1.5 KiB
   of MCCONF rollback/work images on the 3-KiB RTOS worker stack. */
// Variabel s_mc_backup: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_mc_backup[2][VESC6_MCCONF_WIRE_SIZE];
// Variabel s_mc_work: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_mc_work[VESC6_MCCONF_WIRE_SIZE];
/* Satu job blocking bersama cukup karena hanya packet_process yang menjadi
 * producer dan worker memproses satu transaksi pada satu waktu. Queue hanya
 * membawa token 1 byte; ini menghindari dua copy BSS + satu copy queue >500 B
 * tanpa menambah stack task. s_block_busy menjaga buffer tidak ditimpa selama
 * worker melakukan detect/config flash yang dapat berlangsung lama. */
// Variabel s_block_job: state internal modul yang dipertahankan antar pemanggilan fungsi.
static blocking_job_t s_block_job;
// Variabel s_block_busy: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_block_busy = false;
/* GET_VALUES is also reached through one-level COMM_FORWARD_CAN recursion.
 * Keep the comparatively large telemetry/average snapshots in BSS rather than
 * on packet_process' stack. This mirrors upstream's packet-buffer/mempool idea
 * and prevents a motor-2-only stack overrun while preserving recursive routing. */
// Variabel s_get_values_telem: state internal modul yang dipertahankan antar pemanggilan fungsi.
static motor_telemetry_t s_get_values_telem;
// Variabel s_get_values_avg: state internal modul yang dipertahankan antar pemanggilan fungsi.
static motor_telemetry_avg_t s_get_values_avg;
// Variabel s_diag: state internal modul yang dipertahankan antar pemanggilan fungsi.
static comm_diag_t s_diag;
// Variabel g_vesc_comm_trace: state global firmware yang dibagikan antarbagian modul.
vesc_comm_trace_t g_vesc_comm_trace;
// Variabel g_vesc_packet_stack_free_words: ruang stack atau informasi pemakaian stack task.
volatile uint32_t g_vesc_packet_stack_free_words = 0U;
// Variabel s_appdata_handler: state internal modul yang dipertahankan antar pemanggilan fungsi.
static vesc_appdata_handler_t s_appdata_handler = NULL;
// Variabel s_motor_ready: state atau parameter motor yang sedang diproses.
static volatile bool s_motor_ready = false;
// Variabel s_config_ready: data konfigurasi yang mengatur perilaku firmware.
static volatile bool s_config_ready = false;
// Variabel s_comm_initialized: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_comm_initialized = false;
// Variabel s_shutdown_latched: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_shutdown_latched = false;
// Variabel s_motor_ready_recovery_count: jumlah recovery aman dari NOT_READY setelah sampling+kalibrasi sehat.
static volatile uint32_t s_motor_ready_recovery_count = 0U;

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi process_payload: memproses process payload setelah input divalidasi lalu memperbarui state atau output
// sesuai aturan modul.
static void process_payload(const uint8_t *data, uint16_t len);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi process_payload_for_motor: memproses process payload for motor setelah input divalidasi lalu
// memperbarui state atau output sesuai aturan modul.
static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id);
// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi packet_process_thread: memproses packet process thread setelah input divalidasi lalu memperbarui state
// atau output sesuai aturan modul.
void packet_process_thread(void *argument);
// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi blocking_thread: menjalankan operasi blocking thread sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void blocking_thread(void *argument);
// Fungsi vesc_comm_reply_diag: menyusun atau mengirim vesc comm reply diag dengan pemeriksaan panjang buffer
// dan jalur transport yang aman.
static void vesc_comm_reply_diag(void);
// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi vesc_comm_send_payload_low_priority: menyusun atau mengirim vesc comm send payload low priority dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
static void vesc_comm_send_payload_low_priority(const uint8_t *payload, uint16_t len);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi process_terminal_text: melayani process terminal text sebagai diagnostik terminal tanpa menambah beban
// pada loop kontrol real-time.
static void process_terminal_text(const uint8_t *data, uint16_t len, motor_id_t id);
// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi queue_blocking_raw: menjalankan operasi queue blocking raw sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool queue_blocking_raw(uint8_t cmd, motor_id_t id, const uint8_t *data, uint16_t len);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_stats: menyusun atau mengirim reply stats dengan pemeriksaan panjang buffer dan jalur transport
// yang aman.
static void reply_stats(const uint8_t *data, uint16_t len, motor_id_t id);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_mcconf_temp: menyusun atau mengirim reply mcconf temp dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_mcconf_temp(motor_id_t id);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter setup: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi set_mcconf_temp: mengatur set mcconf temp setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
static void set_mcconf_temp(const uint8_t *data, uint16_t len, motor_id_t id, bool setup);

// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_i32_be: membaca get i32 be tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static int32_t get_i32_be(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_u16_be: membaca get u16 be tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static uint16_t get_u16_be(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_u32_be: membaca get u32 be tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static uint32_t get_u32_be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_i16: menyusun put i16 ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_i16(uint8_t *b, uint16_t *i, int16_t v) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t u = (uint16_t)v;
    b[(*i)++] = (uint8_t)(u >> 8);
    b[(*i)++] = (uint8_t)u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_u16: menyusun put u16 ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_u16(uint8_t *b, uint16_t *i, uint16_t v) {
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)v;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_i32: menyusun put i32 ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_i32(uint8_t *b, uint16_t *i, int32_t v) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t u = (uint32_t)v;
    b[(*i)++] = (uint8_t)(u >> 24);
    b[(*i)++] = (uint8_t)(u >> 16);
    b[(*i)++] = (uint8_t)(u >> 8);
    b[(*i)++] = (uint8_t)u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_u32: menyusun put u32 ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_u32(uint8_t *b, uint16_t *i, uint32_t v) {
    put_i32(b, i, (int32_t)v);
}

/* Fully-defined float32-auto representation used by VESC sample/plot packets. */
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi put_float32_auto: menyusun put float32 auto ke buffer/wire format dengan urutan field, skala, dan
// batas data yang konsisten.
static void put_float32_auto(uint8_t *b, uint16_t *i, float number) {
    if (fabsf(number) < 1.5e-38f)
        number = 0.0f;
    // Variabel exp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int exp = 0;
    // Variabel sig: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float sig = frexpf(number, &exp);
    // Variabel mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float mag = fabsf(sig);
    // Variabel fraction: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t fraction = 0U;
    if (mag >= 0.5f) {
        fraction = (uint32_t)((mag - 0.5f) * 16777216.0f);
        exp += 126;
    }
    // Variabel bits: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t bits = (((uint32_t)exp & 0xFFU) << 23) | (fraction & 0x7FFFFFU);
    if (sig < 0.0f)
        bits |= 0x80000000UL;
    put_u32(b, i, bits);
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi scaled_i16: menjalankan operasi scaled i16 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int16_t scaled_i16(float v, float scale) {
    if (!isfinite(v) || !isfinite(scale))
        return 0;
    // Variabel x: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float x = v * scale;
    if (x > 32767.0f)
        x = 32767.0f;
    if (x < -32768.0f)
        x = -32768.0f;
    return (int16_t)x; /* truncation matches upstream VESC buffer_append_float16 */
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi scaled_i32: menjalankan operasi scaled i32 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int32_t scaled_i32(float v, float scale) {
    if (!isfinite(v) || !isfinite(scale))
        return 0;
    // Variabel x: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    double x = (double)v * (double)scale;
    if (x > 2147483647.0)
        x = 2147483647.0;
    if (x < -2147483648.0)
        x = -2147483648.0;
    return (int32_t)x; /* truncation matches upstream VESC buffer_append_float32 */
}


// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi controller_id_for_motor: menjalankan operasi controller id for motor sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static uint8_t controller_id_for_motor(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? VESC_CONTROLLER_ID_RIGHT : VESC_CONTROLLER_ID_LEFT;
}

// Fungsi comm_scheduler_running: menjalankan operasi comm scheduler running sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static bool comm_scheduler_running(void) {
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

// Fungsi payload_begin: menjalankan operasi payload begin sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
static uint8_t *payload_begin(void) {
    if (comm_scheduler_running() && s_payload_mutex != NULL &&
        xSemaphoreTake(s_payload_mutex, portMAX_DELAY) != pdTRUE) {
        return NULL;
    }
    /* Before scheduler start the boot path is single-threaded. Bypass the
     * mutex entirely so early-fatal COMM_FW_VERSION replies cannot block on a
     * FreeRTOS object before task scheduling exists. */
    return s_tx_payload;
}
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi payload_end: menjalankan operasi payload end sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static void payload_end(uint16_t len) {
    // Variabel running: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool running = comm_scheduler_running();
    vesc_comm_send_payload(s_tx_payload, len);
    if (running && s_payload_mutex != NULL)
        (void)xSemaphoreGive(s_payload_mutex);
}

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi commands_send_packet: menyusun atau mengirim commands send packet dengan pemeriksaan panjang buffer
// dan jalur transport yang aman.
void commands_send_packet(unsigned char *data, unsigned int len) {
    if (data == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD)
        return;
    vesc_comm_send_payload((const uint8_t *)data, (uint16_t)len);
}

// Parameter fmt: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_printf: menjalankan operasi commands printf sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
int commands_printf(const char *fmt, ...) {
    if (fmt == NULL)
        return 0;
    // Variabel msg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    char msg[156];
    va_list ap;
    va_start(ap, fmt);
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0)
        return n;
    msg[sizeof(msg) - 1U] = '\0';
    commands_send_print(msg);
    return n;
}

// Parameter msg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_send_print: menyusun atau mengirim commands send print dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
void commands_send_print(const char *msg) {
    if (msg == NULL)
        return;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[160];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_PRINT;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    size_t n = strlen(msg);
    if (n > sizeof(p) - 2U)
        n = sizeof(p) - 2U;
    memcpy(&p[i], msg, n);
    i = (uint16_t)(i + n);
    p[i++] = 0U;
    vesc_comm_send_payload(p, i);
}

// Parameter rotor_pos: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi commands_send_rotor_pos: menyusun atau mengirim commands send rotor pos dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void commands_send_rotor_pos(float rotor_pos) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[5];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    p[i++] = COMM_ROTOR_POSITION;
    buffer_append_int32(p, (int32_t)(rotor_pos*100000.0f), &i);
    commands_send_packet(p, (unsigned)i);
}
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi commands_send_experiment_samples: menyusun atau mengirim commands send experiment samples dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
void commands_send_experiment_samples(float *samples, int len) {
    if (samples == NULL || len <= 0 || len > 63)
        return;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[253];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    p[i++] = COMM_EXPERIMENT_SAMPLE;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (int n = 0; n < len; n++) {
        buffer_append_int32(p, (int32_t)(samples[n] * 10000.0f), &i);
    }
    commands_send_packet(p, (unsigned)i);
}
// Parameter namex: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter namey: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_init_plot: menginisialisasi commands init plot sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void commands_init_plot(const char *namex, const char *namey) {
    if (namex == NULL || namey == NULL)
        return;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0;
    p[i++] = COMM_PLOT_INIT;
    // Variabel nx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ny: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    size_t nx = strlen(namex), ny = strlen(namey);
    if (nx > 100U)
        nx = 100U;
    if (ny > 100U)
        ny = 100U;
    memcpy(&p[i], namex, nx);
    i = (uint16_t)(i + nx);
    p[i++] = 0U;
    memcpy(&p[i], namey, ny);
    i = (uint16_t)(i + ny);
    p[i++] = 0U;
    payload_end(i);
}
// Parameter name: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_plot_add_graph: menjalankan operasi commands plot add graph sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void commands_plot_add_graph(const char *name) {
    if (name == NULL)
        return;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0;
    p[i++] = COMM_PLOT_ADD_GRAPH;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    size_t n = strlen(name);
    if (n > 120U)
        n = 120U;
    memcpy(&p[i], name, n);
    i = (uint16_t)(i + n);
    p[i++] = 0U;
    payload_end(i);
}
// Parameter graph: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_plot_set_graph: mengatur commands plot set graph setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void commands_plot_set_graph(int graph) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[2] = {
        COMM_PLOT_SET_GRAPH, (uint8_t)graph
    }
    ;
    commands_send_packet(p, 2U);
}
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter y: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi commands_send_plot_points: menyusun atau mengirim commands send plot points dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void commands_send_plot_points(float x, float y) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[9];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    p[i++] = COMM_PLOT_DATA;
    buffer_append_float32_auto(p, x, &i);
    buffer_append_float32_auto(p, y, &i);
    commands_send_packet(p, (unsigned)i);
}

/* Hardware CRC reported to VESC Tool in the COMM_FW_VERSION reply.
 * This hoverboard board has no genuine VESC HW CRC calculator, so we build a
 * deterministic board signature: CRC32 (IEEE 802.3, same polynomial as
 * conf_general.c) over the STM32F103 96-bit unique device ID plus the
 * calibration / config flash region. VESC Tool expects a 4-byte CRC appended
 * to the FW_VERSION reply and will otherwise flag the device as unverified. */
// Fungsi main_calc_hw_crc: menangani kalibrasi main calc hw crc agar offset atau parameter hasil ukur valid
// sebelum dipakai kendali.
static uint32_t main_calc_hw_crc(void) {
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t c = 0xFFFFFFFFUL;

    /* STM32F103 unique device ID: 96 bits @ 0x1FFFF7E8. */
    // Variabel uid: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t *uid = (const uint8_t *)0x1FFFF7E8UL;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t n = 0U; n < 12U; n++) {
        c ^= (uint32_t)uid[n];
        // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (uint8_t b = 0U; b < 8U; b++) {
            // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t m = (uint32_t)-(int32_t)(c & 1U);
            c = (c >> 1) ^ (0xEDB88320UL & m);
        }
    }

    /* Calibration / config flash region: 4 pages of 2 KiB @ 0x0803E000. */
    // Variabel cfg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t *cfg = (const uint8_t *)0x0803E000UL;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t n = 0U; n < (4U * 2048U); n++) {
        c ^= (uint32_t)cfg[n];
        // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (uint8_t b = 0U; b < 8U; b++) {
            // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t m = (uint32_t)-(int32_t)(c & 1U);
            c = (c >> 1) ^ (0xEDB88320UL & m);
        }
    }

    return ~c;
}

// Fungsi reply_fw_version: menyusun atau mengirim reply fw version dengan pemeriksaan panjang buffer dan jalur
// transport yang aman.
static void reply_fw_version(void) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;

    /* Match the proven hoverboard_vesc VESC-6.00 handshake field order. The
     * transport and this minimal first reply are deliberately conservative;
     * richer RT data/config support remains implemented by this firmware. */
    p[i++] = COMM_FW_VERSION;
    p[i++] = 6U;
    p[i++] = 0U;

    /* User-visible identity is explicit for this virtual dual-local port:
     * local/controller ID1 = MOTOR_LEFT, forwarded ID2 = MOTOR_RIGHT. */
    // Variabel hw: nama hardware sesuai konteks motor aktif.
    const char *hw = (mc_interface_get_motor_thread() == 2) ? "MOTOR_RIGHT" : "MOTOR_LEFT";
    // Variabel hw_len: panjang data yang sedang diproses atau dikirim.
    uint8_t hw_len = (uint8_t)(strlen(hw) + 1U); /* include NUL terminator */
    memcpy(&p[i], hw, hw_len);
    i = (uint16_t)(i + hw_len);

    // Variabel uid: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t *uid = (const uint32_t *)0x1FFFF7E8UL;
    // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (uint8_t w = 0U; w < 3U; w++) {
        // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t v = uid[w];
        p[i++] = (uint8_t)v;
        p[i++] = (uint8_t)(v >> 8);
        p[i++] = (uint8_t)(v >> 16);
        p[i++] = (uint8_t)(v >> 24);
    }
    if (mc_interface_get_motor_thread() == 2)
        p[i - 1U]++;

    p[i++] = 1U; /* pairing done */
    p[i++] = 0U; /* FW test version */
    p[i++] = 0U; /* HW_TYPE_VESC */
    p[i++] = 0U; /* custom configs */
    p[i++] = 0U; /* phase filters */
    p[i++] = 0U; /* qml hw */
    p[i++] = 0U; /* qml app */
    p[i++] = 0U; /* nrf flags */

    // Variabel fw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const char fw[] = "vesc_stm32f103rct6";
    memcpy(&p[i], fw, sizeof(fw));
    i = (uint16_t)(i + sizeof(fw));

    /* Append the 4-byte hardware CRC that VESC Tool expects at the end of the
     * COMM_FW_VERSION reply (bounds-checked against the payload ceiling). */
    // Variabel hw_crc: nilai CRC untuk memeriksa integritas data.
    uint32_t hw_crc = main_calc_hw_crc();
    // Variabel j: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t j = (int32_t)i;
    if (j + 4 <= (int32_t)VESC_PACKET_MAX_PAYLOAD) {
        buffer_append_uint32(p, hw_crc, &j);
        i = (uint16_t)j;
    }

    payload_end(i);
}

// Fungsi reply_fw_info: menyusun atau mengirim reply fw info dengan pemeriksaan panjang buffer dan jalur
// transport yang aman.
static void reply_fw_info(void) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[16];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_FW_INFO;
    p[i++] = 6U; /* major */
    p[i++] = 0U; /* minor */
    p[i++] = 0U; /* test */
    /* This build is produced outside the upstream git tree, so do not invent
       commit hashes. Empty NUL-terminated strings are the truthful VESC format. */
    p[i++] = 0U;
    p[i++] = 0U;
    vesc_comm_send_payload(p, i);
}

// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter t: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi append_get_values_fields: membaca append get values fields tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
static void append_get_values_fields(uint8_t *p, uint16_t *i,
                                     const motor_telemetry_t *t, uint32_t mask,
                                     motor_id_t id) {
    /* Stock hoverboard hardware has no individual MOSFET NTCs. It does have
     * ADC1's internal MCU sensor, filtered per motor as board_temp_filter_c.
     * Report that honest board-temperature proxy in the VESC FET slots; the
     * motor NTC slot remains unavailable (-300 C), rather than fabricating a
     * motor temperature. */
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel fet_proxy_c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float fet_proxy_c = (m != NULL && m->board_temp_valid)
                             ? m->board_temp_filter_c : -300.0f;
    // Variabel fet_proxy_decic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int16_t fet_proxy_decic = scaled_i16(fet_proxy_c, 10.0f);
    if (mask & (1UL << 0))
        put_i16(p, i, fet_proxy_decic);
    if (mask & (1UL << 1))
        put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    if (mask & (1UL << 2))
        put_i32(p, i, scaled_i32(t->current_motor, 100.0f));
    if (mask & (1UL << 3))
        put_i32(p, i, scaled_i32(t->current_in, 100.0f));
    if (mask & (1UL << 4))
        put_i32(p, i, scaled_i32(t->id_filter, 100.0f));
    if (mask & (1UL << 5))
        put_i32(p, i, scaled_i32(t->iq_filter, 100.0f));
    if (mask & (1UL << 6))
        put_i16(p, i, scaled_i16(t->duty, 1000.0f));
    if (mask & (1UL << 7))
        put_i32(p, i, scaled_i32(t->erpm, 1.0f));
    if (mask & (1UL << 8))
        put_i16(p, i, scaled_i16(t->vbus, 10.0f));
    if (mask & (1UL << 9))
        put_i32(p, i, scaled_i32(t->amp_hours, 10000.0f));
    if (mask & (1UL << 10))
        put_i32(p, i, scaled_i32(t->amp_hours_charged, 10000.0f));
    if (mask & (1UL << 11))
        put_i32(p, i, scaled_i32(t->watt_hours, 10000.0f));
    if (mask & (1UL << 12))
        put_i32(p, i, scaled_i32(t->watt_hours_charged, 10000.0f));
    if (mask & (1UL << 13))
        put_i32(p, i, t->tachometer);
    if (mask & (1UL << 14))
        put_i32(p, i, t->tachometer_abs);
    if (mask & (1UL << 15))
        p[(*i)++] = (uint8_t)motor_fault_to_vesc((motor_fault_t)t->fault);
    if (mask & (1UL << 16))
        put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if (mask & (1UL << 17))
        p[(*i)++] = t->controller_id;
    if (mask & (1UL << 18)) {
        put_i16(p, i, fet_proxy_decic);
        put_i16(p, i, fet_proxy_decic);
        put_i16(p, i, fet_proxy_decic);
    }
    /* VESC upstream selective-values layout: bit 19 = Vd, bit 20 = Vq,
     * bit 21 = status. Keep these as three independent fields; VESC Tool can
     * request each bit separately when it polls a forwarded controller. */
    if (mask & (1UL << 19))
        put_i32(p, i, scaled_i32(t->vd, 1000.0f));
    if (mask & (1UL << 20))
        put_i32(p, i, scaled_i32(t->vq, 1000.0f));
    if (mask & (1UL << 21)) {
        // Variabel status: status runtime untuk diagnostik atau keputusan kendali.
        uint8_t status = 0U;
        if (timeout_has_timeout())
            status |= 1U;
        p[(*i)++] = status;
    }
}

// Parameter command: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter requested_id: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Fungsi reply_get_values: membaca reply get values tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
static void reply_get_values(uint8_t command, uint32_t mask, motor_id_t requested_id) {
    /* VESC dual-motor semantics are thread-selected. A raw UART packet is motor
     * 1; COMM_FORWARD_CAN selects thread 2 before recursively dispatching the
     * inner command. Derive the telemetry source from that thread context so
     * every getter has the same identity rule as upstream commands.c. */
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    const motor_id_t id = (mc_interface_get_motor_thread() == 2) ? MOTOR_RIGHT : MOTOR_LEFT;
    (void)requested_id;
    g_vesc_comm_trace.last_motor_context = (id == MOTOR_RIGHT) ? 2U : 1U;
    if (id == MOTOR_RIGHT)
        g_vesc_comm_trace.get_values_m2++;
    else g_vesc_comm_trace.get_values_m1++;

    telemetry_get_realtime(id, &s_get_values_telem);
    telemetry_read_reset_avg(id, mask, &s_get_values_avg);
    if (mask & (1UL << 2))
        s_get_values_telem.current_motor = s_get_values_avg.current_motor;
    if (mask & (1UL << 3))
        s_get_values_telem.current_in = s_get_values_avg.current_in;
    if (mask & (1UL << 4))
        s_get_values_telem.id_filter = s_get_values_avg.id;
    if (mask & (1UL << 5))
        s_get_values_telem.iq_filter = s_get_values_avg.iq;
    if (mask & (1UL << 19))
        s_get_values_telem.vd = s_get_values_avg.vd;
    if (mask & (1UL << 20))
        s_get_values_telem.vq = s_get_values_avg.vq;

    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = command;
    if (command == COMM_GET_VALUES_SELECTIVE)
        put_u32(p, &i, mask);
    append_get_values_fields(p, &i, &s_get_values_telem, mask, id);
    payload_end(i);
}


// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Parameter t: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sv: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter speed: nilai kecepatan untuk target, pembatas, atau hasil pengukuran.
// Parameter battery_level: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter distance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter distance_abs: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter wh_left: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter odometer: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi append_setup_fields: menyusun append setup fields ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
static void append_setup_fields(uint8_t *p, uint16_t *i,
                                const motor_telemetry_t *t, const setup_values *sv, uint32_t mask,
                                float speed, float battery_level, float distance,
                                float distance_abs, float wh_left, uint32_t odometer) {
    if (mask & (1UL << 0))
        put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    if (mask & (1UL << 1))
        put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    if (mask & (1UL << 2))
        put_i32(p, i, scaled_i32(sv->current_tot, 100.0f));
    if (mask & (1UL << 3))
        put_i32(p, i, scaled_i32(sv->current_in_tot, 100.0f));
    if (mask & (1UL << 4))
        put_i16(p, i, scaled_i16(t->duty, 1000.0f));
    if (mask & (1UL << 5))
        put_i32(p, i, scaled_i32(t->erpm, 1.0f));
    if (mask & (1UL << 6))
        put_i32(p, i, scaled_i32(speed, 1000.0f));
    if (mask & (1UL << 7))
        put_i16(p, i, scaled_i16(t->vbus, 10.0f));
    if (mask & (1UL << 8))
        put_i16(p, i, scaled_i16(battery_level, 1000.0f));
    if (mask & (1UL << 9))
        put_i32(p, i, scaled_i32(sv->ah_tot, 10000.0f));
    if (mask & (1UL << 10))
        put_i32(p, i, scaled_i32(sv->ah_charge_tot, 10000.0f));
    if (mask & (1UL << 11))
        put_i32(p, i, scaled_i32(sv->wh_tot, 10000.0f));
    if (mask & (1UL << 12))
        put_i32(p, i, scaled_i32(sv->wh_charge_tot, 10000.0f));
    if (mask & (1UL << 13))
        put_i32(p, i, scaled_i32(distance, 1000.0f));
    if (mask & (1UL << 14))
        put_i32(p, i, scaled_i32(distance_abs, 1000.0f));
    if (mask & (1UL << 15))
        put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if (mask & (1UL << 16))
        p[(*i)++] = (uint8_t)motor_fault_to_vesc((motor_fault_t)t->fault);
    if (mask & (1UL << 17))
        p[(*i)++] = t->controller_id;
    if (mask & (1UL << 18))
        p[(*i)++] = sv->num_vescs;
    if (mask & (1UL << 19))
        put_i32(p, i, scaled_i32(wh_left, 1000.0f));
    if (mask & (1UL << 20))
        put_u32(p, i, odometer);
    if (mask & (1UL << 21))
        put_u32(p, i, xTaskGetTickCount());
}

// Parameter command: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_setup_values: menyusun atau mengirim reply setup values dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_setup_values(uint8_t command, uint32_t mask, motor_id_t id) {
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_telemetry_t t;
    telemetry_get(id, &t);
    // Variabel old_motor: state atau parameter motor yang sedang diproses.
    int old_motor = mc_interface_get_motor_thread();
    mc_interface_select_motor_thread(id == MOTOR_RIGHT ? 2 : 1);
    // Variabel wh_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float wh_left = 0.0f;
    // Variabel battery_level: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float battery_level = mc_interface_get_battery_level(&wh_left);
    // Variabel speed: nilai kecepatan untuk target atau pengukuran.
    float speed = mc_interface_get_speed();
    // Variabel distance: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float distance = mc_interface_get_distance();
    // Variabel distance_abs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float distance_abs = mc_interface_get_distance_abs();
    // Variabel odo64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t odo64 = mc_interface_get_odometer();
    // Variabel sv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    setup_values sv = mc_interface_get_setup_values();
    mc_interface_select_motor_thread(old_motor);
    // Variabel odometer: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t odometer = odo64 > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : (uint32_t)odo64;

    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = command;
    if (command == COMM_GET_VALUES_SETUP_SELECTIVE)
        put_u32(p, &i, mask);
    append_setup_fields(p, &i, &t, &sv, mask, speed, battery_level, distance, distance_abs, wh_left, odometer);
    payload_end(i);
}


// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi wrap_error_deg: menjalankan operasi wrap error deg sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float wrap_error_deg(float a, float b) {
    // Variabel e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float e = a - b;
    while (e > 180.0f)
        e -= 360.0f;
    while (e < -180.0f)
        e += 360.0f;
    return e;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi send_rotor_position: menyusun atau mengirim send rotor position dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void send_rotor_position(motor_id_t id) {
    // Variabel index: indeks elemen yang sedang diproses.
    uint8_t index = (id == MOTOR_RIGHT) ? 1U : 0U;
    if (s_display_owner != (int8_t)index)
        return;
    // Variabel mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t mode = s_display_mode[index];
    if (mode == DISP_POS_MODE_NONE)
        return;

    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_telemetry_t t;
    telemetry_get(id, &t);
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel observer: state atau hasil estimator sensorless untuk memperkirakan posisi rotor.
    float observer = mcpwm_foc_get_phase_observer_rt(m);
    // Variabel encoder: data encoder untuk pengukuran posisi atau kecepatan rotor.
    float encoder = mcpwm_foc_get_phase_encoder_rt(m);
    // Variabel hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
    float hall = mcpwm_foc_get_phase_hall_rt(m);
    // Variabel pos: nilai posisi rotor atau aktuator.
    float pos;

    switch ((disp_pos_mode_t)mode) {
        case DISP_POS_MODE_ENCODER:
            pos = encoder;
        break;
        case DISP_POS_MODE_PID_POS:
            pos = t.position_deg;
        break;
        case DISP_POS_MODE_PID_POS_ERROR:
            pos = wrap_error_deg(m->position_target_deg, t.position_deg);
            break;
        case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
            pos = wrap_error_deg(encoder, observer);
            break;
        case DISP_POS_MODE_HALL_OBSERVER_ERROR:
            pos = wrap_error_deg(hall, observer);
            break;
        case DISP_POS_MODE_OBSERVER:
        case DISP_POS_MODE_INDUCTANCE:
        default:
            pos = observer;
        break;
    }


    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[5];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_ROTOR_POSITION;
    put_i32(p, &i, scaled_i32(pos, 100000.0f));
    vesc_comm_send_payload_low_priority(p, i);
}

// Fungsi reply_custom_summary: menyusun atau mengirim reply custom summary dengan pemeriksaan panjang buffer
// dan jalur transport yang aman.
static void reply_custom_summary(void) {
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_telemetry_t l, r;
    telemetry_get(MOTOR_LEFT, &l);
    telemetry_get(MOTOR_RIGHT, &r);
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_DUAL_SUMMARY;
    p[i++] = 5U;
    // Variabel arr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const motor_telemetry_t *arr[2] = {
        &l, &r
    }
    ;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 2U; k++) {
        // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const motor_telemetry_t *t = arr[k];
        p[i++] = k;
        p[i++] = t->controller_id;
        p[i++] = t->sensor_mode;
        p[i++] = t->fault;
        p[i++] = t->sensor_detect_state;
        put_i32(p, &i, scaled_i32(t->erpm, 1.0f));
        put_i32(p, &i, scaled_i32(t->iq, 100.0f));
        put_i32(p, &i, scaled_i32(t->id, 100.0f));
        put_i16(p, &i, scaled_i16(t->vbus, 10.0f));
    }
    put_u32(p, &i, l.isr_max_cycles);
    put_u32(p, &i, l.isr_overruns);
    p[i++] = l.calibration_done;
    p[i++] = l.calibration_valid;
    p[i++] = timeout_has_timeout() ? 1U : 0U;
    p[i++] = l.foc_sensor_mode;
    p[i++] = r.foc_sensor_mode;
    payload_end(i);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_extended: menyusun atau mengirim reply extended dengan pemeriksaan panjang buffer dan jalur
// transport yang aman.
static void reply_extended(motor_id_t id) {
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    motor_telemetry_t t;
    telemetry_get(id, &t);
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel cal_count: pencacah kejadian atau sampel.
    // Variabel cal_target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t cal_count = 0U, cal_target = 0U;
    foc_get_calibration_progress(&cal_count, &cal_target);
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_EXT_TELEMETRY;
    p[i++] = (uint8_t)id;
    p[i++] = 8U;
    p[i++] = t.controller_id;
    p[i++] = t.sensor_mode;
    p[i++] = t.fault;
    p[i++] = t.sensor_detect_state;
    p[i++] = t.calibration_done;
    p[i++] = t.calibration_valid;
    p[i++] = m->hall.raw_state;
    p[i++] = m->pole_pairs;
    p[i++] = m->encoder.inverted ? 1U : 0U;
    put_i32(p, &i, scaled_i32(t.phase_current_a, 1000.0f));
    put_i32(p, &i, scaled_i32(t.phase_current_b, 1000.0f));
    put_i32(p, &i, scaled_i32(t.phase_current_c, 1000.0f));
    put_i32(p, &i, scaled_i32(t.id, 1000.0f));
    put_i32(p, &i, scaled_i32(t.iq, 1000.0f));
    put_i32(p, &i, scaled_i32(t.id_filter, 1000.0f));
    put_i32(p, &i, scaled_i32(t.iq_filter, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vd, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vq, 1000.0f));
    put_i32(p, &i, scaled_i32(t.current_motor, 1000.0f));
    put_i32(p, &i, scaled_i32(t.current_in, 1000.0f));
    put_i32(p, &i, scaled_i32(t.erpm, 1.0f));
    put_i32(p, &i, scaled_i32(t.mech_rpm, 10.0f));
    put_i32(p, &i, scaled_i32(t.position_deg, 1000.0f));
    put_i32(p, &i, scaled_i32(t.rotor_elec_deg, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vbus, 1000.0f));
    put_i32(p, &i, scaled_i32(t.duty, 100000.0f));
    put_i32(p, &i, (int32_t)t.current_offset_u);
    put_i32(p, &i, (int32_t)t.current_offset_v);
    put_i32(p, &i, (int32_t)t.dc_current_offset);
    put_u32(p, &i, cal_count);
    put_u32(p, &i, cal_target);
    put_u32(p, &i, t.isr_max_cycles);
    put_u32(p, &i, t.isr_overruns);
    put_i32(p, &i, m->encoder.extended_count);
    put_u16(p, &i, m->encoder.elec_offset_u16);
    /* V22: phase A/B/C ditambahkan sebelum dq untuk audit arus fasa.
     * Observer/model fields tetap mengikuti blok yang sama setelah core FOC. */
    p[i++] = t.observer_valid;
    p[i++] = t.using_encoder;
    p[i++] = t.encoder_synced;
    put_i32(p, &i, scaled_i32(t.observer_phase_deg, 1000.0f));
    put_i32(p, &i, scaled_i32(t.observer_erpm, 1.0f));
    put_i32(p, &i, scaled_i32(t.observer_quality, 100000.0f));
    put_i32(p, &i, scaled_i32(t.foc_motor_r, 1000000.0f));
    put_i32(p, &i, scaled_i32(t.foc_motor_l, 1000000000.0f));
    put_i32(p, &i, scaled_i32(t.foc_motor_ld_lq_diff, 1000000000.0f));
    put_i32(p, &i, scaled_i32(t.foc_motor_flux_linkage, 10000000.0f));
    put_i32(p, &i, scaled_i32(t.foc_sl_erpm_start, 1.0f));
    put_i32(p, &i, scaled_i32(t.foc_sl_erpm, 1.0f));
    put_i32(p, &i, scaled_i32(t.foc_openloop_rpm, 1.0f));
    put_i32(p, &i, scaled_i32(t.foc_openloop_rpm_low, 1.0f));
    put_u32(p, &i, t.current_loop_hz);
    put_u32(p, &i, t.telemetry_snapshot_hz);
    p[i++] = t.foc_sensor_mode;
    /* Revisi 8 menambahkan referensi dq efektif dari snapshot ISR yang sama.
       Field lama tetap berada pada offset yang sama agar parser revisi <=7 aman. */
    put_i32(p, &i, scaled_i32(t.id_target, 1000.0f));
    put_i32(p, &i, scaled_i32(t.iq_target, 1000.0f));
    payload_end(i);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_sensor_info: menyusun atau mengirim reply sensor info dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_sensor_info(motor_id_t id) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_SENSOR_INFO;
    p[i++] = (uint8_t)id;
    p[i++] = controller_id_for_motor(id);
    p[i++] = m->sensor_mode;
    p[i++] = m->sensor_request_mode;
    p[i++] = (uint8_t)m->detect.state;
    p[i++] = m->detect.success ? 1U : 0U;
    p[i++] = m->pole_pairs;
    p[i++] = m->encoder.inverted ? 1U : 0U;
    put_u16(p, &i, m->encoder.elec_offset_u16);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 8U; k++)
        p[i++] = m->foc_hall_table[k];
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 8U; k++)
        put_u16(p, &i, m->hall_angle_u16[k]);

    /* Extended detect/current diagnostics. Legacy fields stay first.
       Revisi 17 menambahkan state startup sensorless/Hall untuk commissioning
       tanpa mengubah field revisi sebelumnya. */
    p[i++] = 17U;
    put_i32(p, &i, scaled_i32(m->detect.drive_current_a, 1000.0f));
    put_i32(p, &i, scaled_i32(m->id_target, 1000.0f));
    put_i32(p, &i, scaled_i32(m->iq_target, 1000.0f));
    put_u32(p, &i, m->detect.sweep_index);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 8U; k++)
        put_u32(p, &i, m->detect.hall_samples[k]);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 8U; k++)
        p[i++] = m->detect.result_hall_table[k];
    put_u16(p, &i, m->current_raw_u);
    put_u16(p, &i, m->current_raw_v);
    put_u16(p, &i, m->dc_current_raw);
    put_i32(p, &i, scaled_i32(m->ia, 1000.0f));
    put_i32(p, &i, scaled_i32(m->ib, 1000.0f));
    put_i32(p, &i, scaled_i32(m->ic, 1000.0f));
    p[i++] = m->pwm_enabled ? 1U : 0U;
    p[i++] = (m->pwm_tim->BDTR & TIM_BDTR_MOE) ? 1U : 0U;
    put_i32(p, &i, scaled_i32(m->current_scale, 1000000.0f));
    /* Synchronized-enable and stock-board sampling state. */
    p[i++] = m->pwm_enable_pending_events;
    put_u16(p, &i, m->pwm_enable_blank_cycles);
    put_u16(p, &i, (uint16_t)m->pwm_tim->CNT);
    put_u16(p, &i, (uint16_t)DMA1_Channel1->CNDTR);
    put_u16(p, &i, (uint16_t)TIM8->RCR);
    put_u16(p, &i, (uint16_t)ADC_MOTOR_PHASE_OFFSET_TICKS);
    p[i++] = (uint8_t)m->foc_sensor_mode;
    p[i++] = m->sensorless_start_failures;
    p[i++] = m->openloop_started ? 1U : 0U;
    p[i++] = m->phase_observer_override ? 1U : 0U;
    put_i32(p, &i, scaled_i32(m->openloop_erpm_now, 1.0f));
    p[i++] = m->observer_valid ? 1U : 0U;
    put_i32(p, &i, scaled_i32(m->observer_erpm, 1.0f));
    p[i++] = m->hall.raw_state;
    p[i++] = m->hall.valid ? 1U : 0U;
    p[i++] = (uint8_t)m->hall.direction;
    put_u16(p, &i, m->hall.invalid_count);
    put_u16(p, &i, m->hall.sequence_error_count);
    payload_end(i);
}

// Fungsi reply_current_cal: menyusun atau mengirim reply current cal dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_current_cal(void) {
    // Variabel count: pencacah kejadian atau sampel.
    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t count = 0U, target = 0U;
    foc_get_calibration_progress(&count, &target);
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_CURRENT_CAL;
    p[i++] = foc_calibration_done() ? 1U : 0U;
    p[i++] = foc_calibration_valid() ? 1U : 0U;
    put_u32(p, &i, count);
    put_u32(p, &i, target);
    put_i32(p, &i, g_motor_left.current_offset_u_counts);
    put_i32(p, &i, g_motor_left.current_offset_v_counts);
    put_i32(p, &i, g_motor_left.dc_current_offset_counts);
    put_i32(p, &i, g_motor_right.current_offset_u_counts);
    put_i32(p, &i, g_motor_right.current_offset_v_counts);
    put_i32(p, &i, g_motor_right.dc_current_offset_counts);
    /* Optional hardware-timing diagnostics. They are after the legacy
       current-cal fields so older host tools can ignore them safely. */
    put_u32(p, &i, foc_adc_isr_count());
    put_u16(p, &i, (uint16_t)DMA1_Channel1->CNDTR);
    put_u16(p, &i, (uint16_t)TIM2->CNT);
    p[i++] = (TIM1->CR1 & TIM_CR1_DIR) ? 1U : 0U;
    p[i++] = (TIM1->CR1 & TIM_CR1_CEN) ? 1U : 0U;
    p[i++] = (TIM8->CR1 & TIM_CR1_CEN) ? 1U : 0U;
    p[i++] = (TIM2->CR1 & TIM_CR1_CEN) ? 1U : 0U;
    p[i++] = (ADC1->CR2 & ADC_CR2_ADON) ? 1U : 0U;
    p[i++] = (ADC2->CR2 & ADC_CR2_ADON) ? 1U : 0U;
    p[i++] = ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U) ? 1U : 0U;

    /* Calibration diagnostics keep the established field prefix and only
       append new timing/synchronized-enable fields at the end. */
    // Variabel cd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_cal_diag_t cd;
    foc_get_calibration_diag(&cd);
    p[i++] = 19U; /* calibration diagnostic revision: unified 2000-frame midpoint */
    put_u16(p, &i, cd.warn_mask);
    put_u16(p, &i, cd.fail_range_mask);
    put_u16(p, &i, cd.fail_noise_mask);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 6U; k++) {
        put_i32(p, &i, cd.ch[k].mean);
        put_u16(p, &i, cd.ch[k].min);
        put_u16(p, &i, cd.ch[k].max);
        put_u32(p, &i, cd.ch[k].variance_x100);
    }
    /* Register snapshots retain legacy TIM2 plus active TIM8/ADC/DMA timing for one-file audits. */
    put_u32(p, &i, RCC->CFGR);
    put_u32(p, &i, ADC1->CR1);
    put_u32(p, &i, ADC1->CR2);
    put_u32(p, &i, ADC1->SQR1);
    put_u32(p, &i, ADC1->SQR3);
    put_u32(p, &i, ADC2->CR1);
    put_u32(p, &i, ADC2->CR2);
    put_u32(p, &i, ADC2->SQR1);
    put_u32(p, &i, ADC2->SQR3);
    put_u32(p, &i, DMA1_Channel1->CCR);
    put_u32(p, &i, DMA1_Channel1->CNDTR);
    put_u32(p, &i, DMA1->ISR);
    put_u32(p, &i, TIM1->CR1);
    put_u32(p, &i, TIM1->ARR);
    put_u32(p, &i, TIM1->CNT);
    put_u32(p, &i, TIM1->BDTR);
    put_u32(p, &i, TIM8->CR1);
    put_u32(p, &i, TIM8->ARR);
    put_u32(p, &i, TIM8->CNT);
    put_u32(p, &i, TIM8->BDTR);
    put_u32(p, &i, TIM2->CR1);
    put_u32(p, &i, TIM2->SMCR);
    put_u32(p, &i, TIM2->CCR2);
    put_u32(p, &i, TIM2->CNT);
    /* Keep the six-word diagnostic wire ABI from Run31, but report the newest
       complete V15-format frame from the 3-frame DMA batch. Word 5 is zero
       because Run35 intentionally has no rank-6 filler. */
    const uint32_t adc_diag_base = ADC_WORDS_PER_FRAME * (ADC_DMA_BATCH_FRAMES - 1U);
    for (uint8_t k = 0U; k < 5U; k++)
        put_u32(p, &i, g_adc_dual_dma[adc_diag_base + k]);
    put_u32(p, &i, 0U);

    /* VESC-style driven/undriven current-offset diagnostics plus the
       first active-drive over-current snapshot. Appended after fields so
       older debug clients still parse the legacy prefix. */
    p[i++] = (uint8_t)cd.stage;
    put_u16(p, &i, cd.shift_warn_mask);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 6U; k++)
        put_i32(p, &i, cd.undriven_mean[k]);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 6U; k++)
        put_i32(p, &i, cd.driven_mean[k]);

    // Variabel fs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_fault_snapshot_t fs;
    foc_get_fault_snapshot(&fs);
    p[i++] = fs.valid;
    p[i++] = fs.motor;
    p[i++] = fs.fault;
    p[i++] = fs.cal_stage;
    put_u16(p, &i, fs.raw_u);
    put_u16(p, &i, fs.raw_v);
    put_u16(p, &i, fs.raw_dc);
    put_i32(p, &i, fs.offset_u);
    put_i32(p, &i, fs.offset_v);
    put_i32(p, &i, fs.offset_dc);
    put_i32(p, &i, fs.ia_q15);
    put_i32(p, &i, fs.ib_q15);
    put_i32(p, &i, fs.ic_q15);
    put_i32(p, &i, fs.trip_q15);
    put_i32(p, &i, fs.id_target_q15);
    put_i32(p, &i, fs.iq_target_q15);
    put_u16(p, &i, fs.ccr1);
    put_u16(p, &i, fs.ccr2);
    put_u16(p, &i, fs.ccr3);
    put_u16(p, &i, fs.tim_cnt);
    put_u16(p, &i, fs.dma_cndtr);
    put_u32(p, &i, fs.adc_isr_count);

    /* Fault edge state and immutable ADC/PWM schedule metadata. */
    put_u16(p, &i, fs.blank_cycles);
    p[i++] = fs.pwm_enabled;
    p[i++] = fs.moe;
    p[i++] = fs.pending_events;
    p[i++] = fs.reserved;
    put_u16(p, &i, (uint16_t)ADC_MOTOR_PHASE_OFFSET_TICKS);
    put_u32(p, &i, FOC_ISR_EVENT_HZ);
    put_u32(p, &i, FOC_ISR_SLOT_CYCLES);
    put_u16(p, &i, (uint16_t)TIM8->RCR);
    put_u16(p, &i, (uint16_t)TIM1->CNT);
    put_u16(p, &i, (uint16_t)TIM8->CNT);

    /* Revision-16 slots dipertahankan byte-for-byte agar debug.py lama tetap
       dapat parsing. Run31 menghapus ADC3/DMA2 dari jalur motor, sehingga
       seluruh field legacy ini sengaja nol dan tidak dereference peripheral. */
    p[i++] = 0U; /* legacy adc3_enabled */
    p[i++] = 0U; /* legacy dma2_ch5_enabled */
    put_u16(p, &i, 0U); /* legacy dma2_ch5_cndtr */
    put_u16(p, &i, 0U); /* legacy ADC3 slot: Run31 tidak memakai ADC3 */
    put_u16(p, &i, 0U); /* legacy ADC3 slot */
    p[i++] = foc_vbus_dma_stale_count();
    p[i++] = 0U;
    put_u32(p, &i, foc_vbus_dma_stale_events());
    put_u32(p, &i, 0U); /* legacy ADC3 CR1 */
    put_u32(p, &i, 0U); /* legacy ADC3 CR2 */
    put_u32(p, &i, 0U); /* legacy ADC3 SQR1 */
    put_u32(p, &i, 0U); /* legacy ADC3 SQR3 */
    put_u32(p, &i, 0U); /* legacy DMA2_CH5 CCR */
    put_u32(p, &i, 0U); /* legacy DMA2_CH5 CNDTR */
    put_u32(p, &i, DMA2->ISR);

    /* Revision 17: robust driven-sample filtering and explicit MOE timing.
       Raw min/max above remain untouched so isolated switching spikes are
       visible, while these counters show whether enough clean samples were
       available for each current offset. */
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 6U; k++) {
        put_u16(p, &i, cd.outlier_count[k]);
    }
    p[i++] = cd.moe_fail_mask;
    p[i++] = cd.moe_confirmed_mask;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 2U; k++) {
        put_u32(p, &i, cd.moe_request_adc[k]);
    }
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 2U; k++) {
        put_u32(p, &i, cd.moe_confirm_adc[k]);
    }
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 2U; k++) {
        put_u32(p, &i, cd.first_sample_adc[k]);
    }
    /* Revision 18: expose power-stage fault latch / PVD status so a stuck
     * MOE handshake can be attributed to a hardware latch rather than a
     * software calibration fault. Appended strictly at the end. */
    put_u32(p, &i, motor_hw_powerstage_fault_flags());
    p[i++] = motor_hw_pvd_low() ? 1U : 0U;
    payload_end(i);
}

// Parameter last_save_ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi reply_config_status: menyusun atau mengirim reply config status dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_config_status(bool last_save_ok) {
    /* Legacy payload already exceeded 16 bytes (21 bytes before Stage2), so
       keep explicit headroom and append boot-state without stack overwrite. */
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[32];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_CONFIG_STATUS;
    p[i++] = conf_general_is_valid() ? 1U : 0U;
    p[i++] = last_save_ok ? 1U : 0U;
    put_u32(p, &i, conf_general_get_save_count());
    put_u32(p, &i, timeout_get_timeout_ms());
    p[i++] = conf_general_integrity_ok() ? 1U : 0U;
    put_u32(p, &i, conf_general_get_integrity_checks());
    put_u32(p, &i, conf_general_get_integrity_failures());
    p[i++] = (uint8_t)conf_general_boot_status();
    vesc_comm_send_payload(p, i);
}

// Parameter action: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter accepted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi reply_buzzer_status: menyusun atau mengirim reply buzzer status dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_buzzer_status(uint8_t action, bool accepted) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_BUZZER_TEST;
    p[i++] = action;
    p[i++] = accepted ? 1U : 0U;
    p[i++] = hw_status_tone_is_running() ? 1U : 0U;
    p[i++] = g_vesc_startup_melody_active ? 1U : 0U;
    p[i++] = g_vesc_startup_melody_index;
    put_u16(p, &i, g_vesc_buzzer_hz);
    put_u32(p, &i, g_vesc_buzzer_remaining);
    payload_end(i);
}

/* Proses command diagnostik/custom VESC. Operasi yang dapat memblokir seperti
 * sensor detect selalu dialihkan ke worker agar parser USART tetap responsif. */
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter context: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi process_custom: memproses process custom setelah input divalidasi lalu memperbarui state atau output
// sesuai aturan modul.
static void process_custom(const uint8_t *data, uint16_t len, motor_id_t context) {
    /* Upstream COMMANDS_APP_DATA callback semantics: `data` starts at the
     * application payload, i.e. COMM_CUSTOM_APP_DATA itself is stripped. */
    if (data == NULL || len < 1U)
        return;
    // Variabel sub: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t sub = data[0];
    // Variabel explicit_id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    motor_id_t explicit_id = context;
    if (len >= 2U && data[1] <= 1U)
        explicit_id = data[1] ? MOTOR_RIGHT : MOTOR_LEFT;

    if (sub == CUSTOM_SELECT_MOTOR) {
        /* Pemilihan eksplisit untuk tool diagnostik custom. UART standar tetap
         * motor-1; motor-2 dapat dicapai melalui local dual-motor forwarding. */
        reply_sensor_info(explicit_id);
    }
    else if (sub == CUSTOM_DUAL_SUMMARY) {
        reply_custom_summary();
    }
    else if (sub == CUSTOM_CLEAR_FAULT && len >= 2U) {
        motor_clear_fault(motor_get(explicit_id));
    }
    else if (sub == CUSTOM_STOP && len >= 2U) {
        app_command_release(explicit_id, true);
    }
    else if ((sub == CUSTOM_SENSOR_SELECT || sub == CUSTOM_SENSOR_DETECT) && len >= 3U) {
        // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        MotorRuntime *m = motor_get(explicit_id);
        // Variabel mode: mode operasi yang menentukan jalur algoritma aktif.
        uint8_t mode = data[2];
        if (sub == CUSTOM_SENSOR_DETECT || mode == SENSOR_MODE_AUTO) {
            /* Detection is blocking in upstream VESC and must never run in the
             * UART packet thread. Queue it on the same comm_block worker used
             * by COMM_DETECT_ENCODER / COMM_DETECT_HALL_FOC. */
            // Variabel current: nilai arus untuk pengukuran, kendali, atau proteksi.
            float current = SENSOR_DETECT_CURRENT_A;
            if (len >= 7U)
                current = (float)get_i32_be(&data[3]) / 1000.0f;
            /* Job detect custom tetap masuk worker blocking. Parameternya hanya
             * 5 byte, jadi aman dibentuk di stack lalu disalin ke single-job
             * buffer melalui helper reservasi di bawah. */
            // Variabel params: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint8_t params[5];
            params[0] = mode;
            // Variabel ca: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int32_t ca = scaled_i32(current, 1000.0f);
            params[1] = (uint8_t)((uint32_t)ca >> 24);
            params[2] = (uint8_t)((uint32_t)ca >> 16);
            params[3] = (uint8_t)((uint32_t)ca >> 8);
            params[4] = (uint8_t)ca;
            if (!queue_blocking_raw(INTERNAL_CUSTOM_SENSOR_DETECT, m->id, params, sizeof(params))) {
                s_diag.blocking_busy_drops++;
                commands_send_print("VESC F103: detection worker busy.");
            }
        }
        else {
            if (motor_select_sensor_mode(m, mode)) {
                /* Explicit sensor selection is a configuration change, not a
                   temporary detect operation. Mirror it into the VESC wire
                   image and EEPROM emulation immediately so Sensorless/Hall/Encoder
                   survives a cold boot exactly like COMM_SET_MCCONF. */
                if (!vesc_config_commit_motor_runtime(m->id)) {
                    commands_send_print("VESC F103: sensor config persistence failed; previous MCCONF restored.");
                }
            }
        }
        reply_sensor_info(m->id);
    }
    else if (sub == CUSTOM_CURRENT_CAL) {
        if (len >= 2U && data[1] == 1U) {
            motor_stop(&g_motor_left);
            motor_stop(&g_motor_right);
            foc_request_recalibration();
        }
        reply_current_cal();
    }
    else if (sub == CUSTOM_SAMPLE_START && len >= 6U) {
        // Variabel count: pencacah kejadian atau sampel.
        uint16_t count = get_u16_be(&data[2]);
        // Variabel decimation: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t decimation = get_u16_be(&data[4]);
        mc_interface_sample_start(explicit_id, count, decimation);
    }
    else if (sub == CUSTOM_EXT_TELEMETRY && len >= 2U) {
        reply_extended(explicit_id);
    }
    else if (sub == CUSTOM_SENSOR_INFO && len >= 2U) {
        reply_sensor_info(explicit_id);
    }
    else if (sub == CUSTOM_COMM_DIAG) {
        vesc_comm_reply_diag();
    }
    else if (sub == CUSTOM_BUZZER_TEST) {
        // Variabel action: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint8_t action = len >= 2U ? data[1] : 0U;
        // Variabel accepted: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        bool accepted = true;
        /* Kebijakan final buzzer: audio otomatis hanya power-on melody dan
         * fault-code. Command diagnostik tidak boleh membunyikan buzzer saat
         * idle/running/calibration/detect. Action 0 hanya query; action 3 boleh
         * mematikan tone sebagai emergency diagnostic. */
        if (action == 3U) {
            hw_status_tone_stop();
        }
        else if (action != 0U) {
            accepted = false;
        }
        reply_buzzer_status(action, accepted);
    }
    else if (sub == CUSTOM_OPENLOOP_PHASE && len >= 12U) {
        /* Wire: [F1][motor][duty_milli i32][phase_mdeg i32][duration_ms u16].
         * Bench diagnostic only. Bypasses current PI via
         * MOTOR_CTRL_OPENLOOP_DUTY_PHASE, clamps modulation to <=2%, and stops
         * automatically after a short hold. Used to verify phase-current raw
         * tracks the applied voltage angle without PI/observer/detect. */
        // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        MotorRuntime *m = motor_get(explicit_id);
        // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
        float duty = (float)get_i32_be(&data[2]) / 1000.0f;
        // Variabel phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
        float phase = (float)get_i32_be(&data[6]) / 1000.0f;
        // Variabel duration_ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t duration_ms = get_u16_be(&data[10]);
        if (duty < -0.02f)
            duty = -0.02f;
        if (duty > 0.02f)
            duty = 0.02f;
        if (duration_ms > 500U)
            duration_ms = 500U;
        if (duration_ms == 0U)
            duration_ms = 50U;
        mc_interface_ignore_input_both((uint32_t)duration_ms + 250U);
        motor_clear_fault(m);
        mcpwm_foc_set_openloop_duty_phase_motor(m, duty, phase);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        mcpwm_foc_release_motor_motor(m);
        mc_interface_ignore_input_both(0U);
        reply_extended(explicit_id);
    }
    else if (sub == CUSTOM_ADC_PHASE_OFFSET && len >= 3U) {
        /* Runtime tuning of TIM8/ADC phase offset (CPU timer ticks). Lets us
         * sweep the shunt current sampling point to find the low-side
         * conduction window without recompiling. Echoes back the applied value. */
        // Variabel ticks: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t ticks = (uint16_t)(data[1] | ((uint16_t)data[2] << 8));
        motor_hw_set_adc_phase_offset_ticks(ticks);
        // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint8_t *p = payload_begin();
        if (p != NULL) {
            // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
            uint16_t i = 0U;
            p[i++] = COMM_CUSTOM_APP_DATA;
            p[i++] = CUSTOM_ADC_PHASE_OFFSET;
            p[i++] = (uint8_t)(ticks & 0xFFU);
            p[i++] = (uint8_t)((ticks >> 8) & 0xFFU);
            payload_end(i);
        }
    }
    else if (sub == CUSTOM_CONFIG_SAVE) {
        // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        bool ok = conf_general_store_all();
        reply_config_status(ok);
    }
    else if (sub == CUSTOM_CONFIG_STATUS) {
        reply_config_status(false);
    }
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi is_blocking_command: menjalankan operasi is blocking command sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool is_blocking_command(uint8_t cmd) {
    switch (cmd) {
        case COMM_DETECT_MOTOR_PARAM:
        case COMM_DETECT_MOTOR_R_L:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_ENCODER:
        case COMM_DETECT_HALL_FOC:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
        case COMM_DETECT_APPLY_ALL_FOC:
        case COMM_SET_MCCONF:
        case COMM_SET_APPCONF:
        case COMM_SET_APPCONF_NO_STORE:
        case COMM_TERMINAL_CMD:
        case COMM_SET_MCCONF_TEMP:
        case COMM_SET_MCCONF_TEMP_SETUP:
        case COMM_SET_BATTERY_CUT:
            return true;
        default:
            return false;
    }
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi blocking_command_length_valid: menjalankan operasi blocking command length valid sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static bool blocking_command_length_valid(uint8_t cmd, uint16_t len) {
    switch (cmd) {
        case COMM_SET_MCCONF:
            return len == (uint16_t)(1U + VESC6_MCCONF_WIRE_SIZE);
        case COMM_SET_APPCONF:
        case COMM_SET_APPCONF_NO_STORE:
            return len == (uint16_t)(1U + VESC6_APPCONF_WIRE_SIZE);
        case COMM_TERMINAL_CMD:
            return len >= 1U && len <= 241U;
        case COMM_SET_BATTERY_CUT:
            /* command + start/end int32(1e3) + store + forward-all flag */
            return len == 11U;
        case COMM_SET_MCCONF_TEMP:
        case COMM_SET_MCCONF_TEMP_SETUP:
            /* command byte + four flag bytes + eight float32_auto values.
             * Two optional input-current values may follow. */
            return len >= 37U && len <= 45U;
        case COMM_DETECT_MOTOR_R_L:
            return len == 1U;
        case COMM_DETECT_ENCODER:
        case COMM_DETECT_HALL_FOC:
            return len == 5U;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
            return len == 17U;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
            return len == 21U;
        case COMM_DETECT_APPLY_ALL_FOC:
            return len == 22U;
        case COMM_DETECT_MOTOR_PARAM:
            /* Legacy BLDC detect is unsupported here; accept its command byte
               so the worker can return the explicit unsupported response. */
            return len >= 1U;
        default:
            return true;
    }
}

/* Reservasi single blocking-job. Critical section hanya check/set satu flag;
 * copy payload dilakukan sesudahnya dan worker belum dapat berjalan sebelum
 * token dikirim. Selama s_block_busy=true producer berikutnya ditolak agar
 * worker tidak pernah membaca job yang sedang ditimpa. */
// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi queue_blocking_raw: menjalankan operasi queue blocking raw sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool queue_blocking_raw(uint8_t cmd, motor_id_t id,
                               const uint8_t *data, uint16_t len) {
    if (s_block_queue == NULL || len > BLOCK_DATA_MAX || (len > 0U && data == NULL))
        return false;

    taskENTER_CRITICAL();
    if (s_block_busy) {
        taskEXIT_CRITICAL();
        return false;
    }
    s_block_busy = true;
    taskEXIT_CRITICAL();

    memset(&s_block_job, 0, sizeof(s_block_job));
    s_block_job.cmd = cmd;
    s_block_job.motor = (uint8_t)id;
    s_block_job.len = len;
    if (len > 0U)
        memcpy(s_block_job.data, data, len);
    __DMB();

    // Variabel token: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t token = 1U;
    if (xQueueSend(s_block_queue, &token, 0U) != pdTRUE) {
        taskENTER_CRITICAL();
        s_block_busy = false;
        taskEXIT_CRITICAL();
        return false;
    }
    return true;
}

/* Validasi payload command standar sebelum memindahkannya ke worker blocking. */
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi queue_blocking_job: menjalankan operasi queue blocking job sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool queue_blocking_job(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len == 0U || len > BLOCK_DATA_MAX)
        return false;
    if (!blocking_command_length_valid(data[0], len))
        return false;
    if (!queue_blocking_raw(data[0], id, data, len)) {
        s_diag.blocking_busy_drops++;
        return false;
    }
    return true;
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter defaults: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi reply_config_wire: menyusun atau mengirim reply config wire dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_config_wire(uint8_t cmd, motor_id_t id, bool defaults) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    p[0] = cmd;
    if (cmd == COMM_GET_MCCONF || cmd == COMM_GET_MCCONF_DEFAULT) {
        // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint8_t *w = vesc_config_mc_wire(id, defaults);
        memcpy(&p[1], w, VESC6_MCCONF_WIRE_SIZE);
        payload_end((uint16_t)(1U + VESC6_MCCONF_WIRE_SIZE));
    }
    else {
        // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint8_t *w = vesc_config_app_wire(defaults);
        memcpy(&p[1], w, VESC6_APPCONF_WIRE_SIZE);
        /* VESC dual-motor semantics expose the second motor with its own
           controller ID even though this port has one physical app instance.
           APPCONF signature occupies bytes 0..3 and controller_id is byte 4. */
        if (id == MOTOR_RIGHT)
            p[1U + 4U] = VESC_LOCAL_MOTOR2_FORWARD_ID;
        payload_end((uint16_t)(1U + VESC6_APPCONF_WIRE_SIZE));
    }
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_ack: menyusun atau mengirim reply ack dengan pemeriksaan panjang buffer dan jalur transport yang
// aman.
static void reply_ack(uint8_t cmd) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[1] = {
        cmd
    }
    ;
    vesc_comm_send_payload(p, 1U);
}

// Parameter hall_table: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Parameter ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_detect_hall_standard: menyusun atau mengirim reply detect hall standard dengan pemeriksaan
// panjang buffer dan jalur transport yang aman.
static void reply_detect_hall_standard(const uint8_t hall_table[8], bool ok) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_DETECT_HALL_FOC;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 8U; k++)
        p[i++] = (ok && hall_table != NULL) ? hall_table[k] : 255U;
    /* VESC COMM_DETECT_HALL_FOC appends the detector boolean result:
       true/success is 1, false/failure is 0. */
    p[i++] = ok ? 1U : 0U;
    payload_end(i);
}

// Parameter offset_deg: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_detect_encoder_standard: menyusun atau mengirim reply detect encoder standard dengan pemeriksaan
// panjang buffer dan jalur transport yang aman.
static void reply_detect_encoder_standard(float offset_deg, float ratio,
                                          bool inverted, bool ok) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_DETECT_ENCODER;
    if (ok && isfinite(offset_deg) && isfinite(ratio) && ratio > 0.0f) {
        put_i32(p, &i, scaled_i32(offset_deg, 1000000.0f));
        put_i32(p, &i, scaled_i32(ratio, 1000000.0f));
        p[i++] = inverted ? 1U : 0U;
    }
    else {
        /* VESC encoder-detect failure sentinel: offset > 1000 degrees. */
        put_i32(p, &i, 1001000000L);
        put_i32(p, &i, 0);
        p[i++] = 0U;
    }
    payload_end(i);
}


// Parameter r_ohm: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter l_h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_detect_rl: menyusun atau mengirim reply detect rl dengan pemeriksaan panjang buffer dan jalur
// transport yang aman.
static void reply_detect_rl(float r_ohm, float l_h, float ld_lq_h, bool ok) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[13];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_DETECT_MOTOR_R_L;
    /* Current VESC Tool decodes R at 1e6 and L/Ld-Lq at 1e3, with L values
       expressed in microhenry on this command. */
    put_i32(p, &i, ok ? scaled_i32(r_ohm, 1.0e6f) : 0);
    put_i32(p, &i, ok ? scaled_i32(l_h*1.0e6f, 1.0e3f) : 0);
    put_i32(p, &i, ok ? scaled_i32(ld_lq_h*1.0e6f, 1.0e3f) : 0);
    vesc_comm_send_payload(p, i);
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter flux_wb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_detect_flux: menyusun atau mengirim reply detect flux dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_detect_flux(uint8_t cmd, float flux_wb, bool ok) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[5];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = cmd;
    put_i32(p, &i, ok ? scaled_i32(flux_wb, 1.0e7f) : 0);
    vesc_comm_send_payload(p, i);
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_unsupported_detect: menyusun atau mengirim reply unsupported detect dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
static void reply_unsupported_detect(uint8_t cmd);

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter run_ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi reply_detect_motor_param_bldc: menyusun atau mengirim reply detect motor param bldc dengan pemeriksaan
// panjang buffer dan jalur transport yang aman.
static void reply_detect_motor_param_bldc(MotorRuntime *m, bool run_ok) {
    (void)m;
    (void)run_ok;
    reply_unsupported_detect(COMM_DETECT_MOTOR_PARAM);
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi reply_unsupported_detect: menyusun atau mengirim reply unsupported detect dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
static void reply_unsupported_detect(uint8_t cmd) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[32];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = cmd;
    switch (cmd) {
        case COMM_DETECT_MOTOR_PARAM:
            put_i32(p, &i, 0);
            put_i32(p, &i, 0);
            // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
            for (uint8_t k = 0U; k < 8U; k++)
                p[i++] = 255U;
            p[i++] = 1U;
            break;
        case COMM_DETECT_MOTOR_R_L:
            put_i32(p, &i, 0);
            put_i32(p, &i, 0);
            put_i32(p, &i, 0);
            break;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
            put_i32(p, &i, 0);
            break;
        case COMM_DETECT_APPLY_ALL_FOC:
            put_i16(p, &i, -1);
            break;
        default:
            break;
    }
    vesc_comm_send_payload(p, i);
    commands_send_print("VESC F103: legacy BLDC motor-param detect is unsupported; use FOC R/L/flux detect.");
}

// Parameter timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi ensure_current_calibration_valid: menangani kalibrasi ensure current calibration valid agar offset
// atau parameter hasil ukur valid sebelum dipakai kendali.
static bool ensure_current_calibration_valid(uint32_t timeout_ms) {
    /* VESC detect-all performs DC offset calibration before reading/applying
       motor parameters. Do the same for every motor-moving detect job. */
    if (foc_calibration_valid())
        return true;
    if (foc_calibration_done())
        foc_request_recalibration();
    // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t start = xTaskGetTickCount();
    while ((uint32_t)(xTaskGetTickCount() - start) < timeout_ms) {
        if (foc_calibration_done())
            return foc_calibration_valid();
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
    return false;
}

// Parameter timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi force_current_calibration_valid: menangani kalibrasi force current calibration valid agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
static bool force_current_calibration_valid(uint32_t timeout_ms) {
    /* Full Detect-All deliberately takes a fresh six-channel offset snapshot.
       The shared ADC calibration covers LEFT/RIGHT phase and DC-current paths,
       including the driven zero-vector stages. Wait for the new transaction,
       not the previous s_cal_done result. */
    motor_stop(&g_motor_left);
    motor_stop(&g_motor_right);
    foc_request_recalibration();
    // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t start = xTaskGetTickCount();
    // Variabel saw_active: penanda bahwa state atau fitur sedang aktif.
    bool saw_active = false;
    while ((uint32_t)(xTaskGetTickCount() - start) < timeout_ms) {
        if (!foc_calibration_done())
            saw_active = true;
        if (saw_active && foc_calibration_done())
            return foc_calibration_valid();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
    return false;
}


// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter table: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi apply_hall_detect_result: menjalankan deteksi apply hall detect result dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
static void apply_hall_detect_result(MotorRuntime *m, const uint8_t table[8]) {
    if (m == NULL || !motor_apply_foc_hall_table(m, table))
        return;

    motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
    m->sensor_mode = SENSOR_MODE_HALL;
    m->sensor_request_mode = SENSOR_MODE_HALL;
    m->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
    m->stats.tachometer_source_valid = false;
    foc_sensorless_startup_abort(m);
    motor_hall_edge_isr(m);
    if (m->hall.valid)
        foc_observer_reset(m, m->hall_angle_u16[m->hall.raw_state & 7U]);
}


// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter offset_deg: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi apply_encoder_detect_result: menjalankan deteksi apply encoder detect result dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
static void apply_encoder_detect_result(MotorRuntime *m, float offset_deg,
                                        float ratio, bool inverted) {
    if (!m || m->id != MOTOR_LEFT || !isfinite(ratio) || ratio < 1.0f)
        return;
    motor_hw_configure_sensor(m, SENSOR_MODE_ENCODER);
    m->encoder.electrical_ratio = ratio;
    m->encoder.electrical_ratio_q16 = (uint32_t)lroundf(ratio * 65536.0f);
    m->encoder.elec_offset_u16 = foc_deg_to_u16(offset_deg);
    m->encoder.inverted = inverted;
    if (m->encoder.cpr > 0U) {
        m->encoder.phase_per_count_q16 =
            (uint32_t)(((uint64_t)m->encoder.electrical_ratio_q16 << 16) /
                       (uint64_t)m->encoder.cpr);
    }
    /* A/B is incremental: applying persistent ratio/offset does not make the
       cold-boot runtime counter absolute. Normal ENCODER_AB startup must sync
       it from the observer before encoder low-speed control is used. */
    m->encoder.synced = false;
    m->encoder.motion_proved = false;
    m->encoder.sync_active = false;
    m->using_encoder = false;
    m->sensor_mode = SENSOR_MODE_ENCODER;
    m->sensor_request_mode = SENSOR_MODE_ENCODER;
    m->foc_sensor_mode = FOC_SENSOR_MODE_ENCODER_AB;
    m->stats.tachometer_source_valid = false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi apply_sensorless_result: menjalankan operasi apply sensorless result sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static void apply_sensorless_result(MotorRuntime *m) {
    if (!m)
        return;
    /* Hasil sensorless harus benar-benar bebas sensor fisik. Lepaskan encoder
       LEFT bila aktif dan mask EXTI Hall pada kedua motor; fase listrik setelah
       forced-openloop tetap dimiliki observer FOC. */
    if (m->id == MOTOR_LEFT)
        encoder_deinit(m);
    motor_hw_configure_sensor(m, SENSOR_MODE_NO_SENSOR);
    m->sensor_mode = SENSOR_MODE_NO_SENSOR;
    m->sensor_request_mode = SENSOR_MODE_NO_SENSOR;
    m->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    m->using_encoder = false;
    m->stats.tachometer_source_valid = false;
    foc_sensorless_startup_abort(m);
    foc_observer_reset(m, m->observer_phase_u16);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Fungsi detect_failure_is_sensor_absent: menjalankan deteksi detect failure is sensor absent dengan proteksi
// motor dan memvalidasi hasil sebelum parameter diterapkan.
static bool detect_failure_is_sensor_absent(MotorRuntime *m, int fault) {
    return m != NULL && m->fault == MOTOR_FAULT_NONE &&
           fault == MOTOR_FAULT_SENSOR_DETECT;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter detect_current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi validate_sensorless_runtime: menjalankan operasi validate sensorless runtime sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static bool validate_sensorless_runtime(MotorRuntime *m, float detect_current_a) {
    if (m == NULL || m->fault != MOTOR_FAULT_NONE ||
        m->foc_sensor_mode != FOC_SENSOR_MODE_SENSORLESS)
        return false;

    /* Sensorless fallback is a real detected result only after the observer
       proves that it can acquire the motor. The motor-service task owns the
       VESC-style forced-startup/blend state machine; this blocking worker only
       applies a small positive-current request and watches the resulting
       observer state. Ordinary APP ADC/UART inputs are already masked by the
       Detect-All input-ignore window. */
    // Variabel test_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float test_current = fabsf(detect_current_a);
    // Variabel current_cap: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_cap = fmaxf(fminf(fabsf(m->current_max_a) * 0.25f, 5.0f), 0.5f);
    if (test_current > current_cap)
        test_current = current_cap;
    if (test_current < 0.5f)
        test_current = 0.5f;

    m->sensorless_start_failures = 0U;
    foc_sensorless_startup_abort(m);
    foc_observer_reset(m, m->observer_phase_u16);
    motor_set_current(m, test_current);

    // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t start = xTaskGetTickCount();
    // Variabel stable_ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t stable_ms = 0U;
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = false;
    while ((uint32_t)(xTaskGetTickCount() - start) < 8000U) {
        if (m->fault != MOTOR_FAULT_NONE || m->sensorless_start_failures >= 3U)
            break;
        // Variabel sp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sp = m->speed_est_fast_erpm_q16;
        if (sp < 0)
            sp = (sp == INT32_MIN) ? INT32_MAX : -sp;
        // Variabel acquired: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        /* Startup pure sensorless sendiri baru melepas override setelah
           threshold foc_openloop_rpm/foc_openloop_rpm_low dan phase-coherence
           terpenuhi. Jangan pakai foc_sl_erpm di sini karena itu threshold
           hybrid sensored dan default-nya lebih tinggi dari openloop startup. */
        const bool acquired = m->observer_valid &&
                              sp >= (50 * 65536) &&
                              !m->phase_observer_override &&
                              !m->openloop_started;
        if (acquired) {
            stable_ms += 10U;
            if (stable_ms >= 150U) {
                ok = true;
                break;
            }
        }
        else {
            stable_ms = 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    motor_stop(m);
    foc_sensorless_startup_abort(m);
    if (!ok && m->fault == MOTOR_FAULT_NONE) {
        commands_send_print("Detect-All: sensorless observer failed acquisition validation; not saving fallback.");
    }
    return ok;
}


// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter max_power_loss: batas atau nilai maksimum untuk validasi dan proteksi.
// Parameter min_input_current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter max_input_current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter openloop: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter sl: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_apply_all_one_runtime: menjalankan deteksi detect apply all one runtime dengan proteksi motor
// dan memvalidasi hasil sebelum parameter diterapkan.
static int16_t detect_apply_all_one_runtime(MotorRuntime *m, float max_power_loss,
                                            float min_input_current, float max_input_current,
                                            float openloop, float sl) {
    if (m == NULL)
        return -1;

    m->foc_openloop_rpm = fabsf(openloop);
    m->foc_sl_erpm = fabsf(sl);

    // Variabel detect_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float detect_current = fabsf(m->current_max_a) / 3.0f;
    if (detect_current < 1.0f)
        detect_current = FOC_DETECT_CURRENT_A;
    if (detect_current > FOC_DETECT_MAX_CURRENT_A)
        detect_current = FOC_DETECT_MAX_CURRENT_A;

    // Variabel result: hasil sementara atau akhir suatu operasi.
    int16_t result = mcpwm_foc_detect_apply_all_motor(m, detect_current);
    if (result != 0)
        return result;

    /* Match VESC Detect-All intent: derive symmetric motor-current capability
       from measured copper resistance and requested maximum copper loss, while
       never exceeding the board/current-protection envelope. */
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = m->foc_motor_r;
    if (!isfinite(r) || r <= 0.00001f)
        return -2;
    // Variabel current_limit: nilai arus untuk pengukuran, kendali, atau proteksi.
    float current_limit = sqrtf(max_power_loss / r);
    // Variabel hard_limit: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float hard_limit = fminf(FOC_MAX_CURRENT_A, m->abs_current_max_a);
    if (!isfinite(current_limit) || current_limit < 0.1f)
        return -3;
    current_limit = fminf(current_limit, hard_limit);
    m->current_max_a = current_limit;
    m->current_min_a = -current_limit;
    m->input_current_min_a = fmaxf(min_input_current, -FOC_MAX_CURRENT_A);
    m->input_current_max_a = fminf(max_input_current, FOC_MAX_CURRENT_A);

    /* VESC-style sensor discovery is independent of the sensor that happened
       to be selected before Detect-All. Physical sensor absence is not a motor
       fault: LEFT tries ABI -> Hall -> sensorless; RIGHT tries Hall ->
       sensorless. A real latched motor/power-stage fault still aborts. */
    if (m->id == MOTOR_LEFT) {
        // Variabel off: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel rat: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float off = 0.0f, rat = 0.0f;
        // Variabel inv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        bool inv = false;
        // Variabel sf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int sf = mcpwm_foc_encoder_detect_motor(m, SENSOR_DETECT_CURRENT_A, false,
                                                &off, &rat, &inv);
        if (sf == MOTOR_FAULT_NONE) {
            apply_encoder_detect_result(m, off, rat, inv);
            return 0;
        }
        if (!detect_failure_is_sensor_absent(m, sf))
            return -11;
    }

    // Variabel hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint8_t hall[8];
    // Variabel hall_ok: data sensor Hall untuk menentukan sektor atau posisi rotor.
    bool hall_ok = false;
    memset(hall, 255, sizeof(hall));
    // Variabel hf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int hf = mcpwm_foc_hall_detect_motor(m, SENSOR_DETECT_CURRENT_A, hall, &hall_ok);
    if (hf == MOTOR_FAULT_NONE && hall_ok) {
        apply_hall_detect_result(m, hall);
        return 0;
    }
    if (!detect_failure_is_sensor_absent(m, hf))
        return -11;

    apply_sensorless_result(m);
    if (!validate_sensorless_runtime(m, detect_current))
        return -13;
    return 0;
}

// Fungsi rollback_detect_all_runtime_both: menjalankan deteksi rollback detect all runtime both dengan proteksi
// motor dan memvalidasi hasil sebelum parameter diterapkan.
static void rollback_detect_all_runtime_both(void) {
    (void)vesc_config_reapply_active_mc(MOTOR_LEFT);
    (void)vesc_config_reapply_active_mc(MOTOR_RIGHT);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi battery_cut_build_wire: menjalankan operasi battery cut build wire sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static bool battery_cut_build_wire(motor_id_t id, float start, float end, uint8_t out[VESC6_MCCONF_WIRE_SIZE]) {
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    mc_configuration c;
    if (!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), &c))
        return false;
    c.l_battery_cut_start = start;
    c.l_battery_cut_end = end;
    return confgenerator_serialize_mcconf_motor(out, &c, id) == (int32_t)VESC6_MCCONF_WIRE_SIZE;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi battery_cut_apply_one: menjalankan operasi battery cut apply one sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static bool battery_cut_apply_one(motor_id_t id, float start, float end, bool store) {
    if (!battery_cut_build_wire(id, start, end, s_mc_work))
        return false;
    return vesc_config_set_mc_wire(id, s_mc_work, VESC6_MCCONF_WIRE_SIZE, store);
}

// Parameter start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi battery_cut_apply_both: menjalankan operasi battery cut apply both sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static bool battery_cut_apply_both(float start, float end, bool store) {
    memcpy(s_mc_backup[MOTOR_LEFT], vesc_config_mc_wire(MOTOR_LEFT, false), VESC6_MCCONF_WIRE_SIZE);
    memcpy(s_mc_backup[MOTOR_RIGHT], vesc_config_mc_wire(MOTOR_RIGHT, false), VESC6_MCCONF_WIRE_SIZE);
    if (!battery_cut_build_wire(MOTOR_LEFT, start, end, s_mc_work) ||
        !vesc_config_set_mc_wire(MOTOR_LEFT, s_mc_work, VESC6_MCCONF_WIRE_SIZE, false))
        return false;
    if (!battery_cut_build_wire(MOTOR_RIGHT, start, end, s_mc_work) ||
        !vesc_config_set_mc_wire(MOTOR_RIGHT, s_mc_work, VESC6_MCCONF_WIRE_SIZE, false)) {
        (void)vesc_config_set_mc_wire(MOTOR_LEFT, s_mc_backup[MOTOR_LEFT], VESC6_MCCONF_WIRE_SIZE, false);
        return false;
    }
    if (store) {
        if (!conf_general_store_all()) {
            (void)vesc_config_set_mc_wire(MOTOR_LEFT, s_mc_backup[MOTOR_LEFT], VESC6_MCCONF_WIRE_SIZE, false);
            (void)vesc_config_set_mc_wire(MOTOR_RIGHT, s_mc_backup[MOTOR_RIGHT], VESC6_MCCONF_WIRE_SIZE, false);
            return false;
        }
    }
    return true;
}

/* Worker tunggal untuk detect dan penulisan konfigurasi yang dapat memblokir.
 * Single-job buffer dijaga busy flag sehingga payload tidak dapat ditimpa. */
// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi blocking_thread: menjalankan operasi blocking thread sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void blocking_thread(void *argument) {
    (void)argument;
    for (;; ) {
        // Variabel token: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint8_t token = 0U;
        if (xQueueReceive(s_block_queue, &token, portMAX_DELAY) != pdTRUE)
            continue;
        (void)token;
        __DMB();
        // Variabel job: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        blocking_job_t *job = &s_block_job;
        // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        MotorRuntime *m = motor_get(job->motor == MOTOR_RIGHT ? MOTOR_RIGHT : MOTOR_LEFT);
        // Variabel old_motor: state atau parameter motor yang sedang diproses.
        int old_motor = mc_interface_get_motor_thread();
        mc_interface_select_motor_thread(m->id == MOTOR_RIGHT ? 2 : 1);
        if (job->cmd == COMM_SET_MCCONF) {
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = job->len == (1U + VESC6_MCCONF_WIRE_SIZE) &&
                      vesc_config_set_mc_wire(m->id, &job->data[1],
                                              (uint16_t)(job->len - 1U), true);
            if (ok)
                reply_ack(COMM_SET_MCCONF);
            else commands_send_print("VESC F103: MCCONF rejected; motor must be OFF and VESC6 signature/layout valid.");
        }
        else if (job->cmd == COMM_SET_APPCONF || job->cmd == COMM_SET_APPCONF_NO_STORE) {
            // Variabel store: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const bool store = job->cmd == COMM_SET_APPCONF;
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = false;
            if (job->len == (1U + VESC6_APPCONF_WIRE_SIZE)) {
                /* Match HW_HAS_DUAL_MOTORS upstream: APPCONF is shared. When
                   a packet is forwarded to motor-thread 2, ignore the public
                   controller-ID carried in that image and restore the local
                   primary ID before applying/storing the shared app config. */
                if (m->id == MOTOR_RIGHT) {
                    job->data[1U + VESC6_APP_OFF_CONTROLLER_ID] = VESC_CONTROLLER_ID_LEFT;
                }
                ok = vesc_config_set_app_wire(&job->data[1], VESC6_APPCONF_WIRE_SIZE, store);
            }
            if (ok)
                reply_ack(job->cmd);
            else commands_send_print("VESC F103: APPCONF rejected; VESC6 signature/layout invalid.");
        }
        else if (job->cmd == COMM_TERMINAL_CMD) {
            process_terminal_text(&job->data[1], (uint16_t)(job->len > 0U ? job->len - 1U : 0U), m->id);
        }
        else if (job->cmd == COMM_SET_MCCONF_TEMP || job->cmd == COMM_SET_MCCONF_TEMP_SETUP) {
            set_mcconf_temp(&job->data[1], (uint16_t)(job->len - 1U), m->id,
                            job->cmd == COMM_SET_MCCONF_TEMP_SETUP);
        }
        else if (job->cmd == COMM_SET_BATTERY_CUT) {
            // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float start = (float)get_i32_be(&job->data[1]) / 1000.0f;
            // Variabel end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float end = (float)get_i32_be(&job->data[5]) / 1000.0f;
            // Variabel store: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool store = job->data[9] != 0U;
            // Variabel forward_all: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool forward_all = job->data[10] != 0U;
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = (m->id == MOTOR_LEFT && forward_all) ?
                      battery_cut_apply_both(start, end, store) :
                      battery_cut_apply_one(m->id, start, end, store);
            if (ok)
                reply_ack(COMM_SET_BATTERY_CUT);
            else commands_send_print("VESC F103: battery-cut update rejected or persistence failed.");
        }
        else if (job->cmd == COMM_DETECT_MOTOR_PARAM) {
            /* Legacy six-step BLDC detector is intentionally unavailable in
             * this FOC-only build. Return the canonical unsupported payload;
             * all FOC detection commands below remain fully active. */
            reply_detect_motor_param_bldc(m, false);
        }
        else if (job->cmd == COMM_DETECT_ENCODER) {
            // Variabel current: nilai arus untuk pengukuran, kendali, atau proteksi.
            float current = (float)get_i32_be(&job->data[1]) / 1000.0f;
            // Variabel off: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            // Variabel rat: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float off = 0.0f, rat = 0.0f;
            // Variabel inv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool inv = false;
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = ensure_current_calibration_valid(10000U) &&
                      mcpwm_foc_encoder_detect_motor(m, fabsf(current), false,
                                               &off, &rat, &inv) == MOTOR_FAULT_NONE;
            /* Never source the reply from m->detect: if current calibration
               fails before detect_begin(), that structure may contain a
               successful result from an older transaction. */
            reply_detect_encoder_standard(off, rat, inv, ok);
        }
        else if (job->cmd == COMM_DETECT_HALL_FOC) {
            // Variabel current: nilai arus untuk pengukuran, kendali, atau proteksi.
            float current = (float)get_i32_be(&job->data[1]) / 1000.0f;
            // Variabel hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
            uint8_t hall[8];
            memset(hall, 255, sizeof(hall));
            // Variabel hall_ok: data sensor Hall untuk menentukan sektor atau posisi rotor.
            bool hall_ok = false;
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = ensure_current_calibration_valid(10000U) &&
                      mcpwm_foc_hall_detect_motor(m, fabsf(current), hall, &hall_ok) == MOTOR_FAULT_NONE &&
                      hall_ok;
            reply_detect_hall_standard(hall, ok);
        }
        else if (job->cmd == COMM_DETECT_MOTOR_R_L) {
            // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            // Variabel ldq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float r = 0.0f, l = 0.0f, ldq = 0.0f;
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = ensure_current_calibration_valid(10000U) &&
                      mcpwm_foc_measure_res_ind_motor(m, &r, &l, &ldq) == MOTOR_FAULT_NONE;
            reply_detect_rl(r, l, ldq, ok);
        } else if (job->cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE ||
                   job->cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
            /* VESC 6.00 standard payloads:
               normal   = current, min_erpm, duty, resistance
               openloop = current, erpm_per_sec, duty, resistance, inductance */
            // Variabel current: nilai arus untuk pengukuran, kendali, atau proteksi.
            float current = (float)get_i32_be(&job->data[1]) / 1000.0f;
            // Variabel second: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float second = (float)get_i32_be(&job->data[5]) / 1000.0f;
            // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
            float duty = (float)get_i32_be(&job->data[9]) / 1000.0f;
            // Variabel resistance: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float resistance = (float)get_i32_be(&job->data[13]) / 1000000.0f;
            // Variabel inductance: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float inductance = m->detect_rl_valid ? m->detect_inductance_h : m->foc_motor_l;
            if (job->cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
                inductance = (float)get_i32_be(&job->data[17]) / 100000000.0f;
            }

            // Variabel flux: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float flux = 0.0f;
            // Variabel params_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool params_ok = isfinite(current) && isfinite(second) && isfinite(duty) &&
                             isfinite(resistance) && isfinite(inductance) &&
                             fabsf(current) >= 0.05f && fabsf(duty) >= 0.01f &&
                             resistance > 0.00001f && inductance >= 0.0f;
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = false;
            if (params_ok && ensure_current_calibration_valid(10000U)) {
                // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float target;
                // Variabel accel: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float accel;
                if (job->cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
                    target = fminf(fmaxf(fabsf(m->max_erpm), FOC_DETECT_FLUX_ERPM), 5000.0f);
                    accel = fmaxf(fabsf(second), 100.0f);
                }
                else {
                    target = fmaxf(fabsf(second), 400.0f);
                    accel = 3000.0f;
                }
                ok = mcpwm_foc_measure_flux_linkage_motor_bounded(m, current, target, accel,
                            fabsf(duty), resistance, inductance, &flux) == MOTOR_FAULT_NONE;
            }
            reply_detect_flux(job->cmd, flux, ok);
        }
        else if (job->cmd == COMM_DETECT_APPLY_ALL_FOC) {
            /* VESC6/VESC Tool payload:
               detect_can, max_power_loss, min_input_current, max_input_current,
               openloop_erpm, sl_erpm. On this board detect_can=true means
               "include the local forwarded Motor-2", not physical CAN. */
            // Variabel result: hasil sementara atau akhir suatu operasi.
            int16_t result = -1;
            // Variabel detect_can: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const bool detect_can = job->data[1] != 0U;
            // Variabel max_power_loss: batas atau nilai maksimum untuk validasi dan proteksi.
            const float max_power_loss = (float)get_i32_be(&job->data[2]) / 1000.0f;
            // Variabel min_input_current: nilai arus untuk pengukuran, kendali, atau proteksi.
            const float min_input_current = (float)get_i32_be(&job->data[6]) / 1000.0f;
            // Variabel max_input_current: nilai arus untuk pengukuran, kendali, atau proteksi.
            const float max_input_current = (float)get_i32_be(&job->data[10]) / 1000.0f;
            // Variabel openloop: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float openloop = (float)get_i32_be(&job->data[14]) / 1000.0f;
            // Variabel sl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float sl = (float)get_i32_be(&job->data[18]) / 1000.0f;
            // Variabel params_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const bool params_ok = isfinite(max_power_loss) && max_power_loss > 0.0f &&
                                   isfinite(min_input_current) && isfinite(max_input_current) &&
                                   min_input_current <= 0.0f && max_input_current >= 0.0f &&
                                   min_input_current <= max_input_current &&
                                   isfinite(openloop) && fabsf(openloop) >= 100.0f &&
                                   isfinite(sl) && fabsf(sl) >= 100.0f;

            if (params_ok) {
                /* Block ordinary duty/current/speed/position commands on both
                   local motors for the complete calibration/detection window. */
                mc_interface_ignore_input_both(180000);
                mc_interface_release_motor_override_both();
                motor_stop(&g_motor_left);
                motor_stop(&g_motor_right);
            }

            if (!params_ok) {
                result = -14;
            }
            else if (!force_current_calibration_valid(15000U)) {
                result = -15;
            }
            else if (detect_can && m->id == MOTOR_LEFT) {
                /* VESC Tool Board Setup calls the directly-connected controller
                   with detect_can=true, then reads each discovered MCCONF.
                   Treat local ID2 exactly as the second controller for that
                   transaction. No physical CAN driver is involved. */
                commands_send_print("Detect-All: calibrating local Motor-1/LEFT...");
                result = detect_apply_all_one_runtime(&g_motor_left, max_power_loss,
                                                       min_input_current, max_input_current,
                                                       openloop, sl);
                if (result == 0) {
                    commands_send_print("Detect-All: calibrating local Motor-2/RIGHT...");
                    result = detect_apply_all_one_runtime(&g_motor_right, max_power_loss,
                                                          min_input_current, max_input_current,
                                                          openloop, sl);
                }

                if (result == 0) {
                    if (!vesc_config_commit_detect_all_runtime_dual()) {
                        result = -12;
                    }
                    else {
                        commands_send_print("Detect-All: both local motors validated and saved atomically.");
                    }
                }

                if (result != 0) {
                    /* Active MCCONF wire images are still the previous known-good
                       values until the atomic dual commit. Reapply both so a
                       failed M2 detection cannot leave M1 running detected-but-
                       unsaved parameters (or vice versa). */
                    rollback_detect_all_runtime_both();
                    commands_send_print("Detect-All: failed; both motor runtimes restored to last committed MCCONF.");
                }
            }
            else {
                /* Individual Detect-All remains valid for the directly selected
                   motor (or forwarded ID2). A forwarded node has no downstream
                   CAN devices, so detect_can=true there still means that motor
                   only. */
                result = detect_apply_all_one_runtime(m, max_power_loss,
                                                      min_input_current, max_input_current,
                                                      openloop, sl);
                if (result == 0) {
                    if (!vesc_config_commit_detect_all_runtime(m->id))
                        result = -12;
                }
                if (result != 0)
                    (void)vesc_config_reapply_active_mc(m->id);
            }

            if (params_ok) {
                mc_interface_ignore_input_both(0);
                timeout_reset();
            }

            // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint8_t p[3];
            // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
            uint16_t i = 0U;
            p[i++] = COMM_DETECT_APPLY_ALL_FOC;
            put_i16(p, &i, result);
            vesc_comm_send_payload(p, i);
        }
        else if (job->cmd == INTERNAL_CUSTOM_SENSOR_DETECT) {
            // Variabel mode: mode operasi yang menentukan jalur algoritma aktif.
            uint8_t mode = job->len >= 1U ? job->data[0] : SENSOR_MODE_AUTO;
            // Variabel current: nilai arus untuk pengukuran, kendali, atau proteksi.
            float current = job->len >= 5U ?
                    (float)get_i32_be(&job->data[1]) / 1000.0f : SENSOR_DETECT_CURRENT_A;
            // Variabel applied: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool applied = false;
            /* Sensor discovery/calibration has exclusive motor ownership just
               like Detect-All. This prevents APP_ADC/APP_UART from replacing
               the excitation command while sensorless fallback is validated. */
            mc_interface_ignore_input_both(30000U);
            motor_stop(&g_motor_left);
            motor_stop(&g_motor_right);
            if (ensure_current_calibration_valid(10000U)) {
                if (mode == SENSOR_MODE_ENCODER ||
                    (mode == SENSOR_MODE_AUTO && m->id == MOTOR_LEFT)) {
                    // Variabel off: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                    // Variabel rat: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                    float off = 0.0f, rat = 0.0f;
                    // Variabel inv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                    bool inv = false;
                    // Variabel sf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                    int sf = mcpwm_foc_encoder_detect_motor(m, current, false,
                                                             &off, &rat, &inv);
                    if (sf == MOTOR_FAULT_NONE) {
                        apply_encoder_detect_result(m, off, rat, inv);
                        applied = true;
                    } else if (mode == SENSOR_MODE_ENCODER ||
                               !detect_failure_is_sensor_absent(m, sf)) {
                        /* Explicit encoder request stays explicit; real motor
                           faults also stop AUTO rather than being hidden. */
                        mode = SENSOR_MODE_ENCODER;
                    }
                }
                if (!applied && mode != SENSOR_MODE_ENCODER) {
                    // Variabel hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
                    uint8_t hall[8];
                    // Variabel hall_ok: data sensor Hall untuk menentukan sektor atau posisi rotor.
                    bool hall_ok = false;
                    // Variabel hf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                    int hf = mcpwm_foc_hall_detect_motor(m, current, hall, &hall_ok);
                    if (hf == MOTOR_FAULT_NONE && hall_ok) {
                        apply_hall_detect_result(m, hall);
                        applied = true;
                    } else if (mode == SENSOR_MODE_AUTO &&
                               detect_failure_is_sensor_absent(m, hf)) {
                        apply_sensorless_result(m);
                        applied = validate_sensorless_runtime(m, current);
                    }
                }
            }
            mc_interface_ignore_input_both(0U);
            timeout_reset();
            if (applied && !vesc_config_commit_motor_runtime(m->id)) {
                commands_send_print("VESC F103: detected sensor config persistence failed; previous MCCONF restored.");
            }
            reply_sensor_info(m->id);
        }
        else {
            reply_unsupported_detect(job->cmd);
        }
        mc_interface_select_motor_thread(old_motor);
        /* Lepaskan single-job buffer hanya setelah seluruh reply/rollback selesai. */
        __DMB();
        taskENTER_CRITICAL();
        s_block_busy = false;
        taskEXIT_CRITICAL();
    }
}


typedef enum {
    CONTROL_CMD_RESULT_NONE = 0,
    CONTROL_CMD_RESULT_ACCEPTED,
    CONTROL_CMD_RESULT_BAD_LENGTH,
    CONTROL_CMD_RESULT_SHUTDOWN,
    CONTROL_CMD_RESULT_MOTOR_NOT_READY,
    CONTROL_CMD_RESULT_UART_REJECTED,
    CONTROL_CMD_RESULT_INPUT_BLOCKED
} control_cmd_result_t;

// Parameter cmd: command VESC yang ingin diperiksa apakah termasuk perintah penggerak motor.
// Fungsi command_is_motor_control: membedakan command penggerak motor dari request telemetry/config.
static bool command_is_motor_control(uint8_t cmd) {
    switch (cmd) {
        case COMM_SET_DUTY:
        case COMM_SET_CURRENT:
        case COMM_SET_CURRENT_BRAKE:
        case COMM_SET_RPM:
        case COMM_SET_POS:
        case COMM_SET_HANDBRAKE:
        case COMM_SET_CURRENT_REL:
            return true;
        default:
            return false;
    }
}

// Parameter id: identitas motor target command.
// Parameter cmd: command VESC yang sedang dicatat.
// Parameter result: hasil penerimaan atau penolakan command.
// Parameter raw_value: nilai int32 wire asli command untuk diagnostik host.
// Fungsi trace_control_command: mencatat command motor tanpa menambah pekerjaan pada ISR FOC.
static void trace_control_command(motor_id_t id, uint8_t cmd,
                                  control_cmd_result_t result, int32_t raw_value) {
    g_vesc_comm_trace.last_control_cmd = cmd;
    g_vesc_comm_trace.last_control_motor = (uint8_t)id;
    g_vesc_comm_trace.last_control_result = (uint8_t)result;
    g_vesc_comm_trace.last_control_app_reject =
        (uint8_t)app_command_uart_reject_reason(id);
    g_vesc_comm_trace.last_control_value_scaled = raw_value;
    if (result == CONTROL_CMD_RESULT_ACCEPTED)
        g_vesc_comm_trace.control_accept_count++;
    else if (result != CONTROL_CMD_RESULT_NONE)
        g_vesc_comm_trace.control_reject_count++;
}

// Parameter id: identitas motor yang akan mengambil ownership UART.
// Parameter cmd: command kendali yang sedang diproses.
// Parameter raw_value: nilai int32 wire asli command.
// Fungsi begin_uart_motor_command: melakukan arbitration UART lalu input-lock check secara konsisten.
static bool begin_uart_motor_command(motor_id_t id, uint8_t cmd, int32_t raw_value) {
    /* Claim aplikasi dilakukan sebelum mc_interface_try_input_motor(). Ini
       mencegah one-shot lock override habis pada command yang kemudian tetap
       ditolak oleh kalibrasi/fault/output-disable. */
    if (!app_command_uart_claim(id)) {
        trace_control_command(id, cmd, CONTROL_CMD_RESULT_UART_REJECTED, raw_value);
        return false;
    }
    if (!mc_interface_try_input_motor(id)) {
        app_command_release(id, false);
        trace_control_command(id, cmd, CONTROL_CMD_RESULT_INPUT_BLOCKED, raw_value);
        return false;
    }
    return true;
}

// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi command_requires_motor_ready: menjalankan operasi command requires motor ready sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static bool command_requires_motor_ready(uint8_t cmd) {
    switch (cmd) {
        case COMM_SET_DUTY:
        case COMM_SET_CURRENT:
        case COMM_SET_CURRENT_BRAKE:
        case COMM_SET_RPM:
        case COMM_SET_POS:
        case COMM_SET_HANDBRAKE:
        case COMM_SET_CURRENT_REL:
        case COMM_SAMPLE_PRINT:
        case COMM_DETECT_MOTOR_PARAM:
        case COMM_DETECT_MOTOR_R_L:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_ENCODER:
        case COMM_DETECT_HALL_FOC:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
        case COMM_DETECT_APPLY_ALL_FOC:
            return true;
        default:
            return false;
    }
}


// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi process_terminal_text: melayani process terminal text sebagai diagnostik terminal tanpa menambah beban
// pada loop kontrol real-time.
static void process_terminal_text(const uint8_t *data, uint16_t len, motor_id_t id) {
    // Variabel line: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    char line[242];
    if (!data)
        return;
    if (len >= sizeof(line))
        len = (uint16_t)(sizeof(line)-1U);
    memcpy(line, data, len);
    line[len] = '\0';
    // Variabel old: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int old = mc_interface_get_motor_thread();
    mc_interface_select_motor_thread(id == MOTOR_RIGHT ? 2 : 1);
    terminal_process_string(line);
    mc_interface_select_motor_thread(old);
}

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_stats: menyusun atau mengirim reply stats dengan pemeriksaan panjang buffer dan jalur transport
// yang aman.
static void reply_stats(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len < 2U)
        return;
    // Variabel bi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t bi = 0;
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mask = (uint32_t)buffer_get_uint16(data, &bi);
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[60];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    p[i++] = COMM_GET_STATS;
    buffer_append_uint32(p, mask, &i);
    // Variabel old: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int old = mc_interface_get_motor_thread();
    mc_interface_select_motor_thread(id == MOTOR_RIGHT ? 2 : 1);
    if (mask&(1UL<<0))
        buffer_append_float32_auto(p, mc_interface_stat_speed_avg(), &i);
    if (mask&(1UL<<1))
        buffer_append_float32_auto(p, mc_interface_stat_speed_max(), &i);
    if (mask&(1UL<<2))
        buffer_append_float32_auto(p, mc_interface_stat_power_avg(), &i);
    if (mask&(1UL<<3))
        buffer_append_float32_auto(p, mc_interface_stat_power_max(), &i);
    if (mask&(1UL<<4))
        buffer_append_float32_auto(p, mc_interface_stat_current_avg(), &i);
    if (mask&(1UL<<5))
        buffer_append_float32_auto(p, mc_interface_stat_current_max(), &i);
    if (mask&(1UL<<6))
        buffer_append_float32_auto(p, mc_interface_stat_temp_mosfet_avg(), &i);
    if (mask&(1UL<<7))
        buffer_append_float32_auto(p, mc_interface_stat_temp_mosfet_max(), &i);
    if (mask&(1UL<<8))
        buffer_append_float32_auto(p, mc_interface_stat_temp_motor_avg(), &i);
    if (mask&(1UL<<9))
        buffer_append_float32_auto(p, mc_interface_stat_temp_motor_max(), &i);
    if (mask&(1UL<<10))
        buffer_append_float32_auto(p, mc_interface_stat_count_time(), &i);
    mc_interface_select_motor_thread(old);
    vesc_comm_send_payload(p, (uint16_t)i);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi temp_conf_apply_one: menjalankan operasi temp conf apply one sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool temp_conf_apply_one(motor_id_t id, mc_configuration *c, bool store) {
    // Variabel wire: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static uint8_t wire[VESC6_MCCONF_WIRE_SIZE];
    if (confgenerator_serialize_mcconf_motor(wire, c, id) != (int32_t)VESC6_MCCONF_WIRE_SIZE)
        return false;
    return vesc_config_set_mc_wire(id, wire, VESC6_MCCONF_WIRE_SIZE, store);
}

// Parameter d: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter setup: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi set_mcconf_temp: mengatur set mcconf temp setelah nilai masukan divalidasi dan dibatasi sesuai aturan
// keselamatan modul.
static void set_mcconf_temp(const uint8_t *d, uint16_t len, motor_id_t id, bool setup) {
    if (d == NULL || len < 36U)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    // Variabel store: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool store = d[i++] != 0U;
    // Variabel forward: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool forward = d[i++] != 0U;
    // Variabel ack: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ack = d[i++] != 0U;
    // Variabel divide: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool divide = d[i++] != 0U;
    (void)forward;
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    mc_configuration c;
    if (!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), &c))
        return;
    // Variabel controllers: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float controllers = divide ? 2.0f : 1.0f;
    c.l_current_min_scale = foc_clampf(buffer_get_float32_auto(d, &i), 0.0f, 1.0f);
    c.l_current_max_scale = foc_clampf(buffer_get_float32_auto(d, &i), 0.0f, 1.0f);
    // Variabel mn: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel mx: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float mn = buffer_get_float32_auto(d, &i), mx = buffer_get_float32_auto(d, &i);
    if (setup) {
        // Variabel fact: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float fact = ((float)c.si_motor_poles*0.5f*60.0f*c.si_gear_ratio)/(c.si_wheel_diameter*3.14159265358979323846f);
        if (!isfinite(fact) || fact <= 0.0f)
            return;
        mn *= fact;
        mx *= fact;
    }
    c.l_min_erpm = mn;
    c.l_max_erpm = mx;
    c.l_min_duty = buffer_get_float32_auto(d, &i);
    c.l_max_duty = buffer_get_float32_auto(d, &i);
    c.l_watt_min = buffer_get_float32_auto(d, &i)/controllers;
    c.l_watt_max = buffer_get_float32_auto(d, &i)/controllers;
    if ((uint16_t)(i+8) <= len) {
        c.l_in_current_min = buffer_get_float32_auto(d, &i);
        c.l_in_current_max = buffer_get_float32_auto(d, &i);
    }
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = temp_conf_apply_one(id, &c, store);
    if (ack && ok)
        reply_ack(setup ? COMM_SET_MCCONF_TEMP_SETUP : COMM_SET_MCCONF_TEMP);
    if (!ok)
        commands_send_print("VESC F103: temporary MCCONF rejected by hardware ownership/limit validation.");
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi reply_mcconf_temp: menyusun atau mengirim reply mcconf temp dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
static void reply_mcconf_temp(motor_id_t id) {
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    mc_configuration c;
    if (!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), &c))
        return;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t p[60];
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    p[i++] = COMM_GET_MCCONF_TEMP;
    buffer_append_float32_auto(p, c.l_current_min_scale, &i);
    buffer_append_float32_auto(p, c.l_current_max_scale, &i);
    buffer_append_float32_auto(p, c.l_min_erpm, &i);
    buffer_append_float32_auto(p, c.l_max_erpm, &i);
    buffer_append_float32_auto(p, c.l_min_duty, &i);
    buffer_append_float32_auto(p, c.l_max_duty, &i);
    buffer_append_float32_auto(p, c.l_watt_min, &i);
    buffer_append_float32_auto(p, c.l_watt_max, &i);
    buffer_append_float32_auto(p, c.l_in_current_min, &i);
    buffer_append_float32_auto(p, c.l_in_current_max, &i);
    p[i++] = c.si_motor_poles;
    buffer_append_float32_auto(p, c.si_gear_ratio, &i);
    buffer_append_float32_auto(p, c.si_wheel_diameter, &i);
    vesc_comm_send_payload(p, (uint16_t)i);
}

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi process_payload_for_motor: memproses process payload for motor setelah input divalidasi lalu
// memperbarui state atau output sesuai aturan modul.
static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len == 0U)
        return;
    // Variabel cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t cmd = data[0];
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);

    /* COMM_FW_VERSION and the local motor-2 forwarding FW_VERSION path must work even when
     * ADC/PWM/FOC initialization has failed. Motor-driving and detection
     * commands stay inhibited until the five-task controller startup marks the motor ready. */
    if (s_shutdown_latched && command_requires_motor_ready(cmd)) {
        if (command_is_motor_control(cmd))
            trace_control_command(id, cmd, CONTROL_CMD_RESULT_SHUTDOWN, 0);
        return;
    }
    if (!s_motor_ready && command_requires_motor_ready(cmd)) {
        /* Re-check once at the command boundary. Ini memperbaiki kondisi boot
         * Run28 ketika readiness sempat dilatch false, lalu DMA/calibration
         * pulih, tetapi tidak pernah ada jalur yang menaikkan flag kembali. */
        (void)vesc_comm_try_recover_motor_ready();
    }
    if (!s_motor_ready && command_requires_motor_ready(cmd)) {
        if (command_is_motor_control(cmd))
            trace_control_command(id, cmd, CONTROL_CMD_RESULT_MOTOR_NOT_READY, 0);
        return;
    }
    /* Config recovery must remain available even if telemetry/control task
       startup fails. The flag is raised only after motor_control_init() and
       config initialization, so SET_MCCONF can safely stop/apply the runtime. */
    if ((cmd == COMM_SET_MCCONF || cmd == COMM_SET_APPCONF ||
         cmd == COMM_SET_APPCONF_NO_STORE || cmd == COMM_SET_BATTERY_CUT) && !s_config_ready) {
        return;
    }

    if (is_blocking_command(cmd)) {
        if (!queue_blocking_job(data, len, id)) {
            if (!blocking_command_length_valid(cmd, len)) {
                commands_send_print("VESC F103: rejected malformed blocking command length.");
            }
        }
        return;
    }

    switch (cmd) {
        case COMM_FW_VERSION:
            reply_fw_version();
            break;
        case COMM_FW_INFO:
            reply_fw_info();
            break;
        case COMM_GET_VALUES:
            reply_get_values(COMM_GET_VALUES, 0x003FFFFFUL, id);
            break;
        case COMM_GET_VALUES_SELECTIVE:
            if (len >= 5U)
                reply_get_values(COMM_GET_VALUES_SELECTIVE, get_u32_be(&data[1]), id);
            break;
        case COMM_GET_VALUES_SETUP:
            reply_setup_values(COMM_GET_VALUES_SETUP, 0x003FFFFFUL, id);
            break;
        case COMM_GET_VALUES_SETUP_SELECTIVE:
            if (len >= 5U)
                reply_setup_values(COMM_GET_VALUES_SETUP_SELECTIVE, get_u32_be(&data[1]), id);
            break;
        case COMM_PING_CAN:
            if (id == MOTOR_LEFT) {
                /* VESC Tool discovers additional controllers by issuing
                 * COMM_PING_CAN on the directly-connected controller. This
                 * board has no physical CAN PHY, but it is a true dual-motor
                 * target. Advertise only the local second-motor ID; the
                 * directly-connected motor-1 is already represented by the
                 * serial connection and must not be duplicated in the scan. */
                // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                uint8_t p[2] = {
                    COMM_PING_CAN, VESC_LOCAL_MOTOR2_FORWARD_ID
                }
                ;
                vesc_comm_send_payload(p, 2U);
            }
            else {
                /* A forwarded ping is not a scan of another CAN bus. Return an
                 * empty list, matching a node with no downstream devices. */
                // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                uint8_t p[1] = {
                    COMM_PING_CAN
                }
                ;
                vesc_comm_send_payload(p, 1U);
            }
            break;
        case COMM_SET_DUTY:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_duty(m, (float)raw / 100000.0f);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_SET_CURRENT:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_current(m, (float)raw / 1000.0f);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_SET_CURRENT_BRAKE:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_brake_current(m, (float)raw / 1000.0f);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_SET_RPM:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_speed(m, (float)raw);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_SET_POS:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_position(m, (float)raw / 1000000.0f);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_SET_HANDBRAKE:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_handbrake(m, (float)raw / 1000.0f);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_SET_CURRENT_REL:
            if (len < 5U) {
                trace_control_command(id, cmd, CONTROL_CMD_RESULT_BAD_LENGTH, 0);
                break;
            }
            {
                const int32_t raw = get_i32_be(&data[1]);
                if (begin_uart_motor_command(id, cmd, raw)) {
                    motor_set_current_rel(m, (float)raw / 100000.0f);
                    timeout_reset();
                    trace_control_command(id, cmd, CONTROL_CMD_RESULT_ACCEPTED, raw);
                }
            }
            break;
        case COMM_GET_DECODED_ADC:
            {
            /* Payload standar VESC harus selalu mendapat balasan, termasuk
             * saat idle, kalibrasi, fault, atau sebelum APP ADC di-arm. Nilai
             * berasal dari snapshot PA2/PA3 terbaru; sebelum frame fisik
             * pertama tersedia getter mengembalikan nol, tetapi transport
             * tidak boleh timeout hanya karena aplikasi belum aktif. */
            // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint8_t p[17];
            // Variabel bi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint16_t bi = 0U;
            p[bi++] = COMM_GET_DECODED_ADC;
            put_i32(p, &bi, scaled_i32(app_adc_get_decoded_level(), 1000000.0f));
            put_i32(p, &bi, scaled_i32(app_adc_get_voltage(), 1000000.0f));
            put_i32(p, &bi, scaled_i32(app_adc_get_decoded_level2(), 1000000.0f));
            put_i32(p, &bi, scaled_i32(app_adc_get_voltage2(), 1000000.0f));
            vesc_comm_send_payload(p, bi);
        } break;
        case COMM_GET_DECODED_PPM:
            {
            /* This build has no app_ppm module (only app_adc is wired). Return
             * the VESC-canonical zeroed payload so VESC Tool does not treat the
             * controller as unresponsive. Format: decoded level, voltage (both
             * scaled 1e6), matching the GET_DECODED_ADC layout. */
            // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint8_t p[9];
            // Variabel bi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint16_t bi = 0U;
            p[bi++] = COMM_GET_DECODED_PPM;
            put_i32(p, &bi, 0);
            put_i32(p, &bi, 0);
            vesc_comm_send_payload(p, bi);
        } break;
        case COMM_GET_BATTERY_CUT:
            {
            // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            mc_configuration c;
            if (confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), &c)) {
                // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                uint8_t p[9];
                // Variabel bi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                uint16_t bi = 0U;
                p[bi++] = COMM_GET_BATTERY_CUT;
                put_i32(p, &bi, scaled_i32(c.l_battery_cut_start, 1000.0f));
                put_i32(p, &bi, scaled_i32(c.l_battery_cut_end, 1000.0f));
                vesc_comm_send_payload(p, bi);
            }
        } break;
        case COMM_SET_DETECT:
            if (len >= 2U && data[1] <= (uint8_t)DISP_POS_MODE_HALL_OBSERVER_ERROR) {
                /* One USART transport can carry only one untagged
                 * COMM_ROTOR_POSITION stream at a time. A forwarded motor-2
                 * request therefore becomes the stream owner instead of
                 * interleaving two indistinguishable command-22 replies. */
                // Variabel index: indeks elemen yang sedang diproses.
                uint8_t index = (id == MOTOR_RIGHT) ? 1U : 0U;
                if (data[1] == (uint8_t)DISP_POS_MODE_NONE) {
                    s_display_mode[index] = (uint8_t)DISP_POS_MODE_NONE;
                    if (s_display_owner == (int8_t)index)
                        s_display_owner = -1;
                }
                else {
                    // Variabel other: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                    uint8_t other = (uint8_t)(index ^ 1U);
                    s_display_mode[other] = (uint8_t)DISP_POS_MODE_NONE;
                    s_display_mode[index] = data[1];
                    s_display_owner = (int8_t)index;
                }
            }
            break;
        case COMM_SAMPLE_PRINT:
            if (len >= 5U) {
                // Variabel mode: mode operasi yang menentukan jalur algoritma aktif.
                uint8_t mode = data[1];
                // Variabel sample_len: panjang data yang sedang diproses atau dikirim.
                uint16_t sample_len = get_u16_be(&data[2]);
                // Variabel decimation: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                uint16_t decimation = data[4];
                // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
                bool raw = (len >= 6U) ? (data[5] != 0U) : false;
                if (mode <= (uint8_t)DEBUG_SAMPLING_SEND_SINGLE_SAMPLE) {
                    if (!mc_interface_sample_control((debug_sampling_mode)mode, id,
                                              sample_len, decimation, raw)) {
                        commands_send_print("VESC F103: sample request busy or no previous capture.");
                    }
                }
            }
            break;
        case COMM_FORWARD_CAN:
            if (len >= 3U) {
                // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                const uint8_t target = data[1];
                // Variabel inner_cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                const uint8_t inner_cmd = data[2];
                (void)inner_cmd;

                /* Match upstream HW_HAS_DUAL_MOTORS literally: only the derived
                 * second-motor CAN ID is handled as an in-MCU virtual node. The
                 * inner command runs recursively with motor-thread 2 selected,
                 * and the packet-processing thread is ALWAYS returned to motor 1
                 * afterwards. Non-local IDs would go to physical CAN upstream;
                 * this target has no CAN PHY, so they are intentionally ignored. */
                if (target == VESC_LOCAL_MOTOR2_FORWARD_ID) {
                    s_diag.motor2_forwards++;
                    g_vesc_comm_trace.last_forward_target = target;
                    g_vesc_comm_trace.last_forward_inner_cmd = inner_cmd;
                    g_vesc_comm_trace.forward_m2_count++;
                    g_vesc_comm_trace.last_motor_context = 2U;
                    mc_interface_select_motor_thread(2);
                    process_payload_for_motor(&data[2], (uint16_t)(len - 2U), MOTOR_RIGHT);
                    mc_interface_select_motor_thread(1);
                    g_vesc_comm_trace.last_motor_context = 1U;
                }
                else {
                    s_diag.unsupported_forward_ids++;
                }
            }
            break;
        case COMM_TERMINAL_CMD_SYNC:
            process_terminal_text(&data[1], (uint16_t)(len > 0U ? len - 1U : 0U), id);
            break;
        case COMM_APP_DISABLE_OUTPUT:
            if (len >= 6U)
                app_disable_output(get_i32_be(&data[2]));
            break;
        /* COMM_SET_MCCONF_TEMP / _SETUP are queued above. On STM32F1 a
         * store request may stall flash fetch, so temporary config writes share
         * the same blocking worker as persistent MCCONF/APPCONF. */
        case COMM_GET_MCCONF_TEMP:
            reply_mcconf_temp(id);
            break;
        case COMM_GET_STATS:
            reply_stats(&data[1], (uint16_t)(len - 1U), id);
            break;
        case COMM_RESET_STATS:
            {
            // Variabel old: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int old = mc_interface_get_motor_thread();
            mc_interface_select_motor_thread(id == MOTOR_RIGHT ? 2 : 1);
            mc_interface_stat_reset();
            mc_interface_select_motor_thread(old);
            if (len >= 2U && data[1])
                reply_ack(COMM_RESET_STATS);
        } break;
        case COMM_SET_ODOMETER:
            if (len >= 5U) {
                mc_interface_set_odometer_motor(id, (uint64_t)get_u32_be(&data[1]));
                conf_general_request_aux_store();
                timeout_reset();
            }
            break;
        case COMM_MOTOR_ESTOP:
            if (len >= 3U) {
                // Variabel old: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                int old = mc_interface_get_motor_thread();
                mc_interface_select_motor_thread(1);
                mc_interface_ignore_input_both((int)get_u16_be(&data[1]));
                mc_interface_release_motor_override_both();
                mc_interface_select_motor_thread(old);
            }
            break;
        case COMM_SHUTDOWN:
            if (len >= 3U) {
                // Variabel force: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                bool force = data[1] == 1U;
                // Variabel restart: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                bool restart = data[2] == 1U;
                if (force || (fabsf(g_motor_left.erpm) <= 100.0f && fabsf(g_motor_right.erpm) <= 100.0f)) {
                    s_shutdown_latched = true;
                    app_disable_output(-1);
                    app_command_release(MOTOR_LEFT, true);
                    app_command_release(MOTOR_RIGHT, true);
                    motor_hw_emergency_all_off();
                    hw_status_tone_stop();
                    motor_hw_led(false);
                    vTaskDelay(pdMS_TO_TICKS(20U));
                    if (restart)
                        NVIC_SystemReset();
                    else hw_status_power_hold(false);
                }
            }
            break;
        case COMM_REBOOT:
            s_shutdown_latched = true;
            motor_hw_emergency_all_off();
            vTaskDelay(pdMS_TO_TICKS(20U));
            NVIC_SystemReset();
            break;
        case COMM_ALIVE:
            timeout_reset();
            app_command_uart_keepalive(id);
            (void)hw_status_power_is_held(); /* keepalive may hold power latch */
            break;
        case COMM_GET_IMU_DATA:
            {
            /* This board has no IMU. Reply with the requested mask and the
             * standard trailing controller id so VESC Tool / scripts do not
             * time out waiting for command 65. Requested fields are reported
             * as 0.0 (auto float32), matching an absent sensor. */
            // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint16_t mask = 0U;
            if (len >= 3U)
                mask = (uint16_t)((uint16_t)data[1] << 8) | data[2];
            // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint8_t *p = payload_begin();
            if (p == NULL)
                break;
            // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
            uint16_t i = 0U;
            p[i++] = COMM_GET_IMU_DATA;
            put_u16(p, &i, mask);
            /* 16 IMU fields (roll/pitch/yaw, accel xyz, gyro xyz, mag xyz,
             * quaternion xyzw). All absent -> 0.0. */
            // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            for (uint8_t b = 0U; b < 16U; b++) {
                if (mask & (1U << b))
                    put_float32_auto(p, &i, 0.0f);
            }
            p[i++] = (id == MOTOR_RIGHT) ? VESC_CONTROLLER_ID_RIGHT
                                         : VESC_CONTROLLER_ID_LEFT;
            payload_end(i);
        } break;
        case COMM_CUSTOM_APP_DATA:
            if (len >= 2U) {
                if (s_appdata_handler != NULL)
                    s_appdata_handler(&data[1], (uint16_t)(len - 1U), id);
                else process_custom(&data[1], (uint16_t)(len - 1U), id);
            }
            break;
        case COMM_GET_MCCONF:
            reply_config_wire(COMM_GET_MCCONF, id, false);
            break;
        case COMM_GET_MCCONF_DEFAULT:
            reply_config_wire(COMM_GET_MCCONF_DEFAULT, id, true);
            break;
        case COMM_GET_APPCONF:
            reply_config_wire(COMM_GET_APPCONF, id, false);
            break;
        case COMM_GET_APPCONF_DEFAULT:
            reply_config_wire(COMM_GET_APPCONF_DEFAULT, id, true);
            break;
        /* SET_MCCONF/SET_APPCONF are queued above and handled by the blocking
           worker because flash programming must never stall uartcomm parsing. */
        default:
            break;
    }
}

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi process_payload: memproses process payload setelah input divalidasi lalu memperbarui state atau output
// sesuai aturan modul.
static void process_payload(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U)
        return;
    s_diag.rx_frames_ok++;
    g_vesc_comm_trace.last_outer_cmd = data[0];
    g_vesc_comm_trace.last_forward_target = 0U;
    g_vesc_comm_trace.last_forward_inner_cmd = 0U;
    g_vesc_comm_trace.last_motor_context = 1U;
    /* A raw UART packet is always the directly-connected controller (motor 1).
     * Force that context before AND after dispatch so a forwarded command can
     * never leak motor-thread 2 into the next local VESC Tool request. */
    mc_interface_select_motor_thread(1);
    process_payload_for_motor(data, len, MOTOR_LEFT);
    mc_interface_select_motor_thread(1);
}

// Fungsi vesc_comm_poll_once: menjalankan operasi vesc comm poll once sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool vesc_comm_poll_once(void) {
    if (!s_comm_initialized)
        return false;

    // Variabel received: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool received = false;
    // Variabel byte: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t byte = 0U;
    while (app_uartcomm_rx_get(&byte)) {
        received = true;
        vesc_packet_process_byte(&s_parser, byte, process_payload);
    }
    return received;
}

// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi packet_process_thread: memproses packet process thread setelah input divalidasi lalu memperbarui state
// atau output sesuai aturan modul.
void packet_process_thread(void *argument) {
    (void)argument;
    // Variabel last_stack_probe: ruang stack atau informasi pemakaian stack task.
    TickType_t last_stack_probe = 0U;
    // Variabel last_ready_probe: waktu probe recovery NOT_READY yang terakhir agar register tidak dibaca tiap loop UART.
    TickType_t last_ready_probe = 0U;
    for (;; ) {
        timeout_heartbeat(TIMEOUT_HEARTBEAT_COMM);
        // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const TickType_t now = xTaskGetTickCount();
        if (!s_motor_ready && (now - last_ready_probe) >= pdMS_TO_TICKS(100U)) {
            (void)vesc_comm_try_recover_motor_ready();
            last_ready_probe = now;
        }
        if ((now - last_stack_probe) >= pdMS_TO_TICKS(1000U)) {
            g_vesc_packet_stack_free_words = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
            last_stack_probe = now;
        }
        /* SmartESC-compatible architecture: USART3 RX DMA writes a circular
         * buffer and this task polls the DMA write pointer (CNDTR). Packet
         * framing/CRC/dispatch is never executed from interrupt context. */
        if (vesc_comm_poll_once()) {
            taskYIELD();
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    }
}

/* Kirim satu payload VESC dengan kelas prioritas. Reply standar boleh menunggu
 * UART, sedangkan paket periodik low-priority tidak boleh memblokir timer. */
// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter low_priority: prioritas task atau interrupt.
// Fungsi vesc_comm_send_payload_class: menyusun atau mengirim vesc comm send payload class dengan pemeriksaan
// panjang buffer dan jalur transport yang aman.
static bool vesc_comm_send_payload_class(const uint8_t *payload, uint16_t len, bool low_priority) {
    if (payload == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD)
        return false;
    // Variabel running: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool running = comm_scheduler_running();
    // Variabel mutex_taken: handle sinkronisasi untuk melindungi resource bersama.
    bool mutex_taken = false;
    if (running && s_send_mutex != NULL) {
        /* Paket low-priority (rotor-position/diagnostik periodik) tidak boleh
         * menahan timer task di belakang reply VESC Tool. Jika UART sedang
         * sibuk, drop paket periodik ini dan biarkan slot berikutnya mencoba
         * lagi; reply request/response standar tetap menunggu sampai terkirim. */
        // Variabel wait: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const TickType_t wait = low_priority ? 0U : portMAX_DELAY;
        if (xSemaphoreTake(s_send_mutex, wait) != pdTRUE)
            return false;
        mutex_taken = true;
    }
    // Variabel frame_len: panjang data yang sedang diproses atau dikirim.
    uint16_t frame_len = vesc_packet_encode(payload, len, s_tx_frame, sizeof(s_tx_frame));
    // Variabel queued: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool queued = frame_len != 0U && (low_priority
         ? app_uartcomm_write_raw_low_priority(s_tx_frame, frame_len)
         : app_uartcomm_write_raw(s_tx_frame, frame_len));
    if (queued) {
        s_diag.tx_frames++;
        g_vesc_comm_trace.last_reply_cmd = payload[0];
        if (g_vesc_comm_trace.last_forward_target == VESC_LOCAL_MOTOR2_FORWARD_ID) {
            g_vesc_comm_trace.forward_m2_reply_count++;
        }
    }
    g_vesc_comm_trace.last_tx_ok = queued ? 1U : 0U;
    if (mutex_taken)
        (void)xSemaphoreGive(s_send_mutex);
    return queued;
}

// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi vesc_comm_send_payload: menyusun atau mengirim vesc comm send payload dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
void vesc_comm_send_payload(const uint8_t *payload, uint16_t len) {
    (void)vesc_comm_send_payload_class(payload, len, false);
}

// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi vesc_comm_send_payload_low_priority: menyusun atau mengirim vesc comm send payload low priority dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
static void vesc_comm_send_payload_low_priority(const uint8_t *payload, uint16_t len) {
    (void)vesc_comm_send_payload_class(payload, len, true);
}

// Fungsi vesc_comm_reply_diag: menyusun atau mengirim vesc comm reply diag dengan pemeriksaan panjang buffer
// dan jalur transport yang aman.
static void vesc_comm_reply_diag(void) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const app_uartcomm_stats_t *u = app_uartcomm_get_stats();
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    app_adc_status_t a;
    app_adc_get_status(&a);
    /* Use the shared bounded payload scratch instead of a hand-sized stack
     * array. This prevents future diagnostic-field additions from recreating
     * the rev10 stack-overflow class. */
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *p = payload_begin();
    if (p == NULL)
        return;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_COMM_DIAG;
    /* Revision 18 mempertahankan breadcrumb command dari rev17 dan menambahkan
     * readiness/recovery tanpa mengubah payload command standar VESC. */
    p[i++] = 18U;
    put_u32(p, &i, u->rx_bytes);
    put_u32(p, &i, u->rx_overruns);
    put_u32(p, &i, s_diag.rx_frames_ok);
    put_u32(p, &i, u->tx_bytes);
    put_u32(p, &i, u->uart_errors);
    put_u32(p, &i, s_diag.tx_frames);
    put_u32(p, &i, u->tx_overruns);
    put_u32(p, &i, u->tx_complete_count);
    put_u32(p, &i, s_diag.blocking_busy_drops);
    put_u32(p, &i, s_diag.motor2_forwards);
    put_u32(p, &i, s_diag.unsupported_forward_ids);
    put_u32(p, &i, VESC_UART_BAUD);
    p[i++] = (uint8_t)uxQueueMessagesWaiting(s_block_queue);
    p[i++] = timeout_has_timeout() ? 1U : 0U;
    p[i++] = conf_general_is_valid() ? 1U : 0U;
    put_u32(p, &i, u->rx_dma_irq_count);
    put_u32(p, &i, u->tx_dma_irq_count);
    put_u32(p, &i, u->idle_irq_count);
    put_u32(p, &i, u->dma_errors);
    put_u32(p, &i, timeout_get_reset_flags());
    p[i++] = timeout_had_iwdg_reset() ? 1U : 0U;
    p[i++] = timeout_watchdog_started() ? 1U : 0U;
    p[i++] = timeout_watchdog_healthy() ? 1U : 0U;
    p[i++] = conf_general_integrity_ok() ? 1U : 0U;
    put_u32(p, &i, conf_general_get_integrity_checks());
    put_u32(p, &i, conf_general_get_integrity_failures());
    p[i++] = hw_status_power_is_held() ? 1U : 0U;
    p[i++] = s_shutdown_latched ? 1U : 0U;
    put_u32(p, &i, timeout_watchdog_required_mask());
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_FOC));
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_MOTOR_SERVICE));
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_COMM));
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_FAULT));
    put_u32(p, &i, timeout_watchdog_unhealthy_mask());
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_FOC));
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_MOTOR_SERVICE));
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_COMM));
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_FAULT));
    p[i++] = (uint8_t)conf_general_boot_status();
    put_u32(p, &i, g_motor_left.sampling_window_clamp_count);
    put_u32(p, &i, g_motor_right.sampling_window_clamp_count);
    put_u16(p, &i, g_motor_left.sampling_margin_min_q15);
    put_u16(p, &i, g_motor_right.sampling_margin_min_q15);
    /* Revision 12: actual PA2/PA3 plus fault-manager/config-health state. */
    put_u16(p, &i, a.raw1);
    put_u16(p, &i, a.raw2);
    put_u16(p, &i, (uint16_t)scaled_i32(a.voltage1, 1000.0f));
    put_u16(p, &i, (uint16_t)scaled_i32(a.voltage2, 1000.0f));
    put_i16(p, &i, (int16_t)scaled_i32(a.decoded1, 1000.0f));
    put_i16(p, &i, (int16_t)scaled_i32(a.decoded2, 1000.0f));
    put_i16(p, &i, (int16_t)scaled_i32(a.command, 1000.0f));
    p[i++] = a.fault_flags;
    p[i++] = a.range_ok ? 1U : 0U;
    p[i++] = a.armed_left ? 1U : 0U;
    p[i++] = a.armed_right ? 1U : 0U;
    p[i++] = (uint8_t)app_command_get_source(MOTOR_LEFT);
    p[i++] = (uint8_t)app_command_get_source(MOTOR_RIGHT);

    /* Revision 13: hardware sampling contract, complete dual-motor ISR timing
     * and RTOS/UART resource headroom. Diagnostics are task-side only. */
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    mc_interface_resource_stats_t r;
    // Variabel cr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    vesc_comm_resource_stats_t cr;
    mc_interface_get_resource_stats(&r);
    vesc_comm_get_resource_stats(&cr);
    put_u32(p, &i, motor_hw_sampling_contract_flags());
    put_u32(p, &i, foc_isr_total_max_cycles());
    put_u32(p, &i, foc_isr_near_deadline_count());
    put_u32(p, &i, foc_isr_period_min_cycles());
    put_u32(p, &i, foc_isr_period_max_cycles());
    put_u32(p, &i, r.heap_free_bytes);
    put_u32(p, &i, r.heap_min_ever_bytes);
    put_u16(p, &i, (uint16_t)(r.motor_service_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.motor_service_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(r.sample_sender_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.sample_sender_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(r.fault_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.fault_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(r.status_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.status_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(cr.packet_stack_free_bytes > UINT16_MAX ? UINT16_MAX : cr.packet_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(cr.blocking_stack_free_bytes > UINT16_MAX ? UINT16_MAX : cr.blocking_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(u->tx_queue_high_water > UINT16_MAX ? UINT16_MAX : u->tx_queue_high_water));
    put_u32(p, &i, u->tx_queue_busy_drops);
    put_u32(p, &i, u->tx_low_priority_drops);
    /* Revision 15: SmartESC-style circular-RX recovery count and explicit
     * boot breadcrumbs for diagnosing a management-UART failure path. */
    put_u32(p, &i, u->rx_restarts);
    put_u32(p, &i, g_vesc_boot_stage);
    put_u32(p, &i, g_vesc_boot_error);

    /* Revision 16: exact VESC dual-motor command breadcrumbs plus buzzer and
     * first current-fault capture. This makes a forwarded GET_VALUES timeout
     * distinguishable from parser, routing, TX and FOC/current failures. */
    p[i++] = g_vesc_comm_trace.last_outer_cmd;
    p[i++] = g_vesc_comm_trace.last_forward_target;
    p[i++] = g_vesc_comm_trace.last_forward_inner_cmd;
    p[i++] = g_vesc_comm_trace.last_motor_context;
    p[i++] = g_vesc_comm_trace.last_reply_cmd;
    p[i++] = g_vesc_comm_trace.last_tx_ok;
    p[i++] = hw_status_tone_is_running() ? 1U : 0U;
    p[i++] = g_vesc_startup_melody_active ? 1U : 0U;
    p[i++] = g_vesc_startup_melody_index;
    p[i++] = 0U;
    put_u32(p, &i, g_vesc_comm_trace.get_values_m1);
    put_u32(p, &i, g_vesc_comm_trace.get_values_m2);
    put_u32(p, &i, g_vesc_comm_trace.forward_m2_count);
    put_u32(p, &i, g_vesc_comm_trace.forward_m2_reply_count);
    put_u16(p, &i, g_vesc_buzzer_hz);
    put_u32(p, &i, g_vesc_buzzer_remaining);
    // Variabel fs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_fault_snapshot_t fs;
    memset(&fs, 0, sizeof(fs));
    foc_get_fault_snapshot(&fs);
    p[i++] = fs.valid;
    p[i++] = fs.motor;
    p[i++] = fs.fault;
    p[i++] = fs.cal_stage;
    put_u16(p, &i, fs.raw_u);
    put_u16(p, &i, fs.raw_v);
    put_u16(p, &i, fs.raw_dc);
    put_i32(p, &i, fs.offset_u);
    put_i32(p, &i, fs.offset_v);
    put_i32(p, &i, fs.offset_dc);
    put_i32(p, &i, fs.ia_q15);
    put_i32(p, &i, fs.ib_q15);
    put_i32(p, &i, fs.ic_q15);
    put_i32(p, &i, fs.trip_q15);
    put_i32(p, &i, fs.iq_target_q15);

    /* Revision 17: breadcrumb command kendali untuk membedakan command yang
     * diterapkan dari silent-reject akibat fault, kalibrasi, arbitration,
     * shutdown, atau input lock. Semua field ini hanya diagnostik task-side. */
    p[i++] = g_vesc_comm_trace.last_control_cmd;
    p[i++] = g_vesc_comm_trace.last_control_motor;
    p[i++] = g_vesc_comm_trace.last_control_result;
    p[i++] = g_vesc_comm_trace.last_control_app_reject;
    put_i32(p, &i, g_vesc_comm_trace.last_control_value_scaled);
    put_u32(p, &i, g_vesc_comm_trace.control_accept_count);
    put_u32(p, &i, g_vesc_comm_trace.control_reject_count);

    /* Revision 18: readiness tidak lagi opaque. Host dapat membedakan
     * boot-warning audit-only, hard sampling failure, calibration belum valid,
     * dan recovery aman yang sudah terjadi. */
    p[i++] = s_motor_ready ? 1U : 0U;
    p[i++] = s_config_ready ? 1U : 0U;
    p[i++] = s_shutdown_latched ? 1U : 0U;
    p[i++] = foc_calibration_done() ? 1U : 0U;
    p[i++] = foc_calibration_valid() ? 1U : 0U;
    p[i++] = 0U;
    put_u32(p, &i, motor_hw_sampling_drive_flags());
    put_u32(p, &i, g_vesc_sampling_contract_flags);
    put_u32(p, &i, s_motor_ready_recovery_count);
    payload_end(i);
}

// Fungsi vesc_comm_periodic_100hz: menjalankan operasi vesc comm periodic 100hz sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void vesc_comm_periodic_100hz(void) {
    /* Standard VESC behaviour: the controller does NOT push COMM_GET_VALUES
     * on its own. VESC Tool issues the request and the firmware replies via
     * the normal packet thread. Pushing telemetry spontaneously both violates
     * the wire convention and, under the polled-TX transport, would contend
     * with command/config replies sent from the higher-priority packet thread.
     * Keep only the non-UART housekeeping here. */
    conf_general_service_100hz();
    /* Kedua bridge lokal memiliki display-position state sendiri. Motor-2
     * mengikuti semantics local dual-motor forwarding, tanpa driver CAN. */
    if (s_display_owner == 0)
        send_rotor_position(MOTOR_LEFT);
    else if (s_display_owner == 1) {
        send_rotor_position(MOTOR_RIGHT);
    }
}

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi sample_default_reply: menyusun atau mengirim sample default reply dengan pemeriksaan panjang buffer
// dan jalur transport yang aman.
static void sample_default_reply(unsigned char *data, unsigned int len) {
    vesc_comm_send_payload_low_priority(data, (uint16_t)len);
}

// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Parameter reply: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi send_sample_buffer_impl: menyusun atau mengirim send sample buffer impl dengan pemeriksaan panjang
// buffer dan jalur transport yang aman.
static void send_sample_buffer_impl(const debug_sample_t *samples, uint16_t count,
                                    void (*reply)(unsigned char *, unsigned int)) {
    if (samples == NULL || count == 0U)
        return;
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    bool raw = mc_interface_sample_raw();
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint16_t n = 0U; n < count; n++) {
        // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const debug_sample_t *d = mc_interface_sample_at(n);
        if (d == NULL)
            d = &samples[n];
        // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint8_t p[56];
        // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        uint16_t i = 0U;
        p[i++] = COMM_SAMPLE_PRINT;
        put_i16(p, &i, (int16_t)n);








        // Variabel current_fir: nilai arus untuk pengukuran, kendali, atau proteksi.
        // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ph1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ph2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ph3: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel vzero: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float ia, ib, ic, ph1, ph2, ph3, vzero, current_fir;
        if (raw) {
            // Variabel sample_motor: state atau parameter motor yang sedang diproses.
            MotorRuntime *sample_motor = motor_get((motor_id_t)d->motor);
            // Variabel phase_a_raw: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
            float phase_a_raw = 0.0f;
            // Variabel phase_b_raw: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
            float phase_b_raw = 0.0f;
            // Variabel phase_c_raw: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
            float phase_c_raw = 0.0f;
            if (sample_motor != NULL) {
                // Variabel synthetic_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
                float synthetic_offset = 0.5f * ((float)sample_motor->current_offset_u_counts +
                                                 (float)sample_motor->current_offset_v_counts);
                // Variabel current_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
                float current_scale = fmaxf(sample_motor->current_scale, 1.0e-6f);
                // Variabel ia_amp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float ia_amp = (float)d->ia_cA / 100.0f;
                // Variabel ib_amp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float ib_amp = (float)d->ib_cA / 100.0f;
                // Variabel ic_amp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float ic_amp = -(ia_amp + ib_amp);
                if (sample_motor->id == MOTOR_LEFT) {
                    phase_a_raw = (float)d->current_raw_u;
                    phase_b_raw = (float)d->current_raw_v;
                    phase_c_raw = synthetic_offset - (ic_amp / current_scale);
                }
                else {
                    phase_a_raw = synthetic_offset - (ia_amp / current_scale);
                    phase_b_raw = (float)d->current_raw_u;
                    phase_c_raw = (float)d->current_raw_v;
                }
            }
            ia = phase_a_raw;
            ib = phase_b_raw;
            ic = phase_c_raw;
            ph1 = (float)d->duty_u_q15 * (4095.0f / 32768.0f);
            ph2 = (float)d->duty_v_q15 * (4095.0f / 32768.0f);
            ph3 = (float)d->duty_w_q15 * (4095.0f / 32768.0f);
            vzero = 2048.0f;
            current_fir = (float)d->iq_cA;
        }
        else {
            ia = (float)d->ia_cA / 100.0f;
            ib = (float)d->ib_cA / 100.0f;
            ic = -(ia + ib);
            // Variabel vbus: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
            float vbus = (float)d->vbus_dV / 10.0f;
            ph1 = ((float)d->duty_u_q15 / 32768.0f) * vbus;
            ph2 = ((float)d->duty_v_q15 / 32768.0f) * vbus;
            ph3 = ((float)d->duty_w_q15 / 32768.0f) * vbus;
            vzero = vbus * 0.5f;
            current_fir = (float)d->iq_cA / 100.0f;
        }

        put_float32_auto(p, &i, ia);
        put_float32_auto(p, &i, ib);
        put_float32_auto(p, &i, ic);
        put_float32_auto(p, &i, ph1);
        put_float32_auto(p, &i, ph2);
        put_float32_auto(p, &i, ph3);
        put_float32_auto(p, &i, vzero);
        put_float32_auto(p, &i, current_fir);
        put_float32_auto(p, &i, (float)PWM_FREQUENCY_HZ);
        p[i++] = 0U; /* reduced sampler status */
        p[i++] = (uint8_t)(((uint32_t)d->phase_u16 * 250U) >> 16);
        put_i32(p, &i, (int32_t)n);
        reply(p, i);
        /* At 115200 the software TX ring is back-pressure aware; yielding also
         * keeps the sample sender from starving control threads. */
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Fungsi vesc_comm_send_sample_buffer: menyusun atau mengirim vesc comm send sample buffer dengan pemeriksaan
// panjang buffer dan jalur transport yang aman.
void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count) {
    send_sample_buffer_impl(samples, count, sample_default_reply);
}

// Parameter reply: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Fungsi vesc_comm_send_sample_buffer_to: menyusun atau mengirim vesc comm send sample buffer to dengan
// pemeriksaan panjang buffer dan jalur transport yang aman.
void vesc_comm_send_sample_buffer_to(void (*reply)(unsigned char *, unsigned int),
                                     uint16_t count) {
    if (reply == NULL) {
        reply = sample_default_reply;
    }
    send_sample_buffer_impl(mc_interface_sample_data(), count, reply);
}

// Parameter handler: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_comm_register_appdata_handler: menangani vesc comm register appdata handler pada konteks
// interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
void vesc_comm_register_appdata_handler(vesc_appdata_handler_t handler) {
    s_appdata_handler = handler;
}

// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_comm_get_resource_stats: membaca vesc comm get resource stats tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
void vesc_comm_get_resource_stats(vesc_comm_resource_stats_t *out) {
    if (out == NULL)
        return;
    out->packet_stack_free_bytes = s_packet_tp == NULL ? 0U :
        (uint32_t)uxTaskGetStackHighWaterMark(s_packet_tp) * sizeof(StackType_t);
    out->blocking_stack_free_bytes = s_blocking_tp == NULL ? 0U :
        (uint32_t)uxTaskGetStackHighWaterMark(s_blocking_tp) * sizeof(StackType_t);
}

// Parameter packet: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter blocking: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_comm_set_thread_ids: mengatur vesc comm set thread ids setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void vesc_comm_set_thread_ids(TaskHandle_t packet, TaskHandle_t blocking) {
    s_packet_tp = packet;
    s_blocking_tp = blocking;
}

/* Inisialisasi resource komunikasi VESC. Queue blocking hanya membawa token
 * 1 byte karena payload tersimpan pada single-job buffer statik yang aman. */
// Fungsi vesc_comm_task_init: menginisialisasi vesc comm task init sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
bool vesc_comm_task_init(void) {
    if (s_comm_initialized)
        return true;
    memset((void *)&s_diag, 0, sizeof(s_diag));
    memset((void *)&g_vesc_comm_trace, 0, sizeof(g_vesc_comm_trace));
    memset((void *)s_display_mode, 0, sizeof(s_display_mode));
    s_display_owner = -1;
    s_motor_ready = false;
    s_config_ready = false;
    s_motor_ready_recovery_count = 0U;
    s_block_busy = false;
    vesc_packet_parser_init(&s_parser);

    s_payload_mutex = xSemaphoreCreateMutex();
    s_send_mutex = xSemaphoreCreateMutex();
    s_block_queue = xQueueCreate(BLOCK_QUEUE_DEPTH, sizeof(uint8_t));

    if (s_payload_mutex == NULL || s_send_mutex == NULL || s_block_queue == NULL) {
        return false;
    }

    /* The packet/blocking threads are spawned centrally in main.c via
     * xTaskCreate and registered through vesc_comm_set_thread_ids(). Validate
     * that the handles were provided before starting the serial peripheral. */
    if (s_packet_tp == NULL || s_blocking_tp == NULL) {
        return false;
    }

    /* Match app_uartcomm_start ordering conceptually: packet state/thread
     * resources exist before the serial peripheral starts receiving bytes. */
    if (!app_uartcomm_init()) {
        return false;
    }

    s_comm_initialized = true;
    return true;
}

// Fungsi commands_init: menginisialisasi commands init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void commands_init(void) {
    (void)vesc_comm_task_init();
}

// Fungsi commands_is_initialized: menjalankan operasi commands is initialized sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool commands_is_initialized(void) {
    return s_comm_initialized;
}

// Parameter ready: penanda bahwa resource atau state siap digunakan.
// Fungsi vesc_comm_set_config_ready: mengatur vesc comm set config ready setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void vesc_comm_set_config_ready(bool ready) {
    s_config_ready = ready;
}

// Parameter ready: penanda bahwa resource atau state siap digunakan.
// Fungsi vesc_comm_set_motor_ready: mengatur vesc comm set motor ready setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void vesc_comm_set_motor_ready(bool ready) {
    s_motor_ready = ready;
}

// Fungsi vesc_comm_motor_ready: menjalankan operasi vesc comm motor ready sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool vesc_comm_motor_ready(void) {
    return s_motor_ready;
}

// Fungsi vesc_comm_try_recover_motor_ready: memulihkan readiness hanya dari bukti runtime yang aman.
bool vesc_comm_try_recover_motor_ready(void) {
    if (s_motor_ready)
        return true;
    if (s_shutdown_latched || motor_hw_powerstage_fault_latched())
        return false;
    if (!foc_calibration_valid() || !motor_hw_sampling_drive_valid())
        return false;

    /* Boot sampling check dapat melatch ADC_DMA sebelum scheduler berjalan.
     * Recalibration yang valid membuktikan current DMA + ISR telah hidup.
     * Hanya fault ADC_DMA historis yang boleh dibersihkan otomatis di sini;
     * fault sensor/current/voltage/temperature lain tetap memerlukan jalur
     * recovery normal dan tidak disamarkan. */
    if (g_motor_left.fault == MOTOR_FAULT_ADC_DMA)
        motor_clear_fault(&g_motor_left);
    if (g_motor_right.fault == MOTOR_FAULT_ADC_DMA)
        motor_clear_fault(&g_motor_right);
    if (g_motor_left.fault != MOTOR_FAULT_NONE ||
        g_motor_right.fault != MOTOR_FAULT_NONE)
        return false;

    s_motor_ready = true;
    s_motor_ready_recovery_count++;
    timeout_watchdog_require_foc(true);
    return true;
}
