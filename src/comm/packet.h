#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* VESC-compatible packet framing. The board uses the standard 512-byte
 * payload ceiling; framing needs up to 6 extra bytes for start/length/CRC/end. */
#define VESC_PACKET_MAX_PAYLOAD 512U
#define VESC_PACKET_BUFFER_SIZE (VESC_PACKET_MAX_PAYLOAD + 8U)

/* Upstream public naming retained so VESC modules can be ported without a
 * board-specific packet shim. */
#ifndef PACKET_MAX_PL_LEN
#define PACKET_MAX_PL_LEN VESC_PACKET_MAX_PAYLOAD
#endif
#ifndef PACKET_BUFFER_LEN
#define PACKET_BUFFER_LEN VESC_PACKET_BUFFER_SIZE
#endif

typedef void (*vesc_payload_cb_t)(const uint8_t *payload, uint16_t len);

typedef struct {
    // Variabel buf: buffer sementara selama pemrosesan data.
    uint8_t buf[VESC_PACKET_BUFFER_SIZE];
    // Variabel write_len: panjang data yang sedang diproses atau dikirim.
    uint16_t write_len;
    // Variabel read_pos: nilai posisi rotor atau aktuator.
    uint16_t read_pos;
    // Variabel bytes_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t bytes_left;
} vesc_packet_parser_t;

/* Source-compatible VESC packet API. The upstream implementation keeps RX/TX
 * state inside PACKET_STATE_t as well; this reduced port embeds the proven
 * streaming parser and exposes the same packet_* entry points. */
typedef void (*packet_send_func_t)(unsigned char *data, unsigned int len);
typedef void (*packet_process_func_t)(unsigned char *data, unsigned int len);
typedef struct {
    // Variabel send_func: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    packet_send_func_t send_func;
    // Variabel process_func: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    packet_process_func_t process_func;
    // Variabel parser: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    vesc_packet_parser_t parser;
    // Variabel tx_buffer: buffer sementara selama pemrosesan data.
    uint8_t tx_buffer[VESC_PACKET_BUFFER_SIZE];
} PACKET_STATE_t;

/* Canonical VESC-style API. */
// Parameter send_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter process_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_init: menginisialisasi packet init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
void packet_init(packet_send_func_t send_func, packet_process_func_t process_func,
                 PACKET_STATE_t *state);
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_reset: mereset packet reset ke kondisi awal yang aman tanpa meninggalkan state lama yang tidak
// konsisten.
void packet_reset(PACKET_STATE_t *state);
// Parameter rx_data: data kerja yang dibaca, diubah, atau dikirim modul.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_process_byte: memproses packet process byte setelah input divalidasi lalu memperbarui state
// atau output sesuai aturan modul.
void packet_process_byte(uint8_t rx_data, PACKET_STATE_t *state);
// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_send_packet: menyusun atau mengirim packet send packet dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
void packet_send_packet(unsigned char *data, unsigned int len, PACKET_STATE_t *state);

/* Board-internal helpers retained for the USART3 implementation. */
// Parameter buf: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi vesc_crc16: menjalankan operasi vesc crc16 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
uint16_t vesc_crc16(const uint8_t *buf, uint16_t len);
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_packet_parser_init: menginisialisasi vesc packet parser init sehingga resource, konfigurasi awal,
// dan state modul siap digunakan dengan aman.
void vesc_packet_parser_init(vesc_packet_parser_t *p);
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter byte: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter cb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_packet_process_byte: memproses vesc packet process byte setelah input divalidasi lalu memperbarui
// state atau output sesuai aturan modul.
void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb);
// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out_max: batas atau nilai maksimum untuk validasi dan proteksi.
// Fungsi vesc_packet_encode: menyusun vesc packet encode ke buffer/wire format dengan urutan field, skala, dan
// batas data yang konsisten.
uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_max);
