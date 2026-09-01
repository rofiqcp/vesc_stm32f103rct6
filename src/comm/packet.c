#include "comm/packet.h"
#include <string.h>

// Parameter buf: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Fungsi vesc_crc16: menjalankan operasi vesc crc16 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
uint16_t vesc_crc16(const uint8_t *buf, uint16_t len) {
    // Variabel crc: nilai CRC untuk memeriksa integritas data.
    uint16_t crc = 0U;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                   : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_packet_parser_init: menginisialisasi vesc packet parser init sehingga resource, konfigurasi awal,
// dan state modul siap digunakan dengan aman.
void vesc_packet_parser_init(vesc_packet_parser_t *p) {
    if (p == NULL)
        return;
    memset(p, 0, sizeof(*p));
}

/* Return values: >=0 number of bytes consumed for one candidate, -1 invalid,
 * -2 need more bytes. This is a clean-room streaming decoder with the same
 * recovery property as upstream packet.c: a bad prefix is skipped and the
 * following bytes are re-scanned rather than flushing the whole stream. */
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter available: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter cb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter need_more: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi try_decode: mengurai try decode dari buffer komunikasi menjadi data internal setelah format dan batas
// input diperiksa.
static int try_decode(const uint8_t *b, uint16_t available,
                      vesc_payload_cb_t cb, uint16_t *need_more) {
    if (available == 0U) {
        if (need_more)
            *need_more = 1U;
        return -2;
    }

    // Variabel payload_len: panjang data yang sedang diproses atau dikirim.
    uint32_t payload_len = 0U;
    // Variabel header: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t header = 0U;
    if (b[0] == 2U) {
        if (available < 2U) {
            if (need_more)
                *need_more = (uint16_t)(2U - available);
            return -2;
        }
        payload_len = b[1];
        header = 2U;
    }
    else if (b[0] == 3U) {
        if (available < 3U) {
            if (need_more)
                *need_more = (uint16_t)(3U - available);
            return -2;
        }
        payload_len = ((uint32_t)b[1] << 8) | b[2];
        header = 3U;
        /* Sama dengan upstream packet.c: bentuk panjang 16-bit hanya sah untuk
         * payload >= 255 byte. Prefix 3 dengan panjang <255 dianggap noise. */
        if (payload_len < 255U)
            return -1;
    }
    else {
        /* PACKET_MAX_PL_LEN is 512, therefore upstream compiles support for
         * 8-bit and 16-bit length prefixes only; start byte 4 is not valid. */
        return -1;
    }

    if (payload_len == 0U || payload_len > VESC_PACKET_MAX_PAYLOAD)
        return -1;
    // Variabel total32: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t total32 = (uint32_t)header + payload_len + 3U;
    if (total32 > VESC_PACKET_BUFFER_SIZE)
        return -1;
    // Variabel total: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t total = (uint16_t)total32;
    if (available < total) {
        if (need_more)
            *need_more = (uint16_t)(total - available);
        return -2;
    }

    // Variabel payload: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t *payload = &b[header];
    // Variabel crc_index: nilai CRC untuk memeriksa integritas data.
    uint16_t crc_index = (uint16_t)(header + payload_len);
    // Variabel rx_crc: nilai CRC untuk memeriksa integritas data.
    uint16_t rx_crc = (uint16_t)(((uint16_t)b[crc_index] << 8) | b[crc_index + 1U]);
    if (b[crc_index + 2U] != 3U || rx_crc != vesc_crc16(payload, (uint16_t)payload_len)) {
        return -1;
    }

    if (cb != NULL)
        cb(payload, (uint16_t)payload_len);
    return (int)total;
}

// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter byte: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter cb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_packet_process_byte: memproses vesc packet process byte setelah input divalidasi lalu memperbarui
// state atau output sesuai aturan modul.
void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb) {
    if (p == NULL)
        return;

    // Variabel live: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t live = (uint16_t)(p->write_len - p->read_pos);
    if (live >= VESC_PACKET_BUFFER_SIZE) {
        p->write_len = p->read_pos = p->bytes_left = 0U;
        live = 0U;
    }

    if (p->write_len >= VESC_PACKET_BUFFER_SIZE) {
        memmove(p->buf, &p->buf[p->read_pos], live);
        p->read_pos = 0U;
        p->write_len = live;
    }

    p->buf[p->write_len++] = byte;
    live++;

    if (p->bytes_left > 1U) {
        p->bytes_left--;
        return;
    }

    for (;; ) {
        // Variabel need: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t need = 0U;
        // Variabel res: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int res = try_decode(&p->buf[p->read_pos], live, cb, &need);
        if (res == -2) {
            p->bytes_left = need;
            break;
        }
        if (res > 0) {
            p->read_pos = (uint16_t)(p->read_pos + (uint16_t)res);
            live = (uint16_t)(p->write_len - p->read_pos);
            p->bytes_left = 0U;
            if (live == 0U) {
                p->write_len = p->read_pos = 0U;
                break;
            }
            continue;
        }

        /* Invalid candidate: advance one byte and retry so a valid frame that
         * follows noise or a bad CRC can still be found. */
        p->read_pos++;
        live--;
        p->bytes_left = 0U;
        if (live == 0U) {
            p->write_len = p->read_pos = 0U;
            break;
        }
    }
}

// Parameter payload: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out_max: batas atau nilai maksimum untuk validasi dan proteksi.
// Fungsi vesc_packet_encode: menyusun vesc packet encode ke buffer/wire format dengan urutan field, skala, dan
// batas data yang konsisten.
uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len,
                            uint8_t *out, uint16_t out_max) {
    if (payload == NULL || out == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD)
        return 0U;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t i = 0U;
    if (len <= 255U) {
        if (out_max < (uint16_t)(len + 5U))
            return 0U;
        out[i++] = 2U;
        out[i++] = (uint8_t)len;
    }
    else {
        if (out_max < (uint16_t)(len + 6U))
            return 0U;
        out[i++] = 3U;
        out[i++] = (uint8_t)(len >> 8);
        out[i++] = (uint8_t)len;
    }
    memcpy(&out[i], payload, len);
    i = (uint16_t)(i + len);
    // Variabel crc: nilai CRC untuk memeriksa integritas data.
    uint16_t crc = vesc_crc16(payload, len);
    out[i++] = (uint8_t)(crc >> 8);
    out[i++] = (uint8_t)crc;
    out[i++] = 3U;
    return i;
}

/* -------------------------------------------------------------------------
 * VESC public packet API compatibility
 * ------------------------------------------------------------------------- */
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_reset: mereset packet reset ke kondisi awal yang aman tanpa meninggalkan state lama yang tidak
// konsisten.
void packet_reset(PACKET_STATE_t *state) {
    if (state == NULL)
        return;
    vesc_packet_parser_init(&state->parser);
}

// Parameter send_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter process_func: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_init: menginisialisasi packet init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
void packet_init(packet_send_func_t send_func, packet_process_func_t process_func,
                 PACKET_STATE_t *state) {
    if (state == NULL)
        return;
    memset(state, 0, sizeof(*state));
    state->send_func = send_func;
    state->process_func = process_func;
    packet_reset(state);
}

/* Standard-state decoder mirrors vesc_packet_process_byte but dispatches via
 * the callback stored in PACKET_STATE_t. It is intentionally independent of
 * the USART3 singleton parser so another VESC module can own a packet state. */
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter available: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter need_more: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi packet_try_decode_state: mengurai packet try decode state dari buffer komunikasi menjadi data internal
// setelah format dan batas input diperiksa.
static int packet_try_decode_state(PACKET_STATE_t *state, const uint8_t *b,
                                   uint16_t available, uint16_t *need_more) {
    if (available == 0U) {
        if (need_more != NULL)
            *need_more = 1U;
        return -2;
    }

    // Variabel payload_len: panjang data yang sedang diproses atau dikirim.
    uint32_t payload_len = 0U;
    // Variabel header: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t header = 0U;
    if (b[0] == 2U) {
        if (available < 2U) {
            if (need_more != NULL)
                *need_more = (uint16_t)(2U - available);
            return -2;
        }
        payload_len = b[1];
        header = 2U;
    }
    else if (b[0] == 3U) {
        if (available < 3U) {
            if (need_more != NULL)
                *need_more = (uint16_t)(3U - available);
            return -2;
        }
        payload_len = ((uint32_t)b[1] << 8) | b[2];
        header = 3U;
        if (payload_len < 255U)
            return -1;
    }
    else {
        return -1;
    }

    if (payload_len == 0U || payload_len > PACKET_MAX_PL_LEN)
        return -1;
    // Variabel total32: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t total32 = (uint32_t)header + payload_len + 3U;
    if (total32 > VESC_PACKET_BUFFER_SIZE)
        return -1;
    // Variabel total: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint16_t total = (uint16_t)total32;
    if (available < total) {
        if (need_more != NULL)
            *need_more = (uint16_t)(total - available);
        return -2;
    }

    // Variabel payload: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *payload = (uint8_t *)&b[header];
    // Variabel crc_index: nilai CRC untuk memeriksa integritas data.
    const uint16_t crc_index = (uint16_t)(header + payload_len);
    // Variabel rx_crc: nilai CRC untuk memeriksa integritas data.
    const uint16_t rx_crc = (uint16_t)(((uint16_t)b[crc_index] << 8) |
                                       b[crc_index + 1U]);
    if (b[crc_index + 2U] != 3U ||
        rx_crc != vesc_crc16(payload, (uint16_t)payload_len)) {
        return -1;
    }

    if (state->process_func != NULL) {
        state->process_func(payload, (unsigned int)payload_len);
    }
    return (int)total;
}

// Parameter rx_data: data kerja yang dibaca, diubah, atau dikirim modul.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_process_byte: memproses packet process byte setelah input divalidasi lalu memperbarui state
// atau output sesuai aturan modul.
void packet_process_byte(uint8_t rx_data, PACKET_STATE_t *state) {
    if (state == NULL)
        return;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    vesc_packet_parser_t *p = &state->parser;

    // Variabel live: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t live = (uint16_t)(p->write_len - p->read_pos);
    if (live >= VESC_PACKET_BUFFER_SIZE) {
        p->write_len = p->read_pos = p->bytes_left = 0U;
        live = 0U;
    }
    if (p->write_len >= VESC_PACKET_BUFFER_SIZE) {
        memmove(p->buf, &p->buf[p->read_pos], live);
        p->read_pos = 0U;
        p->write_len = live;
    }

    p->buf[p->write_len++] = rx_data;
    live++;
    if (p->bytes_left > 1U) {
        p->bytes_left--;
        return;
    }

    for (;; ) {
        // Variabel need: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t need = 0U;
        // Variabel res: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int res = packet_try_decode_state(state, &p->buf[p->read_pos], live, &need);
        if (res == -2) {
            p->bytes_left = need;
            break;
        }
        if (res > 0) {
            p->read_pos = (uint16_t)(p->read_pos + (uint16_t)res);
            live = (uint16_t)(p->write_len - p->read_pos);
            p->bytes_left = 0U;
            if (live == 0U) {
                p->write_len = p->read_pos = 0U;
                break;
            }
            continue;
        }
        p->read_pos++;
        live--;
        p->bytes_left = 0U;
        if (live == 0U) {
            p->write_len = p->read_pos = 0U;
            break;
        }
    }
}

// Parameter data: pointer atau data kerja yang diproses oleh fungsi.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter state: state runtime yang menentukan tahap operasi modul.
// Fungsi packet_send_packet: menyusun atau mengirim packet send packet dengan pemeriksaan panjang buffer dan
// jalur transport yang aman.
void packet_send_packet(unsigned char *data, unsigned int len, PACKET_STATE_t *state) {
    if (state == NULL || state->send_func == NULL || data == NULL || len == 0U ||
        len > PACKET_MAX_PL_LEN)
        return;
    // Variabel frame_len: panjang data yang sedang diproses atau dikirim.
    const uint16_t frame_len = vesc_packet_encode(data, (uint16_t)len,
                                                   state->tx_buffer,
                                                   (uint16_t)sizeof(state->tx_buffer));
    if (frame_len != 0U) {
        state->send_func(state->tx_buffer, (unsigned int)frame_len);
    }
}
