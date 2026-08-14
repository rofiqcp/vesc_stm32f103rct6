#include "vesc_packet.h"
#include <string.h>

uint16_t vesc_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0U;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0; b < 8U; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void vesc_packet_parser_init(vesc_packet_parser_t *p) {
    p->len = 0U;
}

static void parser_reset(vesc_packet_parser_t *p) {
    p->len = 0U;
}

void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb) {
    if (p->len == 0U) {
        if (byte != 2U && byte != 3U) return;
        p->buf[p->len++] = byte;
        return;
    }

    if (p->len >= sizeof(p->buf)) {
        parser_reset(p);
        return;
    }

    p->buf[p->len++] = byte;
    uint16_t payload_len = 0U;
    uint16_t payload_start = 0U;
    uint16_t total = 0U;

    if (p->buf[0] == 2U) {
        if (p->len < 2U) return;
        payload_len = p->buf[1];
        payload_start = 2U;
        total = (uint16_t)(payload_len + 5U);
    } else {
        if (p->len < 3U) return;
        payload_len = (uint16_t)(((uint16_t)p->buf[1] << 8) | p->buf[2]);
        payload_start = 3U;
        total = (uint16_t)(payload_len + 6U);
    }

    if (total > sizeof(p->buf) || payload_len == 0U) {
        parser_reset(p);
        return;
    }
    if (p->len < total) return;

    uint16_t crc_rx = (uint16_t)(((uint16_t)p->buf[payload_start + payload_len] << 8) |
                                  p->buf[payload_start + payload_len + 1U]);
    uint8_t stop = p->buf[payload_start + payload_len + 2U];
    if (stop == 3U && crc_rx == vesc_crc16(&p->buf[payload_start], payload_len)) {
        if (cb != NULL) cb(&p->buf[payload_start], payload_len);
    }
    parser_reset(p);
}

uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_max) {
    if (payload == NULL || out == NULL || len == 0U) return 0U;
    uint16_t i = 0U;
    if (len <= 255U) {
        if (out_max < (uint16_t)(len + 5U)) return 0U;
        out[i++] = 2U;
        out[i++] = (uint8_t)len;
    } else {
        if (out_max < (uint16_t)(len + 6U)) return 0U;
        out[i++] = 3U;
        out[i++] = (uint8_t)(len >> 8);
        out[i++] = (uint8_t)len;
    }
    memcpy(&out[i], payload, len);
    i = (uint16_t)(i + len);
    uint16_t crc = vesc_crc16(payload, len);
    out[i++] = (uint8_t)(crc >> 8);
    out[i++] = (uint8_t)crc;
    out[i++] = 3U;
    return i;
}
