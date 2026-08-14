#include "vesc_packet.h"
#include <string.h>

uint16_t vesc_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void vesc_packet_parser_init(vesc_packet_parser_t *p) {
    if (p == NULL) return;
    memset(p, 0, sizeof(*p));
}

/* Return values: >=0 number of bytes consumed for one candidate, -1 invalid,
 * -2 need more bytes. This is a clean-room streaming decoder with the same
 * recovery property as upstream packet.c: a bad prefix is skipped and the
 * following bytes are re-scanned rather than flushing the whole stream. */
static int try_decode(const uint8_t *b, uint16_t available,
                      vesc_payload_cb_t cb, uint16_t *need_more) {
    if (available == 0U) {
        if (need_more) *need_more = 1U;
        return -2;
    }

    uint32_t payload_len = 0U;
    uint16_t header = 0U;
    if (b[0] == 2U) {
        if (available < 2U) { if (need_more) *need_more = (uint16_t)(2U - available); return -2; }
        payload_len = b[1]; header = 2U;
    } else if (b[0] == 3U) {
        if (available < 3U) { if (need_more) *need_more = (uint16_t)(3U - available); return -2; }
        payload_len = ((uint32_t)b[1] << 8) | b[2]; header = 3U;
    } else if (b[0] == 4U) {
        if (available < 4U) { if (need_more) *need_more = (uint16_t)(4U - available); return -2; }
        payload_len = ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]; header = 4U;
    } else {
        return -1;
    }

    if (payload_len == 0U || payload_len > VESC_PACKET_MAX_PAYLOAD) return -1;
    uint32_t total32 = (uint32_t)header + payload_len + 3U;
    if (total32 > VESC_PACKET_BUFFER_SIZE) return -1;
    uint16_t total = (uint16_t)total32;
    if (available < total) {
        if (need_more) *need_more = (uint16_t)(total - available);
        return -2;
    }

    const uint8_t *payload = &b[header];
    uint16_t crc_index = (uint16_t)(header + payload_len);
    uint16_t rx_crc = (uint16_t)(((uint16_t)b[crc_index] << 8) | b[crc_index + 1U]);
    if (b[crc_index + 2U] != 3U || rx_crc != vesc_crc16(payload, (uint16_t)payload_len)) {
        return -1;
    }

    if (cb != NULL) cb(payload, (uint16_t)payload_len);
    return (int)total;
}

void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb) {
    if (p == NULL) return;

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

    for (;;) {
        uint16_t need = 0U;
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

uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len,
                            uint8_t *out, uint16_t out_max) {
    if (payload == NULL || out == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD) return 0U;
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
