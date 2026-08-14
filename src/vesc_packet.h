#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Current upstream VESC packet.c uses 512-byte payload buffers by default.
 * Framing needs up to 6 extra bytes for start/length/crc/stop. */
#define VESC_PACKET_MAX_PAYLOAD 512U
#define VESC_PACKET_BUFFER_SIZE (VESC_PACKET_MAX_PAYLOAD + 8U)

typedef void (*vesc_payload_cb_t)(const uint8_t *payload, uint16_t len);

typedef struct {
    uint8_t buf[VESC_PACKET_BUFFER_SIZE];
    uint16_t write_len;
    uint16_t read_pos;
    uint16_t bytes_left;
} vesc_packet_parser_t;

uint16_t vesc_crc16(const uint8_t *buf, uint16_t len);
void vesc_packet_parser_init(vesc_packet_parser_t *p);
void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb);
uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_max);
