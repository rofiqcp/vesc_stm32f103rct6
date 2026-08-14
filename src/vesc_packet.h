#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef void (*vesc_payload_cb_t)(const uint8_t *payload, uint16_t len);

typedef struct {
    uint8_t buf[512];
    uint16_t len;
} vesc_packet_parser_t;

uint16_t vesc_crc16(const uint8_t *buf, uint16_t len);
void vesc_packet_parser_init(vesc_packet_parser_t *p);
void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb);
uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_max);
