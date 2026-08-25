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
    uint8_t buf[VESC_PACKET_BUFFER_SIZE];
    uint16_t write_len;
    uint16_t read_pos;
    uint16_t bytes_left;
} vesc_packet_parser_t;

/* Source-compatible VESC packet API. The upstream implementation keeps RX/TX
 * state inside PACKET_STATE_t as well; this reduced port embeds the proven
 * streaming parser and exposes the same packet_* entry points. */
typedef void (*packet_send_func_t)(unsigned char *data, unsigned int len);
typedef void (*packet_process_func_t)(unsigned char *data, unsigned int len);
typedef struct {
    packet_send_func_t send_func;
    packet_process_func_t process_func;
    vesc_packet_parser_t parser;
    uint8_t tx_buffer[VESC_PACKET_BUFFER_SIZE];
} PACKET_STATE_t;

/* Canonical VESC-style API. */
void packet_init(packet_send_func_t send_func, packet_process_func_t process_func,
                 PACKET_STATE_t *state);
void packet_reset(PACKET_STATE_t *state);
void packet_process_byte(uint8_t rx_data, PACKET_STATE_t *state);
void packet_send_packet(unsigned char *data, unsigned int len, PACKET_STATE_t *state);

/* Board-internal helpers retained for the USART3 implementation. */
uint16_t vesc_crc16(const uint8_t *buf, uint16_t len);
void vesc_packet_parser_init(vesc_packet_parser_t *p);
void vesc_packet_process_byte(vesc_packet_parser_t *p, uint8_t byte, vesc_payload_cb_t cb);
uint16_t vesc_packet_encode(const uint8_t *payload, uint16_t len, uint8_t *out, uint16_t out_max);
