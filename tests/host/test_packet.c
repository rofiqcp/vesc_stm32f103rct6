#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vesc_packet.h"

static uint8_t got[512];
static uint16_t got_len;
static void cb(const uint8_t *p, uint16_t n) {
    assert(n <= sizeof(got));
    memcpy(got, p, n);
    got_len = n;
}

int main(void) {
    const uint8_t payload[] = {6,0,0,3,0xE8,0xA5,0x5A};
    uint8_t frame[64];
    uint16_t n = vesc_packet_encode(payload, sizeof(payload), frame, sizeof(frame));
    assert(n == sizeof(payload) + 5U);
    vesc_packet_parser_t parser;
    vesc_packet_parser_init(&parser);
    for (uint16_t i = 0; i < n; ++i) vesc_packet_process_byte(&parser, frame[i], cb);
    assert(got_len == sizeof(payload));
    assert(memcmp(got, payload, sizeof(payload)) == 0);
    assert(vesc_crc16((const uint8_t *)"123456789", 9) == 0x31C3U);
    puts("test_packet: PASS");
    return 0;
}
