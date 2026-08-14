#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vesc_packet.h"

static int got = 0;
static uint16_t got_len = 0;
static uint8_t got_payload[VESC_PACKET_MAX_PAYLOAD];
static void cb(const uint8_t *p, uint16_t n) {
    got++;
    got_len = n;
    memcpy(got_payload, p, n);
}

static void feed(vesc_packet_parser_t *s, const uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) vesc_packet_process_byte(s, p[i], cb);
}

int main(void) {
    vesc_packet_parser_t s;
    uint8_t frame[VESC_PACKET_BUFFER_SIZE];
    uint8_t p1[] = {0,1,2,3,4,5,0xaa,0x55};
    uint16_t n1 = vesc_packet_encode(p1, sizeof(p1), frame, sizeof(frame));
    assert(n1 > 0);
    vesc_packet_parser_init(&s); got=0; feed(&s, frame, n1);
    assert(got==1 && got_len==sizeof(p1) && memcmp(got_payload,p1,sizeof(p1))==0);

    uint8_t pmax[VESC_PACKET_MAX_PAYLOAD];
    for (unsigned i=0;i<sizeof(pmax);++i) pmax[i]=(uint8_t)(i*37u+11u);
    uint16_t nm = vesc_packet_encode(pmax, sizeof(pmax), frame, sizeof(frame));
    assert(nm > 0 && frame[0] == 3); /* 512-byte payload uses 16-bit length framing. */
    vesc_packet_parser_init(&s); got=0; feed(&s, frame, nm);
    assert(got==1 && got_len==sizeof(pmax) && memcmp(got_payload,pmax,sizeof(pmax))==0);

    /* Corrupt one frame, then append a valid frame. Parser must resynchronize. */
    uint8_t bad[VESC_PACKET_BUFFER_SIZE]; memcpy(bad,frame,nm); bad[nm-3] ^= 0x5a;
    vesc_packet_parser_init(&s); got=0;
    uint8_t noise[] = {0x99,0x00,0x7f,0x02,0xff,0xff};
    feed(&s,noise,sizeof(noise)); feed(&s,bad,nm); feed(&s,frame,nm);
    assert(got==1 && got_len==sizeof(pmax) && memcmp(got_payload,pmax,sizeof(pmax))==0);
    puts("test_packet_v6: PASS");
    return 0;
}
