#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vesc_packet.h"

static int got;
static uint16_t got_len;
static uint8_t got_payload[VESC_PACKET_MAX_PAYLOAD];
static void cb(const uint8_t *p, uint16_t n) {
    got++;
    got_len=n;
    memcpy(got_payload,p,n);
}
static void feed(vesc_packet_parser_t *s,const uint8_t *p,uint16_t n) {
    for(uint16_t i=0;i<n;i++) vesc_packet_process_byte(s,p[i],cb);
}
int main(void) {
    vesc_packet_parser_t s;
    uint8_t out[VESC_PACKET_BUFFER_SIZE];

    uint8_t fw[]={0};
    uint16_t n=vesc_packet_encode(fw,1,out,sizeof(out));
    assert(n==6);
    assert(out[0]==2 && out[1]==1 && out[2]==0 && out[3]==0 && out[4]==0 && out[5]==3);
    vesc_packet_parser_init(&s); got=0; feed(&s,out,n);
    assert(got==1 && got_len==1 && got_payload[0]==0);

    /* Match the proven hoverboard parser: a valid 16-bit length prefix is
       accepted even for a one-byte payload. */
    uint8_t long_one[]={3,0,1,0,0,0,3};
    vesc_packet_parser_init(&s); got=0; feed(&s,long_one,sizeof(long_one));
    assert(got==1 && got_len==1 && got_payload[0]==0);

    uint8_t maxp[VESC_PACKET_MAX_PAYLOAD];
    for(unsigned i=0;i<sizeof(maxp);i++) maxp[i]=(uint8_t)(i*17u+3u);
    uint16_t nm=vesc_packet_encode(maxp,sizeof(maxp),out,sizeof(out));
    assert(nm>0 && out[0]==3 && out[1]==2 && out[2]==0);
    vesc_packet_parser_init(&s); got=0; feed(&s,out,nm);
    assert(got==1 && got_len==sizeof(maxp) && memcmp(got_payload,maxp,sizeof(maxp))==0);

    puts("test_packet_v8: PASS");
    return 0;
}
