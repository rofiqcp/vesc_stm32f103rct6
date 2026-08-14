#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vesc_buffer.h"

static int closef(float a,float b,float eps){return fabsf(a-b)<=eps;}

int main(void){
    uint8_t b[64]={0}; int32_t i=0;
    vesc_buf_append_i16(b,(int16_t)-12345,&i);
    vesc_buf_append_u16(b,(uint16_t)54321,&i);
    vesc_buf_append_i32(b,(int32_t)-123456789,&i);
    vesc_buf_append_u32(b,0x89ABCDEFUL,&i);
    /* Upstream VESC scaled append semantics truncate toward zero. */
    vesc_buf_append_float16(b,1.23459f,1000.0f,&i);
    vesc_buf_append_float16(b,-1.23459f,1000.0f,&i);
    vesc_buf_append_float32(b,12.345678f,100000.0f,&i);
    vesc_buf_append_float32_auto(b,37.25f,&i);
    vesc_buf_append_float32_auto(b,-0.125f,&i);

    int32_t r=0;
    assert(vesc_buf_get_i16(b,&r)==-12345);
    assert(vesc_buf_get_u16(b,&r)==54321U);
    assert(vesc_buf_get_i32(b,&r)==-123456789);
    assert(vesc_buf_get_u32(b,&r)==0x89ABCDEFUL);
    assert(vesc_buf_get_i16(b,&r)==1234);
    assert(vesc_buf_get_i16(b,&r)==-1234);
    assert(vesc_buf_get_i32(b,&r)==1234567);
    assert(closef(vesc_buf_get_float32_auto(b,&r),37.25f,0.0001f));
    assert(closef(vesc_buf_get_float32_auto(b,&r),-0.125f,0.000001f));
    assert(r==i);
    puts("test_vesc_buffer: PASS");
    return 0;
}
