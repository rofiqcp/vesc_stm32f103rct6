#include "vesc_buffer.h"
#include <math.h>
#include <limits.h>

void vesc_buf_append_i16(uint8_t *b, int16_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(((uint16_t)v) >> 8);
    b[(*i)++] = (uint8_t)v;
}
void vesc_buf_append_u16(uint8_t *b, uint16_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(v >> 8); b[(*i)++] = (uint8_t)v;
}
void vesc_buf_append_i32(uint8_t *b, int32_t v, int32_t *i) {
    uint32_t u=(uint32_t)v;
    b[(*i)++]=(uint8_t)(u>>24); b[(*i)++]=(uint8_t)(u>>16);
    b[(*i)++]=(uint8_t)(u>>8); b[(*i)++]=(uint8_t)u;
}
void vesc_buf_append_u32(uint8_t *b, uint32_t v, int32_t *i) {
    b[(*i)++]=(uint8_t)(v>>24); b[(*i)++]=(uint8_t)(v>>16);
    b[(*i)++]=(uint8_t)(v>>8); b[(*i)++]=(uint8_t)v;
}
void vesc_buf_append_float16(uint8_t *b, float v, float scale, int32_t *i) {
    float x=v*scale;
    if (x > 32767.0f) x=32767.0f;
    if (x < -32768.0f) x=-32768.0f;
    /* VESC buffer.c uses a C cast here (truncate toward zero), not rounding. */
    vesc_buf_append_i16(b,(int16_t)x,i);
}
void vesc_buf_append_float32(uint8_t *b, float v, float scale, int32_t *i) {
    double x=(double)v*(double)scale;
    if (x > (double)INT32_MAX) x=(double)INT32_MAX;
    if (x < (double)INT32_MIN) x=(double)INT32_MIN;
    /* Match upstream VESC wire semantics exactly. */
    vesc_buf_append_i32(b,(int32_t)x,i);
}
void vesc_buf_append_float32_auto(uint8_t *b, float number, int32_t *i) {
    if (fabsf(number) < 1.5e-38f) number=0.0f;
    int e=0; float sig=frexpf(number,&e); float a=fabsf(sig); uint32_t mant=0U;
    if (a >= 0.5f) { mant=(uint32_t)((a-0.5f)*16777216.0f); e+=126; }
    uint32_t raw=((uint32_t)(e & 0xFF) << 23) | (mant & 0x7FFFFFU);
    if (sig < 0.0f) raw |= 0x80000000UL;
    vesc_buf_append_u32(b,raw,i);
}
int16_t vesc_buf_get_i16(const uint8_t *b, int32_t *i) {
    uint16_t u=((uint16_t)b[*i]<<8)|b[*i+1]; *i+=2; return (int16_t)u;
}
uint16_t vesc_buf_get_u16(const uint8_t *b, int32_t *i) {
    uint16_t u=((uint16_t)b[*i]<<8)|b[*i+1]; *i+=2; return u;
}
int32_t vesc_buf_get_i32(const uint8_t *b, int32_t *i) {
    uint32_t u=((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|((uint32_t)b[*i+2]<<8)|b[*i+3]; *i+=4; return (int32_t)u;
}
uint32_t vesc_buf_get_u32(const uint8_t *b, int32_t *i) {
    uint32_t u=((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|((uint32_t)b[*i+2]<<8)|b[*i+3]; *i+=4; return u;
}
float vesc_buf_get_float16(const uint8_t *b, float scale, int32_t *i) { return (float)vesc_buf_get_i16(b,i)/scale; }
float vesc_buf_get_float32(const uint8_t *b, float scale, int32_t *i) { return (float)vesc_buf_get_i32(b,i)/scale; }
float vesc_buf_get_float32_auto(const uint8_t *b, int32_t *i) {
    uint32_t raw=vesc_buf_get_u32(b,i);
    uint32_t exp=(raw>>23)&0xFFU; uint32_t mant=raw&0x7FFFFFU;
    if (exp==0U && mant==0U) return 0.0f;
    float sig=0.5f + ((float)mant/16777216.0f);
    if (raw & 0x80000000UL) sig=-sig;
    return ldexpf(sig,(int)exp-126);
}
