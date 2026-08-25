#pragma once
#include <stdint.h>

void vesc_buf_append_i16(uint8_t *b, int16_t v, int32_t *i);
void vesc_buf_append_u16(uint8_t *b, uint16_t v, int32_t *i);
void vesc_buf_append_i32(uint8_t *b, int32_t v, int32_t *i);
void vesc_buf_append_u32(uint8_t *b, uint32_t v, int32_t *i);
void vesc_buf_append_float16(uint8_t *b, float v, float scale, int32_t *i);
void vesc_buf_append_float32(uint8_t *b, float v, float scale, int32_t *i);
void vesc_buf_append_float32_auto(uint8_t *b, float v, int32_t *i);
int16_t vesc_buf_get_i16(const uint8_t *b, int32_t *i);
uint16_t vesc_buf_get_u16(const uint8_t *b, int32_t *i);
int32_t vesc_buf_get_i32(const uint8_t *b, int32_t *i);
uint32_t vesc_buf_get_u32(const uint8_t *b, int32_t *i);
float vesc_buf_get_float16(const uint8_t *b, float scale, int32_t *i);
float vesc_buf_get_float32(const uint8_t *b, float scale, int32_t *i);
float vesc_buf_get_float32_auto(const uint8_t *b, int32_t *i);

/* Canonical upstream VESC buffer API wrappers. */
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index);
void buffer_append_int64(uint8_t *buffer, int64_t number, int32_t *index);
void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index);
void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);
void buffer_append_double64(uint8_t *buffer, double number, double scale, int32_t *index);
void buffer_append_float32_auto(uint8_t *buffer, float number, int32_t *index);
void buffer_append_float64_auto(uint8_t *buffer, double number, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index);
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index);
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);
double buffer_get_float64_auto(const uint8_t *buffer, int32_t *index);
