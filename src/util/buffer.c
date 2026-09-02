#include "util/buffer.h"
#include <math.h>
#include <limits.h>

// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_i16: menyusun vesc buf append i16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_i16(uint8_t *b, int16_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(((uint16_t)v) >> 8);
    b[(*i)++] = (uint8_t)v;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_u16: menyusun vesc buf append u16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_u16(uint8_t *b, uint16_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)v;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_i32: menyusun vesc buf append i32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_i32(uint8_t *b, int32_t v, int32_t *i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t u = (uint32_t)v;
    b[(*i)++] = (uint8_t)(u>>24);
    b[(*i)++] = (uint8_t)(u>>16);
    b[(*i)++] = (uint8_t)(u>>8);
    b[(*i)++] = (uint8_t)u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_u32: menyusun vesc buf append u32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_u32(uint8_t *b, uint32_t v, int32_t *i) {
    b[(*i)++] = (uint8_t)(v>>24);
    b[(*i)++] = (uint8_t)(v>>16);
    b[(*i)++] = (uint8_t)(v>>8);
    b[(*i)++] = (uint8_t)v;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_float16: menyusun vesc buf append float16 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void vesc_buf_append_float16(uint8_t *b, float v, float scale, int32_t *i) {
    // Variabel x: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float x = v*scale;
    if (x > 32767.0f)
        x = 32767.0f;
    if (x < -32768.0f)
        x = -32768.0f;
    /* VESC buffer.c uses a C cast here (truncate toward zero), not rounding. */
    vesc_buf_append_i16(b, (int16_t)x, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_float32: menyusun vesc buf append float32 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void vesc_buf_append_float32(uint8_t *b, float v, float scale, int32_t *i) {
    // Variabel x: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    double x = (double)v*(double)scale;
    if (x > (double)INT32_MAX)
        x = (double)INT32_MAX;
    if (x < (double)INT32_MIN)
        x = (double)INT32_MIN;
    /* Match upstream VESC wire semantics exactly. */
    vesc_buf_append_i32(b, (int32_t)x, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_float32_auto: menyusun vesc buf append float32 auto ke buffer/wire format dengan
// urutan field, skala, dan batas data yang konsisten.
void vesc_buf_append_float32_auto(uint8_t *b, float number, int32_t *i) {
    if (fabsf(number) < 1.5e-38f)
        number = 0.0f;
    // Variabel e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int e = 0;
    // Variabel sig: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float sig = frexpf(number, &e);
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float a = fabsf(sig);
    // Variabel mant: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mant = 0U;
    if (a >= 0.5f) {
        mant = (uint32_t)((a-0.5f)*16777216.0f);
        e += 126;
    }
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    uint32_t raw = ((uint32_t)(e & 0xFF) << 23) | (mant & 0x7FFFFFU);
    if (sig < 0.0f)
        raw |= 0x80000000UL;
    vesc_buf_append_u32(b, raw, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_i16: membaca vesc buf get i16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int16_t vesc_buf_get_i16(const uint8_t *b, int32_t *i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t u = ((uint16_t)b[*i]<<8)|b[*i+1];
    *i+=2;
    return (int16_t)u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_u16: membaca vesc buf get u16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint16_t vesc_buf_get_u16(const uint8_t *b, int32_t *i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t u = ((uint16_t)b[*i]<<8)|b[*i+1];
    *i+=2;
    return u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_i32: membaca vesc buf get i32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int32_t vesc_buf_get_i32(const uint8_t *b, int32_t *i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t u = ((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|((uint32_t)b[*i+2]<<8)|b[*i+3];
    *i+=4;
    return (int32_t)u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_u32: membaca vesc buf get u32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint32_t vesc_buf_get_u32(const uint8_t *b, int32_t *i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t u = ((uint32_t)b[*i]<<24)|((uint32_t)b[*i+1]<<16)|((uint32_t)b[*i+2]<<8)|b[*i+3];
    *i+=4;
    return u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_float16: membaca vesc buf get float16 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float vesc_buf_get_float16(const uint8_t *b, float scale, int32_t *i) {
    return (float)vesc_buf_get_i16(b, i)/scale;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_float32: membaca vesc buf get float32 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float vesc_buf_get_float32(const uint8_t *b, float scale, int32_t *i) {
    return (float)vesc_buf_get_i32(b, i)/scale;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_float32_auto: membaca vesc buf get float32 auto tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float vesc_buf_get_float32_auto(const uint8_t *b, int32_t *i) {
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    uint32_t raw = vesc_buf_get_u32(b, i);
    // Variabel exp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t exp = (raw>>23)&0xFFU;
    // Variabel mant: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mant = raw&0x7FFFFFU;
    if (exp == 0U && mant == 0U)
        return 0.0f;
    // Variabel sig: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float sig = 0.5f + ((float)mant/16777216.0f);
    if (raw & 0x80000000UL)
        sig = -sig;
    return ldexpf(sig, (int)exp-126);
}

/* -------------------------------------------------------------------------
 * Canonical VESC public API. Keep the proven local helpers as the one
 * implementation for the common formats; add only the 64-bit variants that
 * were not previously needed by this reduced target. */
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_int16: menyusun buffer append int16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_int16(uint8_t *b, int16_t v, int32_t *i) {
    vesc_buf_append_i16(b, v, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_uint16: menyusun buffer append uint16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_uint16(uint8_t *b, uint16_t v, int32_t *i) {
    vesc_buf_append_u16(b, v, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_int32: menyusun buffer append int32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_int32(uint8_t *b, int32_t v, int32_t *i) {
    vesc_buf_append_i32(b, v, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_uint32: menyusun buffer append uint32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_uint32(uint8_t *b, uint32_t v, int32_t *i) {
    vesc_buf_append_u32(b, v, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_int64: menyusun buffer append int64 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_int64(uint8_t *b, int64_t v, int32_t *i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t u = (uint64_t)v;
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (int s = 56; s >= 0; s -= 8)
        b[(*i)++] = (uint8_t)(u>>(unsigned)s);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_uint64: menyusun buffer append uint64 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_uint64(uint8_t *b, uint64_t v, int32_t *i) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (int s = 56; s >= 0; s -= 8)
        b[(*i)++] = (uint8_t)(v>>(unsigned)s);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_float16: menyusun buffer append float16 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void buffer_append_float16(uint8_t *b, float v, float scale, int32_t *i) {
    vesc_buf_append_float16(b, v, scale, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_float32: menyusun buffer append float32 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void buffer_append_float32(uint8_t *b, float v, float scale, int32_t *i) {
    vesc_buf_append_float32(b, v, scale, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_double64: menyusun buffer append double64 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void buffer_append_double64(uint8_t *b, double v, double scale, int32_t *i) {
    buffer_append_int64(b, (int64_t)(v*scale), i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_float32_auto: menyusun buffer append float32 auto ke buffer/wire format dengan urutan
// field, skala, dan batas data yang konsisten.
void buffer_append_float32_auto(uint8_t *b, float v, int32_t *i) {
    vesc_buf_append_float32_auto(b, v, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_append_float64_auto: menyusun buffer append float64 auto ke buffer/wire format dengan urutan
// field, skala, dan batas data yang konsisten.
void buffer_append_float64_auto(uint8_t *b, double v, int32_t *i) {
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    float n = (float)v;
    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float err = (float)(v-(double)n);
    vesc_buf_append_float32_auto(b, n, i);
    vesc_buf_append_float32_auto(b, err, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_int16: membaca buffer get int16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int16_t buffer_get_int16(const uint8_t*b, int32_t*i) {
    return vesc_buf_get_i16(b, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_uint16: membaca buffer get uint16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint16_t buffer_get_uint16(const uint8_t*b, int32_t*i) {
    return vesc_buf_get_u16(b, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_int32: membaca buffer get int32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int32_t buffer_get_int32(const uint8_t*b, int32_t*i) {
    return vesc_buf_get_i32(b, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_uint32: membaca buffer get uint32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint32_t buffer_get_uint32(const uint8_t*b, int32_t*i) {
    return vesc_buf_get_u32(b, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_int64: membaca buffer get int64 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int64_t buffer_get_int64(const uint8_t*b, int32_t*i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t u = buffer_get_uint64(b, i);
    return (int64_t)u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_uint64: membaca buffer get uint64 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint64_t buffer_get_uint64(const uint8_t*b, int32_t*i) {
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t u = 0U;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 8; n++)
        u = (u<<8)|b[(*i)++];
    return u;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_float16: membaca buffer get float16 tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float buffer_get_float16(const uint8_t*b, float scale, int32_t*i) {
    return vesc_buf_get_float16(b, scale, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_float32: membaca buffer get float32 tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float buffer_get_float32(const uint8_t*b, float scale, int32_t*i) {
    return vesc_buf_get_float32(b, scale, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_double64: membaca buffer get double64 tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
double buffer_get_double64(const uint8_t*b, double scale, int32_t*i) {
    return (double)buffer_get_int64(b, i)/scale;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_float32_auto: membaca buffer get float32 auto tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float buffer_get_float32_auto(const uint8_t*b, int32_t*i) {
    return vesc_buf_get_float32_auto(b, i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi buffer_get_float64_auto: membaca buffer get float64 auto tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
double buffer_get_float64_auto(const uint8_t*b, int32_t*i) {
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    double n = (double)vesc_buf_get_float32_auto(b, i);
    // Variabel e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    double e = (double)vesc_buf_get_float32_auto(b, i);
    return n+e;
}
