#pragma once
#include <stdint.h>

// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_i16: menyusun vesc buf append i16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_i16(uint8_t *b, int16_t v, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_u16: menyusun vesc buf append u16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_u16(uint8_t *b, uint16_t v, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_i32: menyusun vesc buf append i32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_i32(uint8_t *b, int32_t v, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_u32: menyusun vesc buf append u32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void vesc_buf_append_u32(uint8_t *b, uint32_t v, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_float16: menyusun vesc buf append float16 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void vesc_buf_append_float16(uint8_t *b, float v, float scale, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_float32: menyusun vesc buf append float32 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void vesc_buf_append_float32(uint8_t *b, float v, float scale, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_append_float32_auto: menyusun vesc buf append float32 auto ke buffer/wire format dengan
// urutan field, skala, dan batas data yang konsisten.
void vesc_buf_append_float32_auto(uint8_t *b, float v, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_i16: membaca vesc buf get i16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int16_t vesc_buf_get_i16(const uint8_t *b, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_u16: membaca vesc buf get u16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint16_t vesc_buf_get_u16(const uint8_t *b, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_i32: membaca vesc buf get i32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int32_t vesc_buf_get_i32(const uint8_t *b, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_u32: membaca vesc buf get u32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint32_t vesc_buf_get_u32(const uint8_t *b, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_float16: membaca vesc buf get float16 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float vesc_buf_get_float16(const uint8_t *b, float scale, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_float32: membaca vesc buf get float32 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float vesc_buf_get_float32(const uint8_t *b, float scale, int32_t *i);
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi vesc_buf_get_float32_auto: membaca vesc buf get float32 auto tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float vesc_buf_get_float32_auto(const uint8_t *b, int32_t *i);

/* Canonical upstream VESC buffer API wrappers. */
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_int16: menyusun buffer append int16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_uint16: menyusun buffer append uint16 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_int32: menyusun buffer append int32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_uint32: menyusun buffer append uint32 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_int64: menyusun buffer append int64 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_int64(uint8_t *buffer, int64_t number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_uint64: menyusun buffer append uint64 ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_float16: menyusun buffer append float16 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_float32: menyusun buffer append float32 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_double64: menyusun buffer append double64 ke buffer/wire format dengan urutan field,
// skala, dan batas data yang konsisten.
void buffer_append_double64(uint8_t *buffer, double number, double scale, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_float32_auto: menyusun buffer append float32 auto ke buffer/wire format dengan urutan
// field, skala, dan batas data yang konsisten.
void buffer_append_float32_auto(uint8_t *buffer, float number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter number: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_append_float64_auto: menyusun buffer append float64 auto ke buffer/wire format dengan urutan
// field, skala, dan batas data yang konsisten.
void buffer_append_float64_auto(uint8_t *buffer, double number, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_int16: membaca buffer get int16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_uint16: membaca buffer get uint16 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_int32: membaca buffer get int32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_uint32: membaca buffer get uint32 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_int64: membaca buffer get int64 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_uint64: membaca buffer get uint64 tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_float16: membaca buffer get float16 tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_float32: membaca buffer get float32 tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_double64: membaca buffer get double64 tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_float32_auto: membaca buffer get float32 auto tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi buffer_get_float64_auto: membaca buffer get float64 auto tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
double buffer_get_float64_auto(const uint8_t *buffer, int32_t *index);
