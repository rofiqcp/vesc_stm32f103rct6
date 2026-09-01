#pragma once
#include <stdint.h>

/* Portable numeric clamp helpers, mirroring VESC utils_truncate_number /
   utils_truncate_number_abs. Used by config range validation on boards that
   do not define HW_LIM_* macros. */
// Parameter num: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter min: batas atau nilai minimum untuk validasi dan proteksi.
// Parameter max: batas atau nilai maksimum untuk validasi dan proteksi.
// Fungsi utils_truncate_number: menjalankan operasi utils truncate number sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void utils_truncate_number(float *num, float min, float max);
// Parameter num: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter max: batas atau nilai maksimum untuk validasi dan proteksi.
// Fungsi utils_truncate_number_abs: menjalankan operasi utils truncate number abs sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void utils_truncate_number_abs(float *num, float max);
