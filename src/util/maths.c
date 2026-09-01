#include "util/maths.h"

// Parameter num: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter min: batas atau nilai minimum untuk validasi dan proteksi.
// Parameter max: batas atau nilai maksimum untuk validasi dan proteksi.
// Fungsi utils_truncate_number: menjalankan operasi utils truncate number sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void utils_truncate_number(float *num, float min, float max) {
    if (*num < min)
        *num = min;
    if (*num > max)
        *num = max;
}

// Parameter num: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter max: batas atau nilai maksimum untuk validasi dan proteksi.
// Fungsi utils_truncate_number_abs: menjalankan operasi utils truncate number abs sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void utils_truncate_number_abs(float *num, float max) {
    if (*num > max)
        *num = max;
    if (*num < -max)
        *num = -max;
}
