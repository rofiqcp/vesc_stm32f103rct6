#pragma once
#include <stdint.h>
/* F103 hoverboard: the ABI hardware source is TIM4 on LEFT PB6/PB7. The
   abstraction exists so motor code does not hard-code the timer. */
typedef struct {
    // Variabel counts: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t counts;
}
encoder_cfg_ABI_t;
// Variabel encoder_cfg_ABI: data encoder untuk pengukuran posisi atau kecepatan rotor.
extern encoder_cfg_ABI_t encoder_cfg_ABI;
