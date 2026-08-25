#pragma once
#include <stdint.h>
/* F103 hoverboard: the ABI hardware source is TIM4 on LEFT PB6/PB7. The
   abstraction exists so motor code does not hard-code the timer. */
typedef struct { uint32_t counts; } encoder_cfg_ABI_t;
extern encoder_cfg_ABI_t encoder_cfg_ABI;
