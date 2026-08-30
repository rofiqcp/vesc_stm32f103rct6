#pragma once
#include <stdint.h>

/* Portable numeric clamp helpers, mirroring VESC utils_truncate_number /
   utils_truncate_number_abs. Used by config range validation on boards that
   do not define HW_LIM_* macros. */
void utils_truncate_number(float *num, float min, float max);
void utils_truncate_number_abs(float *num, float max);
