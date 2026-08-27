#include "util/maths.h"

void utils_truncate_number(float *num, float min, float max) {
    if (*num < min) *num = min;
    if (*num > max) *num = max;
}

void utils_truncate_number_abs(float *num, float max) {
    if (*num > max) *num = max;
    if (*num < -max) *num = -max;
}
