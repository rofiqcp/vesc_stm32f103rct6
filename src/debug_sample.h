#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

void debug_sample_init(void);
void debug_sample_start(motor_id_t motor, uint16_t len, uint16_t decimation);
void debug_sample_capture_isr(MotorRuntime *left, MotorRuntime *right);
bool debug_sample_ready(void);
uint16_t debug_sample_count(void);
const debug_sample_t *debug_sample_data(void);
void debug_sample_mark_sent(void);
bool debug_sample_active(void);
