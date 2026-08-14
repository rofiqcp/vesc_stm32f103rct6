#pragma once
#include "motor_types.h"

void motor_threads_init(void);
void motor_threads_fault_signal(motor_id_t id);
void motor_threads_sample_signal(void);
