#pragma once
#include "motor_types.h"
#include <stdint.h>

bool status_threads_init(void);
bool motor_threads_init(void);
void motor_threads_fault_signal(motor_id_t id);
void motor_threads_sample_signal(void);

typedef struct {
    uint32_t heap_free_bytes;
    uint32_t heap_min_ever_bytes;
    uint32_t motor_service_stack_free_bytes;
    uint32_t sample_sender_stack_free_bytes;
    uint32_t fault_stack_free_bytes;
    uint32_t status_stack_free_bytes;
} motor_runtime_resource_stats_t;

uint32_t motor_threads_free_heap_bytes(void);
uint32_t motor_threads_min_ever_free_heap_bytes(void);
void motor_threads_get_resource_stats(motor_runtime_resource_stats_t *out);
