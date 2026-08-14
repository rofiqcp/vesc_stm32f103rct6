#pragma once
#include <stdint.h>
#include "motor_types.h"

void vesc_comm_task_init(void);
void vesc_comm_send_payload(const uint8_t *payload, uint16_t len);
void vesc_comm_periodic_100hz(void);
void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count);
