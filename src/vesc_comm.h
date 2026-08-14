#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

typedef void (*vesc_appdata_handler_t)(const uint8_t *data, uint16_t len, motor_id_t motor);

bool vesc_comm_task_init(void);
void vesc_comm_set_motor_ready(bool ready);
bool vesc_comm_motor_ready(void);
void vesc_comm_send_payload(const uint8_t *payload, uint16_t len);
void vesc_comm_periodic_100hz(void);
void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count);

void vesc_comm_register_appdata_handler(vesc_appdata_handler_t handler);
