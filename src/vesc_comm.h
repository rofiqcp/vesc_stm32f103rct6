#pragma once
#include <stdint.h>
#include "motor_types.h"

void vesc_comm_task_init(void);
void vesc_comm_thread(void *argument);
void vesc_comm_uart_rx_isr_byte(uint8_t b);
void vesc_comm_notify_from_isr(void);
void vesc_comm_send_payload(const uint8_t *payload, uint16_t len);
void vesc_comm_periodic_100hz(void);
void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count);
