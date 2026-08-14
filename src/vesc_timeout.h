#pragma once
#include <stdint.h>
#include <stdbool.h>

void vesc_timeout_init(void);
void vesc_timeout_reset(void);
void vesc_timeout_configure(uint32_t timeout_ms, float brake_current_a);
bool vesc_timeout_has_timeout(void);
uint32_t vesc_timeout_get_timeout_ms(void);

float vesc_timeout_get_brake_current(void);
