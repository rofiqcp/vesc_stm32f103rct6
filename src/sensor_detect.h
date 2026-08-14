#pragma once
#include "motor_types.h"

bool sensor_detect_request(MotorRuntime *m, uint8_t requested_mode);
bool sensor_detect_request_current(MotorRuntime *m, uint8_t requested_mode, float current_a);
bool sensor_detect_request_current_ex(MotorRuntime *m, uint8_t requested_mode, float current_a, bool apply_result);
void sensor_detect_update_1khz(MotorRuntime *m, uint32_t now_ms);
