#pragma once
#include "motor_types.h"

bool sensor_detect_request(MotorRuntime *m, uint8_t requested_mode);
void sensor_detect_update_1khz(MotorRuntime *m, uint32_t now_ms);
