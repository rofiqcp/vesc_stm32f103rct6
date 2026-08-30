#pragma once
#include "encoder_datatype.h"

bool encoder_init(MotorRuntime *m);
void encoder_deinit(MotorRuntime *m);
float encoder_read_deg(MotorRuntime *m);
void encoder_set_deg(MotorRuntime *m, float deg);
bool encoder_index_found(const MotorRuntime *m);

void encoder_update_config(MotorRuntime *m);
bool encoder_is_configured(const MotorRuntime *m);
