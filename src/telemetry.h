#pragma once
#include "motor_types.h"

void telemetry_init(void);
void telemetry_update_100hz(void);
void telemetry_stats_update_100hz(void);
void telemetry_snapshot_100hz(void);
void telemetry_get(motor_id_t id,motor_telemetry_t *out);
