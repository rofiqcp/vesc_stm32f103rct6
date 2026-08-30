#pragma once
#include <stdbool.h>
#include "datatypes.h"
#include <stdint.h>

bool telemetry_init(void);
void telemetry_update_100hz(void);
void telemetry_stats_update_100hz(void);
void telemetry_snapshot_100hz(void);
void telemetry_get(motor_id_t id,motor_telemetry_t *out);
/* Lock-free task-side snapshot for VESC command replies. Uses the ISR/cache
 * seqlock so forwarded GET_VALUES tidak pernah menunggu mutex telemetry. */
void telemetry_get_realtime(motor_id_t id, motor_telemetry_t *out);

/* VESC COMM_GET_VALUES uses read-reset averages for these six quantities.
 * The mask uses the standard GET_VALUES bit positions 2,3,4,5,19,20. */
typedef struct {
    float current_motor;
    float current_in;
    float id;
    float iq;
    float vd;
    float vq;
} motor_telemetry_avg_t;

void telemetry_read_reset_avg(motor_id_t id, uint32_t mask, motor_telemetry_avg_t *out);
