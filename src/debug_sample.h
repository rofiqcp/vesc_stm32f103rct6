#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"
#include "datatypes.h"

void debug_sample_init(void);

/* VESC debug-sampler state-machine. Returns false only when a new capture
 * would overwrite a buffer that is still queued for transmission. */
bool debug_sample_control(debug_sampling_mode mode, motor_id_t motor,
                          uint16_t len, uint16_t decimation, bool raw);

/* Convenience APIs retained for custom commands; both map to NOW capture. */
void debug_sample_start(motor_id_t motor, uint16_t len, uint16_t decimation);
void debug_sample_start_ex(motor_id_t motor, uint16_t len, uint16_t decimation, bool raw);

void debug_sample_capture_isr(MotorRuntime *active);
bool debug_sample_ready(void);
bool debug_sample_has_capture(void);
uint16_t debug_sample_count(void);
const debug_sample_t *debug_sample_data(void);
const debug_sample_t *debug_sample_at(uint16_t logical_index);
void debug_sample_mark_sent(void);
bool debug_sample_active(void);
bool debug_sample_raw(void);
debug_sampling_mode debug_sample_mode(void);
