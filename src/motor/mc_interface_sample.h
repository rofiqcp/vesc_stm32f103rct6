#ifndef MC_INTERFACE_SAMPLE_H_
#define MC_INTERFACE_SAMPLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "datatypes.h"

/*
 * The upstream VESC sampler belongs to mc_interface. It remains in a separate
 * translation unit on this F103 port so that the ISR-facing ring buffer does
 * not make mc_interface.c harder to review. All public names retain the
 * mc_interface prefix to keep ownership unambiguous.
 */
void mc_interface_sample_init(void);
bool mc_interface_sample_control(debug_sampling_mode mode, motor_id_t motor,
		uint16_t len, uint16_t decimation, bool raw);
void mc_interface_sample_start(motor_id_t motor, uint16_t len,
		uint16_t decimation);
void mc_interface_sample_start_ex(motor_id_t motor, uint16_t len,
		uint16_t decimation, bool raw);

void mc_interface_sample_capture_isr(MotorRuntime *active);
bool mc_interface_sample_ready(void);
bool mc_interface_sample_has_capture(void);
uint16_t mc_interface_sample_count(void);
const debug_sample_t *mc_interface_sample_data(void);
const debug_sample_t *mc_interface_sample_at(uint16_t logical_index);
void mc_interface_sample_mark_sent(void);
bool mc_interface_sample_active(void);
bool mc_interface_sample_raw(void);
debug_sampling_mode mc_interface_sample_mode(void);

#endif /* MC_INTERFACE_SAMPLE_H_ */
