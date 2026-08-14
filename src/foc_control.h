#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

void foc_control_init(void);
bool foc_calibration_done(void);
bool foc_calibration_valid(void);
void foc_request_recalibration(void);
uint32_t foc_adc_isr_count(void);
void foc_get_calibration_progress(uint32_t *count, uint32_t *target);
void foc_adc_dma_isr(const volatile uint32_t adc_words[6]);
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m);
void motor_hall_edge_isr(MotorRuntime *m);
