#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

void foc_control_init(void);
bool foc_calibration_done(void);
bool foc_calibration_valid(void);
void foc_request_recalibration(void);
void foc_get_calibration_progress(uint32_t *count, uint32_t *target);
void foc_adc_dma_isr(const volatile uint32_t adc_words[4]);
void foc_adc_dma_quick_guard_isr(const volatile uint32_t adc_words[4]);
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m);
void motor_hall_edge_isr(MotorRuntime *m);
