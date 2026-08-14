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

typedef struct {
    int32_t mean;
    uint16_t min;
    uint16_t max;
    uint32_t variance_x100; /* population variance * 100, no float needed in ISR */
} foc_cal_channel_diag_t;

typedef struct {
    uint16_t warn_mask;      /* bits 0..5: soft noise warning per channel */
    uint16_t fail_range_mask;/* bits 0..5: mean outside broad 12-bit sane window */
    uint16_t fail_noise_mask;/* bits 0..5: extreme spread/stddev */
    foc_cal_channel_diag_t ch[6]; /* LU,LV,LDC,RU,RV,RDC */
} foc_cal_diag_t;

void foc_get_calibration_diag(foc_cal_diag_t *out);
void foc_adc_dma_isr(const volatile uint32_t adc_words[6]);
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m);
void motor_hall_edge_isr(MotorRuntime *m);
