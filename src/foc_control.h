#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

void foc_control_init(void);

typedef enum {
    FOC_CAL_STAGE_UNDRIVEN = 0,
    FOC_CAL_STAGE_WAIT_LEFT_DRIVEN,
    FOC_CAL_STAGE_LEFT_WARMUP,
    FOC_CAL_STAGE_LEFT_DRIVEN,
    FOC_CAL_STAGE_WAIT_RIGHT_DRIVEN,
    FOC_CAL_STAGE_RIGHT_WARMUP,
    FOC_CAL_STAGE_RIGHT_DRIVEN,
    FOC_CAL_STAGE_WAIT_FINALIZE,
    FOC_CAL_STAGE_DONE,
    FOC_CAL_STAGE_FAILED
} foc_cal_stage_t;

typedef struct {
    uint8_t valid;
    uint8_t motor;
    uint8_t fault;
    uint8_t cal_stage;
    uint16_t raw_u, raw_v, raw_dc;
    int32_t offset_u, offset_v, offset_dc;
    int32_t ia_q15, ib_q15, ic_q15;
    int32_t trip_q15;
    int32_t id_target_q15, iq_target_q15;
    uint16_t ccr1, ccr2, ccr3;
    uint16_t tim_cnt;
    uint16_t dma_cndtr;
    uint32_t adc_isr_count;
    uint16_t blank_cycles;
    uint8_t pwm_enabled;
    uint8_t moe;
    uint8_t pending_events;
    uint8_t reserved;
} foc_fault_snapshot_t;

bool foc_calibration_done(void);
bool foc_calibration_valid(void);
foc_cal_stage_t foc_calibration_stage(void);
void foc_calibration_service_task(void);
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
    uint8_t stage;
    uint8_t reserved;
    uint16_t shift_warn_mask;
    uint16_t warn_mask;      /* bits 0..5: soft noise warning per channel */
    uint16_t fail_range_mask;/* bits 0..5: mean outside broad 12-bit sane window */
    uint16_t fail_noise_mask;/* bits 0..5: extreme spread/stddev */
    foc_cal_channel_diag_t ch[6]; /* driven: LU,LV,LDC,RU,RV,RDC */
    int32_t undriven_mean[6];
    int32_t driven_mean[6];
} foc_cal_diag_t;

void foc_get_calibration_diag(foc_cal_diag_t *out);
void foc_get_fault_snapshot(foc_fault_snapshot_t *out);
void foc_adc_dma_isr(const volatile uint32_t adc_words[6]);
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m);
void motor_hall_edge_isr(MotorRuntime *m);
