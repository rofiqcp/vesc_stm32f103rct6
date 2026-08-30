#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    APP_ADC_FAULT_NONE = 0,
    APP_ADC_FAULT_NOT_READY = 1U << 0,
    APP_ADC_FAULT_CONFIG = 1U << 1,
    APP_ADC_FAULT_THROTTLE_RANGE = 1U << 2,
    APP_ADC_FAULT_BRAKE_RANGE = 1U << 3,
    APP_ADC_FAULT_IMPLAUSIBLE = 1U << 4,
    APP_ADC_FAULT_START_ACTIVE = 1U << 5
} app_adc_fault_t;

typedef struct {
    uint16_t raw1;
    uint16_t raw2;
    float voltage1;
    float voltage2;
    float decoded1;
    float decoded2;
    float command;
    bool armed_left;
    bool armed_right;
    bool range_ok;
    uint8_t fault_flags;
} app_adc_status_t;

void app_adc_init(void);
void app_adc_service_1khz(uint32_t now_ms);
float app_adc_get_decoded_level(void);
float app_adc_get_voltage(void);
float app_adc_get_decoded_level2(void);
float app_adc_get_voltage2(void);
bool app_adc_range_ok(void);
bool app_adc_data_ready(void);
uint8_t app_adc_fault_flags(void);
void app_adc_get_status(app_adc_status_t *out);
