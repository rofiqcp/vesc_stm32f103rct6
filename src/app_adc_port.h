#pragma once
#include <stdbool.h>

void app_adc_port_init(void);
void app_adc_port_set_enabled(bool enabled);
void app_adc_port_set_inputs(float voltage1, float voltage2);
float app_adc_get_decoded_level(void);
float app_adc_get_decoded_level2(void);
float app_adc_get_voltage(void);
float app_adc_get_voltage2(void);
