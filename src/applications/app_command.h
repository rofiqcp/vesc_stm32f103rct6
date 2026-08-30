#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    APP_CMD_SRC_NONE = 0,
    APP_CMD_SRC_ADC,
    APP_CMD_SRC_UART,
    APP_CMD_SRC_CALIBRATION,
    APP_CMD_SRC_DETECTION,
    APP_CMD_SRC_INTERNAL
} app_command_source_t;

void app_command_init(void);
void app_command_service_1khz(uint32_t now_ms);
void app_command_configuration_changed(void);
bool app_command_uart_claim(motor_id_t id);
void app_command_uart_keepalive(motor_id_t id);
bool app_command_adc_claim(motor_id_t id, bool neutral_stable);
void app_command_adc_block(motor_id_t id);
void app_command_adc_release(motor_id_t id, bool stop_motor);
void app_command_force_adc_rearm(motor_id_t id);
void app_command_release(motor_id_t id, bool stop_motor);
app_command_source_t app_command_get_source(motor_id_t id);
bool app_command_adc_rearm_required(motor_id_t id);
