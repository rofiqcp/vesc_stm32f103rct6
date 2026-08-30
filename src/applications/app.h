#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    UART_PORT_COMM_HEADER = 0,
    UART_PORT_BUILTIN,
    UART_PORT_EXTRA_HEADER
} UART_PORT;

const app_configuration *app_get_configuration(void);
void app_set_configuration(app_configuration *conf);
void app_disable_output(int time_ms);
bool app_is_output_disabled(void);
unsigned short app_calc_crc(app_configuration *conf);
void app_notify_configuration_changed(void);

void app_uartcomm_initialize(void);
void app_uartcomm_start(UART_PORT port_number);
void app_uartcomm_stop(UART_PORT port_number);
void app_uartcomm_configure(uint32_t baudrate, bool permanent_enabled, UART_PORT port_number);
void app_uartcomm_send_packet(unsigned char *data, unsigned int len, UART_PORT port_number);
