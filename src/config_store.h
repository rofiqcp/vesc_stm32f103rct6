#pragma once
#include <stdbool.h>
#include <stdint.h>

/* STM32F103RC has no data EEPROM; this is a small flash-backed emulation
 * record for the reduced F103 port. */
bool config_store_load_apply(void);
bool config_store_save_all(void);
bool config_store_valid(void);
uint32_t config_store_save_count(void);
