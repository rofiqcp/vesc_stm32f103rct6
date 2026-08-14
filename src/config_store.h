#pragma once
#include <stdbool.h>
#include <stdint.h>

/* STM32F103RC has no data EEPROM. V9 emulates it with four transactional
   2-KiB flash pages reserved at the end of flash. */
bool config_store_load_apply(void);
bool config_store_save_all(void);
bool config_store_valid(void);
uint32_t config_store_save_count(void);
