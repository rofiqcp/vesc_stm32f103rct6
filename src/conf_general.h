#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "datatypes.h"

typedef enum {
    CONF_BOOT_VIRGIN = 0,
    CONF_BOOT_VALID = 1,
    CONF_BOOT_CORRUPT = 2
} conf_boot_status_t;

/* STM32F103RC has no data EEPROM. emulates it with four transactional
   2-KiB flash pages reserved at the end of flash. */
/* VESC-style configuration/persistence front end. Persistent payloads are
   exact VESC Tool wire images; runtime hardware clamping is intentionally
   separate so a write/readback does not produce false Parameters truncated. */
bool conf_general_init(void);
bool conf_general_load_apply(void);
bool conf_general_store_all(void);
bool conf_general_store_mc_wire_persistent(motor_id_t id, const uint8_t *wire);
bool conf_general_store_app_wire_persistent(const uint8_t *wire);
bool conf_general_is_valid(void);
conf_boot_status_t conf_general_boot_status(void);
uint32_t conf_general_get_save_count(void);
bool conf_general_integrity_ok(void);
uint32_t conf_general_get_integrity_checks(void);
uint32_t conf_general_get_integrity_failures(void);
void conf_general_request_aux_store(void);
void conf_general_service_100hz(void);

/* Canonical upstream-style typed configuration front end. Persistent storage
 * is still the transactional exact VESC-6 wire record used by this port. */
void conf_general_read_app_configuration(app_configuration *conf);
bool conf_general_store_app_configuration(app_configuration *conf);
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2);
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2);
