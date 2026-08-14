#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_types.h"

#define VESC6_MCCONF_WIRE_SIZE 481U
#define VESC6_APPCONF_WIRE_SIZE 493U
#define VESC6_MCCONF_SIGNATURE 776184161UL
#define VESC6_APPCONF_SIGNATURE 486554156UL

void vesc_config_init_defaults(void);
bool vesc_config_layout_ok(void);
const uint8_t *vesc_config_mc_wire(motor_id_t id, bool defaults);
const uint8_t *vesc_config_app_wire(bool defaults);
bool vesc_config_set_mc_wire(motor_id_t id, const uint8_t *wire, uint16_t len, bool store);
bool vesc_config_set_app_wire(const uint8_t *wire, uint16_t len, bool store);
void vesc_config_sync_motor_runtime(motor_id_t id);

/* Flash persistence interface: exact VESC 6.00 wire images are the source of
   truth for round-trip read/write, including unsupported UI-only fields. */
void vesc_config_export_wire(uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE],
                             uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE],
                             uint8_t app[VESC6_APPCONF_WIRE_SIZE]);
bool vesc_config_import_wire(const uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE],
                             const uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE],
                             const uint8_t app[VESC6_APPCONF_WIRE_SIZE]);
