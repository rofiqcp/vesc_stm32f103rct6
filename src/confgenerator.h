#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

#define VESC6_MCCONF_WIRE_SIZE 481U
#define VESC6_APPCONF_WIRE_SIZE 493U
#define VESC6_MCCONF_SIGNATURE 776184161UL
#define VESC6_APPCONF_SIGNATURE 486554156UL

/* Offsets of VESC firmware 6.00 mc_configuration wire image. Keep these
 * named and unit-tested: a one-byte drift here is enough to make VESC Tool
 * report truncated/corrupt parameters even when the total packet length is
 * correct. */
#define VESC6_MC_OFF_L_CURRENT_MAX          8U
#define VESC6_MC_OFF_L_CURRENT_MIN         12U
#define VESC6_MC_OFF_L_IN_CURRENT_MAX      16U
#define VESC6_MC_OFF_L_IN_CURRENT_MIN      20U
#define VESC6_MC_OFF_L_ABS_CURRENT_MAX     24U
#define VESC6_MC_OFF_L_MIN_ERPM            28U
#define VESC6_MC_OFF_L_MAX_ERPM            32U
#define VESC6_MC_OFF_L_ERPM_START          36U
#define VESC6_MC_OFF_L_MAX_ERPM_FBRAKE     38U
#define VESC6_MC_OFF_L_MAX_ERPM_FBRAKE_CC  42U
#define VESC6_MC_OFF_L_MIN_VIN             46U
#define VESC6_MC_OFF_L_MAX_VIN             50U
#define VESC6_MC_OFF_L_BAT_CUT_START       54U
#define VESC6_MC_OFF_L_BAT_CUT_END         58U
#define VESC6_MC_OFF_L_SLOW_ABS_CURRENT    62U
#define VESC6_MC_OFF_L_TEMP_FET_START       63U
#define VESC6_MC_OFF_L_TEMP_FET_END         65U
#define VESC6_MC_OFF_L_TEMP_MOTOR_START     67U
#define VESC6_MC_OFF_L_TEMP_MOTOR_END       69U
#define VESC6_MC_OFF_L_TEMP_ACCEL_DEC       71U
#define VESC6_MC_OFF_L_MIN_DUTY            73U
#define VESC6_MC_OFF_L_MAX_DUTY            75U
#define VESC6_MC_OFF_L_WATT_MAX            77U
#define VESC6_MC_OFF_L_WATT_MIN            81U
#define VESC6_MC_OFF_L_CURRENT_MAX_SCALE   85U
#define VESC6_MC_OFF_L_CURRENT_MIN_SCALE   87U
#define VESC6_MC_OFF_L_DUTY_START          89U
#define VESC6_MC_OFF_FOC_CURRENT_KP        127U
#define VESC6_MC_OFF_FOC_CURRENT_KI        131U
#define VESC6_MC_OFF_FOC_F_ZV              135U
#define VESC6_MC_OFF_FOC_DT_US             139U
#define VESC6_MC_OFF_FOC_ENCODER_INVERTED  143U
#define VESC6_MC_OFF_FOC_ENCODER_OFFSET    144U
#define VESC6_MC_OFF_FOC_ENCODER_RATIO     148U
#define VESC6_MC_OFF_FOC_SENSOR_MODE       152U
#define VESC6_MC_OFF_FOC_OBSERVER_OFFSET   185U
#define VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP 187U
#define VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI 191U
#define VESC6_MC_OFF_FOC_START_CURR_DEC     195U
#define VESC6_MC_OFF_FOC_START_CURR_DEC_RPM 197U
#define VESC6_MC_OFF_FOC_HALL_TABLE        223U
#define VESC6_MC_OFF_FOC_HALL_INTERP_ERPM  231U
#define VESC6_MC_OFF_FOC_D_GAIN_SCALE_START 207U
#define VESC6_MC_OFF_FOC_D_GAIN_SCALE_MAX_MOD 209U
#define VESC6_MC_OFF_FOC_SAMPLE_V0_V7      239U
#define VESC6_MC_OFF_FOC_SAMPLE_HIGH_CURRENT 240U
#define VESC6_MC_OFF_FOC_SAT_COMP_MODE     241U
#define VESC6_MC_OFF_FOC_SAT_COMP          242U
#define VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST 247U
#define VESC6_MC_OFF_FOC_CC_DECOUPLING     249U
#define VESC6_MC_OFF_FOC_OBSERVER_TYPE     250U
#define VESC6_MC_OFF_FOC_MTPA_MODE         303U
#define VESC6_MC_OFF_FOC_FW_CURRENT_MAX    304U
#define VESC6_MC_OFF_FOC_FW_DUTY_START     308U
#define VESC6_MC_OFF_FOC_FW_RAMP_TIME      310U
#define VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR 312U
#define VESC6_MC_OFF_FOC_SPEED_SOURCE      314U
#define VESC6_MC_OFF_FOC_SL_ERPM           235U
#define VESC6_MC_OFF_S_PID_KP              330U
#define VESC6_MC_OFF_S_PID_KI              334U
#define VESC6_MC_OFF_S_PID_KD              338U
#define VESC6_MC_OFF_P_PID_KP              353U
#define VESC6_MC_OFF_P_PID_KI              357U
#define VESC6_MC_OFF_P_PID_KD              361U
#define VESC6_MC_OFF_M_ENCODER_COUNTS      403U
#define VESC6_MC_OFF_M_SENSOR_PORT_MODE    419U
#define VESC6_MC_OFF_M_INVERT_DIRECTION    420U
#define VESC6_MC_OFF_SI_MOTOR_POLES        451U
#define VESC6_MC_OFF_SI_GEAR_RATIO         452U
#define VESC6_MC_OFF_SI_WHEEL_DIAMETER     456U
#define VESC6_MC_OFF_SI_BATTERY_TYPE       460U
#define VESC6_MC_OFF_SI_BATTERY_CELLS      461U
#define VESC6_MC_OFF_SI_BATTERY_AH         462U
#define VESC6_MC_OFF_SI_MOTOR_NL_CURRENT   466U

#define VESC6_APP_OFF_CONTROLLER_ID          4U
#define VESC6_APP_OFF_TIMEOUT_MSEC           5U
#define VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT  9U
#define VESC6_APP_OFF_APP_TO_USE            33U
/* VESC-6 adc_config occupies bytes 90..138. Keeping the canonical offsets
 * lets VESC Tool configure the real PA2/PA3 backend without a custom ABI. */
#define VESC6_APP_OFF_ADC_CTRL_TYPE           90U
#define VESC6_APP_OFF_ADC_HYST                91U
#define VESC6_APP_OFF_ADC_VOLTAGE_START       95U
#define VESC6_APP_OFF_ADC_VOLTAGE_END         97U
#define VESC6_APP_OFF_ADC_VOLTAGE_MIN         99U
#define VESC6_APP_OFF_ADC_VOLTAGE_MAX        101U
#define VESC6_APP_OFF_ADC_VOLTAGE_CENTER     103U
#define VESC6_APP_OFF_ADC_VOLTAGE2_START     105U
#define VESC6_APP_OFF_ADC_VOLTAGE2_END       107U
#define VESC6_APP_OFF_ADC_USE_FILTER         109U
#define VESC6_APP_OFF_ADC_SAFE_START         110U
#define VESC6_APP_OFF_ADC_BUTTONS            111U
#define VESC6_APP_OFF_ADC_VOLTAGE_INVERTED   112U
#define VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED  113U
#define VESC6_APP_OFF_ADC_THROTTLE_EXP       114U
#define VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE 118U
#define VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE  122U
#define VESC6_APP_OFF_ADC_RAMP_TIME_POS      123U
#define VESC6_APP_OFF_ADC_RAMP_TIME_NEG      127U
#define VESC6_APP_OFF_ADC_MULTI_ESC          131U
#define VESC6_APP_OFF_ADC_TC                 132U
#define VESC6_APP_OFF_ADC_TC_MAX_DIFF        133U
#define VESC6_APP_OFF_ADC_UPDATE_RATE_HZ     137U
#define VESC6_APP_OFF_UART_BAUD              139U

void vesc_config_init_defaults(void);
bool vesc_config_layout_ok(void);
const uint8_t *vesc_config_mc_wire(motor_id_t id, bool defaults);
const uint8_t *vesc_config_app_wire(bool defaults);
bool vesc_config_set_mc_wire(motor_id_t id, const uint8_t *wire, uint16_t len, bool store);
bool vesc_config_set_app_wire(const uint8_t *wire, uint16_t len, bool store);
void vesc_config_sync_motor_runtime(motor_id_t id);
void vesc_config_sync_detect_all_runtime(motor_id_t id);
/* Transactional helpers for detect/custom apply paths. These use the existing
   static rollback scratch so large 481-byte backups never live on RTOS stacks. */
bool vesc_config_commit_motor_runtime(motor_id_t id);
bool vesc_config_commit_detect_all_runtime(motor_id_t id);
/* Commit both local motor Detect-All results as one flash transaction.
 * Either both runtime MCCONFs become active/persistent, or both roll back. */
bool vesc_config_commit_detect_all_runtime_dual(void);
bool vesc_config_reapply_active_mc(motor_id_t id);

/* Flash persistence interface: exact VESC 6.00 wire images are the source of
   truth for round-trip read/write. Unsupported/UI-only fields remain present
   in the image but are immutable when no real runtime backend exists. */
void vesc_config_export_wire(uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE],
                             uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE],
                             uint8_t app[VESC6_APPCONF_WIRE_SIZE]);
bool vesc_config_import_wire(const uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE],
                             const uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE],
                             const uint8_t app[VESC6_APPCONF_WIRE_SIZE]);

/* Canonical VESC confgenerator API. These wrappers intentionally serialize
 * the pinned VESC-6.00 ABI used by this port (481/493 bytes), not current
 * master VESC7 schema. */
int32_t confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf);
int32_t confgenerator_serialize_appconf(uint8_t *buffer, const app_configuration *conf);
bool confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf);
bool confgenerator_deserialize_appconf(const uint8_t *buffer, app_configuration *conf);
void confgenerator_set_defaults_mcconf(mc_configuration *conf);
void confgenerator_set_defaults_appconf(app_configuration *conf);

/* Board extension used by conf_general where the motor is explicit rather
 * than selected through mc_interface thread-local state. */
int32_t confgenerator_serialize_mcconf_motor(uint8_t *buffer,
        const mc_configuration *conf, motor_id_t id);
