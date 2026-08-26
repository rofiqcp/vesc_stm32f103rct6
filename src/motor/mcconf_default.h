#pragma once

/* VESC-style motor configuration ownership for the STM32F103 hoverboard port.
 *
 * Upstream keeps motor defaults in motor/mcconf_default.h. Board/electrical
 * calibration constants still live in applications/appconf_default.h here,
 * but motor code only consumes them through this header so the ownership and
 * diff layout stay close to vedderb/bldc.
 */
#include "applications/appconf_default.h"

/* Backend. This build executes MOTOR_TYPE_FOC only. The VESC wire enums for
 * BLDC/DC/HFI remain in datatypes/confgenerator solely to preserve protocol
 * numbering; they are rejected before reaching the power-stage runtime. */
#define MCCONF_MOTOR_TYPE_DEFAULT            MOTOR_TYPE_FOC
#define MCCONF_PWM_MODE_DEFAULT              PWM_MODE_SYNCHRONOUS
#define MCCONF_COMM_MODE_DEFAULT             COMM_MODE_INTEGRATE

/* Generic motor limits. */
#define MCCONF_L_CURRENT_MAX_DEFAULT          FOC_MAX_CURRENT_A
#define MCCONF_L_CURRENT_MIN_DEFAULT          (-FOC_MAX_CURRENT_A)
#define MCCONF_L_ABS_CURRENT_MAX_DEFAULT      FOC_ABS_CURRENT_TRIP_A
#define MCCONF_L_MIN_ERPM_DEFAULT             MOTOR_DEFAULT_MIN_ERPM
#define MCCONF_L_MAX_ERPM_DEFAULT             MOTOR_DEFAULT_MAX_ERPM
#define MCCONF_L_ERPM_START_DEFAULT            0.80f
#define MCCONF_L_MIN_DUTY_DEFAULT             MOTOR_DEFAULT_MIN_DUTY
#define MCCONF_L_MAX_DUTY_DEFAULT             MOTOR_DEFAULT_MAX_DUTY
#define MCCONF_L_WATT_MAX_DEFAULT             5000.0f
#define MCCONF_L_WATT_MIN_DEFAULT            -5000.0f
#define MCCONF_L_CURRENT_MAX_SCALE_DEFAULT    1.0f
#define MCCONF_L_CURRENT_MIN_SCALE_DEFAULT    1.0f
#define MCCONF_L_DUTY_START_DEFAULT           1.0f
/* Runtime-only on the pinned VESC6 wire. This board has real per-motor DC
 * current sensing, so begin a smooth pre-limit at 90% of the configured input
 * current ceiling instead of waiting for hard clipping at 100%. */
#define MCCONF_L_IN_CURRENT_MAP_START_DEFAULT 0.90f
#define MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT 0.005f
#define MCCONF_FOC_SPEED_SOURCE_DEFAULT       FOC_SPEED_SRC_CORRECTED
/* These temperature bytes already exist in the pinned VESC-6.00 MCCONF.
 * Batch 9.1 gives them a real backend. On this PCB l_temp_fet_* protects the
 * MCU/board-temperature proxy, because there is no validated MOSFET NTC. */
#define MCCONF_L_TEMP_FET_START_DEFAULT       80.0f
#define MCCONF_L_TEMP_FET_END_DEFAULT         100.0f
#define MCCONF_L_TEMP_MOTOR_START_DEFAULT     80.0f
#define MCCONF_L_TEMP_MOTOR_END_DEFAULT       100.0f
#define MCCONF_L_TEMP_ACCEL_DEC_DEFAULT       0.15f
/* Current VESC keeps additional faults disabled by default. VESC-6.00 has no
 * wire byte for this field, so this remains a runtime/board policy. */
#define MCCONF_L_ADDITIONAL_FAULTS_DEFAULT    0U
#define MCCONF_L_ADDITIONAL_FAULT_ENCODER_SLIP (1U << 0)
#define MCCONF_L_ADDITIONAL_FAULT_OVERSPEED  (1U << 1)
#define MCCONF_L_ADDITIONAL_FAULT_UNDERSPEED (1U << 2)
#define MCCONF_L_ADDITIONAL_FAULT_ABS_SPEED  (1U << 3)
/* VESC 6.00 has no wire fields for the newer battery-regeneration cut.
 * Keep it as a board safety policy derived from l_max_vin: full regen is
 * allowed below (max_vin - 2 V) and tapered to zero by (max_vin - 0.5 V). */
#define MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V 2.0f
#define MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V   0.5f

/* VESC 6.00 FOC features that are implemented by this port. Values follow
 * upstream semantics; conservative board defaults keep optional features off
 * until configured from VESC Tool. */
#define MCCONF_FOC_OBSERVER_OFFSET_DEFAULT    -1.0f
#define MCCONF_FOC_DUTY_DOWNRAMP_KP_DEFAULT  10.0f
#define MCCONF_FOC_DUTY_DOWNRAMP_KI_DEFAULT  200.0f
/* Present in the VESC-6.00 wire image at offsets 195/197. A value of 1.0
 * keeps the legacy behavior; values below 1.0 reduce available acceleration
 * current near zero speed and ramp back to full current by the RPM threshold. */
#define MCCONF_FOC_START_CURR_DEC_DEFAULT     1.0f
#define MCCONF_FOC_START_CURR_DEC_RPM_DEFAULT 2500.0f
/* Current VESC can hold all low-side FETs on for an exact zero vector. Keep
 * this OFF on the stock hoverboard until continuous low-side gate drive and
 * bootstrap behavior have been verified physically. */
#define MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT false
#define MCCONF_FOC_CURRENT_FILTER_CONST_DEFAULT 0.1f
#define MCCONF_FOC_CC_DECOUPLING_DEFAULT     FOC_CC_DECOUPLING_DISABLED
#define MCCONF_FOC_SAT_COMP_MODE_DEFAULT     SAT_COMP_DISABLED
#define MCCONF_FOC_SAT_COMP_DEFAULT          0.0f
#define MCCONF_FOC_OBSERVER_TYPE_DEFAULT     FOC_OBSERVER_ORTEGA_ORIGINAL
#define MCCONF_FOC_MTPA_MODE_DEFAULT         MTPA_MODE_OFF
#define MCCONF_FOC_FW_CURRENT_MAX_DEFAULT    0.0f
#define MCCONF_FOC_FW_DUTY_START_DEFAULT     0.90f
#define MCCONF_FOC_FW_RAMP_TIME_DEFAULT      0.20f
#define MCCONF_FOC_FW_Q_CURRENT_FACTOR_DEFAULT 0.02f
/* Newer VESC fields are runtime-only while the wire ABI reports VESC 6.00. */
#define MCCONF_FOC_FW_BACKOFF_DEFAULT         0.0f
#define MCCONF_FOC_MAG_VD_MAX_DEFAULT        1.0f
#define MCCONF_FOC_OVERMOD_FACTOR_DEFAULT    1.0f
/* Newer VESC fields implemented by this port (runtime-only while wire ABI
 * reports VESC 6.00). The F103 port has no motor NTC, so temp compensation
 * uses the STM32 board-temperature proxy as the thermal input. */
#define MCCONF_FOC_TEMP_COMP_DEFAULT               false
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP_DEFAULT     25.0f
/* Offset calibration mode bits: 1 = driven, 2 = undriven, 4 = periodic
 * recalibration on motor-stopped (state OFF). Keep bit2 OFF until the
 * zero-vector/safety behavior of the hoverboard bridge is verified live. */
#define MCCONF_FOC_OFFSETS_CAL_MODE_DEFAULT        1

/* FOC defaults are board-qualified values from appconf_default.h. */
#define MCCONF_FOC_F_ZV_DEFAULT               VESC_FOC_F_ZV_HZ
#define MCCONF_FOC_SENSOR_LEFT_DEFAULT        FOC_SENSOR_MODE_ENCODER_AB
#define MCCONF_FOC_SENSOR_RIGHT_DEFAULT       FOC_SENSOR_MODE_HALL
#define MCCONF_SENSOR_PORT_LEFT_DEFAULT       SENSOR_PORT_MODE_ABI
#define MCCONF_SENSOR_PORT_RIGHT_DEFAULT      SENSOR_PORT_MODE_HALL
