#pragma once
#include "stm32f1xx_hal.h"
#include "applications/appconf_default.h"

/* LEFT HALL */
#define LEFT_HALL_U_PIN GPIO_PIN_5
#define LEFT_HALL_V_PIN GPIO_PIN_6
#define LEFT_HALL_W_PIN GPIO_PIN_7
#define LEFT_HALL_U_PORT GPIOB
#define LEFT_HALL_V_PORT GPIOB
#define LEFT_HALL_W_PORT GPIOB

/* RIGHT HALL */
#define RIGHT_HALL_U_PIN GPIO_PIN_10
#define RIGHT_HALL_V_PIN GPIO_PIN_11
#define RIGHT_HALL_W_PIN GPIO_PIN_12
#define RIGHT_HALL_U_PORT GPIOC
#define RIGHT_HALL_V_PORT GPIOC
#define RIGHT_HALL_W_PORT GPIOC

/* LEFT PWM = TIM8 */
#define LEFT_TIM TIM8
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_6
#define LEFT_TIM_UH_PORT GPIOC
#define LEFT_TIM_UL_PIN GPIO_PIN_7
#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_7
#define LEFT_TIM_VH_PORT GPIOC
#define LEFT_TIM_VL_PIN GPIO_PIN_0
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_8
#define LEFT_TIM_WH_PORT GPIOC
#define LEFT_TIM_WL_PIN GPIO_PIN_1
#define LEFT_TIM_WL_PORT GPIOB

/* RIGHT PWM = TIM1 */
#define RIGHT_TIM TIM1
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_UH_PIN GPIO_PIN_8
#define RIGHT_TIM_UH_PORT GPIOA
#define RIGHT_TIM_UL_PIN GPIO_PIN_13
#define RIGHT_TIM_UL_PORT GPIOB
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_VH_PIN GPIO_PIN_9
#define RIGHT_TIM_VH_PORT GPIOA
#define RIGHT_TIM_VL_PIN GPIO_PIN_14
#define RIGHT_TIM_VL_PORT GPIOB
#define RIGHT_TIM_W CCR3
#define RIGHT_TIM_WH_PIN GPIO_PIN_10
#define RIGHT_TIM_WH_PORT GPIOA
#define RIGHT_TIM_WL_PIN GPIO_PIN_15
#define RIGHT_TIM_WL_PORT GPIOB

/* ADC */
#define LEFT_DC_CUR_PIN GPIO_PIN_0
#define LEFT_U_CUR_PIN GPIO_PIN_0
#define LEFT_V_CUR_PIN GPIO_PIN_3
#define LEFT_DC_CUR_PORT GPIOC
#define LEFT_U_CUR_PORT GPIOA
#define LEFT_V_CUR_PORT GPIOC

#define RIGHT_DC_CUR_PIN GPIO_PIN_1
#define RIGHT_U_CUR_PIN GPIO_PIN_4
#define RIGHT_V_CUR_PIN GPIO_PIN_5
#define RIGHT_DC_CUR_PORT GPIOC
#define RIGHT_U_CUR_PORT GPIOC
#define RIGHT_V_CUR_PORT GPIOC

#define DCLINK_PIN GPIO_PIN_2
#define DCLINK_PORT GPIOC

/* STM32F103 internal temperature sensor. There is no factory two-point
 * calibration constant on this target, so the default conversion uses the
 * datasheet-typical V25/slope and nominal 3.3-V VDDA. These constants are
 * intentionally board-overridable after bench calibration. The result is a
 * board/MCU thermal proxy, NOT a MOSFET-junction measurement. */
#ifndef HOVERBOARD_MCU_TEMP_V25_V
#define HOVERBOARD_MCU_TEMP_V25_V              1.43f
#endif
#ifndef HOVERBOARD_MCU_TEMP_AVG_SLOPE_V_PER_C
#define HOVERBOARD_MCU_TEMP_AVG_SLOPE_V_PER_C  0.0043f
#endif
#ifndef HOVERBOARD_ADC_VDDA_NOMINAL_V
#define HOVERBOARD_ADC_VDDA_NOMINAL_V          3.30f
#endif
#define HOVERBOARD_MCU_TEMP_ADC_MIN_VALID       300U
#define HOVERBOARD_MCU_TEMP_ADC_MAX_VALID      3800U
#define HOVERBOARD_MCU_TEMP_MIN_VALID_C         -40.0f
#define HOVERBOARD_MCU_TEMP_MAX_VALID_C         125.0f


/* Batch 9.3 power-stage safety. The stock schematic/wiring has not been
 * validated to expose a gate-driver/comparator fault on BKIN, so both
 * external-break inputs stay OFF by default. If a board revision wires a
 * fault signal, TIM1 uses default BKIN PB12 and TIM8 uses default BKIN PA6.
 * Hardware break itself clears MOE asynchronously; software only latches and
 * reports the event. Do not enable these macros without checking polarity and
 * the actual PCB net with a scope/multimeter. */
#ifndef HOVERBOARD_TIM1_BREAK_ENABLE
#define HOVERBOARD_TIM1_BREAK_ENABLE          0
#endif
#ifndef HOVERBOARD_TIM8_BREAK_ENABLE
#define HOVERBOARD_TIM8_BREAK_ENABLE          0
#endif
#ifndef HOVERBOARD_TIM1_BREAK_ACTIVE_HIGH
#define HOVERBOARD_TIM1_BREAK_ACTIVE_HIGH     0
#endif
#ifndef HOVERBOARD_TIM8_BREAK_ACTIVE_HIGH
#define HOVERBOARD_TIM8_BREAK_ACTIVE_HIGH     0
#endif
#define HOVERBOARD_TIM1_BKIN_PIN              GPIO_PIN_12
#define HOVERBOARD_TIM1_BKIN_PORT             GPIOB
#define HOVERBOARD_TIM8_BKIN_PIN              GPIO_PIN_6
#define HOVERBOARD_TIM8_BKIN_PORT             GPIOA

/* Internal STM32 PVD is safe to use without external PCB wiring. PLS=111 is
 * the highest STM32F1 threshold band (nominally about 2.9 V). A falling VDD
 * latches both bridges off until reset; it is not an auto-recovery limiter. */
#ifndef HOVERBOARD_PVD_ENABLE
#define HOVERBOARD_PVD_ENABLE                 1
#endif
#define HOVERBOARD_PVD_PLS_BITS               (7U << 5)

/* UI */
#define LED_PIN GPIO_PIN_2
#define LED_PORT GPIOB
#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_PORT GPIOA
#define OFF_PIN GPIO_PIN_5
#define OFF_PORT GPIOA
#define BUTTON_PIN GPIO_PIN_1
#define BUTTON_PORT GPIOA

/* LEFT encoder AB. Runtime sensor selection reuses PB6/PB7, therefore LEFT
 * Hall and encoder are never enabled at the same time. */
#define LEFT_ENCODER_TIM TIM4
#define LEFT_ENCODER_A_PIN GPIO_PIN_6
#define LEFT_ENCODER_A_PORT GPIOB
#define LEFT_ENCODER_B_PIN GPIO_PIN_7
#define LEFT_ENCODER_B_PORT GPIOB

/* Permanent VESC Tool UART interface for this PCB. */
#define VESC_UART USART3
#define VESC_UART_TX_PIN GPIO_PIN_10
#define VESC_UART_TX_PORT GPIOB
#define VESC_UART_RX_PIN GPIO_PIN_11
#define VESC_UART_RX_PORT GPIOB

/* ============================================================================
 * Board capability gates (VESC HW_HAS_* convention, F103 hoverboard port).
 * Derived from the former vesc_f103_capabilities.h capability table.
 * SUPPORTED -> HW_HAS_<FEATURE> defined; HARDWARE_UNAVAILABLE/UNSUPPORTED -> omitted;
 * PLATFORM_EXTENSION -> HW_F103_EXT_<FEATURE> defined.
 * ============================================================================ */
#define HW_HAS_MOTOR_TYPE_FOC
#define HW_HAS_FOC_CURRENT_CONTROL
#define HW_HAS_FOC_DUTY_CONTROL
#define HW_HAS_FOC_SPEED_CONTROL
#define HW_HAS_FOC_POSITION_CONTROL
#define HW_HAS_FOC_CURRENT_PI
#define HW_HAS_FOC_MTPA
#define HW_HAS_FOC_DECOUPLING
#define HW_HAS_FOC_FIELD_WEAKENING
#define HW_HAS_FOC_OBSERVER_ORTEGA
#define HW_HAS_FOC_OBSERVER_SPLINE
#define HW_F103_EXT_FOC_OBSERVER_LUENBERGER
#define HW_HAS_FOC_OBSERVER_SENSORLESS
#define HW_HAS_FOC_ENCODER_LEFT
#define HW_HAS_FOC_HALL_BOTH
#define HW_HAS_FOC_SENSORLESS_STARTUP
#define HW_HAS_FOC_CURRENT_RAMP
#define HW_HAS_FOC_DUTY_RAMP
#define HW_HAS_FOC_MOTOR_R_L_DETECT
#define HW_HAS_FOC_MOTOR_FLUX_DETECT
#define HW_HAS_FOC_MOTOR_HALL_DETECT
#define HW_HAS_FOC_MOTOR_ENCODER_DETECT
#define HW_HAS_FOC_OFFSET_CALIBRATION
#define HW_HAS_FOC_DRIVE_CURRENT_FILTER
#define HW_HAS_FOC_MODULATION_INDEX
#define HW_HAS_FOC_SATURATION_COMP
#define HW_HAS_FOC_DEAD_TIME_COMP
#define HW_HAS_FOC_OBSERVER_GAIN_SCHEME
#define HW_HAS_PWM_CENTER_ALIGNED
#define HW_HAS_PWM_SYNCHRONOUS
#define HW_HAS_SAMPLE_CURRENT
#define HW_HAS_SAMPLE_RING_BUFFER
#define HW_HAS_SAMPLE_NOW
#define HW_HAS_SAMPLE_START
#define HW_HAS_SAMPLE_TRIGGER_START
#define HW_HAS_SAMPLE_TRIGGER_FAULT
#define HW_HAS_SAMPLE_SEND_SINGLE
#define HW_HAS_SAMPLE_SEND_LAST
#define HW_HAS_SAMPLE_CHRONOLOGICAL
#define HW_HAS_SAMPLE_RAW_MODE
#define HW_HAS_SAMPLE_REPLY_FUNC
#define HW_HAS_FAULT_OVERCURRENT
#define HW_HAS_FAULT_UNDERVOLTAGE
#define HW_HAS_FAULT_OVERVOLTAGE
#define HW_HAS_FAULT_OVERTEMP_BOARD
#define HW_HAS_FAULT_OVERTEMP_MOTOR
#define HW_HAS_FAULT_DRV
#define HW_HAS_FAULT_TIMEOUT
#define HW_HAS_FAULT_ADC_DMA
#define HW_HAS_FAULT_HALL_INVALID
#define HW_HAS_FAULT_OVERSPEED
#define HW_HAS_FAULT_UNDERSPEED
#define HW_HAS_FAULT_ENCODER_SLIP
#define HW_HAS_FAULT_FLASH_CONFIG
#define HW_HAS_FAULT_CURRENT_OFFSET
#define HW_HAS_FAULT_FAULT_METADATA
#define HW_HAS_COMM_UART
#define HW_HAS_COMM_PACKET_CRC16
#define HW_HAS_COMM_APP_CONFIG
#define HW_HAS_COMM_MCCONF_V6
#define HW_HAS_COMM_MCCONF_V6_CRC
#define HW_HAS_COMM_FACTORY_RESET
#define HW_HAS_COMM_CONF_PERSISTENCE
#define HW_HAS_AUDIO_BUZZER
#define HW_HAS_AUDIO_BEEP
#define HW_HAS_ISR_DURATION
#define HW_HAS_ISR_DEADLINE_TRACKING
#define HW_HAS_ISR_PERIOD_JITTER
#define HW_HAS_GET_TOT_CURRENT
#define HW_HAS_GET_TOT_CURRENT_FILTERED
#define HW_HAS_GET_DIRECTIONAL_CURRENT
#define HW_HAS_GET_DIRECTIONAL_FILTERED
#define HW_HAS_GET_INPUT_CURRENT
#define HW_HAS_GET_INPUT_VOLTAGE
#define HW_HAS_GET_DUTY
#define HW_HAS_GET_RPM
#define HW_HAS_GET_POSITION_ENCODER
#define HW_HAS_GET_POSITION_HALL
#define HW_HAS_GET_POSITION_OBSERVER
#define HW_HAS_GET_ERPM
#define HW_HAS_GET_VBUS
#define HW_HAS_GET_TEMP_FET
#define HW_HAS_GET_TEMP_MOTOR
#define HW_HAS_GET_ODOMETER
#define HW_HAS_GET_AH
#define HW_HAS_GET_WH
#define HW_HAS_GET_TACHOMETER
#define HW_HAS_GET_MOD_ALPHA_MEASURED
#define HW_HAS_GET_MOD_BETA_MEASURED
#define HW_HAS_GET_V_ALPHA
#define HW_HAS_GET_V_BETA
#define HW_HAS_GET_VOLTAGE_OFFSETS
#define HW_HAS_SET_CURRENT
#define HW_HAS_SET_CURRENT_BRAKE
#define HW_HAS_SET_DUTY
#define HW_HAS_SET_SPEED
#define HW_HAS_SET_POS
#define HW_HAS_SET_HANDBRAKE
#define HW_HAS_SET_OPENLOOP_CURRENT
#define HW_HAS_SET_OPENLOOP_DUTY
#define HW_HAS_SET_OPENLOOP_PHASE
