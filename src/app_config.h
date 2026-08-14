#pragma once


/* ===================== CLOCK / PWM ===================== */
#define CPU_CLOCK_HZ                    64000000UL
#define PWM_FREQUENCY_HZ                16000UL
#define PWM_TIMER_ARR                   (CPU_CLOCK_HZ / (2UL * PWM_FREQUENCY_HZ))
#define PWM_DEADTIME_NS                 750UL
#define PWM_MIN_DUTY                    0.10f
#define PWM_MAX_DUTY                    0.90f
#define PWM_MIN_DUTY_Q15                3277U
#define PWM_MAX_DUTY_Q15                29491U
/* VESC-style V0/V7 current sampling: TIM8 update resets TIM2 twice per
 * center-aligned PWM period. TIM2 CC2 therefore produces 2 ADC events/PWM;
 * each event services one motor according to TIM1 direction. */
#define VESC_CURRENT_SAMP_OFFSET_TICKS  500UL
#define FOC_SAMPLE_EVENTS_PER_PWM       2UL
#define FOC_ISR_EVENT_HZ                (PWM_FREQUENCY_HZ * FOC_SAMPLE_EVENTS_PER_PWM)
#define FOC_ISR_SLOT_CYCLES             (CPU_CLOCK_HZ / FOC_ISR_EVENT_HZ)
#define FOC_DT_S                        (1.0f / (float)PWM_FREQUENCY_HZ)

/* ===================== POWER-STAGE POLARITY =====================
 * User hardware requirement:
 *   high-side driver input: ACTIVE HIGH
 *   low-side  driver input: ACTIVE LOW
 * The advanced timers still generate complementary CHx/CHxN with deadtime.
 * CHx idle is LOW (top FET off), CHxN idle is HIGH (bottom FET off).
 */
#define HIGH_SIDE_ACTIVE_HIGH           1
#define LOW_SIDE_ACTIVE_LOW             1

/* ===================== RUNTIME SENSOR MODE =====================
 * No compile-time LEFT_SENSOR_MODE / RIGHT_SENSOR_MODE is used.
 * LEFT pins PB6/PB7 are shared between TIM4 encoder and Hall V/W, therefore
 * runtime switching safely stops the motor and reconfigures the pins.
 */
#define SENSOR_MODE_AUTO                0U
#define SENSOR_MODE_HALL                1U
#define SENSOR_MODE_ENCODER             2U
#define SENSOR_MODE_FORCED_OPENLOOP     3U
#define LEFT_SENSOR_BOOT_MODE           SENSOR_MODE_AUTO
#define RIGHT_SENSOR_BOOT_MODE          SENSOR_MODE_AUTO

/* ===================== MOTOR ===================== */
#define LEFT_POLE_PAIRS                 15U
#define RIGHT_POLE_PAIRS                15U
#define LEFT_ENCODER_CPR                4000U

/* Electrical calibration values are runtime variables. These defaults are
 * only fallback values until autodetect/calibration has run. */
#define LEFT_ENCODER_ELEC_OFFSET_DEG_DEFAULT   0.0f
#define LEFT_ENCODER_INVERTED_DEFAULT          0
#define LEFT_HALL_ELEC_OFFSET_DEG_DEFAULT      0.0f
#define RIGHT_HALL_ELEC_OFFSET_DEG_DEFAULT     0.0f

/* Fallback Hall sequence only. Runtime Hall auto-detect replaces it. */
#define HALL_TABLE_DEFAULT              { -1, 0, 4, 5, 2, 1, 3, -1 }

/* ===================== CURRENT / VOLTAGE CALIBRATION =====================
 * A/count and V/count depend on shunt, op-amp and divider values. They must
 * match the PCB. Offset-to-zero calibration is automatic at every boot.
 */
#define LEFT_CURRENT_A_PER_COUNT        0.0100f
#define RIGHT_CURRENT_A_PER_COUNT       0.0100f
#define LEFT_DC_CURRENT_A_PER_COUNT     0.0100f
#define RIGHT_DC_CURRENT_A_PER_COUNT    0.0100f
#define DCLINK_V_PER_COUNT              0.0200f

/* VESC-style boot current-offset calibration. */
#define ADC_OFFSET_CAL_SAMPLES          4096U
/* V12 calibration policy:
 * - VESC upstream calibrates by averaging current offsets and does not reject
 *   a board merely because the raw ADC has modest PWM-synchronous noise.
 * - Keep broad 12-bit rail/saturation and extreme-noise checks as HARD safety.
 * - Keep tighter noise limits as WARNING diagnostics only.
 * This avoids the V11 false-negative caused by stddev <= 12 counts while still
 * refusing a disconnected/railed/violently noisy current-sense channel. */
#define ADC_OFFSET_HARD_MIN_COUNT       128
#define ADC_OFFSET_HARD_MAX_COUNT       3967
#define ADC_OFFSET_WARN_SPREAD_COUNT    160U
#define ADC_OFFSET_WARN_STDDEV_COUNT    16U
#define ADC_OFFSET_HARD_SPREAD_COUNT    800U
#define ADC_OFFSET_HARD_STDDEV_COUNT    80U
#define ADC_OFFSET_TRACK_SHIFT          12U /* only when motor is OFF: 1/4096 */

/* Low-pass only for telemetry/slow functions. The raw Id/Iq values remain the
 * feedback to the current PI, matching the VESC design principle. */
#define FOC_CURRENT_FILTER_CONST        0.10f
#define FOC_CURRENT_FILTER_Q15          3277 /* 0.10 * 32768 */
#define VBUS_FILTER_CONST               0.05f
#define DC_CURRENT_FILTER_CONST         0.10f

/* ===================== FAST FOC FIXED-POINT BASES ===================== */
#define FOC_CURRENT_Q_BASE_A            64.0f
#define FOC_VOLTAGE_Q_BASE_V            64.0f

/* ===================== FOC CURRENT PI ===================== */
#define LEFT_FOC_KP                     0.030f
#define LEFT_FOC_KI                     20.0f
#define RIGHT_FOC_KP                    0.030f
#define RIGHT_FOC_KI                    20.0f
#define FOC_MAX_CURRENT_A               30.0f
#define FOC_ABS_CURRENT_TRIP_A          45.0f
#define FOC_MAX_VOLTAGE_MODULATION      0.80f
#define VBUS_MIN_RUN_V                  8.0f
#define VBUS_MAX_RUN_V                  55.0f

/* ===================== SENSOR AUTODETECT =====================
 * Detection deliberately uses a small D-axis lock current. Increase only
 * after current scaling has been verified with a bench supply/current clamp.
 */
#define SENSOR_DETECT_CURRENT_A         1.5f
#define SENSOR_DETECT_LOCK_MS           500U
#define SENSOR_DETECT_STEP_MS           3U
#define SENSOR_DETECT_SWEEPS            3U
#define SENSOR_DETECT_STEP_DEG          3U
#define SENSOR_DETECT_SETTLE_MS         400U
#define SENSOR_DETECT_MIN_ENCODER_COUNTS 20
#define SENSOR_DETECT_MAX_POLE_PAIRS    40U
#define SENSOR_DETECT_HALL_MIN_SAMPLES  30U

/* ===================== SPEED / POSITION PID ===================== */
#define SPEED_PID_KP                    0.0030f
#define SPEED_PID_KI                    0.0015f
#define SPEED_PID_KD                    0.0000f
#define SPEED_PID_MAX_ERPM              30000.0f
#define POSITION_PID_KP_ERPM_PER_DEG    30.0f
#define POSITION_PID_MAX_ERPM           5000.0f

/* ===================== COMMAND / TELEMETRY ===================== */
#define MOTOR_COMMAND_TIMEOUT_MS        500U
#define VESC_UART_BAUD                  115200U
#define STAT_PERIOD_MS                  10U
#define ROTOR_STREAM_PERIOD_MS          10U
#define SAMPLE_BUFFER_LEN               256U
#define SAMPLE_DEFAULT_DECIMATION       8U

/* PA5 is the hoverboard power-latch/OFF control. HIGH keeps the controller
 * powered, as in the known-working hoverboard firmware. Bridge enable/disable
 * is performed only with TIM1/TIM8 MOE. */
#define OFF_PIN_POWER_HOLD_HIGH          1

/* ===================== VESC DUAL-MOTOR IDENTITY =====================
 * UART is always motor-left/local. Motor-right is exposed through the exact
 * COMM_FORWARD_CAN dual-motor semantic, without enabling physical CAN HW. */
#define VESC_CONTROLLER_ID_LEFT         1U
#define VESC_CONTROLLER_ID_RIGHT        2U
#define VESC_VIRTUAL_CAN_RIGHT_ID       VESC_CONTROLLER_ID_RIGHT

/* ===================== V9 VESC-COMPATIBLE CONTROL DEFAULTS ===================== */
#define DUTY_PID_KP_CURRENT_PER_DUTY     18.0f
#define DUTY_PID_KI_CURRENT_PER_DUTY_S   55.0f
#define POSITION_PID_KP_CURRENT_PER_DEG  0.12f
#define POSITION_PID_KI_CURRENT_PER_DEG_S 0.02f
#define POSITION_PID_KD_CURRENT_PER_DEGPS 0.002f
#define POSITION_PID_D_FILTER            0.20f
#define SPEED_PID_D_FILTER               0.20f
#define MOTOR_DEFAULT_MIN_DUTY           (-0.95f)
#define MOTOR_DEFAULT_MAX_DUTY           (0.95f)
#define MOTOR_DEFAULT_MIN_ERPM           (-30000.0f)
#define MOTOR_DEFAULT_MAX_ERPM           (30000.0f)
