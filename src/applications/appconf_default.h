#pragma once


/* ===================== CLOCK / PWM ===================== */
#define CPU_CLOCK_HZ                    64000000UL
#define PWM_FREQUENCY_HZ                16000UL
#define PWM_TIMER_ARR                   (CPU_CLOCK_HZ / (2UL * PWM_FREQUENCY_HZ))
#define PWM_DEADTIME_NS                 750UL
/* VESC foc_f_zv is the zero-vector update frequency. With center-aligned
 * PWM and one current-control sample per PWM period, it is twice the physical
 * 16 kHz bridge switching frequency. foc_dt_us is dead-time compensation,
 * not the FOC loop period. */
#define VESC_FOC_F_ZV_HZ                (2UL * PWM_FREQUENCY_HZ)
#define FOC_DEADTIME_COMP_US             ((float)PWM_DEADTIME_NS / 1000.0f)
#define PWM_MIN_DUTY                    0.10f
#define PWM_MAX_DUTY                    0.90f
#define PWM_MIN_DUTY_Q15                3277U
#define PWM_MAX_DUTY_Q15                29491U
/* Stock EFeru hoverboard timing: one dual-ADC current frame per PWM period.
 * ADC rank order is DC-link currents, LEFT phase pair, RIGHT phase pair. TIM8
 * (LEFT) is advanced from TIM1 (RIGHT) by exactly one phase-current ADC
 * conversion so the two phase pairs are sampled in their respective low-side
 * measurement windows. We retain ADC /6 (10.67 MHz) to stay inside STM32F103
 * limits, therefore 20 ADC clocks = 120 CPU timer ticks. */
#define ADC_CLOCK_DIV                   6UL
#define ADC_PHASE_CONV_CYCLES           20UL /* 7.5 sample + 12.5 conversion */
#define ADC_MOTOR_PHASE_OFFSET_TICKS    (ADC_CLOCK_DIV * ADC_PHASE_CONV_CYCLES)
#define FOC_SAMPLE_EVENTS_PER_PWM       1UL
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
#define LEFT_SENSOR_BOOT_MODE           SENSOR_MODE_ENCODER
#define RIGHT_SENSOR_BOOT_MODE          SENSOR_MODE_HALL

/* ===================== MOTOR ===================== */
#define LEFT_POLE_PAIRS                 4U
#define RIGHT_POLE_PAIRS                15U

/* LEFT quadrature encoder specification is 1024 PPR. STM32 TIM4 encoder
 * mode TI12 counts both edges on both A and B, therefore the runtime/ABI
 * encoder count is 4 * PPR = 4096 counts per mechanical revolution. */
#define LEFT_ENCODER_PPR                1024U
#define LEFT_ENCODER_CPR                (LEFT_ENCODER_PPR * 4U)

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
/* Stock EFeru hoverboard analog front-end baseline: A2BIT_CONV = 50,
 * therefore 1 ADC count ~= 1/50 A = 0.020 A. Keep these board constants
 * separate from MCCONF; verify with a current clamp on non-stock boards. */
#define LEFT_CURRENT_A_PER_COUNT        0.0200f
#define RIGHT_CURRENT_A_PER_COUNT       0.0200f
#define LEFT_DC_CURRENT_A_PER_COUNT     0.0200f
#define RIGHT_DC_CURRENT_A_PER_COUNT    0.0200f
/* Stock EFeru battery divider calibration in current upstream config is
 * 39.70 V at ADC=1492. PC2/ADC12 is the stock DCLINK/battery sense input on
 * this mainboard, so use the same scale for VESC input-voltage telemetry.
 * Board-specific calibration can still replace this constant later. */
#define DCLINK_V_PER_COUNT              (39.70f / 1492.0f)

/* VESC-style boot current-offset calibration. */
#define ADC_OFFSET_CAL_SAMPLES          4096U
/* EFeru-reference boot offset averaging: number of 16-kHz DMA ISR frames the
 * default boot path averages the six raw current channels before the FOC runs
 * with the converging offsets (EFeru semantics: cur = offset - raw). 2000
 * frames ~= 125 ms, matching the reference running-average window. Only used
 * on the default boot path; VESC Tool recalibration uses the driven pipeline. */
#define ADC_BOOT_CAL_SAMPLES            2000U
/* VESC-style driven current-offset calibration. Upstream VESC stores
 * driven and undriven offsets separately; for low-side-shunt hardware its
 * alternative calibration runs all three phases at 50% (zero SVM amplitude)
 * and averages 1000 samples. We preserve an undriven diagnostic pass, then
 * calibrate LEFT and RIGHT one at a time under 50% zero-vector PWM. */
#define ADC_DRIVEN_CAL_SAMPLES          1000U
#define ADC_DRIVEN_CAL_SAMPLE_HZ        1000U
#define ADC_DRIVEN_CAL_DECIMATION       (FOC_ISR_EVENT_HZ / ADC_DRIVEN_CAL_SAMPLE_HZ)
#define ADC_DRIVEN_CAL_WARMUP_EVENTS    64U
#define ADC_DRIVEN_CAL_MAX_DC_A         6.0f
#define ADC_DRIVEN_CAL_MAX_DC_COUNTS    300U /* 6 A at 0.02 A/count */
#define ADC_DRIVEN_CAL_DC_TRIP_SAMPLES  8U
#define ADC_CAL_VBUS_STABLE_5MS_TICKS   400U /* 2.0 s, matching VESC settle window */
#define ADC_CAL_VBUS_STABLE_DELTA_V     1.50f /* restart settle timer when Vbus moves more than this */
#define PWM_ENABLE_BLANK_CYCLES         8U
/* EFeru uses I_DC_MAX=17 A for the stage-2 DC-link current chop on the stock
 * board. Do not reuse the much lower 6 A driven-calibration sanity limit for
 * normal PWM startup; that coupling caused a false ABS_OVER_CURRENT during
 * Hall detect before the current ramp had even begun. */
#define PWM_STARTUP_DC_TRIP_A           17.0f
#define PWM_STARTUP_DC_TRIP_DEBOUNCE_SAMPLES 4U /* 250 us at 16 kHz; fits inside the 8-event startup blanking window while rejecting a one-sample switching spike */
#define PWM_ENABLE_PRELOAD_EVENTS       2U
/* Current-offset validation policy:
 * - VESC upstream calibrates by averaging current offsets and does not reject
 *   a board merely because the raw ADC has modest PWM-synchronous noise.
 * - Keep broad 12-bit rail/saturation and extreme-noise checks as HARD safety.
 * - Keep tighter noise limits as WARNING diagnostics only.
 * This avoids false rejection caused by stddev <= 12 counts while still
 * refusing a disconnected/railed/violently noisy current-sense channel. */
#define ADC_OFFSET_HARD_MIN_COUNT       128
#define ADC_OFFSET_HARD_MAX_COUNT       3967
#define ADC_OFFSET_WARN_SPREAD_COUNT    160U
#define ADC_OFFSET_WARN_STDDEV_COUNT    16U
#define ADC_OFFSET_HARD_STDDEV_COUNT    80U
#define ADC_OFFSET_INLIER_WINDOW_COUNT  256U
#define ADC_OFFSET_HARD_OUTLIER_COUNT   10U
#define ADC_OFFSET_MOE_WAIT_EVENTS      128U

/* Incremental A/B has no absolute electrical origin after cold boot.
 * FOC_SENSOR_MODE_ENCODER_AB uses observer/open-loop startup to establish
 * the runtime phase; encoder_set_deg() rebases the ABI counter. No HFI and
 * no standstill "CNT=0 means electrical zero" shortcut are used. */

/* Low-pass only for telemetry/slow functions. The raw Id/Iq values remain the
 * feedback to the current PI, matching the VESC design principle. */
#define FOC_CURRENT_FILTER_CONST        0.10f
#define FOC_CURRENT_FILTER_Q15          3277 /* 0.10 * 32768 */
#define VBUS_FILTER_CONST               0.05f
#define DC_CURRENT_FILTER_CONST         0.10f

/* ===================== FAST FOC FIXED-POINT BASES ===================== */
#define FOC_CURRENT_Q_BASE_A            64.0f
#define FOC_VOLTAGE_Q_BASE_V            64.0f
/* Fixed-point flux observer base. Rotor/stator flux states use Q30 where
 * 1.0 represents this many webers. 50 mWb covers hoverboard-class motors
 * while retaining fine resolution for the usual few-mWb linkage values. */
#define FOC_FLUX_Q_BASE_WB              0.050f
#define FOC_OBSERVER_SPEED_ALPHA_Q15    1638 /* 0.05 */

/* ===================== FOC CURRENT PI ===================== */
#define LEFT_FOC_KP                     0.030f
#define LEFT_FOC_KI                     20.0f
#define RIGHT_FOC_KP                    0.030f
#define RIGHT_FOC_KI                    20.0f
/* Conservative stock-hoverboard commissioning defaults. VESC MCCONF can
 * lower these further after current scaling is verified. */
#define FOC_MAX_CURRENT_A               15.0f
#define FOC_ABS_CURRENT_TRIP_A          25.0f
#define FOC_ABS_CURRENT_FILTER_ALPHA_Q15 4096  /* 0.125 @ 16 kHz */
#define FOC_ABS_CURRENT_FAULT_DEBOUNCE_SAMPLES 4U
#define FOC_HARD_CURRENT_FAULT_DEBOUNCE_SAMPLES 4U /* 250 us at 16 kHz; hardware BKIN remains immediate */
#define FOC_MAX_VOLTAGE_MODULATION      0.80f
#define VBUS_MIN_RUN_V                  8.0f
#define VBUS_MAX_RUN_V                  55.0f
#define FOC_VBUS_FAULT_DEBOUNCE_SAMPLES 4U
#define FOC_VBUS_DMA_STALE_FAULT_SAMPLES 3U /* ADC3 DMA must advance each PWM trigger */
#define FOC_VBUS_HARD_OV_MARGIN_V       3.0f
#define FOC_VBUS_HARD_UV_MARGIN_V       2.0f
#define FOC_VBUS_HARD_MAX_V             60.0f
#define FOC_VBUS_HARD_MIN_V             4.0f

/* ===================== SENSOR AUTODETECT =====================
 * Detection deliberately uses a small D-axis lock current. Increase only
 * after current scaling has been verified with a bench supply/current clamp.
 */
#define SENSOR_DETECT_CURRENT_A         1.5f
#define SENSOR_DETECT_MAX_CURRENT_A     2.0f
/* Hall detect uses OPEN-LOOP SVPWM (no current PI) with a tiny fixed duty.
 * The closed-loop current PI on this F103 port is not yet trustworthy for
 * sub-2A excitation: a 0.1-0.5 A request drives Id/Iq to several amps and
 * trips ABS_OVER_CURRENT. A 1.5% modulation at ~44 V bus yields a small, safe
 * phase field that is more than enough to sample the Hall states during the
 * electrical sweep. This matches the "try open-loop SVPWM first" request. */
#define SENSOR_DETECT_DUTY              0.015f
#define SENSOR_DETECT_CURRENT_RAMP_MS   1000U
#define SENSOR_DETECT_STEP_MS           5U
#define SENSOR_DETECT_SWEEPS            3U
#define SENSOR_DETECT_STEP_DEG          1U
#define SENSOR_DETECT_SETTLE_MS         500U
#define SENSOR_DETECT_MIN_ENCODER_COUNTS 20
#define SENSOR_DETECT_MAX_POLE_PAIRS    40U
#define SENSOR_DETECT_HALL_MIN_SAMPLES  30U

/* ===================== SPEED / POSITION PID ===================== */
#define SPEED_PID_KP                    0.0030f
#define SPEED_PID_KI                    0.0015f
#define SPEED_PID_KD                    0.0000f
#define SPEED_PID_MIN_ERPM              100.0f
#define SPEED_PID_ALLOW_BRAKING         true
#define SPEED_PID_RAMP_ERPMS_S          35000.0f
#define SPEED_PID_SOURCE_DEFAULT        S_PID_SPEED_SRC_PLL
#define SPEED_PID_MAX_ERPM              30000.0f
#define POSITION_PID_KP_ERPM_PER_DEG    30.0f
#define POSITION_PID_MAX_ERPM           5000.0f

/* ===================== COMMAND / TELEMETRY ===================== */
#define MOTOR_COMMAND_TIMEOUT_MS        500U
#define VESC_UART_BAUD                  115200U
#define STAT_PERIOD_MS                  10U
#define ROTOR_STREAM_PERIOD_MS          10U
#define SAMPLE_BUFFER_LEN                64U
#define SAMPLE_DEFAULT_DECIMATION       8U

/* PA5 is the hoverboard power-latch/OFF control. HIGH keeps the controller
 * powered, as in the known-working hoverboard firmware. Bridge enable/disable
 * is performed only with TIM1/TIM8 MOE. */
#define OFF_PIN_POWER_HOLD_HIGH          1

/* ===================== VESC DUAL-MOTOR IDENTITY =====================
 * UART default adalah motor-left. Untuk kompatibilitas cabang upstream
 * HW_HAS_DUAL_MOTORS, COMM_FORWARD_CAN dengan ID motor-right diperlakukan
 * sebagai pemilihan motor lokal. Tidak ada driver CAN/CAN PHY driver pada build. */
#define VESC_CONTROLLER_ID_LEFT         1U
#define VESC_CONTROLLER_ID_RIGHT        2U
#define VESC_LOCAL_MOTOR2_FORWARD_ID    VESC_CONTROLLER_ID_RIGHT

/* ===================== VESC-COMPATIBLE CONTROL DEFAULTS ===================== */
#define DUTY_PID_KP_CURRENT_PER_DUTY     18.0f
#define DUTY_PID_KI_CURRENT_PER_DUTY_S   55.0f
#define POSITION_PID_KP_CURRENT_PER_DEG  0.12f
#define POSITION_PID_KI_CURRENT_PER_DEG_S 0.02f
#define POSITION_PID_KD_CURRENT_PER_DEGPS 0.002f
#define POSITION_PID_D_FILTER            0.20f
#define POSITION_PID_KD_PROC              0.0f
#define POSITION_PID_ANG_DIV              1.0f
#define POSITION_PID_GAIN_DEC_ANGLE       0.0f
#define POSITION_PID_OFFSET_DEG            0.0f
#define SPEED_PID_D_FILTER               0.20f
#define CURRENT_CTRL_MIN_CURRENT_A        0.0f
#define MOTOR_DEFAULT_MIN_DUTY           (0.0f)
#define MOTOR_DEFAULT_MAX_DUTY           (0.95f)
#define MOTOR_DEFAULT_MIN_ERPM           (-30000.0f)
#define MOTOR_DEFAULT_MAX_ERPM           (30000.0f)


/* VESC-style sensorless/ENCODER_AB startup defaults. These are deliberately
 * conservative for stock hoverboard motors and can be replaced by Detect All. */
#define MCCONF_FOC_MOTOR_R_DEFAULT              0.050f
#define MCCONF_FOC_MOTOR_L_DEFAULT              0.000020f
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE_DEFAULT   0.0030f
#define MCCONF_FOC_OBSERVER_GAIN_DEFAULT        900000.0f
#define MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT   0.05f
#define MCCONF_FOC_PLL_KP_DEFAULT               2000.0f
#define MCCONF_FOC_PLL_KI_DEFAULT               30000.0f
#define MCCONF_FOC_SL_ERPM_START_DEFAULT        1200.0f
#define MCCONF_FOC_SL_ERPM_DEFAULT              2500.0f
#define MCCONF_FOC_OPENLOOP_RPM_DEFAULT         900.0f
#define MCCONF_FOC_OPENLOOP_RPM_LOW_DEFAULT     350.0f
#define MCCONF_FOC_SL_OPENLOOP_HYST_DEFAULT     0.10f
#define MCCONF_FOC_SL_OPENLOOP_T_LOCK_DEFAULT   0.18f
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP_DEFAULT   0.55f
#define MCCONF_FOC_SL_OPENLOOP_TIME_DEFAULT     0.35f
#define MCCONF_FOC_SL_OPENLOOP_BOOST_Q_DEFAULT  1.5f
#define MCCONF_FOC_SL_OPENLOOP_MAX_Q_DEFAULT    6.0f
#define FOC_OBSERVER_MIN_FLUX_FACTOR             0.20f
#define FOC_OBSERVER_MAX_FLUX_FACTOR             3.50f
#define FOC_ENCODER_SWITCH_HYST                  0.05f
#define FOC_DETECT_CURRENT_A                     2.0f
#define FOC_DETECT_MAX_CURRENT_A                 5.0f
#define FOC_DETECT_FLUX_CURRENT_A                1.5f
#define FOC_DETECT_SETTLE_MS                     500U
#define FOC_DETECT_R_SAMPLES                     400U
#define FOC_DETECT_L_PULSE_VOLTAGE               2.0f
#define FOC_DETECT_L_PULSE_MS                    4U
#define FOC_DETECT_FLUX_ERPM                     1800.0f
#define FOC_DETECT_FLUX_SETTLE_MS                1200U
