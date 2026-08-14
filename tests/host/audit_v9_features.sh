#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "AUDIT V9 FAIL: $*" >&2; exit 1; }

# V8 transport must be byte-for-byte frozen.
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256 >/dev/null || fail "proven V8 transport files changed"

# Standard VESC control command scaling and routing.
grep -q 'motor_set_duty(m, (float)get_i32_be(&data\[1\]) / 100000.0f)' src/vesc_comm.c || fail "COMM_SET_DUTY scaling/routing missing"
grep -q 'motor_set_current(m, (float)get_i32_be(&data\[1\]) / 1000.0f)' src/vesc_comm.c || fail "COMM_SET_CURRENT scaling/routing missing"
grep -q 'motor_set_speed(m, (float)get_i32_be(&data\[1\]))' src/vesc_comm.c || fail "COMM_SET_RPM routing missing"
grep -q 'motor_set_position(m, (float)get_i32_be(&data\[1\]) / 1000000.0f)' src/vesc_comm.c || fail "COMM_SET_POS scaling/routing missing"
grep -q 'case MOTOR_CTRL_DUTY: iq=duty_pid_step' src/motor_control.c || fail "duty outer-loop missing"
grep -q 'case MOTOR_CTRL_SPEED: iq=speed_pid_step' src/motor_control.c || fail "speed outer-loop missing"
grep -q 'case MOTOR_CTRL_POSITION: iq=position_pid_step' src/motor_control.c || fail "position outer-loop missing"
grep -q 'motor_set_foc_targets(m,0.0f,iq)' src/motor_control.c || fail "outer loop does not feed Iq"

# LEFT encoder AB on TIM4/PB6/PB7, no-index session alignment and separate electrical ratio.
grep -q '#define LEFT_ENCODER_A_PIN[[:space:]]*GPIO_PIN_6' src/board_pins.h || fail "LEFT encoder A PB6 missing"
grep -q '#define LEFT_ENCODER_B_PIN[[:space:]]*GPIO_PIN_7' src/board_pins.h || fail "LEFT encoder B PB7 missing"
grep -q 'TIM_ENCODERMODE_TI12' src/motor_hw.c || fail "TIM4 quadrature mode missing"
grep -q 'encoder.session_zero_count' src/sensor_detect.c || fail "per-boot encoder electrical alignment missing"
grep -q 'encoder.mechanical_zero_count' src/sensor_detect.c || fail "encoder position zero missing"
grep -q 'encoder.electrical_ratio_q16' src/vesc_config.c || fail "fractional encoder ratio Q16 mapping missing"
grep -q 'm->pole_pairs=pp' src/vesc_config.c || fail "physical pole-pair mapping missing"
grep -q 'speed_sample_valid' src/motor_control.c || fail "encoder first-sample speed guard missing"
grep -q 'id==MOTOR_RIGHT.*SENSOR_MODE_ENCODER' src/vesc_config.c && fail "RIGHT encoder unexpectedly accepted" || true

# Exact VESC6 config wire sizes and full-wire persistence.
grep -q '#define VESC6_MCCONF_WIRE_SIZE 481U' src/vesc_config.h || fail "481-byte MCCONF missing"
grep -q '#define VESC6_APPCONF_WIRE_SIZE 493U' src/vesc_config.h || fail "493-byte APPCONF missing"
grep -q 'COMM_SET_MCCONF' src/vesc_comm.c || fail "SET_MCCONF missing"
grep -q 'COMM_GET_MCCONF_DEFAULT' src/vesc_comm.c || fail "GET_MCCONF_DEFAULT missing"
grep -q 'COMM_SET_APPCONF' src/vesc_comm.c || fail "SET_APPCONF missing"
grep -q 'COMM_GET_APPCONF_DEFAULT' src/vesc_comm.c || fail "GET_APPCONF_DEFAULT missing"
grep -q 'uint8_t mc_left\[VESC6_MCCONF_WIRE_SIZE\]' src/config_store.c || fail "LEFT MCCONF raw persistence missing"
grep -q 'uint8_t mc_right\[VESC6_MCCONF_WIRE_SIZE\]' src/config_store.c || fail "RIGHT MCCONF raw persistence missing"
grep -q 'uint8_t app\[VESC6_APPCONF_WIRE_SIZE\]' src/config_store.c || fail "APPCONF raw persistence missing"
grep -q 'CFG_PAGE_COUNT[[:space:]]*4U' src/config_store.c || fail "four-page transactional flash store missing"
grep -q 'CFG_REGION_ADDR[[:space:]]*0x0803E000UL' src/config_store.c || fail "8KiB config region start wrong"
grep -q 'board_upload.maximum_size = 253952' platformio.ini || fail "upload flash reserve missing"
grep -q 'board_build.ldscript = stm32f103rc_v9.ld' platformio.ini || fail "custom reserved-flash linker script missing"
grep -q 'FLASH (rx).*LENGTH = 248K' stm32f103rc_v9.ld || fail "linker FLASH length does not reserve final 8KiB"

# Fixed-point FOC hot path and precision/CPU features.
grep -q 'current_kp_q16' src/motor_types.h || fail "Q16.16 current Kp missing"
grep -q 'current_ki_dt_q16' src/motor_types.h || fail "Q16.16 current Ki*dt missing"
grep -q 'vd_int_q31' src/foc_control.c || fail "Q31 high-resolution current integrator missing"
grep -q 'q15_mul_q16' src/foc_control.c || fail "Q15 x Q16 PI multiply missing"
grep -q 'foc_sin_lut_q15' src/foc_math.c || fail "flash sine LUT missing"
grep -q 'Linear interpolation' src/foc_math.c || fail "sine LUT interpolation missing"
grep -q 's_inv_vbus_q30' src/foc_control.c || fail "cached reciprocal path missing"
grep -q 'l_trip = (l->abs_current_trip_q15' src/foc_control.c || fail "per-motor LEFT quick-guard limit missing"
grep -q 'r_trip = (r->abs_current_trip_q15' src/foc_control.c || fail "per-motor RIGHT quick-guard limit missing"
! grep -q -- '-ffast-math' platformio.ini || fail "unsafe -ffast-math still enabled"

# Keep excluded hardware out and virtual right CAN in.
grep -q 'COMM_FORWARD_CAN' src/vesc_comm.c || fail "virtual CAN forward missing"
grep -q 'VESC_VIRTUAL_CAN_RIGHT_ID' src/vesc_comm.c || fail "virtual RIGHT ID missing"
! grep -RInE '\bHAL_CAN_|\bCAN1\b|CAN_HandleTypeDef' src >/dev/null || fail "physical CAN hardware unexpectedly present"

echo "audit_v9_features: PASS"
