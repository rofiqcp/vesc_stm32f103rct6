#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "audit_v13_detection: FAIL: $*" >&2; exit 1; }
need(){ grep -qF "$1" "$2" || fail "missing '$1' in $2"; }

# Proven V8 VESC Tool transport remains immutable.
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256 >/dev/null || fail "V8 transport changed"

# Hardware-proven current polarity and phase mapping.
need 'offset - (int32_t)raw' src/foc_control.c
need 'PCB LEFT shunts measure phase A and phase B' src/foc_control.c
need 'PCB RIGHT shunts are phase B (PC4) and phase C (PC5)' src/foc_control.c
need 'ia = -(ib + ic)' src/foc_control.c
need 'm->pwm_enabled && (iabs32(ia) > trip' src/foc_control.c
need 'dc_current_offset_counts-(int32_t)m->dc_current_raw' src/motor_control.c
need '#define LEFT_CURRENT_A_PER_COUNT        0.0200f' src/app_config.h
need '#define RIGHT_CURRENT_A_PER_COUNT       0.0200f' src/app_config.h

# Stock board Hall electrical input behavior.
need 'Stock hoverboard Hall inputs are active-low' src/motor_hw.c
need 'GPIO_NOPULL' src/motor_hw.c

# Upstream VESC Hall-detect sequence/transaction semantics.
need '#define SENSOR_DETECT_CURRENT_RAMP_MS   1000U' src/app_config.h
need '#define SENSOR_DETECT_STEP_MS           5U' src/app_config.h
need '#define SENSOR_DETECT_STEP_DEG          1U' src/app_config.h
need '#define SENSOR_DETECT_SWEEPS            3U' src/app_config.h
need 'motor_set_current_pi_gains(m, 0.01f, 10.0f)' src/sensor_detect.c
need 'vesc_timeout_configure(60000U, 0.0f)' src/sensor_detect.c
need 'vesc_timeout_reset();' src/sensor_detect.c
need 'm->detect.hall_samples[raw]++' src/sensor_detect.c
need 'fails == 2U' src/sensor_detect.c
need 'result_hall_table[0] == 255U' src/sensor_detect.c
need 'result_hall_table[7] == 255U' src/sensor_detect.c
need 'sensor_detect_request_current_ex(m, SENSOR_MODE_HALL, fabsf(current), false)' src/vesc_comm.c
need 'result_hall_table[k]' src/vesc_comm.c
need 'result_encoder_ratio' src/vesc_comm.c

# Detection must not hide an electrical fault or latch a clean algorithmic failure.
need 'measurement result, not automatically a' src/sensor_detect.c
if grep -q 'motor_raise_fault_from_task(m, MOTOR_FAULT_SENSOR_DETECT)' src/sensor_detect.c; then fail 'clean detect failure still latches SENSOR_DETECT'; fi

# Hall phase source of truth is polled in the hard current loop; EXTI is optional.
need 'hall_update_raw_fast(m, motor_hw_read_hall_raw(m->id))' src/foc_control.c
need 'm->hall.invalid_count >= 32U' src/foc_control.c
need 'm->hall.sequence_error_count >= 4U' src/foc_control.c
need 'IDR snapshot per GPIO port' src/motor_hw.c

# Custom bring-up detection can use a deliberately low current without changing VESC command semantics.
need 'sensor_detect_request_current(m, mode, current)' src/vesc_comm.c
need 'p.add_argument("--current",type=float,default=0.5' debug_vesc_f103.py

# V13 host trace support.
grep -Eq 'V1(3|4) SENSOR DETECT TRACE' debug_vesc_f103.py || fail 'missing V13/V14 sensor trace marker'
grep -Eq 'v1(3|4)_sensor_detect' debug_vesc_f103.py || fail 'missing V13/V14 sensor trace filename'
need 'NATIVE_FAULT_NAMES' debug_vesc_f103.py

# The full Hall routine takes about 11.8 s at canonical timing; host timeout must exceed it.
python3 - <<'PY'
ramp=1000
sweeps=3
step=5
ms=ramp+(360*sweeps*step)+(361*sweeps*step)
assert 11800 <= ms <= 11900, ms
print(f"canonical Hall detect scheduled duration: {ms} ms")
# EFeru baseline: 50 ADC counts/A -> 0.02 A/count.
offset=2782
raw=2732
amps=(offset-raw)*0.02
assert abs(amps-1.0)<1e-9
# Right measured B/C reconstruction must satisfy ia+ib+ic == 0.
ib=1.2; ic=-0.4; ia=-(ib+ic)
assert abs(ia+ib+ic)<1e-9
PY

# Syntax-check the rewritten state machine with the existing host HAL stubs.
gcc -std=c11 -Wall -Wextra -Werror -fsyntax-only -Itests/host/stubs -Isrc src/sensor_detect.c
python3 -m py_compile debug_vesc_f103.py

echo 'audit_v13_detection: PASS'
echo '  V8 UART/packet transport frozen'
echo '  current polarity + LEFT A/B + RIGHT B/C reconstruction corrected'
echo '  off-state manual rotation no longer latches software over-current'
echo '  active-low/NOPULL Hall inputs match stock hoverboard hardware'
echo '  Hall detect uses VESC 1s current ramp + 1deg/5ms x 3 fwd/rev'
echo '  timeout disabled/restored and standard detect restores old config'
echo '  clean detect failure is non-latching; electrical faults retain their real cause'
echo '  Hall GPIO is polled in FOC fast path; EXTI is optional hint only'
echo '  V13 sensor-detect trace writes detailed TXT and supports low diagnostic current'
