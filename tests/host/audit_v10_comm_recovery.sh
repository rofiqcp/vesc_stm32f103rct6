#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
V8="/mnt/data/v8_rofiq_work"
# exact files that must remain frozen
for f in src/vesc_uart.c src/vesc_uart.h src/vesc_packet.c src/vesc_packet.h; do
  cmp -s "$ROOT/$f" "$V8/$f" || { echo "FAIL: $f differs from V8"; exit 1; }
done
# comm dispatcher can differ only by motor_set_duty API name
python3 - "$ROOT/src/vesc_comm.c" "$V8/src/vesc_comm.c" <<'PY'
import sys
new=open(sys.argv[1]).read().replace('motor_set_duty(m,','motor_set_duty_approx(m,')
old=open(sys.argv[2]).read()
if new != old:
    print('FAIL: V10 vesc_comm differs from V8 beyond duty API rename')
    raise SystemExit(1)
print('V8 dispatcher/reply freeze: PASS')
PY
# no V9 config handlers exposed in comm
if grep -q 'vesc_config_set_mc_wire\|vesc_config_set_app_wire\|vesc_config_mc_wire\|vesc_config_app_wire' "$ROOT/src/vesc_comm.c"; then
  echo 'FAIL: experimental V9 config path still exposed'; exit 1
fi
# parser is task-side, not in DMA ISR file
if grep -n 'vesc_packet_process_byte' "$ROOT/src/vesc_uart.c"; then
  echo 'FAIL: parser found in UART transport'; exit 1
fi
grep -q 'vesc_packet_process_byte(&s_parser' "$ROOT/src/vesc_comm.c"
grep -q 'packet_process_thread' "$ROOT/src/vesc_comm.c"
grep -q 'blocking_thread' "$ROOT/src/vesc_comm.c"
grep -q 'hoverboard-vesc6-rtos-v8' "$ROOT/src/vesc_comm.c"
echo 'audit_v10_comm_recovery: PASS'
