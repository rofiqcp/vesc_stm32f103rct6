#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/vesc_f103_v8_tests"
rm -rf "$OUT" && mkdir -p "$OUT"
CFLAGS=(-std=c11 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror -I"$ROOT/src")
gcc "${CFLAGS[@]}" "$ROOT/tests/host/test_packet_v8.c" "$ROOT/src/vesc_packet.c" -o "$OUT/test_packet_v8"
"$OUT/test_packet_v8"
bash "$ROOT/tests/host/audit_v8_rofiq_transport.sh"
python3 -m py_compile "$ROOT/debug_vesc_f103.py"
python3 "$ROOT/debug_vesc_f103.py" --self-test
echo "V8 host/audit tests: ALL PASS"
