#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/vesc_f103_v9_tests"
rm -rf "$OUT" && mkdir -p "$OUT"
CFLAGS=(-std=c11 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)

gcc "${CFLAGS[@]}" -I"$ROOT/src" "$ROOT/tests/host/test_foc_math.c" "$ROOT/src/foc_math.c" -lm -o "$OUT/test_foc_math"
"$OUT/test_foc_math"
gcc "${CFLAGS[@]}" -I"$ROOT/src" "$ROOT/tests/host/test_vesc_buffer.c" "$ROOT/src/vesc_buffer.c" -lm -o "$OUT/test_vesc_buffer"
"$OUT/test_vesc_buffer"
gcc "${CFLAGS[@]}" -I"$ROOT/tests/host/stubs" -I"$ROOT/src" "$ROOT/tests/host/test_vesc_config_layout.c" "$ROOT/src/vesc_config.c" "$ROOT/src/vesc_buffer.c" -lm -o "$OUT/test_vesc_config_layout"
"$OUT/test_vesc_config_layout"
bash "$ROOT/tests/host/audit_v9_features.sh"
python3 -m py_compile "$ROOT/debug_vesc_f103.py"
python3 "$ROOT/debug_vesc_f103.py" --self-test

echo "V9 host/audit tests: ALL PASS"
