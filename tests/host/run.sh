#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/vesc_f103_host_tests"
rm -rf "$OUT" && mkdir -p "$OUT"
CFLAGS=(-std=c11 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror -I"$ROOT/src")
gcc "${CFLAGS[@]}" "$ROOT/tests/host/test_packet.c" "$ROOT/src/vesc_packet.c" -o "$OUT/test_packet"
gcc "${CFLAGS[@]}" "$ROOT/tests/host/test_foc_math.c" "$ROOT/src/foc_math.c" -lm -o "$OUT/test_foc_math"
"$OUT/test_packet"
"$OUT/test_foc_math"
python3 -m py_compile "$ROOT/debug_vesc_f103.py"
python3 "$ROOT/debug_vesc_f103.py" --self-test
echo "host tests: ALL PASS"
