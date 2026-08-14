#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CC=${CC:-gcc}
CFLAGS='-std=c11 -O2 -Wall -Wextra -Isrc -Itests/host/stubs'
mkdir -p /tmp/vesc_v11_host

$CC $CFLAGS tests/host/test_packet_v8.c src/vesc_packet.c -o /tmp/vesc_v11_host/test_packet_v8
/tmp/vesc_v11_host/test_packet_v8
$CC $CFLAGS tests/host/test_foc_math.c src/foc_math.c -lm -o /tmp/vesc_v11_host/test_foc_math
/tmp/vesc_v11_host/test_foc_math
$CC $CFLAGS tests/host/test_vesc_buffer.c src/vesc_buffer.c -lm -o /tmp/vesc_v11_host/test_vesc_buffer
/tmp/vesc_v11_host/test_vesc_buffer
$CC $CFLAGS tests/host/test_vesc_config_layout.c src/vesc_config.c src/vesc_buffer.c -lm -o /tmp/vesc_v11_host/test_vesc_config_layout
/tmp/vesc_v11_host/test_vesc_config_layout
$CC $CFLAGS tests/host/test_vesc_config_early_get.c src/vesc_config.c src/vesc_buffer.c -lm -o /tmp/vesc_v11_host/test_vesc_config_early_get
/tmp/vesc_v11_host/test_vesc_config_early_get

bash tests/host/audit_v11_vesc_f103.sh
python3 -m py_compile debug_vesc_f103.py
python3 debug_vesc_f103.py --self-test

echo 'V11 host/audit tests: ALL PASS'
