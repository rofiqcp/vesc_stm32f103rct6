#!/usr/bin/env bash
set -euo pipefail
bash tests/host/run_v12.sh
bash tests/host/audit_v13_detection.sh
python3 debug_vesc_f103.py --self-test
echo 'V13 host/audit tests: ALL PASS'
