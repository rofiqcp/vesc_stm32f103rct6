#!/usr/bin/env bash
set -euo pipefail
bash tests/host/run_v11.sh
bash tests/host/audit_v12_calibration.sh
python3 -m py_compile debug_vesc_f103.py
python3 debug_vesc_f103.py --self-test
echo 'V12 host/audit tests: ALL PASS'
