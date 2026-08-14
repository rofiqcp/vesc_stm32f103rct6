#!/usr/bin/env bash
set -euo pipefail
bash tests/host/run_v11.sh
# V13 physical mapping/detect semantics (without chaining the old V12 text-marker audit).
bash tests/host/audit_v13_detection.sh
bash tests/host/audit_v14_driven_cal.sh
python3 debug_vesc_f103.py --self-test
echo 'V14 host/audit tests: ALL PASS'
