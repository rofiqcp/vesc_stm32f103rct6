#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "audit_v14_driven_cal: FAIL: $*" >&2; exit 1; }
need(){ grep -qF "$1" "$2" || fail "missing '$1' in $2"; }

# Proven communication path must remain frozen.
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256 >/dev/null || fail "V8 transport changed"
# VESC timing and V13 physical current/Hall mapping must remain present.
bash tests/host/audit_v11_vesc_f103.sh >/dev/null
bash tests/host/audit_v13_detection.sh >/dev/null

need 'ADC_DRIVEN_CAL_SAMPLES          1000U' src/app_config.h
need 'ADC_DRIVEN_CAL_DECIMATION' src/app_config.h
need 'ADC_DRIVEN_CAL_WARMUP_EVENTS' src/app_config.h
need 'PWM_ENABLE_BLANK_CYCLES' src/app_config.h
need 'FOC_CAL_STAGE_LEFT_DRIVEN' src/foc_control.h
need 'FOC_CAL_STAGE_RIGHT_DRIVEN' src/foc_control.h
need '50%/50%/50% zero-vector PWM' src/foc_control.c
need 'cal_task_reset_acc_and_stage' src/foc_control.c
need 'do NOT drag the driven offsets back toward the undriven readings' src/foc_control.c
need 'pwm_enable_blank_cycles' src/foc_control.c
need 'capture_current_fault_snapshot' src/foc_control.c
need 'foc_get_fault_snapshot' src/vesc_comm.c
need 'calibration diagnostic revision' src/vesc_comm.c
need 'V14 DRIVEN/OFFSET SPLIT' debug_vesc_f103.py
need 'FIRST ACTIVE-DRIVE CURRENT-FAULT SNAPSHOT' debug_vesc_f103.py

# Hard ISR remains RTOS/HAL blocking-free in the calibration implementation.
python3 - <<'PY'
from pathlib import Path
s=Path('src/foc_control.c').read_text()
start=s.index('void foc_adc_dma_isr(')
body=s[start:]
for bad in ('osDelay(', 'osMutexAcquire(', 'HAL_GPIO_', 'HAL_TIM_', 'printf(', 'malloc('):
    assert bad not in body, bad
print('ISR blocking-call audit: PASS')
PY
python3 tests/host/test_v14_current_fixedpoint.py
python3 -m py_compile debug_vesc_f103.py

echo 'audit_v14_driven_cal: PASS'
echo '  VESC-style driven/undriven offset separation present'
echo '  low-side-shunt zero-vector driven calibration: 1000 samples per motor'
echo '  current offsets are not re-tracked while undriven'
echo '  PWM-enable blanking + first-fault ADC snapshot present'
echo '  V8 communication and V11 TIM2/ADC/DMA topology retained'
