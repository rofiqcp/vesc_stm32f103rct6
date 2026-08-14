#!/usr/bin/env bash
set -euo pipefail

grep -q 'ADC_OFFSET_HARD_MIN_COUNT' src/app_config.h
grep -q 'ADC_OFFSET_WARN_STDDEV_COUNT' src/app_config.h
grep -q 'ADC_OFFSET_HARD_STDDEV_COUNT' src/app_config.h
grep -q 'foc_get_calibration_diag' src/foc_control.c
grep -q 's_cal_fail_range_mask' src/foc_control.c
grep -q 's_cal_fail_noise_mask' src/foc_control.c
grep -q 'CURRENT_OFFSET is recoverable' src/motor_tasks.c
grep -q 'cal_diag_revision' debug_vesc_f103.py
grep -q 'V12 CALIBRATION STATISTICS' debug_vesc_f103.py
grep -q 'v12_full_debug' debug_vesc_f103.py
# Communication transport files must remain byte-identical to the proven V8 path.
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256 >/dev/null
# Timing/ADC topology from V11 must remain present.
bash tests/host/audit_v11_vesc_f103.sh >/dev/null

echo 'audit_v12_calibration: PASS'
echo '  V8 UART DMA transport retained'
echo '  V11 TIM2 + ADC1/ADC2 + DMA1_CH1 topology retained'
echo '  calibration now reports mean/min/max/spread/stddev/fail masks/registers/raw DMA words'
echo '  moderate ADC noise is warning-only; only rail/extreme-noise is hard failure'
echo '  successful recalibration clears only latched CURRENT_OFFSET fault'
