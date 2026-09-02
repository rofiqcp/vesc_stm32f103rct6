#!/usr/bin/env python3
"""Static acceptance audit for Run29. Does not move motors or access serial."""
from pathlib import Path
import re, sys
ROOT=Path(__file__).resolve().parents[1]
checks=[]
def check(name, cond, detail=''):
    checks.append((name,bool(cond),detail))

def text(rel): return (ROOT/rel).read_text(encoding='utf-8',errors='replace')

hw_h=text('src/hwconf/hw.h')
hw_c=text('src/hwconf/hw.c')
main=text('src/main.c')
cmd=text('src/comm/commands.c')
status=text('src/hwconf/hw_status.c')
foc=text('src/motor/foc_math.c')
mcpwm=text('src/motor/mcpwm_foc.c')
dbg=text('tools/debug.py')

check('drive critical mask defined', 'HW_SAMPLING_CONTRACT_DRIVE_CRITICAL_MASK' in hw_h)
check('boot gate uses drive critical mask', 'drive_contract_flags' in main and 'HW_SAMPLING_CONTRACT_DRIVE_CRITICAL_MASK' in main)
check('runtime readiness recovery exists', 'vesc_comm_try_recover_motor_ready' in cmd)
check('recovery requires valid calibration', '!foc_calibration_valid()' in cmd)
check('recovery requires hard sampling validity', '!motor_hw_sampling_drive_valid()' in cmd)
check('recovery only auto-clears ADC_DMA', cmd.count('== MOTOR_FAULT_ADC_DMA') >= 2)
check('command boundary retries readiness', '(void)vesc_comm_try_recover_motor_ready();' in cmd)
check('packet thread probes recovery', 'last_ready_probe' in cmd and 'pdMS_TO_TICKS(100U)' in cmd)
check('COMM_DIAG revision 18', re.search(r'p\[i\+\+\]\s*=\s*18U;',cmd) is not None)
check('COMM_DIAG exports motor_ready', 's_motor_ready ? 1U : 0U' in cmd and 's_motor_ready_recovery_count' in cmd)
check('startup melody begins before scheduler', 'hw_status_startup_melody_begin();' in status)
check('fault audio deferred while melody active', 'fault != MOTOR_FAULT_NONE && g_vesc_startup_melody_active' in status)
check('running does not cancel startup melody', 'Power-on melody berjalan pada TIM3 independen' in status)
check('Hall detect forces electrical angle', 'detect_force_angle = true' in mcpwm)
check('Hall detect drives Id not Hall phase', 'motor_set_foc_targets(m, id_cmd, 0.0f)' in mcpwm or 'motor_set_foc_targets(m, id_target, 0.0f)' in mcpwm)
check('detect phase bypass exists', 'detect_force_angle' in mcpwm and 'detect_phase_u16' in mcpwm and 'forced_detect_phase' in mcpwm)
check('sensorless forced startup exists', 'foc_sensorless_startup_1khz' in foc and 'phase_observer_override = true' in foc)
check('sensorless Q boost participates', 'foc_sl_openloop_boost_q' in foc and 'openloop_current_a' in foc)
check('debug decodes MOTOR_NOT_READY', 'MOTOR_NOT_READY' in dbg and 'CONTROL_RESULT_NAMES' in dbg)
check('debug has startup-check', 'startup-check' in dbg and 'cmd_startup_check' in dbg)
check('sensor select distinguishes detect result', 'detect_success(last)' in dbg and 'selection_active' in dbg)
check('calibrate checks readiness', 'READINESS AFTER CALIBRATION' in dbg)

# Hygiene guards on project-owned C/H.
bad_tabs=[]; trailing=[]; if0=[]
for p in (ROOT/'src').rglob('*'):
    if p.suffix not in ('.c','.h'): continue
    s=p.read_text(encoding='utf-8',errors='replace')
    if '\t' in s: bad_tabs.append(str(p.relative_to(ROOT)))
    if any(line.rstrip('\n\r').endswith((' ','\t')) for line in s.splitlines(True)): trailing.append(str(p.relative_to(ROOT)))
    if re.search(r'^\s*#\s*if\s+0\b',s,re.M): if0.append(str(p.relative_to(ROOT)))
check('no #if 0 dead blocks', not if0, ','.join(if0[:4]))
check('no tabs in project C/H', not bad_tabs, ','.join(bad_tabs[:4]))
check('no trailing whitespace in project C/H', not trailing, ','.join(trailing[:4]))

for name,ok,detail in checks:
    print(f"{'PASS' if ok else 'FAIL':4s}  {name}" + (f" :: {detail}" if detail else ''))
failed=[x for x in checks if not x[1]]
print(f"\nSUMMARY: {len(checks)-len(failed)}/{len(checks)} PASS")
sys.exit(1 if failed else 0)
