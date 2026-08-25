#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys, re
ROOT=Path(__file__).resolve().parents[1]
def text(p): return (ROOT/p).read_text(errors='ignore')
def ok(cond,msg):
    if not cond:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

cfg=text('src/confgenerator.c')
cmd=text('src/comm/commands.c')
mc=text('src/motor/mc_interface.c')
fm=text('src/motor/foc_math.c')
fh=text('src/motor/foc_math.h')
tasks=text('src/motor/mc_interface_tasks.c')
status=text('src/hwconf/hw_status.c')
hw=text('src/hwconf/hw.c')

# MCCONF policy: sensorless is a real VESC mode, HFI remains forbidden.
ok('w[152] != FOC_SENSOR_MODE_SENSORLESS' in cfg,
   'MCCONF validation explicitly accepts sensorless alongside physical sensors')
ok('w[152] >= FOC_SENSOR_MODE_HFI && w[152] <= FOC_SENSOR_MODE_HFI_V5' in cfg,
   'all HFI modes remain rejected')
ok('w[152] != VESC_FOC_SENSOR_ENCODER' in cfg and 'id == MOTOR_LEFT' in cfg,
   'LEFT keeps encoder support')
ok('if (w[152] == VESC_FOC_SENSOR_ENCODER) return false;' in cfg,
   'RIGHT still rejects physical encoder mode')

# Auto detect is independent of prior selection and has VESC-like fallback.
body=re.search(r'static int16_t detect_apply_all_one_runtime\(.*?\n\}',cmd,re.S)
ok(body is not None,'Detect-All per-motor routine found')
b=body.group(0)
ok(b.find('mcpwm_foc_encoder_detect_motor') < b.find('mcpwm_foc_hall_detect_motor'),
   'LEFT discovery order probes encoder before Hall')
ok('apply_sensorless_result(m);' in b,
   'Detect-All falls back to sensorless when no physical sensor is detected')
ok('detect_failure_is_sensor_absent' in b,
   'sensor absence is distinguished from real motor/power-stage fault')
ok('m->sensor_mode == SENSOR_MODE_ENCODER' not in b,
   'Detect-All no longer depends on the sensor selected before detection')

# Sensorless startup/handover safety.
ok('foc_sensorless_startup_1khz' in fh and 'foc_sensorless_startup_abort' in fh,
   'sensorless startup API is explicit')
start=fm.find('bool foc_sensorless_startup_1khz(')
end=fm.find('#endif /* FOC_MATH_UNIT_TEST */',start)
ok(start>=0 and end>start,'sensorless startup implementation found')
sb=fm[start:end]
for token,msg in [
    ('phase_60','stuck-rotor startup includes VESC-style 60-degree phase lead'),
    ('phase_45','observer state is seeded ahead of forced phase'),
    ('pwm_enable_blank_cycles','startup timer waits for current-sense blanking'),
    ('foc_sl_erpm_q16','observer handover uses configured sensorless threshold'),
    ('abs_err <= 10923','handover requires phase coherence'),
    ('foc_sl_openloop_boost_q','startup applies configured q-axis boost'),
    ('foc_sl_openloop_max_q','startup current is bounded by configured max-q')]:
    ok(token in sb,msg)

# 1-kHz outer control applies safe semantics.
ok('const bool sensorless_foc = m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;' in mc,
   'outer-loop service has explicit sensorless policy')
ok('observer_min_ready' in mc and 'observer_control_ready' in mc,
   'sensorless low-speed and full-control observer thresholds are separated')
ok('MOTOR_CTRL_POSITION' in mc and 'motor_set_foc_targets(m, 0.0f, 0.0f);' in mc,
   'position/brake/hold do not invent zero-speed sensorless rotor angle')
abort_start=fm.find('void foc_sensorless_startup_abort(')
abort_end=fm.find('bool foc_sensorless_startup_1khz(',abort_start)
ab=fm[abort_start:abort_end]
ok('motor_set_foc_targets(m, 0.0f, 0.0f);' in ab,
   'sensorless abort removes torque before releasing forced phase')
ok('openloop_erpm_now != 0.0f' in sb and
   '((m->openloop_erpm_now > 0.0f) != (dir > 0))' in sb,
   'sensorless command reversal re-arms startup alignment')
ok(re.search(r'vesc-f103-hoverboard-v(?:2[7-9]|[3-9][0-9])-', cmd) is not None,
   'firmware identity distinguishes Part-1 or later while VESC wire ABI stays 6.00')

# Fault status and gate contract remain safe.
status_fault=re.search(r'static motor_fault_t highest_priority_fault\(void\).*?\n\}',status,re.S)
ok(status_fault is not None and 'MOTOR_FAULT_COMMAND_TIMEOUT' not in status_fault.group(0),
   'non-VESC command timeout no longer produces fake fault-code 00 pulses')
ok('oc.OCPolarity = TIM_OCPOLARITY_HIGH;' in hw,
   'high-side PWM remains active HIGH')
ok('oc.OCNPolarity = TIM_OCNPOLARITY_LOW;' in hw,
   'low-side complementary PWM remains active LOW')

# Excluded subsystems stay excluded.
combined='\n'.join([cfg,cmd,mc,fm,tasks,status,hw]).lower()
for token in ['#include "comm_can.h"','#include "imu/','#include "bms','#include "bm_if',
              '#include "nrf','#include "ledpwm','#include "comm_usb','#include "lispif','#include "lzo']:
    ok(token not in combined,f'excluded subsystem not introduced: {token}')

# Strict portable syntax for changed core files using established host stubs.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for f in ['src/confgenerator.c','src/motor/mc_interface.c','src/motor/foc_math.c']:
    cp=subprocess.run(base+incs+['-fsyntax-only',f],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True,f'{f} passes strict host syntax/warning check')

print('ALL PART-1 SENSORLESS/AUTODETECT REGRESSIONS: PASS')
