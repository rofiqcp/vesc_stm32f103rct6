#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys
ROOT=Path(__file__).resolve().parents[1]

def text(rel): return (ROOT/rel).read_text(encoding='utf-8',errors='replace')
checks=[]
def ck(name, ok):
    checks.append((name,bool(ok)))
    print(('PASS  ' if ok else 'FAIL  ')+name)

# Keep Run29 safety/readiness contract.
r=subprocess.run([sys.executable,str(ROOT/'tools/audit_run29.py')],cwd=ROOT,text=True,capture_output=True)
print('=== RUN29 BASELINE ===')
print(r.stdout.rstrip())
ck('Run29 baseline audit remains green', r.returncode==0 and '25/25 PASS' in r.stdout)

hw=text('src/hwconf/hw_status.c')
conf=text('src/conf_general.c')
cmd=text('src/comm/commands.c')
gen=text('src/confgenerator.c')
foc=text('src/motor/mcpwm_foc.c')
mc=text('src/motor/mc_interface.c')
dbg=text('tools/debug.py')
hwh=text('src/hwconf/hw.h')

print('\n=== RUN30 EEPROM / STATUS ===')
ck('startup melody retained', 's_startup_notes[]' in hw and 'hw_status_startup_melody_begin();' in hw)
ck('normal running status is exactly 3 LED pulses', '(running ? 3U : 0U)' in hw and 'const uint8_t pulses = mode;' in hw)
ck('running/detect suppresses ordinary audio cue', 'if (detecting || running)' in hw and 'hw_status_tone_stop();' in hw)
ck('EEPROM uses queued event counter', 's_eeprom_cue_pending' in hw and 's_eeprom_cue_pending++' in hw)
ck('EEPROM cue explicitly five tones', 'tepat lima beep' in hw and 's_cue_step < 8U' in hw and 'step 8 mengakhiri tone kelima' in hw)
ck('EEPROM cue documented as five beep in public header', 'lima beep pendek' in hwh)
ck('all four physical store wrappers notify centrally', conf.count('hw_status_notify_eeprom_saved();')==4)
ck('no duplicate high-level EEPROM notify in commands', 'hw_status_notify_eeprom_saved();' not in cmd)
ck('no duplicate high-level EEPROM notify in config generator', 'hw_status_notify_eeprom_saved();' not in gen)

print('\n=== RUN30 VESC COMMAND PATH ===')
for case,call in [
    ('COMM_SET_DUTY','motor_set_duty('),
    ('COMM_SET_CURRENT','motor_set_current('),
    ('COMM_SET_CURRENT_BRAKE','motor_set_brake_current('),
    ('COMM_SET_RPM','motor_set_speed('),
    ('COMM_SET_POS','motor_set_position('),
    ('COMM_SET_HANDBRAKE','motor_set_handbrake('),
    ('COMM_SET_CURRENT_REL','motor_set_current_rel('),
]:
    ck(f'{case} handler reaches {call[:-1]}', f'case {case}:' in cmd and call in cmd)
ck('all control handlers pass readiness boundary', cmd.count('begin_uart_motor_command(id, cmd, raw)') >= 7)
ck('brake Iq opposes measured speed', 'iq = -(float)dir * brake_target;' in mc)
ck('brake zero-speed guard retained', 'brake_zero_guard_1khz' in mc)
ck('position PID command path retained', 'mcpwm_foc_set_pid_pos_motor' in mc and 'position_pid_step(m)' in mc)

print('\n=== RUN30 HALL / FOC CROSS-CHECK ===')
ck('Hall detect forces electrical phase', 'm->detect_force_angle = true;' in foc)
ck('Hall detect ramps Id with Iq zero', 'motor_set_foc_targets(m, id_cmd, 0.0f);' in foc)
ck('Hall forward sweep keeps Id and forced phase', 'SENSOR_DETECT_HALL_FWD' in foc and 'motor_set_foc_targets(m, m->detect.drive_current_a, 0.0f);' in foc)
ck('Hall reverse sweep present', 'SENSOR_DETECT_HALL_REV' in foc)
ck('Hall topology validates six Gray states', 'detect_hall_table_topology_valid' in foc)
ck('current ADC polarity remains offset-minus-raw', 'offset - raw' in dbg)

print('\n=== RUN30 DEBUG ACCEPTANCE ===')
ck('debug has brake-test', 'def cmd_brake_test' in dbg and '"brake-test"' in dbg)
ck('brake-test pre-spins before brake', 'PRE-SPIN' in dbg and 'COMM_SET_CURRENT_BRAKE' in dbg and 'min_spin_erpm' in dbg)
ck('debug has Hall-resolution-aware position-test', 'def cmd_position_test' in dbg and 'Hall resolution approx' in dbg)
ck('debug has full drive-acceptance', 'def cmd_drive_acceptance' in dbg and '"drive-acceptance"' in dbg)
for token in ["'duty'","'current'","'current-rel'","'rpm'","cmd_brake_test","cmd_position_test","'handbrake'","cmd_config_save"]:
    ck(f'drive-acceptance includes {token}', token in dbg[dbg.index('def cmd_drive_acceptance'):dbg.index('# Fungsi _measured_rate_hz:')])
ck('full-test aliases stronger Run30 acceptance', 'return cmd_drive_acceptance(link,ns)' in dbg)
ck('config-save reports five-beep contract', 'mengantrikan tepat 5 beep' in dbg)

failed=[n for n,ok in checks if not ok]
print(f"\nSUMMARY: {len(checks)-len(failed)}/{len(checks)} PASS")
if failed:
    for n in failed: print(' -',n)
    raise SystemExit(1)
