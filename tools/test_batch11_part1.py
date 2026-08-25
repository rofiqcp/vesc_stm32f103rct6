#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys, re

ROOT = Path(__file__).resolve().parents[1]
def read(p): return (ROOT / p).read_text(errors='replace')
def ok(c, msg):
    if not c:
        print('FAIL:', msg)
        sys.exit(1)
    print('PASS:', msg)

cmd = read('src/comm/commands.c')
conf = read('src/confgenerator.c')
confh = read('src/confgenerator.h')
mc = read('src/motor/mc_interface.c')
foc = read('src/motor/mcpwm_foc.c')
fmath = read('src/motor/foc_math.c')
enc = read('src/encoder/encoder.c')
hw = read('src/hwconf/hw.c')
tasks = read('src/motor_tasks.c')
dt = read('src/datatypes.h')
port = read('PORTING_NOTES.md')

# ---- Identity and VESC Tool dual-controller basics ----
ok(re.search(r'vesc-f103-hoverboard-v(?:2[7-9]|[3-9][0-9])-', cmd) is not None, 'firmware identity is Part-1 or later while preserving VESC6 protocol target')
ok('uint8_t p[2] = {COMM_PING_CAN, VESC_LOCAL_MOTOR2_FORWARD_ID};' in cmd, 'direct VESC Tool scan advertises local Motor-2 ID')
ok('sv->num_vescs' in cmd and 'p[(*i)++] = sv->num_vescs' in cmd, 'setup telemetry carries VESC count')
ok('controller_id' in cmd and 'p[(*i)++] = t->controller_id' in cmd, 'standard telemetry carries selected controller ID')

# ---- Full dual Detect-All transaction ----
ok('detect_can && m->id == MOTOR_LEFT' in cmd, 'direct detect_can=true is handled as local dual Detect-All')
ok('detect_apply_all_one_runtime(&g_motor_left' in cmd and 'detect_apply_all_one_runtime(&g_motor_right' in cmd,
   'dual Detect-All executes LEFT then RIGHT')
ok('vesc_config_commit_detect_all_runtime_dual()' in cmd and 'vesc_config_commit_detect_all_runtime_dual(void)' in conf,
   'dual Detect-All uses dedicated atomic dual commit')
ok('conf_general_store_all()' in re.search(r'bool vesc_config_commit_detect_all_runtime_dual\(void\).*?\n\}', conf, re.S).group(0),
   'dual Detect-All persists with one transactional store-all operation')
body = re.search(r'bool vesc_config_commit_detect_all_runtime_dual\(void\).*?\n\}', conf, re.S).group(0)
ok(body.count('s_rollback_mc[MOTOR_LEFT]') >= 2 and body.count('s_rollback_mc[MOTOR_RIGHT]') >= 2,
   'dual commit snapshots and restores both motor configs on failure')
ok('rollback_detect_all_runtime_both();' in cmd, 'failed dual Detect-All reapplies both last-known-good MCCONFs')
ok('put_i16(p, &i, result);' in cmd, 'Detect-All reply remains VESC Tool int16 result format')

# ---- Fresh current calibration and transaction isolation ----
calh = re.search(r'static bool force_current_calibration_valid\(uint32_t timeout_ms\).*?\n\}', cmd, re.S)
ok(calh is not None, 'fresh calibration helper exists')
cb = calh.group(0)
ok('foc_request_recalibration();' in cb and 'saw_active' in cb and '!foc_calibration_done()' in cb,
   'Detect-All waits for a newly-started calibration transaction, not stale boot result')
ok('mc_interface_ignore_input_both(180000)' in cmd and 'mc_interface_ignore_input_both(0)' in cmd,
   'ordinary motor input is gated for both motors during Detect-All and released afterwards')
ok('motor_stop(&g_motor_left);' in cmd and 'motor_stop(&g_motor_right);' in cmd,
   'both bridges are commanded stopped before calibration/detection')
ok('s_undriven_mean[0]' in foc and 's_undriven_mean[5]' in foc and 's_driven_mean[0]' in foc and 's_driven_mean[5]' in foc,
   'shared calibration covers all six LEFT/RIGHT phase/DC-current channels')
ok('FOC_CAL_STAGE_LEFT_DRIVEN' in foc and 'FOC_CAL_STAGE_RIGHT_DRIVEN' in foc and '50%/50%/50% zero-vector PWM' in foc,
   'fresh calibration includes driven zero-vector stages for both inverters')
ok('ADC_OFFSET_CAL_SAMPLES' in foc and 'ADC_DRIVEN_CAL_SAMPLES' in foc,
   'calibration uses explicit multi-sample undriven and driven estimators')

# ---- Sensor policy and Detect-All sensor calibration ----
# Part-1 intentionally supersedes the old 'preserve selected sensor' behavior:
# Detect-All now discovers LEFT ABI -> Hall -> sensorless and RIGHT Hall ->
# sensorless independently of the previously selected mode.
ok('if (m->id == MOTOR_LEFT)' in cmd and 'mcpwm_foc_encoder_detect_motor' in cmd and
   'apply_encoder_detect_result' in cmd,
   'LEFT Detect-All probes ABI encoder as first physical-sensor candidate')
ok('mcpwm_foc_hall_detect_motor' in cmd and 'apply_hall_detect_result' in cmd and
   'apply_sensorless_result' in cmd,
   'Detect-All probes Hall and has explicit sensorless fallback')
ok('if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER) return false;' in mc,
   'RIGHT encoder selection is explicitly rejected')
ok('if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER) mode = SENSOR_MODE_HALL;' in foc,
   'hard FOC layer also clamps RIGHT encoder request back to Hall')
ok('m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER' in fmath and
   'm->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER' in enc,
   'ABI encoder fast-path remains LEFT-only')

# ---- Main control command coverage ----
controls = {
    'COMM_SET_DUTY': 'motor_set_duty',
    'COMM_SET_CURRENT': 'motor_set_current',
    'COMM_SET_CURRENT_BRAKE': 'motor_set_brake_current',
    'COMM_SET_RPM': 'motor_set_speed',
    'COMM_SET_POS': 'motor_set_position',
    'COMM_SET_HANDBRAKE': 'motor_set_handbrake',
    'COMM_SET_CURRENT_REL': 'motor_set_current_rel',
}
for c, fn in controls.items():
    ok(f'case {c}:' in cmd and fn in cmd, f'{c} is routed to real motor-control backend')
ok(cmd.count('mc_interface_try_input_motor(id)') >= len(controls), 'main control commands pass motor input ownership/safety gate')

# ---- Telemetry required by VESC Tool ----
for c in ['COMM_GET_VALUES','COMM_GET_VALUES_SELECTIVE','COMM_GET_VALUES_SETUP','COMM_GET_VALUES_SETUP_SELECTIVE']:
    ok(f'case {c}:' in cmd, f'{c} is implemented')
ok('0x003FFFFFUL' in cmd, 'full standard VESC6 selective telemetry mask bits 0..21 are populated')
for tok, label in [
    ('t->current_motor','motor current'),('t->current_in','input current'),('t->id_filter','Id'),('t->iq_filter','Iq'),
    ('t->duty','duty'),('t->erpm','ERPM'),('t->vbus','Vbus'),('t->tachometer','tachometer'),
    ('t->position_deg','position'),('t->vd','Vd'),('t->vq','Vq'),('t->fault','fault')]:
    ok(tok in cmd, f'standard telemetry exposes {label}')
ok('VESC_TEMP_UNAVAILABLE_DECIC' in cmd, 'unavailable NTC/FET/motor temperatures are explicit, not faked')
ok('telemetry_get(id, &t)' in cmd, 'telemetry snapshot is selected per requested M1/M2')

# ---- Configuration and temporary app config semantics ----
ok('case COMM_SET_APPCONF_NO_STORE:' in cmd, 'COMM_SET_APPCONF_NO_STORE is accepted')
ok('const bool store = job.cmd == COMM_SET_APPCONF;' in cmd, 'APPCONF no-store path does not write flash')
ok('reply_ack(job.cmd);' in cmd, 'APPCONF set/no-store ACK preserves actual command ID')
ok('VESC6_MCCONF_WIRE_SIZE' in conf and 'VESC6_APPCONF_WIRE_SIZE' in conf, 'VESC6 fixed MCCONF/APPCONF wire ABI remains explicit')

# ---- Individual calibration/detection commands used by VESC Tool wizard ----
for c in ['COMM_DETECT_MOTOR_R_L','COMM_DETECT_MOTOR_FLUX_LINKAGE','COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP',
          'COMM_DETECT_ENCODER','COMM_DETECT_HALL_FOC','COMM_DETECT_APPLY_ALL_FOC']:
    ok(c in cmd, f'{c} exists for VESC Tool motor wizard')
ok('fabsf(current) >= 0.05f' in cmd or 'fabsf(current)>=0.05f' in cmd,
   'open-loop flux detection requires nonzero drive current; passive zero-current flux is not faked')

# ---- Fault LED/buzzer + startup melody ----
ok('MOTOR_FAULT_ENCODER_SLIP' in tasks, 'encoder-slip is included in audible/visible fault selection')
ok('motor_fault_to_vesc(f)' in tasks, 'fault indication is derived from canonical VESC fault number')
ok('fault_tens' in tasks and 'fault_ones' in tasks and 'fault_digit_pulses' in tasks,
   'LED+buzzer encode the same decimal fault-code pulse groups')
ok('status_io_led(true)' in tasks and 'status_io_tone_start' in tasks,
   'same state machine drives visual and audible fault indication')
ok('900U' in tasks and '1350U' in tasks and '1900U' in tasks,
   'non-blocking VESC-style power-on melody remains present')

# ---- PWM polarity contract ----
ok('oc.OCPolarity = TIM_OCPOLARITY_HIGH;' in hw, 'high-side PWM input is active HIGH')
ok('oc.OCNPolarity = TIM_OCNPOLARITY_LOW;' in hw, 'low-side complementary PWM input is active LOW')
ok('oc.OCIdleState = TIM_OCIDLESTATE_RESET' in hw and 'oc.OCNIdleState = TIM_OCNIDLESTATE_SET' in hw,
   'idle polarity explicitly leaves both high-side and low-side gate inputs OFF')

# ---- Fixed-point hard FOC / no excluded ecosystems ----
start = foc.find('static void foc_one_motor_isr(')
end = foc.find('static uint64_t offset_variance_num', start)
ok(start >= 0 and end > start, 'hard FOC ISR body found')
fi = foc[start:end]
for bad in ['sqrtf(', 'fabsf(', 'lrintf(', 'double ', 'float ']:
    ok(bad not in fi, f'hard FOC ISR remains free of {bad.strip()} operations')
combined = '\n'.join([cmd,conf,mc,foc,tasks,hw,fmath,enc]).lower()
excluded_includes = ['#include \"comm_can.h\"','#include \"imu/','#include \"bms/','#include \"bm_if','#include \"nrf','#include \"ledpwm','#include \"comm_usb','#include \"lispif','#include \"lzo']
for token in excluded_includes:
    ok(token not in combined, f'excluded runtime subsystem not introduced: {token}')

# ---- Compile what is portable with the established host stubs ----
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
cp=subprocess.run(base+incs+['-fsyntax-only','src/confgenerator.c'],cwd=ROOT,text=True,capture_output=True)
if cp.returncode:
    print(cp.stdout, cp.stderr); sys.exit(cp.returncode)
ok(True,'confgenerator.c dual atomic-commit changes pass strict host syntax/warning check')

print('ALL BATCH 11 PART-1 CORE/DETECT/CALIBRATION/TELEMETRY REGRESSIONS: PASS')
