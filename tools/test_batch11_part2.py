#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys, re
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text(errors='replace')
def ok(c,m):
    if not c:
        print('FAIL:',m); sys.exit(1)
    print('PASS:',m)
cmd=read('src/comm/commands.c'); conf=read('src/conf_general.c'); confh=read('src/conf_general.h')
mc=read('src/motor/mc_interface.c'); foc=read('src/motor/mcpwm_foc.c'); foch=read('src/motor/mcpwm_foc.h')
status=read('src/status_io.c'); statush=read('src/status_io.h'); tasks=read('src/motor_tasks.c')
cfg=read('src/confgenerator.c'); dt=read('src/datatypes.h'); term=read('src/terminal.c')

ok(re.search(r'vesc-f103-hoverboard-v(?:2[7-9]|[3-9][0-9])-', cmd) is not None, 'firmware identity is Part-1 or later while protocol remains VESC6')
ok('case COMM_SET_BATTERY_CUT:' in cmd and 'case COMM_GET_BATTERY_CUT:' in cmd, 'VESC battery-cut SET/GET commands are implemented')
ok('return len == 11U;' in cmd and 'forward_all' in cmd, 'battery-cut payload length and forward-all flag follow VESC command format')
ok('battery_cut_apply_both' in cmd and 'conf_general_store_all()' in cmd, 'forward-all battery cut applies local M1+M2 with one transactional store')
body=re.search(r'static bool battery_cut_apply_both\(.*?\n\}',cmd,re.S)
ok(body is not None and 's_mc_backup[MOTOR_LEFT]' in body.group(0) and 's_mc_backup[MOTOR_RIGHT]' in body.group(0) and body.group(0).count('vesc_config_set_mc_wire') >= 4,
   'dual battery-cut update snapshots and rolls back both active MCCONFs on failure')
ok('put_i32(p, &bi, scaled_i32(c.l_battery_cut_start, 1000.0f))' in cmd and 'c.l_battery_cut_end' in cmd,
   'GET_BATTERY_CUT uses canonical 1e3 int32 representation')

ok('case COMM_FW_INFO:' in cmd and 'reply_fw_info();' in cmd, 'COMM_FW_INFO is implemented')
ok('p[i++] = 6U; p[i++] = 0U; p[i++] = 0U;' in cmd and 'p[i++] = 0U;\n    p[i++] = 0U;' in cmd,
   'FW_INFO truthfully reports 6.00/test0 with empty local git hashes')
ok('case COMM_SHUTDOWN:' in cmd, 'COMM_SHUTDOWN is implemented without a bootloader')
ok('fabsf(g_motor_left.erpm) <= 100.0f' in cmd and 'fabsf(g_motor_right.erpm) <= 100.0f' in cmd,
   'shutdown safety checks both local motors, not only the selected virtual controller')
ok('motor_hw_emergency_all_off()' in cmd and 'status_io_power_hold(false)' in cmd,
   'shutdown clears power-stage outputs before dropping PA5 power hold')
ok('s_shutdown_latched' in cmd and 's_shutdown_latched && command_requires_motor_ready(cmd)' in cmd,
   'shutdown remains input-latched if external power does not immediately collapse')

ok('status_io_power_hold(bool on)' in status and 'OFF_PORT, OFF_PIN' in status, 'PA5 power-hold has a single explicit StatusIO backend')
ok('status_io_tone_start_for' in status and 's_tone_toggle_remaining' in status, 'timed buzzer tones are non-blocking and IRQ-counted')
ok('status_io_tone_start_for' in status and 's_tone_toggle_remaining' in status,
   'buzzer tone engine remains non-blocking after dead VESC audio-wrapper cleanup')
ok(all(name not in foc for name in ('mcpwm_foc_beep','mcpwm_foc_play_tone','mcpwm_foc_stop_audio',
                                    'mcpwm_foc_set_audio_sample_table','mcpwm_foc_get_audio_sample_table',
                                    'mcpwm_foc_play_audio_samples')),
   'unused VESC motor-audio compatibility stubs are removed instead of returning fake/unsupported state')

ok('s_integrity_fault' in conf and 's_integrity_checks' in conf and 's_integrity_failures' in conf,
   'transactional flash config has persistent integrity-supervision state')
ok('++s_integrity_div >= 100U' in conf, 'config scrub runs at 1 Hz rather than burdening the FOC or 1-kHz control math')
ok('!g_motor_left.pwm_enabled && !g_motor_right.pwm_enabled' in conf, 'CRC scrub is deferred until both bridges are idle')
ok('page_hdr((uint32_t)bp)->sequence == s_save_count' in conf, 'scrub verifies the exact last committed sequence, not merely any older valid page')
ok('MOTOR_FAULT_FLASH_CONFIG' in dt and 'FAULT_CODE_FLASH_CORRUPTION' in mc, 'config corruption maps to canonical VESC flash-corruption fault')
ok('motor_hw_emergency_all_off();' in conf and 'MOTOR_FAULT_FLASH_CONFIG' in conf, 'detected config corruption hard-disables both bridges and faults both motors')
ok('m->fault == MOTOR_FAULT_FLASH_CONFIG && !conf_general_integrity_ok()' in mc,
   'persistent config corruption fault cannot be cleared while integrity remains bad')
ok('MOTOR_FAULT_FLASH_CONFIG' in tasks, 'flash integrity fault participates in canonical LED+buzzer fault reporting')
ok('s_integrity_fault=false;s_save_count=s_stage.h.sequence' in conf, 'a verified replacement record can repair the integrity latch before fault clear')
ok(any(f'p[i++] = {n}U' in cmd for n in range(10,20)), 'custom communications diagnostics retain Part-2 watchdog/sampling fields or later')
ok('conf_general_integrity_ok()' in cmd and 'status_io_power_is_held()' in cmd, 'communications diagnostics expose integrity and power-latch status')
ok('strcmp(str, "integrity")' in term, 'terminal exposes config integrity counters')

ok('legacy trapezoidal BLDC current-to-duty' in cfg and 'cc_min_current' in cfg, 'cc_* semantics are corrected: only cc_min_current is applicable to FOC')
ok('no backend yet' not in cfg, 'misleading unfinished-current-controller marker is removed')
# Prove upstream-inapplicable fields remain immutable in VESC6 byte map.
mut=re.search(r'static bool mc_wire_byte_runtime_mutable\(.*?\n\}',cfg,re.S).group(0)
ok('byte_in_range(i, 383U, 4U)' in mut and '381U' not in mut and '387U' not in mut and '391U' not in mut,
   'BLDC-only cc_startup_boost/cc_gain/cc_ramp remain immutable; FOC cc_min_current remains writable')

combined='\n'.join([cmd,conf,mc,foc,status,tasks,cfg,term]).lower()
for token in ['#include "comm_can.h"','#include "imu/','#include "bms/','#include "bm_if','#include "nrf','#include "ledpwm','#include "comm_usb','#include "lispif','#include "lzo']:
    ok(token not in combined, f'excluded subsystem not introduced: {token}')

# Strict host compile the portable/touched FOC/config units using established stubs.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for unit,extra in [('src/motor/mc_interface.c',[]),('src/motor/mcpwm_foc.c',['-DDMA1_Channel1=DMA2_Channel5']),('src/confgenerator.c',[]),('src/terminal.c',[])]:
    cp=subprocess.run(base+incs+extra+['-fsyntax-only',unit],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True, unit+' passes strict host syntax/warning check')

cp=subprocess.run([sys.executable,'tools/debug.py','--self-test'],cwd=ROOT,text=True,capture_output=True)
if cp.returncode:
    print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
ok('SELF-TEST PASS' in cp.stdout,'debug parser accepts extended COMM_DIAG integrity/power fields')
print('ALL BATCH 11 PART-2 COMMAND/INTEGRITY/CLEANUP REGRESSIONS: PASS')
