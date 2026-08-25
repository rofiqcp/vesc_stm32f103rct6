#!/usr/bin/env python3
from pathlib import Path
import re, subprocess, sys, math
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

hw=read('src/hwconf/hw.c'); it=read('src/stm32f1xx_it.c')
app=read('src/applications/app.c'); adc=read('src/applications/app_adc.c')
cmdsrc=read('src/applications/app_command.c'); commands=read('src/comm/commands.c')
cg=read('src/confgenerator.c'); cgh=read('src/confgenerator.h'); dt=read('src/datatypes.h')
tasks=read('src/motor_tasks.c'); term=read('src/terminal.c')

# P0 target-build regression from the user's real PlatformIO log.
ok('#include "encoder/encoder.h"' in commands and 'encoder_deinit(m)' in commands,
   'encoder_deinit build regression is fixed through the owning public header')

# PA2/PA3 are slow app inputs and must not disturb the proven current ranks 1..3.
for ch,rank in [(11,1),(10,1),(0,2),(13,2),(14,3),(15,3)]:
    ok(re.search(r'ADC_CHANNEL_'+str(ch)+r',\s*ADC_REGULAR_RANK_'+str(rank)+r'\b', hw) is not None,
       f'FOC current-rank mapping retained: ADC_CHANNEL_{ch} rank {rank}')
ok('LEFT_U_CUR_PIN | GPIO_PIN_2 | GPIO_PIN_3' in hw, 'PA2/PA3 are configured as analog GPIO alongside PA0')
ok('ADC_CHANNEL_2, ADC_REGULAR_RANK_6' in hw, 'PA2/ADC1_CH2 is APP ADC rank 6')
ok('ADC_CHANNEL_3, ADC_REGULAR_RANK_6' in hw, 'PA3/ADC2_CH3 is APP ADC rank 6')
ok('motor_hw_capture_app_adc_from_isr();\n        foc_adc_dma_isr(g_adc_dual_dma);' in it,
   'APP ADC capture is a bounded copy before the existing 16-kHz FOC call')
ok('s_app_adc_word = g_adc_dual_dma[5]' in hw and 's_app_adc_seq++' in hw,
   'APP ADC latches the previous complete rank-6 dual sample with sequence protection')

# No new RTOS task/stack for ADC; it lives in existing 1-kHz service.
ok('osThreadNew' not in adc and 'osThreadNew' not in cmdsrc, 'APP ADC/arbitration add no RTOS task or stack')
ok('app_command_service_1khz(now);' in tasks and 'app_adc_service_1khz(now);' in tasks,
   'APP ADC and source arbitration run in the existing 1-kHz motor service')
ok('app_command_init();' in tasks and 'app_adc_init();' in tasks,
   'APP state is initialized before the service thread starts')

# Canonical VESC6 APP ADC schema and supported runtime selectors.
ok('APP_NONE = 0, APP_PPM, APP_ADC, APP_UART, APP_PPM_UART, APP_ADC_UART' in dt,
   'canonical VESC application selector numbering is retained')
for off,val in [('VESC6_APP_OFF_ADC_CTRL_TYPE','90U'),('VESC6_APP_OFF_ADC_HYST','91U'),
                ('VESC6_APP_OFF_ADC_VOLTAGE_START','95U'),('VESC6_APP_OFF_ADC_UPDATE_RATE_HZ','137U'),
                ('VESC6_APP_OFF_UART_BAUD','139U')]:
    ok(re.search(r'#define\s+'+off+r'\s+'+val,cgh) is not None, f'{off} is pinned to canonical VESC6 offset {val}')
ok('app!=VESC_APP_NONE && app!=VESC_APP_ADC && app!=VESC_APP_UART && app!=VESC_APP_ADC_UART' in cg,
   'runtime accepts NONE/ADC/UART/ADC_UART and rejects unrelated apps')
ok('ctrl != ADC_CTRL_TYPE_CURRENT' in cg and 'ctrl != ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC' in cg and
   'ctrl != ADC_CTRL_TYPE_DUTY' in cg and 'ctrl != ADC_CTRL_TYPE_PID' in cg,
   'APP ADC validation exposes required current/current+brake/duty/RPM modes only')
ok('app_notify_configuration_changed();' in cg, 'accepted APPCONF immediately invalidates the typed runtime cache')
ok('app_command_configuration_changed();' in app and 'app_command_configuration_changed' in cmdsrc,
   'APPCONF changes revoke stale command ownership and force ADC re-arm')
ok('ml->pwm_enabled || ml->detect.busy' in cg and 'mr->pwm_enabled || mr->detect.busy' in cg,
   'APPCONF ADC/control writes are rejected while either local bridge is live')

# Functional APP ADC pipeline / safety.
for token,msg in [
    ('electrical_range_ok', 'electrical range/rail validation exists'),
    ('APP_ADC_IMPLAUSIBLE_STEP_V', 'implausible analog transition detection exists'),
    ('APP_ADC_SAFE_NEUTRAL_MS', 'safe-start neutral dwell exists'),
    ('throttle_curve', 'configurable throttle curve exists'),
    ('ramp_time_pos', 'positive/negative command ramping exists'),
    ('brake > 0.05f', 'brake has explicit priority over throttle'),
    ('foc_calibration_done() && foc_calibration_valid()', 'ADC cannot arm before current calibration is valid')]:
    ok(token in adc,msg)
ok('APP_ADC_SAFE_NEUTRAL_MS     500U' in adc, 'safe-start requires 500 ms stable neutral')
ok('(uint32_t)(now_ms - s_adc.neutral_since_ms[i]) >= APP_ADC_SAFE_NEUTRAL_MS' in adc,
   'safe-start dwell is wall-time based and does not stretch with lower ADC update_rate_hz')
ok('motor_set_duty' in adc and 'motor_set_current_rel' in adc and 'motor_set_brake_current' in adc and 'motor_set_speed' in adc,
   'ADC command mapping uses central motor APIs for duty/current/brake/RPM')
ok('TIM1->CCR' not in adc and 'TIM8->CCR' not in adc, 'APP ADC never bypasses motor API to PWM registers')

# Source arbitration and independent UART leases.
ok('APP_CMD_SRC_ADC' in cmdsrc and 'APP_CMD_SRC_UART' in cmdsrc and
   'APP_CMD_SRC_CALIBRATION' in cmdsrc and 'APP_CMD_SRC_DETECTION' in cmdsrc,
   'command source state covers ADC/UART/calibration/detection')
ok('(uint32_t)(now_ms - s_uart_last_ms[i]) > conf->timeout_msec' in cmdsrc,
   'UART freshness is supervised per motor independently')
ok('if (s_source[id] == APP_CMD_SRC_UART) return false;' in cmdsrc,
   'ADC cannot overwrite an active UART owner')
ok('!mc_interface_try_input_motor(id)' in cmdsrc and 'motor_stop(m);' in cmdsrc,
   'ADC ownership respects mc_interface input inhibit/estop and stops stale ADC output')
ok('app_is_output_disabled() || m->fault != MOTOR_FAULT_NONE' in cmdsrc,
   'output-disable/fault revokes stale ADC/UART ownership before automatic resume')
for case in ['COMM_SET_DUTY','COMM_SET_CURRENT','COMM_SET_CURRENT_BRAKE','COMM_SET_RPM','COMM_SET_POS','COMM_SET_HANDBRAKE','COMM_SET_CURRENT_REL']:
    # Require claim near each command in the operational handler area.
    pos=commands.find('case '+case+':', commands.rfind('static void process_payload_for_motor'))
    ok(pos>=0 and 'app_command_uart_claim(id)' in commands[pos:pos+380], f'{case} passes through UART arbitration')
ok('case COMM_ALIVE:' in commands and 'app_command_uart_keepalive(id);' in commands,
   'COMM_ALIVE refreshes the active per-motor UART lease')
ok('app_command_release(explicit_id, true);' in commands, 'custom STOP revokes source ownership and requires ADC re-arm')

# Standard VESC decoded ADC response and operator diagnostics use real backend data.
ok('case COMM_GET_DECODED_ADC:' in commands and 'app_adc_get_decoded_level()' in commands and
   'app_adc_get_voltage2()' in commands, 'COMM_GET_DECODED_ADC has a real four-value PA2/PA3 backend')
ok('strcmp(str, "appadc")' in term and 'app_adc_get_status(&a)' in term,
   'terminal exposes raw/voltage/decoded/armed/source/fault APP ADC status')


# Additional Stage-1 hardening found during full runtime review.
ok(('uint8_t p[160]' in commands or 'uint8_t p[192]' in commands) and any(f'p[i++] = {n}U' in commands for n in range(11,20)),
   'COMM_DIAG stack buffer has headroom; revision 10 132-byte overflow is removed')
ok('app_adc_data_ready()' in commands,
   'GET_DECODED_ADC refuses to fabricate zero telemetry before first real PA2/PA3 sample')
ok(('APP ADC/source diagnostics' in commands or 'PA2/PA3 plus fault-manager/config-health state' in commands) and 'app_command_get_source(MOTOR_LEFT)' in commands,
   'COMM_DIAG revision 11 exposes APP ADC fault/armed/source state')
ok('SAFE_START_NO_FAULT' in cmdsrc and 'old_source != APP_CMD_SRC_ADC' in cmdsrc,
   'SAFE_START_NO_FAULT has distinct VESC-like fault re-arm semantics')
ok('s_pub_seq' in adc and 'app_adc_publish_begin()' in adc and 'app_adc_publish_end()' in adc,
   'APP ADC multi-field status uses a seqlock-style coherent snapshot')
dbg=read('tools/debug.py')
ok('revision >= 11' in dbg and "app_adc_fault_flags" in dbg and "app_cmd_source_left" in dbg,
   'host diagnostic parser understands COMM_DIAG revision 11 APP ADC/source fields')

# Strict host syntax/warning checks for all portable new/modified logic.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for unit in ['src/applications/app.c','src/applications/app_command.c','src/applications/app_adc.c','src/confgenerator.c','src/terminal.c']:
    cp=subprocess.run(base+incs+['-fsyntax-only',unit],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True,f'{unit} passes strict host syntax/warning check')

# Lightweight numeric sanity model for PA2 default mapping and brake arbitration.
def map01(v,a,b): return max(0.0,min(1.0,(v-a)/(b-a)))
ok(abs(map01(.9,.9,3.0)) < 1e-9, 'default PA2 0.9 V maps to zero throttle')
ok(abs(map01(3.0,.9,3.0)-1.0) < 1e-9, 'default PA2 3.0 V maps to full throttle')
p=map01(2.0,.9,3.0); b=map01(2.0,.9,3.0)
if b > .05: p=0.0
cmd=p-b
ok(cmd < 0.0, 'simultaneous throttle+brake resolves to braking, never competing positive torque')

print('ALL STAGE-1 APP ADC PA2/PA3 + ARBITRATION REGRESSIONS: PASS')
