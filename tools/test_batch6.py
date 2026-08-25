#!/usr/bin/env python3
"""Batch 6 source/numeric regressions: VESC Tool ecosystem and persistence."""
from pathlib import Path
import hashlib, re, sys
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(cond,msg):
    if not cond:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

cmd=read('src/comm/commands.c'); cmdh=read('src/comm/commands.h')
term=read('src/terminal.c'); mc=read('src/motor/mc_interface.c'); mch=read('src/motor/mc_interface.h')
conf=read('src/conf_general.c'); confh=read('src/conf_general.h'); app=read('src/applications/app.c')
dt=read('src/datatypes.h'); cg=read('src/confgenerator.h')

# Terminal/plot/experiment compatibility.
ok('COMM_TERMINAL_CMD_SYNC' in cmd and 'COMM_TERMINAL_CMD' in cmd and 'terminal_process_string' in cmd,
   'async and sync terminal command paths are present')
ok('motor_take_pending_fault_mask()' not in term, 'terminal diagnostics do not destructively consume pending faults')
for sym in ['commands_send_experiment_samples','commands_init_plot','commands_plot_add_graph','commands_plot_set_graph','commands_send_plot_points']:
    ok(sym in cmd and sym in cmdh, f'{sym} VESC Tool helper exists')
ok('COMM_EXPERIMENT_SAMPLE' in cmd and 'COMM_PLOT_INIT' in cmd and 'COMM_PLOT_DATA' in cmd and 'COMM_PLOT_ADD_GRAPH' in cmd and 'COMM_PLOT_SET_GRAPH' in cmd,
   'plot/experiment helpers use canonical command IDs')

# Temporary MCCONF behavior and flash-stall isolation.
ok('case COMM_SET_MCCONF_TEMP:' in cmd and 'case COMM_SET_MCCONF_TEMP_SETUP:' in cmd,
   'temporary MCCONF commands are recognized')
block=cmd[cmd.index('static bool is_blocking_command'):cmd.index('static bool blocking_command_length_valid')]
ok('COMM_SET_MCCONF_TEMP' in block and 'COMM_SET_MCCONF_TEMP_SETUP' in block,
   'temporary MCCONF is routed through blocking worker on STM32F1')
ok('job.cmd == COMM_SET_MCCONF_TEMP || job.cmd == COMM_SET_MCCONF_TEMP_SETUP' in cmd,
   'blocking worker executes temporary MCCONF')
ok('divide?2.0f:1.0f' in cmd and '(void)forward' in cmd,
   'temporary limits understand two local bridges without adding physical CAN')
ok('COMM_GET_MCCONF_TEMP' in cmd and 'reply_mcconf_temp' in cmd,
   'temporary MCCONF setup is readable by VESC Tool')

# Application output-disable behavior (app-only, as upstream semantics).
ok('case COMM_APP_DISABLE_OUTPUT:' in cmd and 'app_disable_output(get_i32_be(&data[2]))' in cmd,
   'COMM_APP_DISABLE_OUTPUT uses app output gate')
ok('s_disabled_indefinite' in app and 'app_is_output_disabled' in app,
   'reduced app manager implements timed/indefinite output disable state')

# Real statistics, not instantaneous aliases.
ok('static setup_stats s_setup_stats[2]' in mc and 'st->samples += 1.0' in mc and 'st->speed_sum += speed' in mc and 'st->power_sum += power' in mc,
   'setup statistics are accumulated per bridge task-side')
ok('s->samples>0.0?(float)(s->speed_sum/s->samples)' in mc and 's->samples>0.0?(float)(s->power_sum/s->samples)' in mc,
   'statistics getters return real averages')
ok('case COMM_GET_STATS:' in cmd and 'reply_stats(' in cmd and 'case COMM_RESET_STATS:' in cmd and 'mc_interface_stat_reset()' in cmd,
   'VESC Tool GET_STATS and RESET_STATS are implemented')
ok('buffer_append_uint32(p,mask' in cmd and all(f'mc_interface_stat_{n}' in cmd for n in ['speed_avg','speed_max','power_avg','power_max','current_avg','current_max','count_time']),
   'GET_STATS response follows masked upstream statistics layout')

# Odometer persistence and legacy migration.
ok('#define CFG_VERSION_LEGACY   0x0012U' in conf and '#define CFG_VERSION          0x0013U' in conf,
   'B6 config record explicitly migrates B5 v0x0012 to v0x0013')
ok('CFG_PAYLOAD_V12_LEN' in conf and 'CFG_PAYLOAD_LEN      (CFG_PAYLOAD_V12_LEN + 16U)' in conf,
   'v0x0013 appends exactly two 64-bit odometers after unchanged VESC wire payload')
ok('h->version==CFG_VERSION_LEGACY' in conf and 'vesc_config_import_wire(oldp->mc_left,oldp->mc_right,oldp->app)' in conf,
   'legacy B5 records remain loadable without losing MCCONF/APPCONF')
ok('mc_interface_set_odometer_motor(MOTOR_LEFT,get_u64_be8' in conf and 'mc_interface_set_odometer_motor(MOTOR_RIGHT,get_u64_be8' in conf,
   'v0x0013 restores left and right odometers')
ok('put_u64_be8(s_stage.payload.odometer_left_be' in conf and 'put_u64_be8(s_stage.payload.odometer_right_be' in conf,
   'v0x0013 stores both odometers')
ok('if(dl>=1000U||dr>=1000U)s_aux_store_pending=true' in conf,
   'automatic odometer persistence is wear-limited to 1 km delta')
ok('if(g_motor_left.pwm_enabled||g_motor_right.pwm_enabled||g_motor_left.command_active||g_motor_right.command_active)return' in conf,
   'automatic flash persistence is deferred while either motor is active')
ok('static void stage_persistent_base(void)' in conf and 'page_payload((uint32_t)p)' in conf and 'vesc_config_mc_wire(MOTOR_LEFT,true)' in conf,
   'persistent base is read from committed flash/defaults without an SRAM shadow')
ok('s_persist_mc' not in conf and 's_persist_app' not in conf,
   'B6 avoids the ~1.45 KiB persistent-config SRAM shadow')
ok('conf_general_store_aux_only()' in conf and 'stage_persistent_base();' in conf,
   'odometer-only save writes last persistent config, never temporary MCCONF')
ok('conf_general_store_mc_wire_persistent' in conf and 'conf_general_store_app_wire_persistent' in conf,
   'motor/app stores update only their persistent config component')
ok('case COMM_SET_ODOMETER:' in cmd and 'conf_general_request_aux_store()' in cmd,
   'COMM_SET_ODOMETER updates selected motor and schedules safe persistence')

# E-stop must block direct UART commands, not merely stop once.
ok('case COMM_MOTOR_ESTOP:' in cmd and 'mc_interface_ignore_input_both' in cmd and 'mc_interface_release_motor_override_both' in cmd,
   'COMM_MOTOR_ESTOP uses canonical ignore-input plus release behavior')
start=cmd.index('static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id) {')
end=cmd.index('static void process_payload(const uint8_t *data, uint16_t len) {', start)
motor_switch=cmd[start:end]
for c in ['COMM_SET_DUTY','COMM_SET_CURRENT','COMM_SET_CURRENT_BRAKE','COMM_SET_RPM','COMM_SET_POS','COMM_SET_HANDBRAKE','COMM_SET_CURRENT_REL']:
    pos=motor_switch.index('case '+c+':')
    nxt=motor_switch.find('case ',pos+6)
    chunk=motor_switch[pos:nxt if nxt!=-1 else pos+500]
    ok('mc_interface_try_input_motor(id)' in chunk, f'{c} obeys ESTOP/input-ignore gate')
ok('int mc_interface_try_input_motor(motor_id_t id)' in mc and 's_ignore_until[id]' in mc,
   'per-motor input gate is backed by ignore-input deadline')

# Keep pinned VESC6 ABI and preserve hard-control files from B5.
ok('VESC6_MCCONF_WIRE_SIZE 481U' in cg and 'VESC6_APPCONF_WIRE_SIZE 493U' in cg,
   'B6 retains pinned VESC6 481/493 config ABI')
base=Path('/mnt/data/vesc_b5_baseline')
if base.exists():
    for rel in ['src/motor/mcpwm_foc.c','src/motor/foc_math.c','src/hwconf/hw.c','src/timeout.c','src/applications/app_uartcomm.c','src/comm/packet.c','src/comm/packet.h']:
        ok(hashlib.sha256((ROOT/rel).read_bytes()).digest()==hashlib.sha256((base/rel).read_bytes()).digest(),
           f'B6 leaves {rel} byte-identical to B5')

# Numeric persistence layout sanity, independent of C struct padding.
legacy=2*481+493
new=legacy+16
ok(legacy==1455 and new==1471 and 16+new < 2048,
   'v0x0013 payload and 16-byte header fit one 2 KiB STM32F1 flash page')

print('ALL BATCH 6 VESC-TOOL/DIAGNOSTIC REGRESSIONS: PASS')
