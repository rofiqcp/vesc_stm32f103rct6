#!/usr/bin/env python3
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]
cmd=(ROOT/'src/comm/commands.c').read_text()
appdef=(ROOT/'src/applications/appconf_default.h').read_text()
mc=(ROOT/'src/motor/mc_interface.c').read_text()
dbg=(ROOT/'tools/debug.py').read_text()

def ok(cond,msg):
    if not cond:
        raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

ok('#define VESC_CONTROLLER_ID_LEFT         1U' in appdef, 'direct VESC identity is motor-1 / ID 1')
ok('#define VESC_CONTROLLER_ID_RIGHT        2U' in appdef, 'local second VESC identity is motor-2 / ID 2')
ok('#define VESC_LOCAL_MOTOR2_FORWARD_ID    VESC_CONTROLLER_ID_RIGHT' in appdef, 'forward target equals advertised motor-2 ID')

# VESC Tool CAN scan must see only the extra local controller, never duplicate root ID 1.
ping_block=re.search(r'case COMM_PING_CAN:(.*?)(?:case COMM_|default:)',cmd,re.S)
ok(ping_block is not None, 'COMM_PING_CAN is implemented')
pb=ping_block.group(1)
ok('{COMM_PING_CAN, VESC_LOCAL_MOTOR2_FORWARD_ID}' in pb, 'root scan replies with local motor-2 ID')
ok('MOTOR_LEFT' in pb and 'uint8_t p[1] = {COMM_PING_CAN}' in pb, 'forwarded M2 ping reports no downstream nodes')
ok('VESC_CONTROLLER_ID_LEFT' not in pb, 'root ID 1 is not duplicated in scan response')

# Routing selected VESC Tool node 2 must process the inner command as right motor.
ok('case COMM_FORWARD_CAN:' in cmd, 'COMM_FORWARD_CAN is implemented')
ok('target == VESC_LOCAL_MOTOR2_FORWARD_ID' in cmd, 'only advertised ID 2 is accepted as local forward target')
ok('process_payload_for_motor(&data[2], (uint16_t)(len - 2U), MOTOR_RIGHT)' in cmd, 'forwarded payload executes against right motor runtime')

# Both logical controllers must expose distinct identities to Tool.
ok('if (id == MOTOR_RIGHT) p[i - 1U]++' in cmd, 'M2 FW_VERSION UUID differs from M1 for backup identity')
ok('controller_id_for_motor' in cmd and 'VESC_CONTROLLER_ID_RIGHT' in cmd, 'telemetry has per-motor controller ID')
ok('if (id == MOTOR_RIGHT) p[1U + 4U] = VESC_LOCAL_MOTOR2_FORWARD_ID' in cmd, 'M2 APPCONF reads as ID 2')
ok('v.num_vescs=2U' in mc.replace(' ',''), 'setup values report two VESC motor controllers')

# Config write semantics: M2 exact readback image may be written, but attempts to mutate its fixed ID are rejected.
ok('job.data[1U + VESC6_APP_OFF_CONTROLLER_ID] == VESC_CONTROLLER_ID_RIGHT' in cmd, 'M2 APPCONF write validates public ID 2 before normalization')
ok('job.data[1U + VESC6_APP_OFF_CONTROLLER_ID] = VESC_CONTROLLER_ID_LEFT' in cmd, 'validated M2 APPCONF is normalized only for shared internal persistence')
ok('vesc_config_set_mc_wire(m->id' in cmd, 'MCCONF writes are stored per selected motor')

# Basic commands Tool needs after selecting either node.
for token in ('COMM_FW_VERSION','COMM_GET_VALUES','COMM_GET_MCCONF','COMM_GET_APPCONF',
              'COMM_SET_MCCONF','COMM_SET_APPCONF','COMM_SET_DUTY','COMM_SET_CURRENT',
              'COMM_SET_CURRENT_BRAKE','COMM_SET_RPM','COMM_SET_POS','COMM_ALIVE'):
    ok(token in cmd, f'{token} path is present')

# Host-side commissioning tool mirrors VESC Tool routing and scan semantics.
ok('COMM_PING_CAN = 62' in dbg, 'debug tool knows canonical COMM_PING_CAN ID 62')
ok('def cmd_vesc_tool_dual_basic' in dbg, 'passive dual VESC Tool end-to-end checker is available')
ok('parse_ping_can(bytes((COMM_PING_CAN,2))) == [2]' in dbg, 'debug self-test covers motor-2 discovery payload')

print('ALL VESC TOOL DUAL BASIC SOURCE REGRESSIONS: PASS')
