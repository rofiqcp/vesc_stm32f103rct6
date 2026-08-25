#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def read(p): return (R/p).read_text(errors='ignore')
def req(c,m):
    if not c: raise AssertionError(m)

dt=read('src/datatypes.h'); fm=read('src/motor/foc_math.c'); mi=read('src/motor/mc_interface.c')
cmd=read('src/comm/commands.c'); cg=read('src/conf_general.c'); th=read('src/timeout.h'); mt=read('src/motor/mc_interface_tasks.c'); tel=read('src/telemetry.c')
req('sensorless_start_failures' in dt,'missing sensorless failure counter')
req('True handover blend' in fm and 'max_corr = 546' in fm,'missing bounded sensorless phase blend')
req('MOTOR_FAULT_SENSORLESS_OBSERVER' in dt and 'sensorless_start_failures >= 3U' in mi,'missing sensorless lock fault escalation')
req('validate_sensorless_runtime' in cmd and 'stable_ms >= 150U' in cmd,'Detect-All sensorless fallback is not validated')
req('foc_sensor_mode' in dt and 't->foc_sensor_mode' in tel,'logical FOC sensor telemetry missing')
req('linear_position' in mi and 'position_target_deg = pos_deg' in read('src/motor/mcpwm_foc.c'),'LEFT encoder no-wrap position missing')
req('config_region_blank' in cg and 'CONF_BOOT_CORRUPT' in cg,'virgin/corrupt flash distinction missing')
req('TIMEOUT_HEARTBEAT_FAULT' in th and 'timeout_heartbeat(TIMEOUT_HEARTBEAT_FAULT)' in mt,'fault manager watchdog heartbeat missing')
req('uint8_t p[32]' in cmd and 'conf_general_boot_status()' in cmd,'config status buffer/boot status missing')
req((('uint8_t p[192]' in cmd or 'uint8_t p[224]' in cmd) or ('payload_begin()' in cmd and 'payload_end(i)' in cmd)),'diagnostic bounded-payload headroom missing')
print('STAGE2 SENSOR/SAFETY STATIC TEST PASS')
