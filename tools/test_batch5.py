#!/usr/bin/env python3
from pathlib import Path
import re, sys
root=Path(__file__).resolve().parents[1]
def read(p): return (root/p).read_text()
def ok(cond,msg):
    if not cond:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

dt=read('src/datatypes.h'); cmd=read('src/comm/commands.c'); cg=read('src/confgenerator.c'); cgh=read('src/confgenerator.h')
app=read('src/applications/app.c'); apph=read('src/applications/app.h'); uart=read('src/applications/app_uartcomm.c')
cfg=read('src/conf_general.c'); cfgh=read('src/conf_general.h'); buf=read('src/util/buffer.c'); bufh=read('src/util/buffer.h')
mc=read('src/motor/mc_interface.c')

ok(all(x in dt for x in ['float ah_tot;','float current_in_tot;','uint8_t num_vescs;']), 'setup_values uses canonical cumulative totals')
ok(all(x in dt for x in ['double samples;','double speed_sum;','float max_power;','double current_sum;']), 'setup_stats is separated from setup_values')
ok('APP_NONE = 0' in dt and 'APP_UART' in dt and 'typedef struct {\n    uint8_t controller_id;' in dt, 'reduced canonical app_use/app_configuration are present')
ok('COMM_FW_VERSION=0' in dt and 'COMM_APP_DISABLE_OUTPUT=63' in dt and 'COMM_GET_STATS=128' in dt and 'COMM_MOTOR_ESTOP=159' in dt, 'COMM_PACKET_ID is canonical through current ID 159')
ok('enum {\n    COMM_FW_VERSION' not in cmd, 'commands.c no longer owns a private COMM enum')
ok('v.current_tot=l->motor_current+r->motor_current' in mc and 'v.num_vescs=2U' in mc, 'setup values aggregate both local bridges')
ok('scaled_i32(sv->current_tot' in cmd and 'scaled_i32(sv->ah_tot' in cmd, 'COMM_GET_VALUES_SETUP consumes canonical setup_values')
# Standard GET_VALUES must remain per-selected-motor telemetry, never setup aggregate.
get_start=cmd.index('static void append_get_values_fields')
get_end=cmd.index('static void reply_get_values',get_start)
get_chunk=cmd[get_start:get_end]
ok('sv->' not in get_chunk and 't->current_motor' in get_chunk and 't->amp_hours' in get_chunk, 'COMM_GET_VALUES remains per-motor telemetry after setup_values refactor')

# Typed serializers preserve represented fields even if the hardware ownership gate
# rejects applying fields whose runtime backend is deferred to a later batch.
ok('put_f16_at(w,211U,c->foc_sl_openloop_hyst' in cg and 'put_auto_at(w,344U,c->s_pid_min_erpm)' in cg and 'put_auto_at(w,365U,c->p_pid_kd_proc)' in cg and 'put_f16_at(w,381U,c->cc_startup_boost_duty' in cg, 'mcconf typed serializer round-trips represented deferred fields')
ok('buffer[VESC6_APP_OFF_CONTROLLER_ID]=conf->controller_id' in cg, 'appconf typed serializer round-trips represented controller ID before apply-time ownership validation')

for sym in ['confgenerator_serialize_mcconf','confgenerator_serialize_appconf','confgenerator_deserialize_mcconf','confgenerator_deserialize_appconf','confgenerator_set_defaults_mcconf','confgenerator_set_defaults_appconf']:
    ok(sym in cgh and sym in cg, f'{sym} wrapper exists')
ok('VESC6_MCCONF_WIRE_SIZE 481U' in cgh and 'VESC6_APPCONF_WIRE_SIZE 493U' in cgh, 'typed wrappers retain pinned VESC6 wire sizes')

for sym in ['conf_general_read_app_configuration','conf_general_store_app_configuration','conf_general_read_mc_configuration','conf_general_store_mc_configuration']:
    ok(sym in cfgh and sym in cfg, f'{sym} typed persistence wrapper exists')

for sym in ['app_get_configuration','app_set_configuration','app_disable_output','app_is_output_disabled','app_calc_crc']:
    ok(sym in apph and sym in app, f'{sym} reduced app manager exists')
ok('APP_PPM' not in app and 'APP_NRF' not in app and 'APP_ADC' in app and 'APP_ADC_UART' in app, 'app manager enables only the required ADC/UART application backends')
ok('app_uartcomm_send_packet' in apph and 'vesc_packet_encode' in uart and 'app_uartcomm_write_raw' in uart, 'canonical UART send API preserves VESC packet framing over B4 DMA')
ok('s_transport_initialized' in uart and 'if(s_transport_initialized)return true' in uart, 'B4 DMA transport initialization is idempotent for canonical app API')

for sym in ['buffer_append_int64','buffer_append_uint64','buffer_append_float64_auto','buffer_get_int64','buffer_get_uint64','buffer_get_float64_auto']:
    ok(sym in bufh and sym in buf, f'{sym} canonical buffer API exists')
ok('vesc_buf_append_float32_auto' in buf and 'vesc_buf_get_float32_auto' in buf, 'canonical buffer wrappers reuse proven VESC wire helpers')

# Structural-only batch: verify key FOC implementation files are byte-identical to B4 baseline if available.
base=Path('/mnt/data/vesc_b4_baseline')
if base.exists():
    import hashlib
    for rel in ['src/motor/mcpwm_foc.c','src/motor/foc_math.c','src/hwconf/hw.c','src/timeout.c']:
        a=(root/rel).read_bytes(); b=(base/rel).read_bytes()
        ok(hashlib.sha256(a).digest()==hashlib.sha256(b).digest(), f'B5 leaves {rel} unchanged')
print('ALL BATCH 5 STRUCTURAL/API REGRESSIONS: PASS')
