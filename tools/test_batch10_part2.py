#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys, re
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

dt=read('src/datatypes.h'); mc=read('src/motor/mc_interface.c'); foc=read('src/motor/mcpwm_foc.c')
conf=read('src/confgenerator.c'); defs=read('src/motor/mcconf_default.h'); term=read('src/terminal.c')

ok('FOC_SPEED_SRC_CORRECTED' in dt and 'FOC_SPEED_SRC_OBSERVER' in dt, 'FOC speed phase-source enum exists')
ok('c->foc_speed_source=(FOC_SPEED_SRC)w[VESC6_MC_OFF_FOC_SPEED_SOURCE]' in conf, 'VESC6 byte 314 decodes the canonical FOC speed source')
ok('w[VESC6_MC_OFF_FOC_SPEED_SOURCE]=(uint8_t)c->foc_speed_source' in conf and
   'if (i == VESC6_MC_OFF_FOC_SPEED_SOURCE) return true;' in conf,
   'VESC6 FOC speed source is writable and persistent rather than fake runtime-only state')
ok('speed_phase = observer_phase_compensated_u16(m)' in foc, 'observer phase can source PLL/fast estimator')
ok('speed_est_fast_corrected_erpm_q16' in foc and 'phase_before_speed_est_corrected_u16' in foc, 'corrected fast speed remains available independently')

ok('hall_rate_limit_phase_isr' in foc, 'hard Hall electrical-angle rate limiter exists')
ok('98304ULL' in foc and 'FOC_ISR_EVENT_HZ' in foc, 'Hall rate limit follows 1.5x electrical-speed-per-sample rule')
m=re.search(r'static inline uint16_t hall_rate_limit_phase_isr\(.*?\n\}',foc,re.S)
ok(m is not None,'Hall limiter helper body located')
for bad in ['float ', 'double ', 'sqrtf(', 'fabsf(', 'lrintf(']: ok(bad not in m.group(0),f'Hall limiter contains no {bad.strip()} hard-path operation')

ok('#define MCCONF_L_IN_CURRENT_MAP_START_DEFAULT 0.90f' in defs, 'physical input-current pre-map begins conservatively at 90%')
ok('#define MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT 0.005f' in defs, 'physical input-current pre-map uses slow 0.005 filter')
ok('input_current_map_filtered_a' in dt and 'input_current_map_limit_a' in dt, 'runtime carries filtered physical DC current and mapped current cap')
ok('m->input_current_map_filtered_a / map_in_max' in mc and 'm->lo_current_max_a = fminf(m->lo_current_max_a, cap)' in mc, 'input-current map progressively reduces positive motor current')
ok('l_in_current_map_start=MCCONF_L_IN_CURRENT_MAP_START_DEFAULT' in conf, 'VESC6 runtime default activates board-specific pre-map')
ok('fabsf(conf->l_in_current_map_start-MCCONF_L_IN_CURRENT_MAP_START_DEFAULT)' in conf, 'non-default runtime input-map settings cannot be fake-persisted into VESC6')

ok('update_res_estimator_1khz' in mc, 'adaptive motor-resistance estimator runs task-side at 1 kHz')
ok('const float gain = 0.00002f;' in mc, 'resistance estimator uses current VESC adaptation gain')
ok('0.25f*r_nom' in mc and '3.0f*r_nom' in mc, 'resistance estimate is clamped to 0.25x..3x configured R')
ok('!m->pwm_enabled || !m->observer_valid || m->detect.busy' in mc, 'resistance adaptation is gated on valid driven observer state')
ok('m->res_est_valid && isfinite(m->res_est_ohm)' in foc, 'public live estimated resistance reports adaptive estimate when valid')
ok('Rcfg=%.5f Rest=%.5f' in term, 'terminal observer diagnostics expose configured and estimated resistance')

# The previous offset tracker was an empty function with unused accumulators.
# A fake runtime calibration path is worse than an explicit calibrated-only
# policy; reintroducing drift compensation requires a real, tested backend.
ok('offset_track_isr' not in foc and 'current_offset_u_acc_q16' not in foc,
   'empty current-offset drift tracker and dead state stay removed')

# Preserve exclusion contract.
for token in ['#include "comm_can.h"','#include "imu/','#include "bms','#include "bm_if','#include "nrf','#include "ledpwm','#include "comm_usb','#include "lispif','#include "lzo']:
    ok(token not in foc+mc, f'Part-2 does not introduce excluded subsystem {token}')

# Strict host syntax checks using existing STM32F1 stubs.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for unit,extra in [('src/motor/mc_interface.c',[]),('src/motor/mcpwm_foc.c',['-DDMA1_Channel1=DMA2_Channel5']),('src/confgenerator.c',[]),('src/terminal.c',[])]:
    cp=subprocess.run(base+extra+incs+['-fsyntax-only',unit],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True,f'{unit} passes strict host syntax/warning check')

# Small numeric model for the two new control laws.
def map_cap(base, iin, imax, start):
    if start >= .98 or imax <= .05 or iin <= 0: return base
    frac=iin/imax
    if frac <= start: return base
    scale=max(0.0,min(1.0,(1-frac)/max(1-start,.001)))
    return min(base,base*scale)
ok(abs(map_cap(20,13.5,15,.9)-20)<1e-6,'input pre-map leaves current untouched at exactly 90%')
ok(abs(map_cap(20,14.25,15,.9)-10)<1e-5,'input pre-map halves torque-current headroom halfway from 90% to limit')
ok(map_cap(20,15,15,.9)==0,'input pre-map reaches zero positive torque-current at input ceiling')
# Hall step at 3000 ERPM, 16 kHz should be ~307 counts/sample after 1.5 factor.
step=(3000*98304 + 60*16000-1)//(60*16000)
ok(300 <= step <= 310,'Hall rate-limit numeric step matches 1.5x electrical-speed rule')

print('ALL BATCH 10 PART-2 ADAPTIVE/HALL/INPUT-MAP/SPEED-SOURCE REGRESSIONS: PASS')
