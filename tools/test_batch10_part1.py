#!/usr/bin/env python3
from pathlib import Path
import subprocess, tempfile, sys, re
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

foc=read('src/motor/mcpwm_foc.c'); mc=read('src/motor/mc_interface.c')
fm=read('src/motor/foc_math.c'); dt=read('src/datatypes.h'); defs=read('src/motor/mcconf_default.h')
port=read('PORTING_NOTES.md')

ok('#define MCCONF_L_ADDITIONAL_FAULT_ENCODER_SLIP (1U << 0)' in defs,
   'additional-fault bit0 is encoder slip without shifting existing RPM bits')
ok('MOTOR_FAULT_ENCODER_SLIP' in dt and 'FAULT_CODE_ENCODER_SLIP' in dt,
   'dedicated local and canonical encoder-slip fault codes exist')
ok('ENCODER_SLIP_LIMIT_PHASE_U16 ((uint16_t)2731U)' in mc and
   'ENCODER_SLIP_TIME_TICKS      ((uint16_t)500U)' in mc,
   'encoder slip threshold is 15 electrical degrees for 500 ms')
ok('erpm_abs <= openloop_q16' in mc and '* 1.10f * 65536.0f' in mc and 'm->encoder.synced' in mc and 'm->observer_valid' in mc,
   'slip checking is gated on valid encoder/observer and above 110% open-loop ERPM')
ok('rt_snapshot_seq' in mc and 'phase_observer_u16' in mc and 'phase_encoder_u16' in mc,
   'encoder slip uses a coherent ISR snapshot rather than torn phase reads')
ok('observer_offset_factor_q15' in mc and 'FOC_ISR_EVENT_HZ' in mc,
   'slip comparison compensates observer PWM/sample delay')
ok('case MOTOR_FAULT_ENCODER_SLIP:     return FAULT_CODE_ENCODER_SLIP;' in mc,
   'encoder-slip maps to canonical VESC fault reporting')

ok('foc_apply_fast_fw_targets_isr' in foc and 'foc_apply_fast_fw_targets_isr(m, &id_target_eff, &iq_target_eff);' in foc,
   'field weakening is applied inside the hard current-control path before PI error')
ok('foc_run_fw(m, 0.001f)' not in mc,
   'old 1-kHz automatic field-weakening ramp is removed from motor service')
ok('abs_i32_sat_local(fw_id) > abs_i32_sat_local(id_base) ? fw_id : id_base' in foc,
   'MTPA and FW Id use VESC7 max-absolute composition instead of addition')
ok('id -= m->foc_fw_current_now' not in mc,
   'legacy additive field-weakening Id composition is absent')
ok('iq_cmd * iq_cmd - id_mtpa * id_mtpa' in mc,
   'MTPA rotates the requested current vector by reducing Iq magnitude')
ok('foc_fw_current_acc_q31' in foc and 'foc_fw_ramp_step_q31' in foc and 'volatile int32_t foc_fw_current_acc_q31' in dt,
   'fast FW uses an atomic 32-bit fractional accumulator for sub-Q15-LSB ramp steps')
ok('fw_override || step <= 0 || m->foc_fw_ramp_direct' in foc,
   'manual FW override follows upstream direct-set semantics instead of automatic ramp')
ok('foc_fw_duty_filter_q15' in foc and '* 328) >> 15' in foc,
   'fast FW thresholds use a fixed-point 0.01 duty low-pass like upstream')
ok('foc_fw_duty_norm_scale_q16' in foc and 'PWM_MAX_DUTY - PWM_MIN_DUTY' in fm,
   'FW duty threshold is normalized to the board usable 10..90% modulation window')
ok('m->iq_filter_q15 - m->iq_target_q15' in foc,
   'FW backoff compares measured Iq against the active effective Iq target')
ok('isqrt_u32' in foc and 'current_lim * current_lim' in foc,
   'fast FW re-applies the Id/Iq current circle with integer sqrt')
ok('fw_max_a < fmaxf(m->cc_min_current, 0.001f)' in fm,
   'FW is disabled when configured maximum is below useful minimum current')

# Hard helper must remain fixed-point: no float/math operations in helper body.
m=re.search(r'static inline void foc_apply_fast_fw_targets_isr\(.*?\n\}', foc, re.S)
ok(m is not None, 'fast FW helper body located')
body=m.group(0)
for bad in ['float ', 'double ', 'sqrtf(', 'fabsf(', 'lrintf(', 'fmaxf(', 'fminf(']:
    ok(bad not in body, f'fast FW helper contains no hard-path {bad.strip()} operation')

# Excluded ecosystems must not be introduced by this part.
for token in ['#include "comm_can.h"', '#include "imu/', '#include "bms', '#include "bm_if',
              '#include "nrf', '#include "ledpwm', '#include "comm_usb', '#include "lispif', '#include "lzo']:
    ok(token not in foc+mc+fm, f'Part-1 does not introduce excluded subsystem {token}')

# Strict host compile of changed portable units.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for unit,extra in [('src/motor/mc_interface.c',[]),('src/motor/mcpwm_foc.c',['-DDMA1_Channel1=DMA2_Channel5']),('src/motor/foc_math.c',[])]:
    cp=subprocess.run(base+extra+incs+['-fsyntax-only',unit],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True,f'{unit} passes strict host syntax/warning check')

with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'b10p1_numeric'
    cp=subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','tools/test_batch10_part1_control.c','-lm','-o',str(exe)],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    rp=subprocess.run([str(exe)],cwd=ROOT,text=True,capture_output=True)
    if rp.returncode:
        print(rp.stdout,rp.stderr); sys.exit(rp.returncode)
    ok('ALL BATCH 10 PART-1 NUMERIC TESTS: PASS' in rp.stdout,
       'numeric max-abs/current-circle/duty-normalization/ramp/phase-wrap tests pass')

print('ALL BATCH 10 PART-1 FAST-FW/ENCODER-SLIP REGRESSIONS: PASS')
