#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,sys,math
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

app=read('src/applications/appconf_default.h')
dt=read('src/datatypes.h')
cg=read('src/confgenerator.c'); cgh=read('src/confgenerator.h')
mc=read('src/motor/mc_interface.c'); foc=read('src/motor/mcpwm_foc.c')

# VESC6 protection field and runtime ownership.
ok('bool l_slow_abs_current;' in dt, 'VESC6 l_slow_abs_current is represented in typed MCCONF')
ok(re.search(r'#define\s+VESC6_MC_OFF_L_SLOW_ABS_CURRENT\s+62U',cgh) is not None, 'VESC6 slow-absolute-current byte stays at canonical offset 62')
ok('w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT]=c->l_slow_abs_current?1U:0U' in cg and
   'c->l_slow_abs_current=w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT]!=0U' in cg,
   'typed MCCONF round-trips l_slow_abs_current without changing wire size')
ok('byte_in_range(i, VESC6_MC_OFF_L_MIN_VIN, 17U)' in cg,
   'SET_MCCONF ownership includes the real VESC6 slow-absolute-current byte')

# Protection layers.
for sym in ['FOC_ABS_CURRENT_FILTER_ALPHA_Q15','FOC_ABS_CURRENT_FAULT_DEBOUNCE_SAMPLES',
            'FOC_VBUS_FAULT_DEBOUNCE_SAMPLES','FOC_VBUS_HARD_OV_MARGIN_V',
            'FOC_VBUS_HARD_UV_MARGIN_V','FOC_VBUS_HARD_MAX_V','FOC_VBUS_HARD_MIN_V']:
    ok(sym in app, f'{sym} protection constant exists')
ok('abs_phase > s_current_trip_q15' in foc,
   'raw board absolute-current ceiling remains an immediate first-sample trip')
ok('m->slow_abs_current ?' in foc and 'm->abs_phase_current_filter_q15 > trip' in foc and
   'FOC_ABS_CURRENT_FAULT_DEBOUNCE_SAMPLES' in foc,
   'slow absolute-current mode filters/debounces only the configurable threshold')
ok('vbus_q15 > hard_max_q15' in foc and 'vbus_q15 < hard_min_q15' in foc,
   'absolute hardware voltage envelope remains immediate')
isr_start=foc.index('static void foc_one_motor_isr')
isr_end=foc.index('static inline uint16_t snapshot_encoder_phase_u16',isr_start) if 'static inline uint16_t snapshot_encoder_phase_u16' in foc[isr_start:] else foc.index('static void detect_clear_acc',isr_start)
isr=foc[isr_start:isr_end]
ok(isr.index('vbus_q15 > hard_max_q15') < isr.index('pwm_enable_blank_cycles > 0U'),
   'hard over-voltage check executes before startup current blanking')
ok(isr.index('vbus_q15 < hard_min_q15') < isr.index('pwm_enable_blank_cycles > 0U'),
   'hard under-voltage check executes before startup current blanking')
ok('abs_current_peak_q15' in dt and 'abs_current_peak=' in read('src/telemetry.c'),
   'non-safety peak-current diagnostics are retained')
ok('m->over_voltage_fault_count >= FOC_VBUS_FAULT_DEBOUNCE_SAMPLES' in foc and
   'm->under_voltage_fault_count >= FOC_VBUS_FAULT_DEBOUNCE_SAMPLES' in foc,
   'configured VIN faults require consecutive PWM-rate samples')
ok('fminf(m->max_vin+FOC_VBUS_HARD_OV_MARGIN_V,FOC_VBUS_HARD_MAX_V)' in cg and
   'fmaxf(m->min_vin-FOC_VBUS_HARD_UV_MARGIN_V,FOC_VBUS_HARD_MIN_V)' in cg,
   'hard VIN thresholds are derived task-side from configured limits plus board envelope')

# Ld/Lq capture and detection semantics.
ok('volatile uint8_t l_capture_axis;' in dt and 'l_capture_i_q15' in dt and 'l_capture_v_prev_q15' in dt,
   'inductance capture supports both d and q axes')
ok('l_capture_id_q15' not in foc and 'l_capture_vd_prev_q15' not in foc,
   'obsolete d-only capture arrays are removed')
cap=foc[foc.index('static inline void detect_inductance_capture_isr'):foc.index('static inline uint16_t snapshot_encoder_phase_u16')]
ok('d->l_capture_axis == 0U' in cap and 'm->vd_q15' in cap and 'm->vq_q15' in cap,
   'hard capture selects Id/Vd or Iq/Vq without task-side ambiguity')
# hard capture helper must remain no-float
cap_clean=re.sub(r'/\*.*?\*/','',cap,flags=re.S)
ok(not any(x in cap_clean for x in ['float ','double ','sqrtf','atan2f','sinf','cosf']),
   'd/q capture helper remains fixed-point/no-float')

start=foc.index('int mcpwm_foc_measure_inductance_current_motor(')
end=foc.index('static int measure_res_ind_ex(',start)
ind=foc[start:end]
ok('for (uint8_t axis = 0U; axis < 2U' in ind and 'm->detect.l_capture_axis = axis' in ind,
   'blocking inductance detect explicitly measures both d and q axes')
ok('q_sign = (pulse & 1U) ? -1.0f : 1.0f' in ind,
   'Lq detection alternates torque-pulse polarity to cancel average torque')
ok('motor_set_foc_targets(m, lock_current, q_sign * goal)' in ind,
   'Lq is measured with rotor-lock Id retained')
ok('speed_est_fast_erpm_q16' in ind and '(250 * 65536)' in ind,
   'Lq probing aborts if the supposedly locked rotor accelerates materially')
ok('l_axis[axis] = 0.90f *' in ind,
   'measured axis inductance retains VESC-style conservative 0.90 scale')
ok('0.5f * (ld + lq)' in ind and '(lq - ld)' in ind,
   'runtime convention stores average L and Lq-Ld saliency separately')
ok('m->detect_ld_lq_diff_h = ldq_out' in ind,
   'detected Ld-Lq result is retained for detect-all')
ok('m->foc_motor_l = l;' in foc and 'm->foc_motor_ld_lq_diff = ldq;' in foc,
   'detect-all applies measured L and Lq-Ld to active motor parameters')
ind_code=re.sub(r'/\*.*?\*/','',ind,flags=re.S)
ok('HFI' not in ind_code and 'foc_f_zv' not in ind_code,
   'B8 saliency detection does not introduce operational HFI or retime the PWM/ADC loop')

# Numerical sanity: reconstruct midpoint-discretized RL steps then apply the
# same estimator algebra used by mc_math_estimate_inductance_q15.
def synth(L,R,V,n=24,fs=16000.0):
    dt=1/fs; i=[0.0]
    a=R*dt/(2*L); b=V*dt/L
    for _ in range(1,n): i.append(((1-a)*i[-1]+b)/(1+a))
    return i,[V]*n

def estimate(i,v,R,fs=16000.0):
    vals=[]
    for k in range(1,len(i)):
        didt=(i[k]-i[k-1])*fs
        if abs(didt)<20: continue
        im=.5*(i[k]+i[k-1])
        L=(v[k]-R*im)/didt
        if 0.2e-6 <= L <= .02: vals.append(L)
    return sum(vals)/len(vals)
Ld=240e-6; Lq=360e-6; R=.18
ed=estimate(*synth(Ld,R,3.0),R); eq=estimate(*synth(Lq,R,3.0),R)
ok(abs(ed-Ld)/Ld < 1e-9 and abs(eq-Lq)/Lq < 1e-9,
   'synthetic d/q RL steps recover separate Ld and Lq with estimator algebra')
avg=.5*(.9*ed+.9*eq); diff=.9*eq-.9*ed
ok(abs(avg-270e-6)<1e-12 and abs(diff-108e-6)<1e-12,
   'B8 average-L and Lq-Ld convention is numerically consistent after 0.90 scaling')

# Hardware/communication/sensor infrastructure is outside B8 scope.
base=Path('/mnt/data/vesc_b7_base')
if base.exists():
    for rel in ['src/hwconf/hw.c','src/timeout.c','src/applications/app_uartcomm.c',
                'src/comm/commands.c','src/comm/packet.c','src/comm/packet.h',
                'src/terminal.c','src/motor/foc_math.c']:
        ok(hashlib.sha256((ROOT/rel).read_bytes()).digest()==hashlib.sha256((base/rel).read_bytes()).digest(),
           f'B8 leaves {rel} byte-identical to B7')
print('ALL BATCH 8 MOTOR-DETECTION/PROTECTION REGRESSIONS: PASS')
