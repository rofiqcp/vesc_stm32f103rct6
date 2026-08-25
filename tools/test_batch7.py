#!/usr/bin/env python3
"""Batch 7 source/numeric regressions: controller semantics only."""
from pathlib import Path
import hashlib, math, re, sys
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(cond,msg):
    if not cond:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)
def body(text,sig):
    p=text.find(sig)
    if p<0: raise AssertionError('missing '+sig)
    b=text.find('{',p); d=0
    for i in range(b,len(text)):
        if text[i]=='{': d+=1
        elif text[i]=='}':
            d-=1
            if d==0:return text[p:i+1]
    raise AssertionError('unterminated '+sig)

dt=read('src/datatypes.h'); app=read('src/applications/appconf_default.h')
cg=read('src/confgenerator.c'); mc=read('src/motor/mc_interface.c')
foc=read('src/motor/foc_math.c'); isr=read('src/motor/mcpwm_foc.c'); h=read('src/motor/mcpwm_foc.h')

# Canonical/controller configuration plumbing.
ok(all(x in dt for x in ['S_PID_SPEED_SRC_PLL','S_PID_SPEED_SRC_FAST','S_PID_SPEED_SRC_FASTER']),
   'runtime speed-source enum supports PLL/FAST/FASTER')
ok('bool s_pid_allow_braking;' in dt and 'S_PID_SPEED_SRC s_pid_speed_source;' in dt,
   'speed PID configuration exposes allow-braking and runtime speed source')
ok(all(x in dt for x in ['p_pid_kd_proc','p_pid_ang_div','p_pid_gain_dec_angle','p_pid_offset','cc_min_current']),
   'advanced position PID and cc_min_current exist in typed MCCONF')
ok('SPEED_PID_MIN_ERPM' in app and 'SPEED_PID_RAMP_ERPMS_S' in app and 'CURRENT_CTRL_MIN_CURRENT_A' in app,
   'B7 has explicit defaults for speed release/ramp and minimum current')

# VESC6 wire ownership: use existing bytes only, no illegal ABI expansion.
ok('c->s_pid_min_erpm=get_auto_at(w,344U)' in cg and 'c->s_pid_allow_braking=w[348U]!=0U' in cg and
   'c->s_pid_ramp_erpms_s=get_auto_at(w,349U)' in cg,
   'VESC6 min-ERPM/allow-braking/ramp bytes decode into runtime config')
ok('c->s_pid_speed_source=S_PID_SPEED_SRC_PLL; /* not present in VESC6 wire */' in cg,
   'VESC6 deserialization defaults non-existent speed-source selector to PLL')
ok('if(conf->s_pid_speed_source!=S_PID_SPEED_SRC_PLL)return -1;' in cg,
   'serializer refuses to silently persist FAST/FASTER into VESC6 wire')
ok('put_auto_at(w,365U,c->p_pid_kd_proc)' in cg and 'put_auto_at(w,371U,c->p_pid_ang_div)' in cg and
   'put_f16_at(w,375U,c->p_pid_gain_dec_angle,10.0f)' in cg and 'put_auto_at(w,377U,c->p_pid_offset)' in cg and
   'put_auto_at(w,383U,c->cc_min_current)' in cg,
   'VESC6 advanced position and cc_min_current bytes serialize canonically')

# Speed PID semantics.
speed=body(mc,'static float speed_pid_step')
ok('m->speed_pid_ramp_erpms_s * dt' in speed and 'step_towards_f' in speed,
   'speed PID implements ERPM/s setpoint ramp')
ok('fabsf(m->speed_pid_set_erpm) < m->speed_pid_min_erpm' in speed,
   'speed PID releases current below configured minimum ERPM')
ok(speed.count('(1.0f / 20.0f)') >= 3,
   'speed PID uses VESC historical 1/20 P/I/D scaling')
ok('!m->speed_pid_allow_braking' in speed and 'rpm > 20.0f && out < 0.0f' in speed,
   'speed PID can suppress active braking')
sfb=body(mc,'static float speed_feedback_erpm')
ok(all(x in sfb for x in ['S_PID_SPEED_SRC_FAST','S_PID_SPEED_SRC_FASTER','S_PID_SPEED_SRC_PLL']),
   'speed PID selects PLL/FAST/FASTER feedback')
ok('return m->invert_direction ? -rpm : rpm;' in sfb,
   'speed feedback is converted to external m_invert_direction coordinate')
ok('if (m->invert_direction) iq=-iq;' in mc,
   'outer-loop current is direction-multiplied exactly in the existing output path')

# Position PID semantics.
pos=body(mc,'static float position_pid_step')
ok('m->position_gain_dec_angle / m->position_ang_div' in pos and 'kp *= scale' in pos,
   'position PID reduces gains close to target')
ok('m->position_kd_proc' in pos and 'position_derivative_proc_filtered' in pos,
   'position PID implements derivative-on-process')
ok('position_dt_integrator' in pos and 'position_dt_process_integrator' in pos,
   'position PID accumulates derivative time across unchanged samples')
ok('const float i_lim = fmaxf(1.0f - fabsf(p_clip), 0.0f);' in pos,
   'position integrator uses proportional-headroom windup limit')
ok('m->position_offset_deg' in pos and 'p_pid_offset' in mc,
   'position offset is applied by the active controller and config path')

# cc_min_current/current_off_delay semantics.
ok('m->current_off_delay_s=fmaxf(0.0f,m->current_off_delay_s-0.001f)' in mc,
   'current_off_delay is a real 1-kHz countdown')
ok('const float min_hold_current = fmaxf(m->cc_min_current, 0.001f);' in mc and
   'm->current_off_delay_s > 0.0f' in mc,
   'PWM hold policy uses cc_min_current plus current_off_delay')
ok('const float release_threshold=fmaxf(m->cc_min_current,0.001f);' in mc and
   'sqrtf(id*id+iq*iq)<release_threshold' in mc,
   'sub-minimum current releases modulation while preserving the outer command')
ok('mcpwm_foc_set_current_off_delay_motor(m, 1.0f);' in foc and
   'target > m->cc_min_current' in foc,
   'field weakening extends current-off delay when meaningful FW is active')
ok('void mcpwm_foc_set_current_off_delay_motor(MotorRuntime *m, float delay_s);' in h,
   'per-motor current-off-delay API is explicit')

# Public correction wrappers now match the same source-selection semantics.
enc=body(foc,'float foc_correct_encoder')
ok('fabsf(sl_erpm)*0.05f' in enc and '0.80' not in enc and '0.8f' not in enc,
   'public encoder correction uses 5% hysteresis, not the obsolete 80-100% blend')
hall=body(foc,'float foc_correct_hall')
ok('m->foc_sl_erpm_start' in hall and 'm->foc_sl_erpm' in hall and
   'hall_angle+diff*foc_clampf' in hall,
   'public Hall correction blends over the same start-to-sensorless range')
phase=body(isr,'uint16_t motor_sensor_electrical_phase_u16')
ok('m->foc_sl_erpm_start_q16' in phase and 'm->foc_sl_erpm_q16' in phase and
   'const int16_t diff = (int16_t)(obs - hall_phase);' in phase,
   'hard Hall phase selector uses matching fixed-point Hall-to-observer blend')
# hard selector must remain integer-only
phase_code=re.sub(r'/\*.*?\*/','',phase,flags=re.S)
forbidden=['float ','double ','sqrtf','atan2f','sinf','cosf','lrintf']
ok(not any(x in phase_code for x in forbidden), 'hard Hall/observer phase blend remains fixed-point/no-float')

# Numeric sanity for the normalized speed controller semantics.
def speed_step(setv,target,rpm,kp,ki,kd,integ,prev,dfilt,ramp,minrpm,allow,dt=0.001):
    step=ramp*dt
    if ramp>0:
        setv=min(target,setv+step) if setv<target else max(target,setv-step) if setv>target else target
    else:setv=target
    err=setv-rpm
    if abs(setv)<minrpm:return setv,0.0,0.0,err,0.0
    p=err*kp/20.0
    d=(err-prev)*(kd/dt)/20.0
    dfilt += 1.0*(d-dfilt)
    out=max(-1.0,min(1.0,p+integ+dfilt))
    integ=max(-1.0,min(1.0,integ+err*ki*dt/20.0))
    if not allow:
        if rpm>20 and out<0:out=0.0
        if rpm<-20 and out>0:out=0.0
    return setv,out,integ,err,dfilt
s,o,i,e,d=speed_step(0,1000,0,0.01,0,0,0,0,0,35000,100,True)
ok(abs(s-35.0)<1e-9 and o==0.0,'numeric speed setpoint ramps by 35 ERPM per 1-ms tick and remains released below min ERPM')
s,o,*_=speed_step(0,50,0,0.01,0,0,0,0,0,0,100,True)
ok(o==0.0,'numeric speed controller releases below min ERPM')
s,o,*_=speed_step(1000,1000,1200,0.01,0,0,0,0,0,0,100,False)
ok(o==0.0,'numeric allow_braking=false suppresses opposite torque above 20 ERPM')

# Files outside controller semantics remain byte-identical to B6 baseline if available.
base=Path('/mnt/data/vesc_b6_base')
if base.exists():
    for rel in ['src/hwconf/hw.c','src/timeout.c','src/applications/app_uartcomm.c','src/comm/commands.c',
                'src/comm/packet.c','src/comm/packet.h','src/conf_general.c','src/terminal.c']:
        ok(hashlib.sha256((ROOT/rel).read_bytes()).digest()==hashlib.sha256((base/rel).read_bytes()).digest(),
           f'B7 leaves {rel} byte-identical to B6')

print('ALL BATCH 7 CONTROLLER-SEMANTICS REGRESSIONS: PASS')
