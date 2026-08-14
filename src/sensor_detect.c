#include "sensor_detect.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_control.h"
#include "foc_math.h"
#include "app_config.h"
#include "vesc_config.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

static uint16_t deg_to_u16_int(uint32_t deg) { return (uint16_t)((deg * 65536UL) / 360UL); }

static void detect_clear_acc(MotorRuntime *m) {
    memset(m->detect.hall_sin_sum,0,sizeof(m->detect.hall_sin_sum));
    memset(m->detect.hall_cos_sum,0,sizeof(m->detect.hall_cos_sum));
    memset(m->detect.hall_samples,0,sizeof(m->detect.hall_samples));
    m->detect.hall_valid_states=0U;
}

bool sensor_detect_request_current(MotorRuntime *m, uint8_t requested_mode, float current_a) {
    if (m==NULL || requested_mode>SENSOR_MODE_ENCODER) return false;
    if (m->id==MOTOR_RIGHT && requested_mode==SENSOR_MODE_ENCODER) return false;
    motor_stop(m);
    m->detect.requested=true; m->detect.busy=false; m->detect.success=false;
    m->detect.result_mode=SENSOR_MODE_AUTO; m->detect.state=SENSOR_DETECT_PREPARE;
    if (current_a < 0.2f) current_a = 0.2f;
    if (current_a > FOC_MAX_CURRENT_A * 0.25f) current_a = FOC_MAX_CURRENT_A * 0.25f;
    m->detect.drive_current_a=current_a;
    m->sensor_request_mode=requested_mode;
    return true;
}

bool sensor_detect_request(MotorRuntime *m, uint8_t requested_mode) {
    return sensor_detect_request_current(m, requested_mode, SENSOR_DETECT_CURRENT_A);
}

static void detect_fail(MotorRuntime *m) {
    if (m->id==MOTOR_LEFT && m->sensor_mode==SENSOR_MODE_ENCODER) m->encoder.synced=false;
    motor_set_foc_targets(m,0.0f,0.0f); m->detect_force_angle=false; m->detect.busy=false;
    m->detect.success=false; m->detect.state=SENSOR_DETECT_FAILED; m->control_mode=MOTOR_CTRL_OFF;
    motor_hw_set_pwm_enabled(m,false);
    motor_raise_fault_from_task(m,MOTOR_FAULT_SENSOR_DETECT);
}

static void detect_done(MotorRuntime *m,uint8_t mode) {
    motor_set_foc_targets(m,0.0f,0.0f); m->detect_force_angle=false; m->detect.busy=false;
    m->detect.success=true; m->detect.result_mode=mode; m->detect.state=SENSOR_DETECT_DONE;
    m->control_mode=MOTOR_CTRL_OFF; m->sensor_mode=mode; m->sensor_request_mode=mode;
    vesc_config_sync_motor_runtime(m->id);
    motor_hw_set_pwm_enabled(m,false);
}

static void hall_sample(MotorRuntime *m) {
    uint8_t raw=motor_hw_read_hall_raw(m->id)&7U;
    if(raw==0U||raw==7U) return;
    int32_t sn,cs; foc_fast_sincos_u16_q15(m->detect_phase_u16,&sn,&cs);
    m->detect.hall_sin_sum[raw]+=sn; m->detect.hall_cos_sum[raw]+=cs; m->detect.hall_samples[raw]++;
}

static bool hall_evaluate(MotorRuntime *m) {
    typedef struct { uint8_t raw; uint16_t ang; } item_t;
    item_t items[6]; uint8_t n=0;
    for(uint8_t raw=0; raw<8U; raw++) {
        if(m->detect.hall_samples[raw] >= SENSOR_DETECT_HALL_MIN_SAMPLES) {
            float a=atan2f((float)m->detect.hall_sin_sum[raw],(float)m->detect.hall_cos_sum[raw]);
            if(a<0.0f)a+=2.0f*3.14159265358979323846f;
            uint16_t u=(uint16_t)(a*(65536.0f/(2.0f*3.14159265358979323846f)));
            if(n<6U){ items[n].raw=raw; items[n].ang=u; n++; }
        }
    }
    if(n!=6U) return false;
    for(uint8_t i=0;i<n;i++) for(uint8_t j=i+1;j<n;j++) if(items[j].ang<items[i].ang){item_t t=items[i];items[i]=items[j];items[j]=t;}
    for(uint8_t i=0;i<8U;i++){m->hall_table[i]=-1;m->foc_hall_table[i]=255U;m->hall_angle_u16[i]=0U;}
    for(uint8_t i=0;i<6U;i++) {
        uint8_t raw=items[i].raw; m->hall_table[raw]=(int8_t)i; m->hall_angle_u16[raw]=items[i].ang;
        m->foc_hall_table[raw]=(uint8_t)(((uint32_t)items[i].ang*200U)>>16);
    }
    m->hall.valid=false; m->hall.sector=-1; m->hall_offset_u16=0U;
    motor_hall_edge_isr(m);
    return true;
}

void sensor_detect_update_1khz(MotorRuntime *m,uint32_t now_ms) {
    if(m==NULL || (!m->detect.requested && !m->detect.busy)) return;
    sensor_detect_t *d=&m->detect;
    const uint32_t steps_per_rev=360U/SENSOR_DETECT_STEP_DEG;
    const uint32_t total_steps=steps_per_rev*SENSOR_DETECT_SWEEPS;

    switch(d->state) {
    case SENSOR_DETECT_PREPARE:
        /* Boot offset calibration must finish before any motor-moving detection.
           If it finished but failed, do not wait forever: abort detection. */
        if(!foc_calibration_done()) return;
        if(!foc_calibration_valid() || m->fault!=MOTOR_FAULT_NONE) { detect_fail(m); return; }
        d->requested=false; d->busy=true; d->success=false; d->sweep_index=0U; d->sweep_pass=0U;
        m->control_mode=MOTOR_CTRL_DETECT; m->command_active=false; m->detect_force_angle=true; m->detect_phase_u16=0U;
        detect_clear_acc(m);
        if(m->sensor_request_mode==SENSOR_MODE_ENCODER && m->id==MOTOR_LEFT) {
            motor_hw_configure_sensor(m,SENSOR_MODE_ENCODER); motor_hw_encoder_reset();
            d->state=SENSOR_DETECT_ENCODER_LOCK0;
        } else {
            motor_hw_configure_sensor(m,SENSOR_MODE_HALL); motor_hall_edge_isr(m); d->state=SENSOR_DETECT_HALL_LOCK;
        }
        motor_set_foc_targets(m,d->drive_current_a,0.0f); d->step_tick=now_ms;
        break;

    case SENSOR_DETECT_HALL_LOCK:
        m->detect_phase_u16=0U; motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if((uint32_t)(now_ms-d->step_tick)>=SENSOR_DETECT_LOCK_MS){d->step_tick=now_ms;d->sweep_index=0U;d->state=SENSOR_DETECT_HALL_FWD;}
        break;

    case SENSOR_DETECT_HALL_FWD:
        if((uint32_t)(now_ms-d->step_tick)>=SENSOR_DETECT_STEP_MS){
            d->step_tick=now_ms; m->detect_phase_u16=deg_to_u16_int((d->sweep_index*SENSOR_DETECT_STEP_DEG)%360U); hall_sample(m); d->sweep_index++;
            if(d->sweep_index>=total_steps){d->sweep_index=0U;d->state=SENSOR_DETECT_HALL_REV;}
        }
        break;

    case SENSOR_DETECT_HALL_REV:
        if((uint32_t)(now_ms-d->step_tick)>=SENSOR_DETECT_STEP_MS){
            d->step_tick=now_ms; uint32_t deg=360U-((d->sweep_index*SENSOR_DETECT_STEP_DEG)%360U); if(deg==360U)deg=0U;
            m->detect_phase_u16=deg_to_u16_int(deg); hall_sample(m); d->sweep_index++;
            if(d->sweep_index>=total_steps){d->state=SENSOR_DETECT_HALL_EVAL;}
        }
        break;

    case SENSOR_DETECT_HALL_EVAL:
        if(hall_evaluate(m)){detect_done(m,SENSOR_MODE_HALL);}
        else if(m->id==MOTOR_LEFT && m->sensor_request_mode==SENSOR_MODE_AUTO){
            motor_set_foc_targets(m,0.0f,0.0f); motor_hw_set_pwm_enabled(m,false); motor_hw_configure_sensor(m,SENSOR_MODE_ENCODER); motor_hw_encoder_reset();
            d->busy=true; m->detect_force_angle=true; m->detect_phase_u16=0U; motor_set_foc_targets(m,d->drive_current_a,0.0f);
            d->step_tick=now_ms; d->state=SENSOR_DETECT_ENCODER_LOCK0;
        } else detect_fail(m);
        break;

    case SENSOR_DETECT_ENCODER_LOCK0:
        m->detect_phase_u16=0U; motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if((uint32_t)(now_ms-d->step_tick)>=SENSOR_DETECT_LOCK_MS){
            motor_hw_encoder_reset(); d->encoder_start_count=0; d->sweep_index=0U; d->step_tick=now_ms; d->state=SENSOR_DETECT_ENCODER_SWEEP;
        }
        break;

    case SENSOR_DETECT_ENCODER_SWEEP:
        if((uint32_t)(now_ms-d->step_tick)>=SENSOR_DETECT_STEP_MS){
            d->step_tick=now_ms; uint32_t deg_total=d->sweep_index*SENSOR_DETECT_STEP_DEG;
            m->detect_phase_u16=deg_to_u16_int(deg_total%360U); d->sweep_index++;
            if(d->sweep_index>=total_steps){d->encoder_end_count=motor_encoder_extended_count(m);d->state=SENSOR_DETECT_ENCODER_EVAL;}
        }
        break;

    case SENSOR_DETECT_ENCODER_EVAL: {
        int32_t delta=d->encoder_end_count-d->encoder_start_count; int32_t ad=abs(delta);
        if(ad<SENSOR_DETECT_MIN_ENCODER_COUNTS){detect_fail(m);break;}
        float counts_per_elec=(float)ad/(float)SENSOR_DETECT_SWEEPS;
        uint32_t pp=(uint32_t)lroundf((float)m->encoder.cpr/counts_per_elec);
        if(pp<1U||pp>SENSOR_DETECT_MAX_POLE_PAIRS){detect_fail(m);break;}
        m->pole_pairs=(uint8_t)pp;
        m->encoder.electrical_ratio=(float)pp;
        m->encoder.electrical_ratio_q16=(uint32_t)pp << 16;
        m->encoder.inverted=(delta<0); m->encoder.motion_proved=true; m->encoder.synced=false;
        m->encoder.phase_per_count_q16=(uint32_t)(((uint64_t)m->encoder.electrical_ratio_q16<<16)/m->encoder.cpr);
        /* AB has no index. Return the rotor to a controlled electrical phase 0
           and capture the current extended count as this boot's phase origin. */
        m->detect_phase_u16=0U; motor_set_foc_targets(m,d->drive_current_a,0.0f);
        d->step_tick=now_ms; d->state=SENSOR_DETECT_ENCODER_RETURN0;
    } break;

    case SENSOR_DETECT_ENCODER_RETURN0:
        m->detect_phase_u16=0U; motor_set_foc_targets(m,d->drive_current_a,0.0f);
        if((uint32_t)(now_ms-d->step_tick)>=SENSOR_DETECT_SETTLE_MS){d->step_tick=now_ms;d->state=SENSOR_DETECT_ENCODER_ALIGN;}
        break;

    case SENSOR_DETECT_ENCODER_ALIGN: {
        int32_t zero=motor_encoder_extended_count(m);
        m->encoder.session_zero_count=zero; m->encoder.mechanical_zero_count=zero;
        m->encoder.elec_offset_u16=0U; m->encoder.synced=true;
        detect_done(m,SENSOR_MODE_ENCODER);
    } break;

    default: break;
    }
}
