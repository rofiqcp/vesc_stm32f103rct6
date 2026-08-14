#include "telemetry.h"
#include "motor_control.h"
#include "foc_control.h"
#include "app_config.h"
#include "foc_math.h"
#include <string.h>
#include <math.h>

static motor_telemetry_t s_telem[2];

void telemetry_init(void){memset(s_telem,0,sizeof(s_telem));}

static void update_stats(MotorRuntime *m,float dt_s){
    float ih=m->input_current*dt_s/3600.0f;
    float wh=m->vbus_filter*m->input_current*dt_s/3600.0f;
    if(ih>=0.0f)m->stats.amp_hours+=ih;else m->stats.amp_hours_charged+=-ih;
    if(wh>=0.0f)m->stats.watt_hours+=wh;else m->stats.watt_hours_charged+=-wh;
    int32_t tach=(int32_t)lroundf((m->position_deg/360.0f)*(6.0f*(float)m->pole_pairs));
    int32_t delta=tach-m->stats.tachometer_last;
    m->stats.tachometer=tach;
    m->stats.tachometer_abs += (delta < 0) ? -delta : delta;
    m->stats.tachometer_last=tach;
    float ac=fabsf(m->motor_current);if(ac>m->stats.max_current)m->stats.max_current=ac;
    float ai=fabsf(m->input_current);if(ai>m->stats.max_input_current)m->stats.max_input_current=ai;
    float ar=fabsf(m->erpm);if(ar>m->stats.max_erpm)m->stats.max_erpm=ar;
    m->stats.runtime_ms+=STAT_PERIOD_MS;
}

static void snapshot(MotorRuntime *m,motor_telemetry_t *t){
    t->id=m->id_meas;t->iq=m->iq_meas;t->id_filter=m->id_filter;t->iq_filter=m->iq_filter;
    t->vd=m->vd_filter;t->vq=m->vq_filter;t->current_motor=m->motor_current;t->current_in=m->input_current;
    t->duty=m->duty_now;t->erpm=m->erpm;t->mech_rpm=m->mech_rpm;t->vbus=m->vbus_filter;
    t->position_deg=foc_wrap_deg(m->position_deg);t->rotor_elec_deg=m->rotor_elec_deg;
    t->current_offset_u=(float)m->current_offset_u_counts;t->current_offset_v=(float)m->current_offset_v_counts;t->dc_current_offset=(float)m->dc_current_offset_counts;
    t->amp_hours=m->stats.amp_hours;t->amp_hours_charged=m->stats.amp_hours_charged;t->watt_hours=m->stats.watt_hours;t->watt_hours_charged=m->stats.watt_hours_charged;
    t->tachometer=m->stats.tachometer;t->tachometer_abs=m->stats.tachometer_abs;
    t->fault=(uint8_t)m->fault;t->controller_id=(m->id==MOTOR_LEFT)?VESC_CONTROLLER_ID_LEFT:VESC_CONTROLLER_ID_RIGHT;t->sensor_mode=m->sensor_mode;t->sensor_detect_state=(uint8_t)m->detect.state;
    t->calibration_done=foc_calibration_done()?1U:0U;t->calibration_valid=foc_calibration_valid()?1U:0U;t->timeout_active=m->timeout_active?1U:0U;
    t->isr_max_cycles=m->isr_max_cycles;t->isr_overruns=m->isr_overruns;
}

void telemetry_stats_update_100hz(void){
    update_stats(&g_motor_left,STAT_PERIOD_MS/1000.0f);
    update_stats(&g_motor_right,STAT_PERIOD_MS/1000.0f);
}

void telemetry_snapshot_100hz(void){
    snapshot(&g_motor_left,&s_telem[0]);
    snapshot(&g_motor_right,&s_telem[1]);
}

void telemetry_update_100hz(void){
    telemetry_stats_update_100hz();
    telemetry_snapshot_100hz();
}

void telemetry_get(motor_id_t id,motor_telemetry_t *out){if(out==NULL)return;*out=s_telem[(id==MOTOR_RIGHT)?1:0];}
