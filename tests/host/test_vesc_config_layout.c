#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vesc_config.h"
#include "vesc_buffer.h"
#include "motor_control.h"

MotorRuntime g_motor_left;
MotorRuntime g_motor_right;

MotorRuntime *motor_get(motor_id_t id) { return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left; }
void motor_stop(MotorRuntime *m){if(m){m->pwm_enabled=false;m->command_active=false;}}
void motor_hall_edge_isr(MotorRuntime *m){(void)m;}
void motor_hw_configure_sensor(MotorRuntime *m,uint8_t mode){m->sensor_mode=mode;}
void vesc_timeout_configure(uint32_t timeout_ms,float brake){(void)timeout_ms;(void)brake;}
void app_adc_port_set_enabled(bool en){(void)en;}
bool config_store_save_all(void){return true;}

static void put_auto(uint8_t *b,int off,float v){int32_t i=off;vesc_buf_append_float32_auto(b,v,&i);}
static void put_f16(uint8_t *b,int off,float v,float s){int32_t i=off;vesc_buf_append_float16(b,v,s,&i);}
static void put_u32(uint8_t *b,int off,uint32_t v){int32_t i=off;vesc_buf_append_u32(b,v,&i);}
static int closef(float a,float b){return fabsf(a-b)<0.0015f;}

static void init_m(MotorRuntime *m,motor_id_t id){
    memset(m,0,sizeof(*m)); m->id=id; m->pole_pairs=15; m->current_max_a=15.0f; m->current_min_a=-15.0f;
    m->abs_current_max_a=22.0f; m->min_erpm=-30000.0f; m->max_erpm=30000.0f; m->min_duty=-0.95f; m->max_duty=0.95f;
    m->current_kp=0.02f; m->current_ki=20.0f; m->speed_pid.kp=0.001f; m->speed_pid.ki=0.01f;
    m->position_pid.kp=0.1f; m->position_pid.ki=0.01f; m->encoder.cpr=4000; m->encoder.electrical_ratio=15.0f; m->encoder.electrical_ratio_q16=15U<<16;
    m->sensor_mode=1U; for(int k=0;k<8;k++)m->foc_hall_table[k]=(k==0||k==7)?255U:(uint8_t)(k*20);
}
int main(void){
    init_m(&g_motor_left,MOTOR_LEFT); init_m(&g_motor_right,MOTOR_RIGHT);
    vesc_config_init_defaults();
    if(!vesc_config_layout_ok()){fprintf(stderr,"layout mismatch\n");return 1;}

    uint8_t mc[VESC6_MCCONF_WIRE_SIZE]; memcpy(mc,vesc_config_mc_wire(MOTOR_LEFT,false),sizeof(mc));
    put_auto(mc,8,12.5f); put_auto(mc,12,-11.0f); put_auto(mc,24,18.0f); put_auto(mc,28,-25000.0f); put_auto(mc,32,25000.0f);
    put_f16(mc,75,0.72f,10000.0f); put_auto(mc,127,0.025f); put_auto(mc,131,37.0f);
    mc[143]=1U; put_auto(mc,144,12.0f); put_auto(mc,148,17.25f); mc[152]=1U;
    put_auto(mc,330,0.002f); put_auto(mc,334,0.015f); put_auto(mc,353,0.22f); put_auto(mc,357,0.03f);
    put_u32(mc,403,4096U); mc[419]=1U; mc[420]=1U; mc[451]=30U;
    mc[300]^=0xA5U; /* runtime-unsupported MCCONF byte must survive canonicalization */
    if(!vesc_config_set_mc_wire(MOTOR_LEFT,mc,sizeof(mc),false)){fprintf(stderr,"set mc failed\n");return 2;}
    const uint8_t *mc_active=vesc_config_mc_wire(MOTOR_LEFT,false);
    if(mc_active[300]!=mc[300]){fprintf(stderr,"mc unsupported byte lost\n");return 3;}
    if(g_motor_left.pole_pairs!=15U || !closef(g_motor_left.encoder.electrical_ratio,17.25f) || g_motor_left.encoder.cpr!=4096U || g_motor_left.sensor_mode!=2U){fprintf(stderr,"encoder mapping wrong pp=%u ratio=%.4f cpr=%lu mode=%u\n",g_motor_left.pole_pairs,(double)g_motor_left.encoder.electrical_ratio,(unsigned long)g_motor_left.encoder.cpr,g_motor_left.sensor_mode);return 4;}
    if(!closef(g_motor_left.max_duty,0.72f)||!closef(g_motor_left.current_max_a,12.5f)||!closef(g_motor_left.current_kp,0.025f)){fprintf(stderr,"mc applied values wrong\n");return 5;}

    uint8_t app[VESC6_APPCONF_WIRE_SIZE]; memcpy(app,vesc_config_app_wire(false),sizeof(app));
    put_u32(app,5,1500U); app[33]=3U; app[100]^=0x5AU; /* unsupported byte must still persist/round-trip */
    if(!vesc_config_set_app_wire(app,sizeof(app),false)){fprintf(stderr,"set app failed\n");return 6;}
    if(memcmp(app,vesc_config_app_wire(false),sizeof(app))!=0){fprintf(stderr,"app raw roundtrip changed\n");return 7;}

    printf("test_vesc_config_layout: PASS (MCCONF=%u APPCONF=%u unsupported-byte preservation + runtime canonicalization + encoder-ratio separation)\n",VESC6_MCCONF_WIRE_SIZE,VESC6_APPCONF_WIRE_SIZE);
    return 0;
}
