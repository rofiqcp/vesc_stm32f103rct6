#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vesc_config.h"
#include "vesc_buffer.h"
#include "motor_control.h"
#include "app_config.h"

/* Intentionally left zeroed: this models VESC Tool sending GET_MCCONF right
 * after FW_VERSION while motor_boot_thread is still below-normal priority. */
MotorRuntime g_motor_left;
MotorRuntime g_motor_right;

MotorRuntime *motor_get(motor_id_t id) { return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left; }
void motor_stop(MotorRuntime *m){(void)m;}
void motor_hall_edge_isr(MotorRuntime *m){(void)m;}
void motor_hw_configure_sensor(MotorRuntime *m,uint8_t mode){if(m)m->sensor_mode=mode;}
void vesc_timeout_configure(uint32_t timeout_ms,float brake){(void)timeout_ms;(void)brake;}
void app_adc_port_set_enabled(bool en){(void)en;}
bool config_store_save_all(void){return true;}

static float auto_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_float32_auto(b,&i);}
static uint32_t u32_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_u32(b,&i);}

int main(void) {
    memset(&g_motor_left,0,sizeof(g_motor_left));
    memset(&g_motor_right,0,sizeof(g_motor_right));
    vesc_config_init_defaults();
    if(!vesc_config_layout_ok()){fprintf(stderr,"layout invalid\n");return 1;}
    const uint8_t *l=vesc_config_mc_wire(MOTOR_LEFT,false);
    const uint8_t *r=vesc_config_mc_wire(MOTOR_RIGHT,false);
    if(u32_at(l,0)!=VESC6_MCCONF_SIGNATURE || u32_at(r,0)!=VESC6_MCCONF_SIGNATURE){fprintf(stderr,"signature\n");return 2;}
    if(!(auto_at(l,8)>0.1f) || !(auto_at(r,8)>0.1f)){fprintf(stderr,"zero current defaults\n");return 3;}
    if(l[451]!=(uint8_t)(LEFT_POLE_PAIRS*2U) || r[451]!=(uint8_t)(RIGHT_POLE_PAIRS*2U)){fprintf(stderr,"zero pole defaults L=%u R=%u\n",l[451],r[451]);return 4;}
    if(l[223]!=255U || l[230]!=255U){fprintf(stderr,"hall endpoints not invalid\n");return 5;}
    for(int k=1;k<7;k++){if(l[223+k]>200U){fprintf(stderr,"hall entry invalid\n");return 6;}}
    if(u32_at(l,403)!=LEFT_ENCODER_CPR){fprintf(stderr,"encoder CPR fallback wrong\n");return 7;}
    if(u32_at(r,403)!=0U){fprintf(stderr,"right encoder must be absent\n");return 8;}
    printf("test_vesc_config_early_get: PASS (safe exact-size MCCONF before motor runtime init)\n");
    return 0;
}
