#include <stdio.h>
#include <math.h>
#include "confgenerator.h"
#include "motor/mc_interface.h"

MotorRuntime g_motor_left, g_motor_right;
MotorRuntime *motor_get(motor_id_t id){return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left;}
int mc_interface_get_motor_thread(void){return 1;}
static int fail(const char *s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
int main(void){
    mc_configuration a,b; uint8_t w[VESC6_MCCONF_WIRE_SIZE];
    confgenerator_set_defaults_mcconf(&a);
    a.s_pid_min_erpm=123.0f; a.s_pid_allow_braking=false; a.s_pid_ramp_erpms_s=23456.0f;
    a.p_pid_kd_proc=0.125f; a.p_pid_ang_div=2.5f; a.p_pid_gain_dec_angle=17.3f; a.p_pid_offset=-6.25f;
    a.cc_min_current=1.75f; a.s_pid_speed_source=S_PID_SPEED_SRC_PLL;
    if(confgenerator_serialize_mcconf(w,&a)!=(int32_t)VESC6_MCCONF_WIRE_SIZE)return fail("serialize VESC6 B7");
    if(!confgenerator_deserialize_mcconf(w,&b))return fail("deserialize VESC6 B7");
    if(fabsf(b.s_pid_min_erpm-123.0f)>0.2f || b.s_pid_allow_braking || fabsf(b.s_pid_ramp_erpms_s-23456.0f)>1.0f)return fail("speed fields roundtrip");
    if(fabsf(b.p_pid_kd_proc-0.125f)>0.002f || fabsf(b.p_pid_ang_div-2.5f)>0.01f || fabsf(b.p_pid_gain_dec_angle-17.3f)>0.11f || fabsf(b.p_pid_offset+6.25f)>0.02f)return fail("position fields roundtrip");
    if(fabsf(b.cc_min_current-1.75f)>0.02f)return fail("cc_min roundtrip");
    if(b.s_pid_speed_source!=S_PID_SPEED_SRC_PLL)return fail("VESC6 source defaults PLL");
    a.s_pid_speed_source=S_PID_SPEED_SRC_FAST;
    if(confgenerator_serialize_mcconf(w,&a)>=0)return fail("VESC6 must reject runtime FAST persistence");
    a.s_pid_speed_source=S_PID_SPEED_SRC_FASTER;
    if(confgenerator_serialize_mcconf(w,&a)>=0)return fail("VESC6 must reject runtime FASTER persistence");
    puts("ALL BATCH 7 CONFIG ROUNDTRIPS: PASS");
    return 0;
}
