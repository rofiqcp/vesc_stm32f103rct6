#include <stdio.h>
#include <math.h>
#include "confgenerator.h"
#include "motor/mc_interface.h"
MotorRuntime g_motor_left, g_motor_right;
MotorRuntime *motor_get(motor_id_t id){return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left;}
int mc_interface_get_motor_thread(void){return 1;}
static int fail(const char*s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
int main(void){
    mc_configuration a,b; uint8_t w[VESC6_MCCONF_WIRE_SIZE];
    confgenerator_set_defaults_mcconf(&a);
    a.l_slow_abs_current=true;
    a.l_min_vin=22.5f; a.l_max_vin=41.5f;
    a.foc_motor_l=300e-6f; a.foc_motor_ld_lq_diff=90e-6f;
    if(confgenerator_serialize_mcconf(w,&a)!=(int32_t)VESC6_MCCONF_WIRE_SIZE)return fail("serialize size");
    if(w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT]!=1U)return fail("slow abs byte");
    if(!confgenerator_deserialize_mcconf(w,&b))return fail("deserialize");
    if(!b.l_slow_abs_current)return fail("slow abs roundtrip");
    if(fabsf(b.l_min_vin-22.5f)>0.02f||fabsf(b.l_max_vin-41.5f)>0.02f)return fail("vin roundtrip");
    if(fabsf(b.foc_motor_l-300e-6f)>2e-7f||fabsf(b.foc_motor_ld_lq_diff-90e-6f)>2e-7f)return fail("L/Ldq roundtrip");
    puts("ALL BATCH 8 CONFIG ROUNDTRIPS: PASS");
    return 0;
}
