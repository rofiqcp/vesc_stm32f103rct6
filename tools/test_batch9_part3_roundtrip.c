#include <stdio.h>
#include "confgenerator.h"
#include "motor/mc_interface.h"

MotorRuntime g_motor_left, g_motor_right;
MotorRuntime *motor_get(motor_id_t id){return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left;}
int mc_interface_get_motor_thread(void){return 1;}
static int fail(const char*s){fprintf(stderr,"FAIL: %s\n",s);return 1;}

int main(void){
    mc_configuration a,b;
    uint8_t w[VESC6_MCCONF_WIRE_SIZE];
    confgenerator_set_defaults_mcconf(&a);
    if(a.foc_short_ls_on_zero_duty)return fail("short-ls default must be false");
    if(confgenerator_serialize_mcconf(w,&a)!=(int32_t)VESC6_MCCONF_WIRE_SIZE)return fail("default serialize");
    if(!confgenerator_deserialize_mcconf(w,&b))return fail("deserialize");
    if(b.foc_short_ls_on_zero_duty)return fail("VESC6 decode must default short-ls false");
    a.foc_short_ls_on_zero_duty=true;
    if(confgenerator_serialize_mcconf(w,&a)>=0)return fail("VESC6 must reject fake short-ls persistence");
    puts("ALL BATCH 9 PART-3 CONFIG ROUNDTRIPS: PASS");
    return 0;
}
