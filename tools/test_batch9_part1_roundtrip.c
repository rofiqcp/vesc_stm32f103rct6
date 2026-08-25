#include <math.h>
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

    a.l_temp_fet_start=72.0f;
    a.l_temp_fet_end=92.0f;
    a.l_temp_motor_start=75.0f;
    a.l_temp_motor_end=105.0f;
    a.l_temp_accel_dec=0.25f;
    a.foc_start_curr_dec=0.35f;
    a.foc_start_curr_dec_rpm=1800.0f;
    a.l_additional_faults=0U;

    if(confgenerator_serialize_mcconf(w,&a)!=(int32_t)VESC6_MCCONF_WIRE_SIZE)return fail("serialize size");
    if(!confgenerator_deserialize_mcconf(w,&b))return fail("deserialize");
    if(fabsf(b.l_temp_fet_start-72.0f)>0.11f||fabsf(b.l_temp_fet_end-92.0f)>0.11f)return fail("board temp limits");
    if(fabsf(b.l_temp_motor_start-75.0f)>0.11f||fabsf(b.l_temp_motor_end-105.0f)>0.11f)return fail("motor temp limits");
    if(fabsf(b.l_temp_accel_dec-0.25f)>0.0002f)return fail("temp accel dec");
    if(fabsf(b.foc_start_curr_dec-0.35f)>0.0002f)return fail("start current fraction");
    if(fabsf(b.foc_start_curr_dec_rpm-1800.0f)>0.2f)return fail("start current rpm");
    if(b.l_additional_faults!=0U)return fail("VESC6 additional faults default");

    /* VESC6 has no l_additional_faults byte. Refuse fake persistence. */
    a.l_additional_faults=(1U<<1);
    if(confgenerator_serialize_mcconf(w,&a)>=0)return fail("newer fault state must not fake-persist into VESC6");

    puts("ALL BATCH 9 PART-1 CONFIG ROUNDTRIPS: PASS");
    return 0;
}
