#include <stdio.h>
#include <string.h>
#include <math.h>
#include "confgenerator.h"
#include "motor/mc_interface.h"
#include "util/buffer.h"

MotorRuntime g_motor_left, g_motor_right;
MotorRuntime *motor_get(motor_id_t id){return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left;}
int mc_interface_get_motor_thread(void){return 1;}

static int fail(const char *s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
int main(void){
    mc_configuration a,b; uint8_t mc[VESC6_MCCONF_WIRE_SIZE];
    confgenerator_set_defaults_mcconf(&a);
    a.l_current_max=12.5f; a.l_current_min=-11.5f; a.foc_pll_kp=2345.0f;
    a.foc_observer_gain_slow=0.07f; a.foc_fw_current_max=4.0f;
    /* Serializer must round-trip represented fields even when the hardware
       ownership gate will reject changing some of them at apply-time. */
    a.foc_sl_openloop_hyst=6.5f; a.s_pid_min_erpm=321.0f; a.s_pid_ramp_erpms_s=12345.0f;
    a.p_pid_kd_proc=0.125f; a.p_pid_ang_div=2.0f; a.p_pid_gain_dec_angle=12.3f; a.p_pid_offset=-4.5f;
    a.cc_startup_boost_duty=0.12f; a.cc_min_current=1.5f; a.cc_gain=0.75f; a.cc_ramp_step_max=0.02f;
    a.si_motor_nl_current=2.25f;
    if(confgenerator_serialize_mcconf(mc,&a)!=(int32_t)VESC6_MCCONF_WIRE_SIZE)return fail("mc serialize length");
    int32_t ix=0;if(buffer_get_uint32(mc,&ix)!=VESC6_MCCONF_SIGNATURE)return fail("mc signature");
    if(!confgenerator_deserialize_mcconf(mc,&b))return fail("mc deserialize");
    if(fabsf(b.l_current_max-12.5f)>0.02f||fabsf(b.l_current_min+11.5f)>0.02f)return fail("mc current roundtrip");
    if(fabsf(b.foc_pll_kp-2345.0f)>0.5f||fabsf(b.foc_observer_gain_slow-0.07f)>0.002f)return fail("mc observer/pll roundtrip");
    if(fabsf(b.foc_sl_openloop_hyst-6.5f)>0.02f||fabsf(b.s_pid_min_erpm-321.0f)>0.5f||fabsf(b.s_pid_ramp_erpms_s-12345.0f)>1.0f)return fail("mc speed/openloop typed roundtrip");
    if(fabsf(b.p_pid_kd_proc-0.125f)>0.002f||fabsf(b.p_pid_ang_div-2.0f)>0.002f||fabsf(b.p_pid_gain_dec_angle-12.3f)>0.11f||fabsf(b.p_pid_offset+4.5f)>0.02f)return fail("mc position typed roundtrip");
    if(fabsf(b.cc_startup_boost_duty-0.12f)>0.002f||fabsf(b.cc_min_current-1.5f)>0.02f||fabsf(b.cc_gain-0.75f)>0.02f||fabsf(b.cc_ramp_step_max-0.02f)>0.002f||fabsf(b.si_motor_nl_current-2.25f)>0.02f)return fail("mc cc/si typed roundtrip");

    app_configuration ap,ap2; uint8_t aw[VESC6_APPCONF_WIRE_SIZE];
    confgenerator_set_defaults_appconf(&ap); ap.controller_id=42U; ap.timeout_msec=1234U; ap.timeout_brake_current=2.5f; ap.app_to_use=APP_UART; ap.app_uart_baudrate=115200U;
    if(confgenerator_serialize_appconf(aw,&ap)!=(int32_t)VESC6_APPCONF_WIRE_SIZE)return fail("app serialize length");
    ix=0;if(buffer_get_uint32(aw,&ix)!=VESC6_APPCONF_SIGNATURE)return fail("app signature");
    if(!confgenerator_deserialize_appconf(aw,&ap2))return fail("app deserialize");
    if(ap2.controller_id!=42U||ap2.timeout_msec!=1234U||fabsf(ap2.timeout_brake_current-2.5f)>0.02f||ap2.app_to_use!=APP_UART||ap2.app_uart_baudrate!=115200U)return fail("app roundtrip");

    uint8_t x[32]={0};ix=0;buffer_append_int64(x,-1234567890123LL,&ix);buffer_append_uint64(x,0xFEDCBA9876543210ULL,&ix);buffer_append_float64_auto(x,12345.678901234,&ix);
    int32_t j=0;if(buffer_get_int64(x,&j)!=-1234567890123LL)return fail("int64 buffer");if(buffer_get_uint64(x,&j)!=0xFEDCBA9876543210ULL)return fail("uint64 buffer");double d=buffer_get_float64_auto(x,&j);if(fabs(d-12345.678901234)>1e-3)return fail("float64_auto buffer");
    puts("ALL BATCH 5 TYPED CONFIG/BUFFER ROUNDTRIPS: PASS"); return 0;
}
