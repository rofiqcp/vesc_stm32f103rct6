#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "confgenerator.h"
#include "motor/mc_interface.h"

MotorRuntime g_motor_left, g_motor_right;
MotorRuntime *motor_get(motor_id_t id){return id==MOTOR_RIGHT?&g_motor_right:&g_motor_left;}
int mc_interface_get_motor_thread(void){return 1;}
void timeout_configure(uint32_t timeout_ms,float brake){(void)timeout_ms;(void)brake;}
void app_notify_configuration_changed(void){}
bool conf_general_store_app_wire_persistent(const uint8_t *wire){(void)wire;return true;}
bool conf_general_store_mc_wire_persistent(motor_id_t id,const uint8_t *wire){(void)id;(void)wire;return true;}
bool conf_general_store_all(void){return true;}
static int fail(const char*s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
static bool nearf(float a,float b,float e){return fabsf(a-b)<=e;}

int main(void){
    app_configuration a,b;
    uint8_t w[VESC6_APPCONF_WIRE_SIZE];
    confgenerator_set_defaults_appconf(&a);
    a.controller_id=1U;
    a.timeout_msec=750U;
    a.timeout_brake_current=0.0f;
    a.permanent_uart_enabled=true;
    a.app_to_use=APP_ADC_UART;
    a.app_uart_baudrate=115200U;
    a.app_adc_conf.ctrl_type=ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC;
    a.app_adc_conf.hyst=0.05f;
    a.app_adc_conf.voltage_start=0.90f;
    a.app_adc_conf.voltage_end=3.00f;
    a.app_adc_conf.voltage_min=0.10f;
    a.app_adc_conf.voltage_max=3.20f;
    a.app_adc_conf.voltage_center=1.65f;
    a.app_adc_conf.voltage2_start=0.80f;
    a.app_adc_conf.voltage2_end=2.90f;
    a.app_adc_conf.use_filter=true;
    a.app_adc_conf.safe_start=SAFE_START_REGULAR;
    a.app_adc_conf.buttons=0U;
    a.app_adc_conf.voltage_inverted=false;
    a.app_adc_conf.voltage2_inverted=true;
    a.app_adc_conf.throttle_exp=0.20f;
    a.app_adc_conf.throttle_exp_brake=-0.10f;
    a.app_adc_conf.throttle_exp_mode=THR_EXP_POLY;
    a.app_adc_conf.ramp_time_pos=0.40f;
    a.app_adc_conf.ramp_time_neg=0.20f;
    a.app_adc_conf.multi_esc=true;
    a.app_adc_conf.tc=false;
    a.app_adc_conf.tc_max_diff=0.0f;
    a.app_adc_conf.update_rate_hz=250U;

    if(confgenerator_serialize_appconf(w,&a)!=(int32_t)VESC6_APPCONF_WIRE_SIZE)return fail("serialize size");
    if(!confgenerator_deserialize_appconf(w,&b))return fail("deserialize");
    if(b.app_to_use!=APP_ADC_UART || b.app_adc_conf.ctrl_type!=ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC)return fail("selector/control");
    if(b.app_uart_baudrate!=115200U || b.app_adc_conf.update_rate_hz!=250U)return fail("baud/rate");
    if(!nearf(b.app_adc_conf.voltage_start,.9f,.002f)||!nearf(b.app_adc_conf.voltage_end,3.0f,.002f))return fail("voltage span");
    if(!nearf(b.app_adc_conf.voltage_min,.1f,.002f)||!nearf(b.app_adc_conf.voltage_max,3.2f,.002f))return fail("voltage range");
    if(!nearf(b.app_adc_conf.voltage2_start,.8f,.002f)||!nearf(b.app_adc_conf.voltage2_end,2.9f,.002f))return fail("brake span");
    if(!nearf(b.app_adc_conf.hyst,.05f,.0002f)||!nearf(b.app_adc_conf.throttle_exp,.2f,.0002f))return fail("hyst/curve");
    if(!nearf(b.app_adc_conf.ramp_time_pos,.4f,.0002f)||!nearf(b.app_adc_conf.ramp_time_neg,.2f,.0002f))return fail("ramps");
    if(!b.app_adc_conf.use_filter || b.app_adc_conf.safe_start!=SAFE_START_REGULAR || !b.app_adc_conf.voltage2_inverted || !b.app_adc_conf.multi_esc)return fail("flags");

    /* Application calibration/control writes are rejected while either bridge is live. */
    g_motor_left.pwm_enabled=true;
    if(vesc_config_set_app_wire(w,VESC6_APPCONF_WIRE_SIZE,false))return fail("live bridge accepted appconf");
    g_motor_left.pwm_enabled=false;
    if(!vesc_config_set_app_wire(w,VESC6_APPCONF_WIRE_SIZE,false))return fail("stopped bridge rejected appconf");

    /* Invalid app input settings must be rejected rather than ACKed as shadow state. */
    uint8_t bad[VESC6_APPCONF_WIRE_SIZE]; memcpy(bad,w,sizeof(bad));
    bad[VESC6_APP_OFF_ADC_CTRL_TYPE]=(uint8_t)ADC_CTRL_TYPE_DUTY_REV_CENTER;
    if(vesc_config_set_app_wire(bad,VESC6_APPCONF_WIRE_SIZE,false))return fail("unsupported ADC mode accepted");

    puts("ALL STAGE-1 APPCONF ROUNDTRIPS: PASS");
    return 0;
}
