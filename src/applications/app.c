#include "applications/app.h"
#include "confgenerator.h"
#include "applications/appconf_default.h"
#include "motor/mc_interface.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "cmsis_os2.h"
#include <string.h>

static app_configuration s_app_conf;
static bool s_loaded=false;
static volatile uint32_t s_disabled_until=0U;
static volatile bool s_disabled_indefinite=false;
static uint8_t s_set_wire[VESC6_APPCONF_WIRE_SIZE];
static uint8_t s_crc_wire[VESC6_APPCONF_WIRE_SIZE];

static bool app_conf_supported(const app_configuration *c){
    if(!c)return false;
    if(c->controller_id!=VESC_CONTROLLER_ID_LEFT)return false;
    if(c->timeout_msec>600000U)return false;
    if(c->app_to_use!=APP_NONE && c->app_to_use!=APP_ADC &&
       c->app_to_use!=APP_UART && c->app_to_use!=APP_ADC_UART)return false;
    if(c->app_uart_baudrate!=VESC_UART_BAUD)return false;
    /* USART3 is the permanent management/VESC Tool transport on this board. */
    if(!c->permanent_uart_enabled)return false;
    return true;
}

static void app_refresh(void){
    const uint8_t *w=vesc_config_app_wire(false);
    if(w && confgenerator_deserialize_appconf(w,&s_app_conf))s_loaded=true;
}

const app_configuration *app_get_configuration(void){
    if(!s_loaded)app_refresh();
    return &s_app_conf;
}

void app_notify_configuration_changed(void){
    s_loaded=false;
    app_command_configuration_changed();
}

void app_set_configuration(app_configuration *conf){
    if(!app_conf_supported(conf))return;
    if(confgenerator_serialize_appconf(s_set_wire,conf)!=(int32_t)VESC6_APPCONF_WIRE_SIZE)return;
    if(vesc_config_set_app_wire(s_set_wire,VESC6_APPCONF_WIRE_SIZE,false)){
        s_app_conf=*conf;s_loaded=true;
    }
}

void app_disable_output(int time_ms){
    if(time_ms<0){s_disabled_indefinite=true;s_disabled_until=0U;return;}
    s_disabled_indefinite=false;
    if(time_ms==0){s_disabled_until=0U;return;}
    s_disabled_until=osKernelGetTickCount()+(uint32_t)time_ms;
}

bool app_is_output_disabled(void){
    if(s_disabled_indefinite)return true;
    uint32_t until=s_disabled_until;
    if(until==0U)return false;
    uint32_t now=osKernelGetTickCount();
    if((int32_t)(until-now)>0)return true;
    s_disabled_until=0U;return false;
}

unsigned short app_calc_crc(app_configuration *conf){
    if(!conf)return 0U;
    if(confgenerator_serialize_appconf(s_crc_wire,conf)!=(int32_t)VESC6_APPCONF_WIRE_SIZE)return 0U;
    uint16_t crc=0U;
    for(uint32_t n=0;n<VESC6_APPCONF_WIRE_SIZE;n++){
        crc^=(uint16_t)s_crc_wire[n]<<8;
        for(unsigned b=0;b<8;b++)crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
    }
    return crc;
}
