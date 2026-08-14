#include "vesc_config.h"
#include "vesc_buffer.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_control.h"
#include "vesc_timeout.h"
#include "app_adc_port.h"
#include "config_store.h"
#include "app_config.h"
#include <string.h>
#include <math.h>

#define VESC_PWM_SYNCHRONOUS 1U
#define VESC_COMM_INTEGRATE  0U
#define VESC_MOTOR_FOC       2U
#define VESC_SENSOR_SENSORED 1U
#define VESC_FOC_SENSOR_ENCODER 1U
#define VESC_FOC_SENSOR_HALL    2U
#define VESC_SENSOR_PORT_HALL   0U
#define VESC_SENSOR_PORT_ABI    1U
#define VESC_APP_UART           3U

static uint8_t s_mc_factory[2][VESC6_MCCONF_WIRE_SIZE];
static uint8_t s_mc_active[2][VESC6_MCCONF_WIRE_SIZE];
static uint8_t s_app_factory[VESC6_APPCONF_WIRE_SIZE];
static uint8_t s_app_active[VESC6_APPCONF_WIRE_SIZE];
static bool s_initialized=false;
static bool s_layout_ok=false;

static void A(uint8_t *b,float v,int32_t *i){vesc_buf_append_float32_auto(b,v,i);}
static void F(uint8_t *b,float v,float scale,int32_t *i){vesc_buf_append_float16(b,v,scale,i);}
static float get_auto_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_float32_auto(b,&i);}
static uint32_t get_u32_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_u32(b,&i);}
static void put_auto_at(uint8_t *b,int off,float v){int32_t i=off;vesc_buf_append_float32_auto(b,v,&i);}
static void put_f16_at(uint8_t *b,int off,float v,float scale){int32_t i=off;vesc_buf_append_float16(b,v,scale,&i);}
static void put_u32_at(uint8_t *b,int off,uint32_t v){int32_t i=off;vesc_buf_append_u32(b,v,&i);}
static float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static int32_t gain_q16(float v){
    if (!isfinite(v)) return 0;
    v=clampf(v,-32768.0f,32767.999f); return (int32_t)lrintf(v*65536.0f);
}
static int32_t current_gain_to_fast_q16(float kp){
    return gain_q16(kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V);
}
static int32_t current_ki_to_fast_q16(float ki){
    return gain_q16(ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V);
}
static int32_t amp_to_q15(float a){
    float q=(a/FOC_CURRENT_Q_BASE_A)*32768.0f; q=clampf(q,-32768.0f,32767.0f); return (int32_t)lrintf(q);
}
static bool sig_ok(const uint8_t *w,uint32_t sig){int32_t i=0;return vesc_buf_get_u32(w,&i)==sig;}

static bool runtime_mc_ready(const MotorRuntime *m) {
    if (m == NULL) return false;
    if (m->pole_pairs < 1U || m->pole_pairs > 60U) return false;
    if (!isfinite(m->current_max_a) || m->current_max_a < 0.1f) return false;
    if (!isfinite(m->max_duty) || fabsf(m->max_duty) < 0.01f) return false;
    return true;
}

static void build_foc_hall(const MotorRuntime *m,uint8_t out[8]) {
    /* GET_MCCONF can arrive immediately after FW_VERSION, before motor_boot_thread
       has initialized MotorRuntime. Never serialize the zeroed BSS as a Hall table. */
    static const uint8_t safe[8]={255,17,83,50,150,183,117,255};
    if (!runtime_mc_ready(m)) { memcpy(out,safe,8); return; }
    bool sane = (m->foc_hall_table[0] == 255U && m->foc_hall_table[7] == 255U);
    for (unsigned k=1;k<7U;k++) sane = sane && (m->foc_hall_table[k] <= 200U);
    if (!sane) { memcpy(out,safe,8); return; }
    memcpy(out,m->foc_hall_table,8);
}

/* VESC 6.00 wire image. Unsupported subsystems keep conservative values but
   their bytes are still retained exactly after a SET_MCCONF. */
static bool build_mc_default(uint8_t *b,motor_id_t id) {
    MotorRuntime *m=motor_get(id); bool right=id==MOTOR_RIGHT; bool mr=runtime_mc_ready(m); int32_t i=0;
    const uint8_t legacy[8]={255,1,3,2,5,6,4,255}; uint8_t hall[8]; build_foc_hall(m,hall);
    memset(b,0,VESC6_MCCONF_WIRE_SIZE);
    vesc_buf_append_u32(b,VESC6_MCCONF_SIGNATURE,&i);
    b[i++]=VESC_PWM_SYNCHRONOUS; b[i++]=VESC_COMM_INTEGRATE; b[i++]=VESC_MOTOR_FOC; b[i++]=VESC_SENSOR_SENSORED;
    A(b,mr?m->current_max_a:FOC_MAX_CURRENT_A,&i); A(b,mr?m->current_min_a:-FOC_MAX_CURRENT_A,&i);
    A(b,mr?m->current_max_a:FOC_MAX_CURRENT_A,&i); A(b,mr?m->current_min_a:-FOC_MAX_CURRENT_A,&i);
    A(b,mr?m->abs_current_max_a:FOC_ABS_CURRENT_TRIP_A,&i);
    A(b,mr?m->min_erpm:MOTOR_DEFAULT_MIN_ERPM,&i); A(b,mr?m->max_erpm:MOTOR_DEFAULT_MAX_ERPM,&i);
    F(b,0.8f,10000,&i); A(b,mr?m->max_erpm:MOTOR_DEFAULT_MAX_ERPM,&i); A(b,mr?m->max_erpm:MOTOR_DEFAULT_MAX_ERPM,&i);
    A(b,VBUS_MIN_RUN_V,&i); A(b,VBUS_MAX_RUN_V,&i); A(b,36.0f,&i); A(b,32.0f,&i); b[i++]=0;
    F(b,80,10,&i);F(b,100,10,&i);F(b,80,10,&i);F(b,100,10,&i);F(b,0.15f,10000,&i);
    /* VESC l_min_duty is a positive low-duty threshold, not a negative
       reverse limit. Reverse is represented by the sign of COMM_SET_DUTY. */
    F(b,0.0f,10000,&i);F(b,mr?m->max_duty:MOTOR_DEFAULT_MAX_DUTY,10000,&i);
    A(b,5000,&i);A(b,-5000,&i);F(b,1,10000,&i);F(b,1,10000,&i);F(b,0,10000,&i);
    A(b,250,&i);A(b,250,&i);A(b,10,&i);F(b,0,10,&i);F(b,0,10000,&i);A(b,1000,&i);A(b,0,&i);
    for(unsigned k=0;k<8;k++){b[i++]=legacy[k];} A(b,2000,&i);
    A(b,mr?m->current_kp:(right?RIGHT_FOC_KP:LEFT_FOC_KP),&i); A(b,mr?m->current_ki:(right?RIGHT_FOC_KI:LEFT_FOC_KI),&i); A(b,(float)PWM_FREQUENCY_HZ,&i); A(b,1000000.0f/PWM_FREQUENCY_HZ,&i);
    b[i++]=(!right&&mr&&m->encoder.inverted)?1:0;
    A(b,(!right&&mr)?((float)m->encoder.elec_offset_u16*360.0f/65536.0f):0.0f,&i);
    A(b,(!right&&mr)?m->encoder.electrical_ratio:(float)(mr?m->pole_pairs:(right?RIGHT_POLE_PAIRS:LEFT_POLE_PAIRS)),&i);
    b[i++]=(!right&&mr&&m->sensor_mode==SENSOR_MODE_ENCODER)?VESC_FOC_SENSOR_ENCODER:VESC_FOC_SENSOR_HALL;
    A(b,2000,&i);A(b,30000,&i);A(b,20e-6f,&i);A(b,0,&i);A(b,0.05f,&i);A(b,0.003f,&i);A(b,1000,&i);A(b,1000,&i);
    F(b,0,1000,&i);A(b,0,&i);A(b,0,&i);F(b,0,10000,&i);A(b,0,&i);A(b,0,&i);
    F(b,0,1000,&i);F(b,0,1000,&i);F(b,0,1000,&i);F(b,0,100,&i);F(b,0,100,&i);F(b,0,100,&i);F(b,0,100,&i);F(b,0,100,&i);F(b,0,100,&i);
    for(unsigned k=0;k<8;k++){b[i++]=hall[k];} A(b,500,&i);A(b,0,&i); b[i++]=1;b[i++]=0;b[i++]=0;F(b,0,1000,&i);b[i++]=0;
    F(b,25,100,&i);F(b,0.1f,10000,&i);b[i++]=0;b[i++]=0;F(b,0,10,&i);F(b,0,10,&i);F(b,0,10,&i);F(b,0,1000,&i);F(b,0,100,&i);A(b,0,&i);vesc_buf_append_u16(b,0,&i);A(b,0,&i);b[i++]=0;
    b[i++]=1; /* offsets calibrated on boot */ A(b,0,&i);A(b,0,&i);A(b,0,&i);
    for(unsigned k=0;k<6;k++){F(b,0,10000,&i);} b[i++]=0;b[i++]=0;A(b,0,&i);b[i++]=0;
    A(b,0,&i);F(b,0.85f,10000,&i);F(b,0.4f,1000,&i);F(b,0.0f,10000,&i);b[i++]=0;
    vesc_buf_append_i16(b,0,&i);vesc_buf_append_i16(b,0,&i);F(b,0,10000,&i);A(b,0,&i);A(b,0,&i);b[i++]=5; /* 1 kHz */
    A(b,mr?m->speed_pid.kp:SPEED_PID_KP,&i);A(b,mr?m->speed_pid.ki:SPEED_PID_KI,&i);A(b,mr?m->speed_pid.kd:SPEED_PID_KD,&i);F(b,mr?m->speed_kd_filter:SPEED_PID_D_FILTER,10000,&i);A(b,100,&i);b[i++]=1;A(b,35000,&i);
    A(b,mr?m->position_pid.kp:POSITION_PID_KP_CURRENT_PER_DEG,&i);A(b,mr?m->position_pid.ki:POSITION_PID_KI_CURRENT_PER_DEG_S,&i);A(b,mr?m->position_pid.kd:POSITION_PID_KD_CURRENT_PER_DEGPS,&i);A(b,0,&i);F(b,mr?m->position_kd_filter:POSITION_PID_D_FILTER,10000,&i);A(b,1,&i);F(b,0,10,&i);A(b,0,&i);
    F(b,0,10000,&i);A(b,0,&i);A(b,1,&i);F(b,0.01f,10000,&i);vesc_buf_append_i32(b,500,&i);F(b,0.02f,10000,&i);A(b,1,&i);
    vesc_buf_append_u32(b,(!right&&mr)?m->encoder.cpr:(!right?LEFT_ENCODER_CPR:0U),&i); for(unsigned k=0;k<6;k++)F(b,0,1000,&i);
    b[i++]=(!right&&mr&&m->sensor_mode==SENSOR_MODE_ENCODER)?VESC_SENSOR_PORT_ABI:VESC_SENSOR_PORT_HALL; b[i++]=(mr&&m->invert_direction)?1:0; b[i++]=0;b[i++]=0;
    A(b,PWM_FREQUENCY_HZ,&i);A(b,PWM_FREQUENCY_HZ,&i);A(b,PWM_FREQUENCY_HZ,&i);A(b,3435,&i);b[i++]=0;b[i++]=8;A(b,1,&i);F(b,10000,0.1f,&i);F(b,25,10,&i);b[i++]=0;b[i++]=8;
    b[i++]=(uint8_t)((mr?m->pole_pairs:(right?RIGHT_POLE_PAIRS:LEFT_POLE_PAIRS))*2U); A(b,1,&i);A(b,0.1f,&i);b[i++]=0;b[i++]=10;A(b,10,&i);A(b,1,&i);b[i++]=0;b[i++]=0;F(b,60,100,&i);F(b,80,100,&i);F(b,0.8f,1000,&i);F(b,0.9f,1000,&i);b[i++]=0;
    return i==(int32_t)VESC6_MCCONF_WIRE_SIZE;
}

static bool build_app_default(uint8_t *b) {
    int32_t i=0; memset(b,0,VESC6_APPCONF_WIRE_SIZE);
    vesc_buf_append_u32(b,VESC6_APPCONF_SIGNATURE,&i); b[i++]=VESC_CONTROLLER_ID_LEFT;
    vesc_buf_append_u32(b,MOTOR_COMMAND_TIMEOUT_MS,&i); A(b,0,&i);
    vesc_buf_append_u16(b,0,&i);vesc_buf_append_u16(b,0,&i);b[i++]=0;b[i++]=0;b[i++]=0;b[i++]=1;b[i++]=1;
    b[i++]=0;b[i++]=0;b[i++]=0;b[i++]=0;A(b,100000,&i);b[i++]=0;b[i++]=0;b[i++]=0;b[i++]=VESC_APP_UART;
    b[i++]=0;for(unsigned n=0;n<5;n++)A(b,0,&i);b[i++]=0;b[i++]=1;A(b,0,&i);A(b,0,&i);b[i++]=0;A(b,0,&i);A(b,0,&i);b[i++]=0;b[i++]=0;A(b,0,&i);F(b,0,1,&i);A(b,0,&i);A(b,0,&i);
    b[i++]=0;A(b,0.05f,&i);F(b,0.9f,1000,&i);F(b,3.0f,1000,&i);F(b,0.0f,1000,&i);F(b,3.3f,1000,&i);F(b,1.65f,1000,&i);F(b,0.9f,1000,&i);F(b,3.0f,1000,&i);
    b[i++]=1;b[i++]=1;b[i++]=0;b[i++]=0;b[i++]=0;A(b,0,&i);A(b,0,&i);b[i++]=0;A(b,0.4f,&i);A(b,0.2f,&i);b[i++]=0;b[i++]=0;A(b,0,&i);vesc_buf_append_u16(b,500,&i);vesc_buf_append_u32(b,VESC_UART_BAUD,&i);
    b[i++]=0;for(unsigned n=0;n<6;n++)A(b,0,&i);b[i++]=0;b[i++]=0;b[i++]=0;A(b,0,&i);b[i++]=0;A(b,0,&i);A(b,0,&i);
    for(unsigned n=0;n<10;n++)b[i++]=0;
    b[i++]=0;for(unsigned n=0;n<6;n++)A(b,0,&i);vesc_buf_append_u16(b,0,&i);vesc_buf_append_u16(b,0,&i);for(unsigned n=0;n<5;n++)A(b,0,&i);for(unsigned n=0;n<6;n++)vesc_buf_append_u16(b,0,&i);b[i++]=0;
    F(b,0,100,&i);F(b,0,100,&i);F(b,0,1000,&i);F(b,0,100,&i);F(b,0,100,&i);A(b,0,&i);F(b,0,100,&i);F(b,0,100,&i);A(b,0,&i);F(b,0,100,&i);A(b,0,&i);vesc_buf_append_u16(b,0,&i);A(b,0,&i);A(b,0,&i);F(b,0,100,&i);for(unsigned n=0;n<4;n++)A(b,0,&i);b[i++]=0;for(unsigned n=0;n<6;n++)A(b,0,&i);vesc_buf_append_u16(b,0,&i);A(b,0,&i);A(b,0,&i);vesc_buf_append_u16(b,0,&i);vesc_buf_append_u16(b,0,&i);for(unsigned n=0;n<12;n++)A(b,0,&i);vesc_buf_append_u16(b,0,&i);A(b,0,&i);vesc_buf_append_u16(b,0,&i);vesc_buf_append_u16(b,0,&i);
    b[i++]=0;b[i++]=0;F(b,0,1000,&i);F(b,0,10,&i);F(b,0,10,&i);b[i++]=0;vesc_buf_append_u16(b,0,&i);b[i++]=0;F(b,0,100,&i);F(b,0,100,&i);vesc_buf_append_u16(b,0,&i);
    b[i++]=0;b[i++]=0;b[i++]=0;for(unsigned n=0;n<4;n++)F(b,0,1,&i);vesc_buf_append_u16(b,0,&i);b[i++]=0;for(unsigned n=0;n<13;n++)A(b,0,&i);
    return i==(int32_t)VESC6_APPCONF_WIRE_SIZE;
}

void vesc_config_init_defaults(void) {
    if(s_initialized) return;
    bool ml=build_mc_default(s_mc_factory[MOTOR_LEFT],MOTOR_LEFT);
    bool mr=build_mc_default(s_mc_factory[MOTOR_RIGHT],MOTOR_RIGHT);
    bool ap=build_app_default(s_app_factory);
    s_layout_ok=ml&&mr&&ap;
    memcpy(s_mc_active,s_mc_factory,sizeof(s_mc_active)); memcpy(s_app_active,s_app_factory,sizeof(s_app_active));
    s_initialized=true;
}
bool vesc_config_layout_ok(void){vesc_config_init_defaults();return s_layout_ok;}
const uint8_t *vesc_config_mc_wire(motor_id_t id,bool defaults){vesc_config_init_defaults();return defaults?s_mc_factory[id]:s_mc_active[id];}
const uint8_t *vesc_config_app_wire(bool defaults){vesc_config_init_defaults();return defaults?s_app_factory:s_app_active;}

static bool apply_mc(motor_id_t id,const uint8_t *w) {
    MotorRuntime *m=motor_get(id); if(!m||!sig_ok(w,VESC6_MCCONF_SIGNATURE))return false;
    float current_max=fabsf(get_auto_at(w,8)), current_min=-fabsf(get_auto_at(w,12));
    float abs_current=fabsf(get_auto_at(w,24)); float min_erpm=get_auto_at(w,28), max_erpm=get_auto_at(w,32);
    int32_t duty_i=73; float vesc_min_duty=vesc_buf_get_float16(w,10000.0f,&duty_i);
    duty_i=75; float max_duty=vesc_buf_get_float16(w,10000.0f,&duty_i);
    float current_kp=get_auto_at(w,127), current_ki=get_auto_at(w,131);
    if(!isfinite(current_max)||!isfinite(abs_current)||!isfinite(max_erpm)||!isfinite(max_duty)||!isfinite(vesc_min_duty)||!isfinite(current_kp)||!isfinite(current_ki))return false;
    if(current_max<0.1f||abs_current<current_max)return false;
    uint8_t poles=w[451]; if(poles<2U||(poles&1U)||poles>120U)return false;
    uint8_t pp=poles/2U;
    /* Equivalent of VESC commands_apply_mcconf_hw_limits(): the complete UI
       value is retained in the raw wire image, while the hardware runtime is
       capped by the board-defined current limits. */
    m->current_max_a=clampf(current_max,0.1f,FOC_MAX_CURRENT_A);
    m->current_min_a=clampf(current_min,-FOC_MAX_CURRENT_A,-0.1f);
    m->abs_current_max_a=clampf(abs_current,m->current_max_a,FOC_ABS_CURRENT_TRIP_A);
    m->abs_current_trip_q15=amp_to_q15(m->abs_current_max_a);
    m->min_erpm=clampf(min_erpm,-100000.0f,-1.0f);m->max_erpm=clampf(max_erpm,1.0f,100000.0f);
    (void)vesc_min_duty; /* threshold retained in raw mcconf for exact round-trip */
    m->max_duty=clampf(fabsf(max_duty),0.01f,0.98f);m->min_duty=-m->max_duty;
    m->current_kp=clampf(current_kp,0.00001f,10.0f);m->current_ki=clampf(current_ki,0.0f,200000.0f);
    m->current_kp_q16=current_gain_to_fast_q16(m->current_kp);m->current_ki_dt_q16=current_ki_to_fast_q16(m->current_ki);
    m->speed_pid.kp=clampf(get_auto_at(w,330),0.0f,1000.0f);m->speed_pid.ki=clampf(get_auto_at(w,334),0.0f,1000.0f);m->speed_pid.kd=clampf(get_auto_at(w,338),0.0f,1000.0f);
    {int32_t q=342;m->speed_kd_filter=clampf(vesc_buf_get_float16(w,10000.0f,&q),0.0f,1.0f);}
    m->position_pid.kp=clampf(get_auto_at(w,353),0.0f,1000.0f);m->position_pid.ki=clampf(get_auto_at(w,357),0.0f,1000.0f);m->position_pid.kd=clampf(get_auto_at(w,361),0.0f,1000.0f);
    {int32_t q=369;m->position_kd_filter=clampf(vesc_buf_get_float16(w,10000.0f,&q),0.0f,1.0f);}
    m->pole_pairs=pp; m->invert_direction=w[420]!=0U;
    for(unsigned k=0;k<8;k++){m->foc_hall_table[k]=w[223+k]; if(w[223+k]==255U){m->hall_table[k]=-1;m->hall_angle_u16[k]=0;} else {m->hall_angle_u16[k]=(uint16_t)(((uint32_t)w[223+k]*65536U)/200U);m->hall_table[k]=(int8_t)(((uint32_t)w[223+k]*6U)/200U);}}
    if(id==MOTOR_LEFT){
        uint32_t cpr=get_u32_at(w,403); if(cpr>=4U&&cpr<=65535U)m->encoder.cpr=cpr;
        m->encoder.inverted=w[143]!=0U; float off=get_auto_at(w,144); while(off<0)off+=360;while(off>=360)off-=360;m->encoder.elec_offset_u16=(uint16_t)lrintf(off*(65536.0f/360.0f));
        float ratio=get_auto_at(w,148);
        if(!isfinite(ratio)||ratio<=0.0f||ratio>1000.0f)return false;
        uint64_t rq=(uint64_t)llrintf(ratio*65536.0f);
        uint64_t step=(rq<<16)/m->encoder.cpr;
        if(rq==0U||rq>0xFFFFFFFFULL||step>0xFFFFFFFFULL)return false;
        m->encoder.electrical_ratio=ratio;
        m->encoder.electrical_ratio_q16=(uint32_t)rq;
        m->encoder.phase_per_count_q16=(uint32_t)step;
        bool enc=(w[152]==VESC_FOC_SENSOR_ENCODER)||(w[419]==VESC_SENSOR_PORT_ABI);
        m->encoder.synced=false;m->encoder.motion_proved=false;
        motor_hw_configure_sensor(m,enc?SENSOR_MODE_ENCODER:SENSOR_MODE_HALL);
        if(!enc)motor_hall_edge_isr(m);
    } else { motor_hw_configure_sensor(m,SENSOR_MODE_HALL); motor_hall_edge_isr(m); }
    return true;
}

static bool apply_app(const uint8_t *w) {
    if(!sig_ok(w,VESC6_APPCONF_SIGNATURE))return false;
    uint32_t timeout=get_u32_at(w,5);float brake=get_auto_at(w,9);if(timeout>600000U||!isfinite(brake))return false;
    /* Transport is intentionally immutable: APP config may round-trip every byte,
       but cannot turn off or retime the proven USART3 DMA link. */
    vesc_timeout_configure(timeout,brake);
    uint8_t app=w[33]; app_adc_port_set_enabled(app==2U||app==5U||app==11U);
    return true;
}

bool vesc_config_set_mc_wire(motor_id_t id,const uint8_t *wire,uint16_t len,bool store){
    vesc_config_init_defaults(); if(!wire||len!=VESC6_MCCONF_WIRE_SIZE||!sig_ok(wire,VESC6_MCCONF_SIGNATURE))return false;
    /* Same-value writes are idempotent. In particular, do not revoke the
       LEFT no-index encoder alignment just because VESC Tool writes back the
       configuration it has just read. */
    if(memcmp(wire,s_mc_active[id],VESC6_MCCONF_WIRE_SIZE)==0){return !store||config_store_save_all();}
    MotorRuntime *m=motor_get(id); if(m->pwm_enabled||m->detect.busy)return false; motor_stop(m);
    uint8_t old[VESC6_MCCONF_WIRE_SIZE];memcpy(old,s_mc_active[id],sizeof(old));memcpy(s_mc_active[id],wire,len);
    if(!apply_mc(id,s_mc_active[id])){memcpy(s_mc_active[id],old,sizeof(old));(void)apply_mc(id,old);return false;}
    /* Publish the values actually accepted by board limits while preserving
       every unsupported/UI-only byte from the incoming 481-byte wire image. */
    vesc_config_sync_motor_runtime(id);
    if(store&&!config_store_save_all()){memcpy(s_mc_active[id],old,sizeof(old));(void)apply_mc(id,old);return false;} return true;
}
bool vesc_config_set_app_wire(const uint8_t *wire,uint16_t len,bool store){
    vesc_config_init_defaults();if(!wire||len!=VESC6_APPCONF_WIRE_SIZE||!sig_ok(wire,VESC6_APPCONF_SIGNATURE))return false;
    if(memcmp(wire,s_app_active,VESC6_APPCONF_WIRE_SIZE)==0){return !store||config_store_save_all();}
    uint8_t old[VESC6_APPCONF_WIRE_SIZE];memcpy(old,s_app_active,sizeof(old));memcpy(s_app_active,wire,len);
    if(!apply_app(s_app_active)){memcpy(s_app_active,old,sizeof(old));(void)apply_app(old);return false;}
    if(store&&!config_store_save_all()){memcpy(s_app_active,old,sizeof(old));(void)apply_app(old);return false;}return true;
}

void vesc_config_sync_motor_runtime(motor_id_t id){
    vesc_config_init_defaults();MotorRuntime *m=motor_get(id);uint8_t *w=s_mc_active[id];
    put_auto_at(w,8,m->current_max_a);put_auto_at(w,12,m->current_min_a);put_auto_at(w,24,m->abs_current_max_a);put_auto_at(w,28,m->min_erpm);put_auto_at(w,32,m->max_erpm);
    put_f16_at(w,73,0.0f,10000.0f);put_f16_at(w,75,m->max_duty,10000.0f);
    put_auto_at(w,127,m->current_kp);put_auto_at(w,131,m->current_ki);for(unsigned k=0;k<8;k++)w[223+k]=m->foc_hall_table[k];
    put_auto_at(w,330,m->speed_pid.kp);put_auto_at(w,334,m->speed_pid.ki);put_auto_at(w,338,m->speed_pid.kd);put_f16_at(w,342,m->speed_kd_filter,10000.0f);
    put_auto_at(w,353,m->position_pid.kp);put_auto_at(w,357,m->position_pid.ki);put_auto_at(w,361,m->position_pid.kd);put_f16_at(w,369,m->position_kd_filter,10000.0f);
    w[451]=(uint8_t)(m->pole_pairs*2U);w[420]=m->invert_direction?1:0;
    if(id==MOTOR_LEFT){w[143]=m->encoder.inverted?1:0;put_auto_at(w,144,(float)m->encoder.elec_offset_u16*360.0f/65536.0f);put_auto_at(w,148,m->encoder.electrical_ratio);put_u32_at(w,403,m->encoder.cpr);bool enc=m->sensor_mode==SENSOR_MODE_ENCODER;w[152]=enc?VESC_FOC_SENSOR_ENCODER:VESC_FOC_SENSOR_HALL;w[419]=enc?VESC_SENSOR_PORT_ABI:VESC_SENSOR_PORT_HALL;}
}
void vesc_config_export_wire(uint8_t l[VESC6_MCCONF_WIRE_SIZE],uint8_t r[VESC6_MCCONF_WIRE_SIZE],uint8_t a[VESC6_APPCONF_WIRE_SIZE]){vesc_config_init_defaults();memcpy(l,s_mc_active[0],VESC6_MCCONF_WIRE_SIZE);memcpy(r,s_mc_active[1],VESC6_MCCONF_WIRE_SIZE);memcpy(a,s_app_active,VESC6_APPCONF_WIRE_SIZE);}
bool vesc_config_import_wire(const uint8_t l[VESC6_MCCONF_WIRE_SIZE],const uint8_t r[VESC6_MCCONF_WIRE_SIZE],const uint8_t a[VESC6_APPCONF_WIRE_SIZE]){
    vesc_config_init_defaults();if(!sig_ok(l,VESC6_MCCONF_SIGNATURE)||!sig_ok(r,VESC6_MCCONF_SIGNATURE)||!sig_ok(a,VESC6_APPCONF_SIGNATURE))return false;
    memcpy(s_mc_active[0],l,VESC6_MCCONF_WIRE_SIZE);memcpy(s_mc_active[1],r,VESC6_MCCONF_WIRE_SIZE);memcpy(s_app_active,a,VESC6_APPCONF_WIRE_SIZE);
    if(!apply_mc(MOTOR_LEFT,s_mc_active[0])||!apply_mc(MOTOR_RIGHT,s_mc_active[1])||!apply_app(s_app_active))return false;
    vesc_config_sync_motor_runtime(MOTOR_LEFT);
    vesc_config_sync_motor_runtime(MOTOR_RIGHT);
    return true;
}
