#include "confgenerator.h"
#include "util/buffer.h"
#include "util/maths.h"
#include "motor/mc_interface.h"
#include "motor/mcconf_default.h"
#include "encoder/encoder.h"
#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "motor/mcpwm_foc.h"
#include "motor/foc_math.h"
#include "timeout.h"
#include "conf_general.h"
#include "applications/appconf_default.h"
#include "applications/app.h"
#include <string.h>
#include <math.h>

#define VESC_PWM_SYNCHRONOUS 1U
#define VESC_COMM_INTEGRATE  0U
#define VESC_MOTOR_BLDC      0U
#define VESC_MOTOR_DC        1U
#define VESC_MOTOR_FOC       2U
#define VESC_SENSOR_SENSORED 1U
/* VESC firmware 6.00 wire enum. ENCODER_AB=9 was introduced later and must
 * never appear on a 6.00 MCCONF wire image. Internally this port can still use
 * FOC_SENSOR_MODE_ENCODER_AB for the incremental A/B implementation. */
#define VESC_FOC_SENSOR_ENCODER    1U
#define VESC_FOC_SENSOR_HALL       2U
#define VESC_SENSOR_PORT_HALL   0U
#define VESC_SENSOR_PORT_ABI    1U
#define VESC_APP_NONE           0U
#define VESC_APP_ADC            2U
#define VESC_APP_UART           3U
#define VESC_APP_ADC_UART       5U

static uint8_t s_mc_factory[2][VESC6_MCCONF_WIRE_SIZE];
static uint8_t s_mc_active[2][VESC6_MCCONF_WIRE_SIZE];
static uint8_t s_app_factory[VESC6_APPCONF_WIRE_SIZE];
static uint8_t s_app_active[VESC6_APPCONF_WIRE_SIZE];
/* Import runs from the 2-KiB boot thread. Keep the ~1.5-KiB transactional
 * rollback image out of that thread stack. vesc_config_import_wire() is only
 * used by the serialized boot-time flash loader. */
/* Shared transactional rollback scratch. SET_MCCONF/APPCONF runs in the
 * serialized comm_block worker, while import runs during serialized boot, so
 * these paths cannot overlap. Reusing the same exact-size images removes the
 * risky RTOS-stack copies without spending another ~1 KiB of F103 SRAM. */
static uint8_t s_rollback_mc[2][VESC6_MCCONF_WIRE_SIZE];
static uint8_t s_rollback_app[VESC6_APPCONF_WIRE_SIZE];
static bool s_initialized=false;
static bool s_layout_ok=false;

static void A(uint8_t *b,float v,int32_t *i){vesc_buf_append_float32_auto(b,v,i);}
static void F(uint8_t *b,float v,float scale,int32_t *i){vesc_buf_append_float16(b,v,scale,i);}
static float get_auto_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_float32_auto(b,&i);}
static float get_f16_at(const uint8_t *b,int off,float scale){int32_t i=off;return vesc_buf_get_float16(b,scale,&i);}
static uint16_t get_u16_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_u16(b,&i);}
static uint32_t get_u32_at(const uint8_t *b,int off){int32_t i=off;return vesc_buf_get_u32(b,&i);}
static void put_auto_at(uint8_t *b,int off,float v){int32_t i=off;vesc_buf_append_float32_auto(b,v,&i);}
static void put_f16_at(uint8_t *b,int off,float v,float scale){int32_t i=off;vesc_buf_append_float16(b,v,scale,&i);}
static void put_u16_at(uint8_t *b,int off,uint16_t v){int32_t i=off;vesc_buf_append_u16(b,v,&i);}
static void put_u32_at(uint8_t *b,int off,uint32_t v){int32_t i=off;vesc_buf_append_u32(b,v,&i);}
/* Forward declaration so vesc_config_set_mc_wire() can reuse the wire decoder
   for the range-validation clamp. Defined further below. */
static void mcconf_decode_wire(const uint8_t *w, mc_configuration *c);
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
static int32_t volt_to_q15(float v){
    float q=(v/FOC_VOLTAGE_Q_BASE_V)*32768.0f; q=clampf(q,0.0f,32767.0f); return (int32_t)lrintf(q);
}
static bool sig_ok(const uint8_t *w,uint32_t sig){int32_t i=0;return vesc_buf_get_u32(w,&i)==sig;}

static bool runtime_mc_auto_fields_finite(const uint8_t *w) {
    /* Every float32-auto field consumed by this hardware backend must decode to
       a finite value before MotorRuntime is touched. Unsupported wire-only
       fields remain byte-preserved and are intentionally not interpreted. */
    static const uint16_t off[] = {
        VESC6_MC_OFF_L_CURRENT_MAX, VESC6_MC_OFF_L_CURRENT_MIN,
        VESC6_MC_OFF_L_IN_CURRENT_MAX, VESC6_MC_OFF_L_IN_CURRENT_MIN,
        VESC6_MC_OFF_L_ABS_CURRENT_MAX, VESC6_MC_OFF_L_MIN_ERPM,
        VESC6_MC_OFF_L_MAX_ERPM, VESC6_MC_OFF_L_MIN_VIN, VESC6_MC_OFF_L_MAX_VIN,
        VESC6_MC_OFF_L_BAT_CUT_START, VESC6_MC_OFF_L_BAT_CUT_END,
        VESC6_MC_OFF_L_WATT_MAX, VESC6_MC_OFF_L_WATT_MIN,
        VESC6_MC_OFF_FOC_CURRENT_KP, VESC6_MC_OFF_FOC_CURRENT_KI,
        153U, 157U, 161U, 165U, 169U, 173U, 177U, 181U,
        VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI,
        VESC6_MC_OFF_FOC_START_CURR_DEC_RPM, 201U, VESC6_MC_OFF_FOC_FW_CURRENT_MAX,
        VESC6_MC_OFF_FOC_HALL_INTERP_ERPM, VESC6_MC_OFF_FOC_SL_ERPM,
        VESC6_MC_OFF_S_PID_KP, VESC6_MC_OFF_S_PID_KI, VESC6_MC_OFF_S_PID_KD,
        VESC6_MC_OFF_P_PID_KP, VESC6_MC_OFF_P_PID_KI, VESC6_MC_OFF_P_PID_KD,
        VESC6_MC_OFF_FOC_ENCODER_OFFSET, VESC6_MC_OFF_FOC_ENCODER_RATIO,
        VESC6_MC_OFF_SI_GEAR_RATIO, VESC6_MC_OFF_SI_WHEEL_DIAMETER,
        VESC6_MC_OFF_SI_BATTERY_AH, VESC6_MC_OFF_SI_MOTOR_NL_CURRENT
    };
    for (unsigned k = 0U; k < sizeof(off) / sizeof(off[0]); k++) {
        if (!isfinite(get_auto_at(w, (int)off[k]))) return false;
    }
    return true;
}

static bool runtime_mc_supported_ranges_valid(const uint8_t *w) {
    /* For every VESC-6.00 field that this reduced controller actually applies
       to hardware/runtime, ACK only values that can be represented without a
       hidden clamp. This keeps the canonical VESC Tool image and MotorRuntime
       semantically identical. Unsupported UI/subsystem fields remain raw-wire
       preserved and are intentionally not interpreted here. */
    const float current_max = get_auto_at(w, VESC6_MC_OFF_L_CURRENT_MAX);
    const float current_min = get_auto_at(w, VESC6_MC_OFF_L_CURRENT_MIN);
    const float input_max = get_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MAX);
    const float input_min = get_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MIN);
    const float abs_current = get_auto_at(w, VESC6_MC_OFF_L_ABS_CURRENT_MAX);
    const float min_erpm = get_auto_at(w, VESC6_MC_OFF_L_MIN_ERPM);
    const float max_erpm = get_auto_at(w, VESC6_MC_OFF_L_MAX_ERPM);
    const float erpm_start = get_f16_at(w, VESC6_MC_OFF_L_ERPM_START, 10000.0f);
    const float min_vin = get_auto_at(w, VESC6_MC_OFF_L_MIN_VIN);
    const float max_vin = get_auto_at(w, VESC6_MC_OFF_L_MAX_VIN);
    const float min_duty = get_f16_at(w, VESC6_MC_OFF_L_MIN_DUTY, 10000.0f);
    const float max_duty = get_f16_at(w, VESC6_MC_OFF_L_MAX_DUTY, 10000.0f);
    const float watt_max = get_auto_at(w, VESC6_MC_OFF_L_WATT_MAX);
    const float watt_min = get_auto_at(w, VESC6_MC_OFF_L_WATT_MIN);
    const float cur_max_scale = get_f16_at(w, VESC6_MC_OFF_L_CURRENT_MAX_SCALE, 10000.0f);
    const float cur_min_scale = get_f16_at(w, VESC6_MC_OFF_L_CURRENT_MIN_SCALE, 10000.0f);
    const float duty_start = get_f16_at(w, VESC6_MC_OFF_L_DUTY_START, 10000.0f);
    const float temp_fet_start = get_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_START, 10.0f);
    const float temp_fet_end = get_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_END, 10.0f);
    const float temp_motor_start = get_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_START, 10.0f);
    const float temp_motor_end = get_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_END, 10.0f);
    const float temp_accel_dec = get_f16_at(w, VESC6_MC_OFF_L_TEMP_ACCEL_DEC, 10000.0f);

    if (current_max < 0.1f || current_max > FOC_MAX_CURRENT_A ||
        current_min < -FOC_MAX_CURRENT_A || current_min > 0.0f ||
        input_max < 0.0f || input_max > FOC_MAX_CURRENT_A ||
        input_min < -FOC_MAX_CURRENT_A || input_min > 0.0f ||
        abs_current < fmaxf(current_max, fabsf(current_min)) ||
        abs_current > FOC_ABS_CURRENT_TRIP_A ||
        min_erpm < -MOTOR_DEFAULT_MAX_ERPM || min_erpm > -1.0f ||
        max_erpm < 1.0f || max_erpm > MOTOR_DEFAULT_MAX_ERPM || min_erpm >= max_erpm ||
        !isfinite(erpm_start) || erpm_start < 0.0f || erpm_start > 1.0f ||
        min_vin < VBUS_MIN_RUN_V || min_vin > (VBUS_MAX_RUN_V - 0.5f) ||
        max_vin > VBUS_MAX_RUN_V || max_vin < (min_vin + 0.5f) ||
        get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_END) < min_vin ||
        get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_START) > max_vin ||
        get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_START) <= get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_END) ||
        min_duty < 0.0f || max_duty < 0.01f || max_duty > 0.98f ||
        min_duty > max_duty || !isfinite(watt_max) || !isfinite(watt_min) ||
        watt_max < 0.0f || watt_min > 0.0f || watt_min >= watt_max ||
        cur_max_scale < 0.0f || cur_max_scale > 1.0f ||
        cur_min_scale < 0.0f || cur_min_scale > 1.0f ||
        duty_start < 0.0f || duty_start > 1.0f ||
        !isfinite(temp_fet_start) || !isfinite(temp_fet_end) ||
        temp_fet_start < HOVERBOARD_MCU_TEMP_MIN_VALID_C ||
        temp_fet_end > HOVERBOARD_MCU_TEMP_MAX_VALID_C ||
        temp_fet_end <= temp_fet_start + 0.5f ||
        !isfinite(temp_motor_start) || !isfinite(temp_motor_end) ||
        temp_motor_start < -100.0f || temp_motor_end > 250.0f ||
        temp_motor_end <= temp_motor_start + 0.5f ||
        !isfinite(temp_accel_dec) || temp_accel_dec < 0.0f || temp_accel_dec > 1.0f) return false;

    const float current_kp = get_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KP);
    const float current_ki = get_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KI);
    const float pll_kp = get_auto_at(w, 153U);
    const float pll_ki = get_auto_at(w, 157U);
    const float motor_l = get_auto_at(w, 161U);
    const float ld_lq = get_auto_at(w, 165U);
    const float motor_r = get_auto_at(w, 169U);
    const float flux = get_auto_at(w, 173U);
    const float observer_gain = get_auto_at(w, 177U);
    const float observer_gain_slow = get_auto_at(w, 181U);
    const uint8_t sat_comp_mode = w[VESC6_MC_OFF_FOC_SAT_COMP_MODE];
    const float sat_comp = get_f16_at(w, VESC6_MC_OFF_FOC_SAT_COMP, 1000.0f);
    const uint8_t observer_type = w[VESC6_MC_OFF_FOC_OBSERVER_TYPE];
    const bool sample_v0_v7 = w[VESC6_MC_OFF_FOC_SAMPLE_V0_V7] != 0U;
    const bool sample_high_current = w[VESC6_MC_OFF_FOC_SAMPLE_HIGH_CURRENT] != 0U;
    const float openloop_rpm = get_auto_at(w, 201U);
    const float openloop_rpm_low = get_f16_at(w, 205U, 1000.0f);
    const float sl_hyst = get_f16_at(w, 211U, 100.0f);
    const float sl_lock = get_f16_at(w, 213U, 100.0f);
    const float sl_ramp = get_f16_at(w, 215U, 100.0f);
    const float sl_time = get_f16_at(w, 217U, 100.0f);
    const float sl_boost = get_f16_at(w, 219U, 100.0f);
    const float sl_max_q = get_f16_at(w, 221U, 100.0f);
    const float hall_interp = get_auto_at(w, VESC6_MC_OFF_FOC_HALL_INTERP_ERPM);
    const float sl_erpm = get_auto_at(w, VESC6_MC_OFF_FOC_SL_ERPM);
    const float obs_offset = get_f16_at(w, VESC6_MC_OFF_FOC_OBSERVER_OFFSET, 1000.0f);
    const float down_kp = get_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP);
    const float down_ki = get_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI);
    const float start_curr_dec = get_f16_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC, 10000.0f);
    const float start_curr_dec_rpm = get_auto_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC_RPM);
    const float current_filter = get_f16_at(w, VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST, 10000.0f);
    const uint8_t decoupling = w[VESC6_MC_OFF_FOC_CC_DECOUPLING];
    const uint8_t mtpa_mode = w[VESC6_MC_OFF_FOC_MTPA_MODE];
    const float fw_max = get_auto_at(w, VESC6_MC_OFF_FOC_FW_CURRENT_MAX);
    const float fw_duty = get_f16_at(w, VESC6_MC_OFF_FOC_FW_DUTY_START, 10000.0f);
    const float fw_ramp = get_f16_at(w, VESC6_MC_OFF_FOC_FW_RAMP_TIME, 1000.0f);
    const float fw_q = get_f16_at(w, VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR, 10000.0f);
    const uint8_t foc_speed_source = w[VESC6_MC_OFF_FOC_SPEED_SOURCE];
    if (current_kp < 0.00001f || current_kp > 10.0f ||
        current_ki < 0.0f || current_ki > 200000.0f ||
        pll_kp < 0.0f || pll_kp > 100000.0f ||
        pll_ki < 0.0f || pll_ki > 1000000.0f ||
        motor_l < 1.0e-7f || motor_l > 0.1f ||
        ld_lq < -0.1f || ld_lq > 0.1f ||
        motor_r < 1.0e-5f || motor_r > 100.0f ||
        flux < 1.0e-6f || flux > (FOC_FLUX_Q_BASE_WB * 1.90f) ||
        observer_gain < 0.0f || observer_gain > 1000000.0f ||
        observer_gain_slow < 0.0f || observer_gain_slow > 1.0f ||
        sat_comp_mode > SAT_COMP_LAMBDA_AND_FACTOR ||
        !isfinite(sat_comp) || sat_comp < 0.0f || sat_comp > 1.0f ||
        observer_type > FOC_OBSERVER_MXV_LAMBDA_COMP_LIN ||
        foc_speed_source > FOC_SPEED_SRC_OBSERVER ||
        /* Shared ADC1/2 dual-motor topology has one coherent sample instant
           per PWM period. V0/V7 alternation and high-current resampling cannot
           be represented safely on this board and must never be ACKed. */
        sample_v0_v7 || sample_high_current ||
        openloop_rpm < 10.0f || openloop_rpm > MOTOR_DEFAULT_MAX_ERPM ||
        openloop_rpm_low < 0.0f || openloop_rpm_low > MOTOR_DEFAULT_MAX_ERPM ||
        sl_hyst < 0.0f || sl_hyst > 100.0f ||
        sl_lock < 0.0f || sl_lock > 20.0f ||
        sl_ramp < 0.01f || sl_ramp > 20.0f ||
        sl_time < 0.01f || sl_time > 20.0f ||
        sl_boost < 0.0f || sl_boost > FOC_MAX_CURRENT_A ||
        sl_max_q < 0.1f || sl_max_q > FOC_MAX_CURRENT_A ||
        hall_interp < 0.0f || hall_interp > MOTOR_DEFAULT_MAX_ERPM ||
        sl_erpm < 10.0f || sl_erpm > MOTOR_DEFAULT_MAX_ERPM ||
        !isfinite(obs_offset) || obs_offset < -10.0f || obs_offset > 10.0f ||
        !isfinite(down_kp) || down_kp < 0.0f || down_kp > 100000.0f ||
        !isfinite(down_ki) || down_ki < 0.0f || down_ki > 1000000.0f ||
        !isfinite(start_curr_dec) || start_curr_dec < 0.0f || start_curr_dec > 1.0f ||
        !isfinite(start_curr_dec_rpm) || start_curr_dec_rpm < 0.0f || start_curr_dec_rpm > MOTOR_DEFAULT_MAX_ERPM ||
        !isfinite(current_filter) || current_filter < 0.0f || current_filter > 1.0f ||
        decoupling > FOC_CC_DECOUPLING_CROSS_BEMF ||
        mtpa_mode > MTPA_MODE_IQ_MEASURED ||
        !isfinite(fw_max) || fw_max < 0.0f || fw_max > FOC_MAX_CURRENT_A ||
        !isfinite(fw_duty) || fw_duty < 0.0f || fw_duty > 1.0f ||
        !isfinite(fw_ramp) || fw_ramp < 0.01f || fw_ramp > 30.0f ||
        !isfinite(fw_q) || fw_q < 0.0f || fw_q > 1.0f) return false;

    const float s_kp = get_auto_at(w, VESC6_MC_OFF_S_PID_KP);
    const float s_ki = get_auto_at(w, VESC6_MC_OFF_S_PID_KI);
    const float s_kd = get_auto_at(w, VESC6_MC_OFF_S_PID_KD);
    const float s_kdf = get_f16_at(w, 342U, 10000.0f);
    const float s_min_erpm = get_auto_at(w, 344U);
    const uint8_t s_allow_braking = w[348U];
    const float s_ramp = get_auto_at(w, 349U);
    const float p_kp = get_auto_at(w, VESC6_MC_OFF_P_PID_KP);
    const float p_ki = get_auto_at(w, VESC6_MC_OFF_P_PID_KI);
    const float p_kd = get_auto_at(w, VESC6_MC_OFF_P_PID_KD);
    const float p_kd_proc = get_auto_at(w, 365U);
    const float p_kdf = get_f16_at(w, 369U, 10000.0f);
    const float p_ang_div = get_auto_at(w, 371U);
    const float p_gain_dec = get_f16_at(w, 375U, 10.0f);
    const float p_offset = get_auto_at(w, 377U);
    const float cc_min_current = get_auto_at(w, 383U);
    const float enc_off = get_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_OFFSET);
    const float enc_ratio = get_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_RATIO);
    const float gear = get_auto_at(w, VESC6_MC_OFF_SI_GEAR_RATIO);
    const float wheel = get_auto_at(w, VESC6_MC_OFF_SI_WHEEL_DIAMETER);
    const float batt_ah = get_auto_at(w, VESC6_MC_OFF_SI_BATTERY_AH);
    const float nl_current = get_auto_at(w, VESC6_MC_OFF_SI_MOTOR_NL_CURRENT);
    if (s_kp < 0.0f || s_kp > 1000.0f || s_ki < 0.0f || s_ki > 1000.0f ||
        s_kd < 0.0f || s_kd > 1000.0f || s_kdf < 0.0f || s_kdf > 1.0f ||
        !isfinite(s_min_erpm) || s_min_erpm < 0.0f || s_min_erpm > MOTOR_DEFAULT_MAX_ERPM ||
        s_allow_braking > 1U || !isfinite(s_ramp) || s_ramp < 0.0f || s_ramp > 1000000.0f ||
        p_kp < 0.0f || p_kp > 1000.0f || p_ki < 0.0f || p_ki > 1000.0f ||
        p_kd < 0.0f || p_kd > 1000.0f || !isfinite(p_kd_proc) || p_kd_proc < 0.0f || p_kd_proc > 1000.0f ||
        p_kdf < 0.0f || p_kdf > 1.0f || !isfinite(p_ang_div) || p_ang_div < 0.01f || p_ang_div > 1000.0f ||
        !isfinite(p_gain_dec) || p_gain_dec < 0.0f || p_gain_dec > 3600.0f ||
        !isfinite(p_offset) || p_offset < -36000.0f || p_offset > 36000.0f ||
        !isfinite(cc_min_current) || cc_min_current < 0.0f || cc_min_current > FOC_MAX_CURRENT_A ||
        enc_off < 0.0f || enc_off >= 360.0f || enc_ratio <= 0.0f || enc_ratio > 1000.0f ||
        gear < 0.01f || gear > 1000.0f || wheel < 0.001f || wheel > 10.0f ||
        batt_ah < 0.0f || batt_ah > 10000.0f ||
        nl_current < 0.0f || nl_current > FOC_MAX_CURRENT_A) {
        return false;
    }
    return true;
}

static bool byte_in_range(uint16_t i, uint16_t off, uint16_t len) {
    return i >= off && i < (uint16_t)(off + len);
}

static bool mc_wire_byte_runtime_mutable(uint16_t i) {
    /* Only bytes whose VESC-6.00 fields have a real runtime backend in this
       STM32F103 build are writable. Every other byte still participates in
       the exact 481-byte wire image and flash CRC, but changing it is rejected
       rather than ACKed and silently ignored. */
    if (byte_in_range(i, VESC6_MC_OFF_L_CURRENT_MAX, 30U)) return true; /* current + ERPM + l_erpm_start */
    if (byte_in_range(i, VESC6_MC_OFF_L_MIN_VIN, 17U)) return true;    /* VIN + battery cut + slow abs */
    if (byte_in_range(i, VESC6_MC_OFF_L_TEMP_FET_START, 10U)) return true; /* thermal limits + accel derate */
    if (byte_in_range(i, VESC6_MC_OFF_L_MIN_DUTY, 4U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_L_WATT_MAX, 14U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_CURRENT_KP, 8U)) return true;
    if (i == VESC6_MC_OFF_FOC_ENCODER_INVERTED) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_ENCODER_OFFSET, 8U)) return true;
    if (i == VESC6_MC_OFF_FOC_SENSOR_MODE) return true;
    if (byte_in_range(i, 153U, 8U)) return true; /* fixed-point PLL Kp/Ki */
    /* L, Ld-Lq, R and flux all have real fixed-point or slow-loop backends. */
    if (byte_in_range(i, 161U, 8U)) return true; /* foc_motor_l + Ld-Lq (MTPA) */
    if (byte_in_range(i, 169U, 8U)) return true; /* foc_motor_r + flux */
    if (byte_in_range(i, 177U, 8U)) return true; /* observer gain + slow gain */
    if (byte_in_range(i, VESC6_MC_OFF_FOC_OBSERVER_OFFSET, 10U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_START_CURR_DEC, 6U)) return true;
    if (byte_in_range(i, 201U, 6U)) return true; /* openloop rpm + low */
    /* foc_sl_openloop_hyst (211..212) has no verified VESC-6.00 backend in
       this reduced state machine. Keep it immutable rather than ACKing a shadow
       value. Lock/ramp/time/boost/max-Q and Hall table are real backends. */
    if (byte_in_range(i, 213U, 18U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_HALL_INTERP_ERPM, 4U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_SL_ERPM, 4U)) return true;
    if (i == VESC6_MC_OFF_FOC_SAT_COMP_MODE ||
        byte_in_range(i, VESC6_MC_OFF_FOC_SAT_COMP, 2U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST, 3U)) return true; /* filter + decoupling */
    /* Part 2 implements the VESC-6 observer family (0..3) plus the later MXV
       family (4..6) in the fixed-point F103 backend. The same one-byte wire
       slot is used without changing the advertised 6.00 MCCONF size. */
    if (i == VESC6_MC_OFF_FOC_OBSERVER_TYPE) return true;
    if (i == VESC6_MC_OFF_FOC_MTPA_MODE) return true;
    if (byte_in_range(i, VESC6_MC_OFF_FOC_FW_CURRENT_MAX, 10U)) return true;
    /* VESC 6.00 already owns byte 314 as foc_speed_soure (sic upstream). */
    if (i == VESC6_MC_OFF_FOC_SPEED_SOURCE) return true;
    /* VESC6 speed PID: kp/ki/kd, D-filter, min-ERPM, allow-braking, ramp. */
    if (byte_in_range(i, VESC6_MC_OFF_S_PID_KP, 23U)) return true;
    /* VESC6 position PID: kp/ki/kd/kd_proc/filter/ang_div/gain-dec/offset. */
    if (byte_in_range(i, VESC6_MC_OFF_P_PID_KP, 28U)) return true;
    /* FOC upstream consumes cc_min_current for modulation release. The other
       three cc_* fields belong to the legacy trapezoidal BLDC current-to-duty
       controller (motor/mcpwm.c) and are not consumed by upstream mcpwm_foc.c.
       Preserve them byte-exact but immutable in this FOC-only firmware. */
    if (byte_in_range(i, 383U, 4U)) return true;
    if (byte_in_range(i, VESC6_MC_OFF_M_ENCODER_COUNTS, 4U)) return true;
    if (i == VESC6_MC_OFF_M_SENSOR_PORT_MODE || i == VESC6_MC_OFF_M_INVERT_DIRECTION) return true;
    /* SI motor poles/gear/wheel/battery type/cells/Ah feed setup telemetry.
       si_motor_nl_current has no runtime consumer in this port. */
    if (byte_in_range(i, VESC6_MC_OFF_SI_MOTOR_POLES, 15U)) return true;
    return false;
}

static bool app_wire_byte_runtime_mutable(uint16_t i) {
    /* Controller ID is normalized by commands.c for the local motor-2 view.
       APP ADC owns the canonical VESC-6 adc_config bytes 90..138 and USART3
       owns the canonical baud field. PPM/NRF/PAS/etc remain immutable. */
    if (byte_in_range(i, VESC6_APP_OFF_TIMEOUT_MSEC, 8U)) return true;
    if (i == VESC6_APP_OFF_APP_TO_USE) return true;
    if (byte_in_range(i, VESC6_APP_OFF_ADC_CTRL_TYPE, 49U)) return true;
    if (byte_in_range(i, VESC6_APP_OFF_UART_BAUD, 4U)) return true;
    return false;
}

static bool unsupported_mc_bytes_unchanged(motor_id_t id, const uint8_t *wire) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT) return false;
    for (uint16_t i = 0U; i < VESC6_MCCONF_WIRE_SIZE; i++) {
        if (!mc_wire_byte_runtime_mutable(i) && wire[i] != s_mc_active[id][i]) return false;
    }
    return true;
}

static bool unsupported_app_bytes_unchanged(const uint8_t *wire) {
    for (uint16_t i = 0U; i < VESC6_APPCONF_WIRE_SIZE; i++) {
        if (!app_wire_byte_runtime_mutable(i) && wire[i] != s_app_active[i]) return false;
    }
    return true;
}

static bool runtime_mc_ready(const MotorRuntime *m) {
    if (m == NULL) return false;
    /* A zeroed BSS has motor_type==BLDC numerically. Require the hardware
       timer binding as the initialization sentinel so early GET_MCCONF does
       not accidentally publish a non-FOC backend before motor_defaults(). */
    if (m->pwm_tim == NULL) return false;
    if (m->pole_pairs < 1U || m->pole_pairs > 60U) return false;
    if (!isfinite(m->current_max_a) || m->current_max_a < 0.1f) return false;
    if (!isfinite(m->max_duty) || fabsf(m->max_duty) < 0.01f) return false;
    return true;
}

static void build_foc_hall(const MotorRuntime *m,uint8_t out[8]) {
    /* GET_MCCONF may be requested before the motor runtime is fully ready.
       Never serialize zeroed BSS as a Hall table. Never serialize the zeroed BSS as a Hall table. */
    static const uint8_t safe[8]={255,17,83,50,150,183,117,255};
    if (!runtime_mc_ready(m)) { memcpy(out,safe,8); return; }
    bool sane = (m->foc_hall_table[0] == 255U && m->foc_hall_table[7] == 255U);
    for (unsigned k=1;k<7U;k++) sane = sane && (m->foc_hall_table[k] <= 200U);
    if (!sane) { memcpy(out,safe,8); return; }
    memcpy(out,m->foc_hall_table,8);
}

/* VESC 6.00 wire image. Unsupported subsystems keep conservative values.
   Their bytes remain part of the exact 481-byte ABI and flash record, but are
   immutable through SET_MCCONF so the firmware never ACKs a setting that has
   no real runtime backend. */
static bool build_mc_default(uint8_t *b,motor_id_t id) {
    MotorRuntime *m=motor_get(id); bool right=id==MOTOR_RIGHT; bool mr=runtime_mc_ready(m); int32_t i=0;
    bool left_encoder = !right && ((mr && m->sensor_mode==SENSOR_MODE_ENCODER) ||
                                   (!mr && LEFT_SENSOR_BOOT_MODE==SENSOR_MODE_ENCODER));
    const uint8_t legacy[8]={255,1,3,2,5,6,4,255}; uint8_t hall[8]; build_foc_hall(m,hall);
    memset(b,0,VESC6_MCCONF_WIRE_SIZE);
    vesc_buf_append_u32(b,VESC6_MCCONF_SIGNATURE,&i);
    b[i++]=mr?(uint8_t)m->pwm_mode:VESC_PWM_SYNCHRONOUS;
    b[i++]=mr?(uint8_t)m->comm_mode:VESC_COMM_INTEGRATE;
    b[i++]=VESC_MOTOR_FOC;
    b[i++]=VESC_SENSOR_SENSORED;
    A(b,mr?m->current_max_a:FOC_MAX_CURRENT_A,&i); A(b,mr?m->current_min_a:-FOC_MAX_CURRENT_A,&i);
    A(b,mr?m->input_current_max_a:FOC_MAX_CURRENT_A,&i); A(b,mr?m->input_current_min_a:-FOC_MAX_CURRENT_A,&i);
    A(b,mr?m->abs_current_max_a:FOC_ABS_CURRENT_TRIP_A,&i);
    A(b,mr?m->min_erpm:MOTOR_DEFAULT_MIN_ERPM,&i); A(b,mr?m->max_erpm:MOTOR_DEFAULT_MAX_ERPM,&i);
    F(b,mr?m->erpm_start:MCCONF_L_ERPM_START_DEFAULT,10000,&i); A(b,mr?m->max_erpm:MOTOR_DEFAULT_MAX_ERPM,&i); A(b,mr?m->max_erpm:MOTOR_DEFAULT_MAX_ERPM,&i);
    A(b,mr?m->min_vin:VBUS_MIN_RUN_V,&i); A(b,mr?m->max_vin:VBUS_MAX_RUN_V,&i);
    A(b,mr?m->battery_cut_start:36.0f,&i); A(b,mr?m->battery_cut_end:32.0f,&i); b[i++]=(mr&&m->slow_abs_current)?1U:0U;
    F(b,mr?m->temp_fet_start:MCCONF_L_TEMP_FET_START_DEFAULT,10,&i);
    F(b,mr?m->temp_fet_end:MCCONF_L_TEMP_FET_END_DEFAULT,10,&i);
    F(b,mr?m->temp_motor_start:MCCONF_L_TEMP_MOTOR_START_DEFAULT,10,&i);
    F(b,mr?m->temp_motor_end:MCCONF_L_TEMP_MOTOR_END_DEFAULT,10,&i);
    F(b,mr?m->temp_accel_dec:MCCONF_L_TEMP_ACCEL_DEC_DEFAULT,10000,&i);
    /* VESC l_min_duty is a positive low-duty threshold, not a negative
       reverse limit. Reverse is represented by the sign of COMM_SET_DUTY. */
    F(b,mr?m->min_duty:MCCONF_L_MIN_DUTY_DEFAULT,10000,&i);F(b,mr?m->max_duty:MCCONF_L_MAX_DUTY_DEFAULT,10000,&i);
    A(b,mr?m->watt_max:MCCONF_L_WATT_MAX_DEFAULT,&i);A(b,mr?m->watt_min:MCCONF_L_WATT_MIN_DEFAULT,&i);
    F(b,mr?m->current_max_scale:MCCONF_L_CURRENT_MAX_SCALE_DEFAULT,10000,&i);
    F(b,mr?m->current_min_scale:MCCONF_L_CURRENT_MIN_SCALE_DEFAULT,10000,&i);
    F(b,mr?m->duty_start:MCCONF_L_DUTY_START_DEFAULT,10000,&i);
    A(b,250,&i);A(b,250,&i);A(b,10,&i);F(b,0,10,&i);F(b,0,10000,&i);A(b,1000,&i);A(b,0,&i);
    for(unsigned k=0;k<8;k++){b[i++]=legacy[k];}
    A(b,2000,&i);
    A(b,mr?m->current_kp:(right?RIGHT_FOC_KP:LEFT_FOC_KP),&i); A(b,mr?m->current_ki:(right?RIGHT_FOC_KI:LEFT_FOC_KI),&i); A(b,(float)VESC_FOC_F_ZV_HZ,&i); A(b,mr?m->foc_dt_us:FOC_DEADTIME_COMP_US,&i);
    b[i++]=(!right&&mr&&m->encoder.inverted)?1:0;
    A(b,(!right&&mr)?((float)m->encoder.elec_offset_u16*360.0f/65536.0f):0.0f,&i);
    A(b,(!right&&mr)?m->encoder.electrical_ratio:(float)(mr?m->pole_pairs:(right?RIGHT_POLE_PAIRS:LEFT_POLE_PAIRS)),&i);
    b[i++]=left_encoder?VESC_FOC_SENSOR_ENCODER:VESC_FOC_SENSOR_HALL;
    A(b,mr?m->foc_pll_kp:MCCONF_FOC_PLL_KP_DEFAULT,&i);
    A(b,mr?m->foc_pll_ki:MCCONF_FOC_PLL_KI_DEFAULT,&i);
    A(b,mr?m->foc_motor_l:MCCONF_FOC_MOTOR_L_DEFAULT,&i);
    A(b,mr?m->foc_motor_ld_lq_diff:0.0f,&i);
    A(b,mr?m->foc_motor_r:MCCONF_FOC_MOTOR_R_DEFAULT,&i);
    A(b,mr?m->foc_motor_flux_linkage:MCCONF_FOC_MOTOR_FLUX_LINKAGE_DEFAULT,&i);
    A(b,mr?m->foc_observer_gain:MCCONF_FOC_OBSERVER_GAIN_DEFAULT,&i);
    A(b,mr?m->foc_observer_gain_slow:MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT,&i);
    F(b,mr?m->foc_observer_offset:MCCONF_FOC_OBSERVER_OFFSET_DEFAULT,1000,&i);
    A(b,mr?m->foc_duty_dowmramp_kp:MCCONF_FOC_DUTY_DOWNRAMP_KP_DEFAULT,&i);
    A(b,mr?m->foc_duty_dowmramp_ki:MCCONF_FOC_DUTY_DOWNRAMP_KI_DEFAULT,&i);
    F(b,mr?m->foc_start_curr_dec:MCCONF_FOC_START_CURR_DEC_DEFAULT,10000,&i);
    A(b,mr?m->foc_start_curr_dec_rpm:MCCONF_FOC_START_CURR_DEC_RPM_DEFAULT,&i);
    A(b,mr?m->foc_openloop_rpm:MCCONF_FOC_OPENLOOP_RPM_DEFAULT,&i);
    /* Exact VESC 6.00 MCCONF order. Two d-gain scaling fields exist before
       sensorless open-loop hysteresis. VESC 6.00 has one foc_sl_erpm field;
       foc_sl_erpm_start is a private runtime parameter in this F103 port. */
    F(b,mr?m->foc_openloop_rpm_low:MCCONF_FOC_OPENLOOP_RPM_LOW_DEFAULT,1000,&i);
    F(b,0.0f,1000,&i); /* foc_d_gain_scale_start: wire-only */
    F(b,0.0f,1000,&i); /* foc_d_gain_scale_max_mod: wire-only */
    F(b,mr?m->foc_sl_openloop_hyst:MCCONF_FOC_SL_OPENLOOP_HYST_DEFAULT,100,&i);
    F(b,mr?m->foc_sl_openloop_time_lock:MCCONF_FOC_SL_OPENLOOP_T_LOCK_DEFAULT,100,&i);
    F(b,mr?m->foc_sl_openloop_time_ramp:MCCONF_FOC_SL_OPENLOOP_T_RAMP_DEFAULT,100,&i);
    F(b,mr?m->foc_sl_openloop_time:MCCONF_FOC_SL_OPENLOOP_TIME_DEFAULT,100,&i);
    F(b,mr?m->foc_sl_openloop_boost_q:MCCONF_FOC_SL_OPENLOOP_BOOST_Q_DEFAULT,100,&i);
    F(b,mr?m->foc_sl_openloop_max_q:MCCONF_FOC_SL_OPENLOOP_MAX_Q_DEFAULT,100,&i);
    for(unsigned k=0;k<8;k++){b[i++]=hall[k];}
    A(b,mr?m->foc_hall_interp_erpm:500.0f,&i);
    A(b,mr?m->foc_sl_erpm:MCCONF_FOC_SL_ERPM_DEFAULT,&i);
    /* Dua-shunt stock hoverboard hanya menjalankan satu current-control sample
       per periode PWM. Jangan iklankan foc_sample_v0_v7=true ke VESC Tool. */
    b[i++]=0; /* foc_sample_v0_v7 */
    b[i++]=0; /* foc_sample_high_current */
    b[i++]=(uint8_t)(mr?m->foc_sat_comp_mode:MCCONF_FOC_SAT_COMP_MODE_DEFAULT);
    F(b,mr?m->foc_sat_comp:MCCONF_FOC_SAT_COMP_DEFAULT,1000,&i);
    b[i++]=0; /* foc_temp_comp: no NTC runtime */
    F(b,25,100,&i);
    F(b,mr?m->foc_current_filter_const:MCCONF_FOC_CURRENT_FILTER_CONST_DEFAULT,10000,&i);
    b[i++]=(uint8_t)(mr?m->foc_cc_decoupling:MCCONF_FOC_CC_DECOUPLING_DEFAULT);
    b[i++]=(uint8_t)(mr?m->foc_observer_type:MCCONF_FOC_OBSERVER_TYPE_DEFAULT); /* Ortega remains compiled default */
    F(b,0,10,&i);F(b,0,10,&i);F(b,0,10,&i);F(b,0,1000,&i);F(b,0,100,&i);A(b,0,&i);vesc_buf_append_u16(b,0,&i);A(b,0,&i);b[i++]=0;
    b[i++]=1; /* offsets calibrated on boot */ A(b,0,&i);A(b,0,&i);A(b,0,&i);
    for(unsigned k=0;k<6;k++){F(b,0,10000,&i);} b[i++]=0;b[i++]=0;A(b,0,&i);
    /* Offset 303 in the exact VESC-6.00 wire image is foc_mtpa_mode. The
       original placeholder byte already occupied this slot; do not append a
       second byte here or every following field shifts by one. */
    b[i++]=(uint8_t)(mr?m->foc_mtpa_mode:MCCONF_FOC_MTPA_MODE_DEFAULT);
    A(b,mr?m->foc_fw_current_max:MCCONF_FOC_FW_CURRENT_MAX_DEFAULT,&i);
    F(b,mr?m->foc_fw_duty_start:MCCONF_FOC_FW_DUTY_START_DEFAULT,10000,&i);
    F(b,mr?m->foc_fw_ramp_time:MCCONF_FOC_FW_RAMP_TIME_DEFAULT,1000,&i);
    F(b,mr?m->foc_fw_q_current_factor:MCCONF_FOC_FW_Q_CURRENT_FACTOR_DEFAULT,10000,&i);
    b[i++]=(uint8_t)(mr?m->foc_speed_source:MCCONF_FOC_SPEED_SOURCE_DEFAULT);
    vesc_buf_append_i16(b,0,&i);vesc_buf_append_i16(b,0,&i);F(b,0,10000,&i);A(b,0,&i);A(b,0,&i);b[i++]=5; /* 1 kHz */
    A(b,mr?m->speed_pid.kp:SPEED_PID_KP,&i);A(b,mr?m->speed_pid.ki:SPEED_PID_KI,&i);A(b,mr?m->speed_pid.kd:SPEED_PID_KD,&i);
    F(b,mr?m->speed_kd_filter:SPEED_PID_D_FILTER,10000,&i);
    A(b,mr?m->speed_pid_min_erpm:SPEED_PID_MIN_ERPM,&i);
    b[i++]=(mr?m->speed_pid_allow_braking:SPEED_PID_ALLOW_BRAKING)?1U:0U;
    A(b,mr?m->speed_pid_ramp_erpms_s:SPEED_PID_RAMP_ERPMS_S,&i);
    A(b,mr?m->position_pid.kp:POSITION_PID_KP_CURRENT_PER_DEG,&i);A(b,mr?m->position_pid.ki:POSITION_PID_KI_CURRENT_PER_DEG_S,&i);A(b,mr?m->position_pid.kd:POSITION_PID_KD_CURRENT_PER_DEGPS,&i);
    A(b,mr?m->position_kd_proc:POSITION_PID_KD_PROC,&i);
    F(b,mr?m->position_kd_filter:POSITION_PID_D_FILTER,10000,&i);
    A(b,mr?m->position_ang_div:POSITION_PID_ANG_DIV,&i);
    F(b,mr?m->position_gain_dec_angle:POSITION_PID_GAIN_DEC_ANGLE,10,&i);
    A(b,mr?m->position_offset_deg:POSITION_PID_OFFSET_DEG,&i);
    F(b,0,10000,&i); /* BLDC-only upstream field; preserved for VESC6 ABI */
    A(b,mr?m->cc_min_current:CURRENT_CTRL_MIN_CURRENT_A,&i);
    A(b,1,&i);F(b,0.01f,10000,&i);vesc_buf_append_i32(b,500,&i);F(b,0.02f,10000,&i);A(b,1,&i);
    vesc_buf_append_u32(b,(!right&&mr)?m->encoder.cpr:(!right?LEFT_ENCODER_CPR:0U),&i); for(unsigned k=0;k<6;k++)F(b,0,1000,&i);
    b[i++]=left_encoder?VESC_SENSOR_PORT_ABI:VESC_SENSOR_PORT_HALL; b[i++]=(mr&&m->invert_direction)?1:0; b[i++]=0;b[i++]=0;
    A(b,PWM_FREQUENCY_HZ,&i);A(b,PWM_FREQUENCY_HZ,&i);A(b,PWM_FREQUENCY_HZ,&i);A(b,3435,&i);b[i++]=0;b[i++]=8;A(b,1,&i);F(b,10000,0.1f,&i);F(b,25,10,&i);b[i++]=0;b[i++]=8;
    b[i++]=(uint8_t)((mr?m->pole_pairs:(right?RIGHT_POLE_PAIRS:LEFT_POLE_PAIRS))*2U);
    A(b,mr?m->si_gear_ratio:1.0f,&i);A(b,mr?m->si_wheel_diameter:0.1f,&i);
    b[i++]=mr?m->si_battery_type:0U;b[i++]=mr?m->si_battery_cells:10U;
    A(b,mr?m->si_battery_ah:10.0f,&i);A(b,mr?m->si_motor_nl_current:1.0f,&i);
    b[i++]=0;b[i++]=0;F(b,60,100,&i);F(b,80,100,&i);F(b,0.8f,1000,&i);F(b,0.9f,1000,&i);b[i++]=0;
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
    /* Preserve the live incremental coordinate across unrelated MCCONF writes.
       VESC Tool commonly writes the complete 481-byte image when only one
       limit/PID field changed; that must not silently rebase TIM4. */
    const bool encoder_was_active = (id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER);
    const uint8_t old_pole_pairs = m->pole_pairs;
    const bool old_invert_direction = m->invert_direction;
    const mc_foc_sensor_mode old_foc_sensor_mode = m->foc_sensor_mode;
    const uint32_t encoder_old_cpr = m->encoder.cpr;
    const bool encoder_old_inverted = m->encoder.inverted;
    const uint16_t encoder_old_offset_u16 = m->encoder.elec_offset_u16;
    const float encoder_old_ratio = m->encoder.electrical_ratio;
    /* Port F103 ini FOC-only. The complete VESC MCCONF wire schema is preserved,
       but BLDC/DC and HFI execution are deliberately rejected instead of
       silently mapping them to a different algorithm. */
    if (w[4] > PWM_MODE_BIPOLAR || w[5] > COMM_MODE_DELAY) return false;
    if (w[6] != VESC_MOTOR_FOC) return false;
    if (w[152] >= FOC_SENSOR_MODE_HFI && w[152] <= FOC_SENSOR_MODE_HFI_V5) return false;
    /* Part-1 sensor policy follows the VESC FOC model while respecting this
       board's physical inputs: LEFT may use sensorless, Hall or incremental
       A/B; RIGHT may use sensorless or Hall only. HFI stays rejected above. */
    if (id == MOTOR_LEFT) {
        if (w[152] != FOC_SENSOR_MODE_SENSORLESS &&
            w[152] != VESC_FOC_SENSOR_ENCODER &&
            w[152] != VESC_FOC_SENSOR_HALL) return false;
    } else {
        if (w[152] != FOC_SENSOR_MODE_SENSORLESS &&
            w[152] != VESC_FOC_SENSOR_HALL) return false;
    }
    if (!runtime_mc_auto_fields_finite(w) || !runtime_mc_supported_ranges_valid(w)) return false;
    m->pwm_mode=(mc_pwm_mode)w[4];
    m->comm_mode=(mc_comm_mode)w[5];
    m->motor_type=MOTOR_TYPE_FOC;
    (void)w[7]; /* BLDC sensor byte is wire-only in this FOC-only build. */
    const float current_max=get_auto_at(w,VESC6_MC_OFF_L_CURRENT_MAX);
    const float current_min=get_auto_at(w,VESC6_MC_OFF_L_CURRENT_MIN);
    const float input_current_max=get_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MAX);
    const float input_current_min=get_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MIN);
    const float abs_current=get_auto_at(w,VESC6_MC_OFF_L_ABS_CURRENT_MAX);
    const float min_erpm=get_auto_at(w,VESC6_MC_OFF_L_MIN_ERPM);
    const float max_erpm=get_auto_at(w,VESC6_MC_OFF_L_MAX_ERPM);
    const float erpm_start=get_f16_at(w,VESC6_MC_OFF_L_ERPM_START,10000.0f);
    const float min_vin=get_auto_at(w,VESC6_MC_OFF_L_MIN_VIN);
    const float max_vin=get_auto_at(w,VESC6_MC_OFF_L_MAX_VIN);
    const float battery_cut_start=get_auto_at(w,VESC6_MC_OFF_L_BAT_CUT_START);
    const float battery_cut_end=get_auto_at(w,VESC6_MC_OFF_L_BAT_CUT_END);
    const float temp_fet_start=get_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_START,10.0f);
    const float temp_fet_end=get_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_END,10.0f);
    const float temp_motor_start=get_f16_at(w,VESC6_MC_OFF_L_TEMP_MOTOR_START,10.0f);
    const float temp_motor_end=get_f16_at(w,VESC6_MC_OFF_L_TEMP_MOTOR_END,10.0f);
    const float temp_accel_dec=get_f16_at(w,VESC6_MC_OFF_L_TEMP_ACCEL_DEC,10000.0f);
    const float start_curr_dec=get_f16_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC,10000.0f);
    const float start_curr_dec_rpm=get_auto_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC_RPM);
    int32_t duty_i=(int32_t)VESC6_MC_OFF_L_MIN_DUTY;
    const float vesc_min_duty=vesc_buf_get_float16(w,10000.0f,&duty_i);
    duty_i=(int32_t)VESC6_MC_OFF_L_MAX_DUTY;
    const float max_duty=vesc_buf_get_float16(w,10000.0f,&duty_i);
    const float watt_max=get_auto_at(w,VESC6_MC_OFF_L_WATT_MAX);
    const float watt_min=get_auto_at(w,VESC6_MC_OFF_L_WATT_MIN);
    const float current_max_scale=get_f16_at(w,VESC6_MC_OFF_L_CURRENT_MAX_SCALE,10000.0f);
    const float current_min_scale=get_f16_at(w,VESC6_MC_OFF_L_CURRENT_MIN_SCALE,10000.0f);
    const float duty_start=get_f16_at(w,VESC6_MC_OFF_L_DUTY_START,10000.0f);
    const float current_kp=get_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KP);
    const float current_ki=get_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KI);
    float si_gear_ratio=get_auto_at(w,VESC6_MC_OFF_SI_GEAR_RATIO);
    float si_wheel_diameter=get_auto_at(w,VESC6_MC_OFF_SI_WHEEL_DIAMETER);
    float si_battery_ah=get_auto_at(w,VESC6_MC_OFF_SI_BATTERY_AH);
    float si_motor_nl_current=get_auto_at(w,VESC6_MC_OFF_SI_MOTOR_NL_CURRENT);
    if(!isfinite(current_max)||!isfinite(current_min)||
       !isfinite(input_current_max)||!isfinite(input_current_min)||
       !isfinite(abs_current)||!isfinite(min_erpm)||!isfinite(max_erpm)||!isfinite(erpm_start)||
       !isfinite(min_vin)||!isfinite(max_vin)||
       !isfinite(battery_cut_start)||!isfinite(battery_cut_end)||
       !isfinite(temp_fet_start)||!isfinite(temp_fet_end)||!isfinite(temp_motor_start)||!isfinite(temp_motor_end)||!isfinite(temp_accel_dec)||
       !isfinite(start_curr_dec)||!isfinite(start_curr_dec_rpm)||
       !isfinite(max_duty)||!isfinite(vesc_min_duty)||
       !isfinite(watt_max)||!isfinite(watt_min)||!isfinite(current_max_scale)||
       !isfinite(current_min_scale)||!isfinite(duty_start)||
       !isfinite(current_kp)||!isfinite(current_ki)||
       !isfinite(si_gear_ratio)||!isfinite(si_wheel_diameter)||
       !isfinite(si_battery_ah)||!isfinite(si_motor_nl_current)) return false;

    /* Do not reinterpret malformed VESC limits. The canonical wire image and
       the runtime must agree on direction/sign; only magnitude clipping to the
       physical F103 power-stage envelope is allowed afterwards. */
    if(current_max < 0.1f || current_min > 0.0f ||
       input_current_max < 0.0f || input_current_min > 0.0f ||
       abs_current < fmaxf(current_max, fabsf(current_min)) ||
       min_erpm >= max_erpm || max_erpm <= 0.0f || min_erpm >= 0.0f ||
       max_duty < 0.0f || max_duty > 1.0f ||
       vesc_min_duty < 0.0f || vesc_min_duty > max_duty) return false;
    if(max_vin<=min_vin||si_gear_ratio<=0.0f||si_wheel_diameter<=0.0f||si_battery_ah<0.0f)return false;
    uint8_t poles=w[VESC6_MC_OFF_SI_MOTOR_POLES]; if(poles<2U||(poles&1U)||poles>120U)return false;
    uint8_t pp=poles/2U;
    /* The preflight above has already guaranteed that accepted VESC values fit
       the physical F103 envelope. The clamp calls below are defensive bounds,
       not a hidden reinterpretation of an ACKed configuration. */
    m->current_max_a=clampf(current_max,0.1f,FOC_MAX_CURRENT_A);
    m->current_min_a=clampf(current_min,-FOC_MAX_CURRENT_A,0.0f);
    m->input_current_max_a=clampf(input_current_max,0.0f,FOC_MAX_CURRENT_A);
    m->input_current_min_a=clampf(input_current_min,-FOC_MAX_CURRENT_A,0.0f);
    m->abs_current_max_a=clampf(abs_current,m->current_max_a,FOC_ABS_CURRENT_TRIP_A);
    m->abs_current_trip_q15=amp_to_q15(m->abs_current_max_a);
    m->min_vin=clampf(min_vin,VBUS_MIN_RUN_V,VBUS_MAX_RUN_V-0.5f);
    m->max_vin=clampf(max_vin,m->min_vin+0.5f,VBUS_MAX_RUN_V);
    m->battery_cut_start=battery_cut_start;m->battery_cut_end=battery_cut_end;
    m->slow_abs_current=w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT]!=0U;
    m->temp_fet_start=temp_fet_start; m->temp_fet_end=temp_fet_end;
    m->temp_motor_start=temp_motor_start; m->temp_motor_end=temp_motor_end;
    m->temp_accel_dec=temp_accel_dec;
    m->additional_faults=MCCONF_L_ADDITIONAL_FAULTS_DEFAULT;
    m->foc_start_curr_dec=start_curr_dec; m->foc_start_curr_dec_rpm=start_curr_dec_rpm;
    m->battery_regen_cut_start=m->max_vin-MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    m->battery_regen_cut_end=m->max_vin-MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    m->min_vin_q15=volt_to_q15(m->min_vin);m->max_vin_q15=volt_to_q15(m->max_vin);
    m->hard_max_vin_q15=volt_to_q15(fminf(m->max_vin+FOC_VBUS_HARD_OV_MARGIN_V,FOC_VBUS_HARD_MAX_V));
    m->hard_min_vin_q15=volt_to_q15(fmaxf(m->min_vin-FOC_VBUS_HARD_UV_MARGIN_V,FOC_VBUS_HARD_MIN_V));
    m->over_voltage_fault_count=0U;m->under_voltage_fault_count=0U;
    m->min_erpm=clampf(min_erpm,-MOTOR_DEFAULT_MAX_ERPM,-1.0f);m->max_erpm=clampf(max_erpm,1.0f,MOTOR_DEFAULT_MAX_ERPM);m->erpm_start=clampf(erpm_start,0.0f,1.0f);
    m->max_duty=clampf(max_duty,0.01f,0.98f);
    m->min_duty=clampf(vesc_min_duty,0.0f,m->max_duty);
    m->watt_max=watt_max; m->watt_min=watt_min;
    m->current_max_scale=current_max_scale; m->current_min_scale=current_min_scale;
    m->duty_start=duty_start;
    m->current_kp=clampf(current_kp,0.00001f,10.0f);m->current_ki=clampf(current_ki,0.0f,200000.0f);
    m->current_kp_q16=current_gain_to_fast_q16(m->current_kp);m->current_ki_dt_q16=current_ki_to_fast_q16(m->current_ki);
    m->foc_pll_kp=clampf(get_auto_at(w,153),0.0f,100000.0f);
    m->foc_pll_ki=clampf(get_auto_at(w,157),0.0f,1000000.0f);
    m->foc_motor_l=clampf(get_auto_at(w,161),1.0e-7f,0.1f);
    m->foc_motor_ld_lq_diff=clampf(get_auto_at(w,165),-0.1f,0.1f);
    m->foc_motor_r=clampf(get_auto_at(w,169),1.0e-5f,100.0f);
    m->foc_motor_flux_linkage=clampf(get_auto_at(w,173),1.0e-6f,FOC_FLUX_Q_BASE_WB*1.90f);
    m->foc_observer_gain=clampf(get_auto_at(w,177),0.0f,1000000.0f);
    m->foc_observer_gain_slow=clampf(get_auto_at(w,181),0.0f,1.0f);
    m->foc_observer_offset=clampf(get_f16_at(w,VESC6_MC_OFF_FOC_OBSERVER_OFFSET,1000.0f),-10.0f,10.0f);
    m->foc_duty_dowmramp_kp=clampf(get_auto_at(w,VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP),0.0f,100000.0f);
    m->foc_duty_dowmramp_ki=clampf(get_auto_at(w,VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI),0.0f,1000000.0f);
    m->foc_current_filter_const=clampf(get_f16_at(w,VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST,10000.0f),0.0f,1.0f);
    m->foc_cc_decoupling=(mc_foc_cc_decoupling_mode)w[VESC6_MC_OFF_FOC_CC_DECOUPLING];
    m->foc_sat_comp_mode=(SAT_COMP_MODE)w[VESC6_MC_OFF_FOC_SAT_COMP_MODE];
    m->foc_sat_comp=clampf(get_f16_at(w,VESC6_MC_OFF_FOC_SAT_COMP,1000.0f),0.0f,1.0f);
    m->foc_observer_type=(mc_foc_observer_type)w[VESC6_MC_OFF_FOC_OBSERVER_TYPE];
    m->foc_mtpa_mode=(MTPA_MODE)w[VESC6_MC_OFF_FOC_MTPA_MODE];
    m->foc_fw_current_max=clampf(get_auto_at(w,VESC6_MC_OFF_FOC_FW_CURRENT_MAX),0.0f,FOC_MAX_CURRENT_A);
    m->foc_fw_duty_start=clampf(get_f16_at(w,VESC6_MC_OFF_FOC_FW_DUTY_START,10000.0f),0.0f,1.0f);
    m->foc_fw_ramp_time=clampf(get_f16_at(w,VESC6_MC_OFF_FOC_FW_RAMP_TIME,1000.0f),0.01f,30.0f);
    m->foc_fw_q_current_factor=clampf(get_f16_at(w,VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR,10000.0f),0.0f,1.0f);
    m->foc_speed_source=(FOC_SPEED_SRC)w[VESC6_MC_OFF_FOC_SPEED_SOURCE];
    m->foc_fw_backoff=MCCONF_FOC_FW_BACKOFF_DEFAULT;
    m->foc_mag_vd_max=MCCONF_FOC_MAG_VD_MAX_DEFAULT;
    m->foc_overmod_factor=MCCONF_FOC_OVERMOD_FACTOR_DEFAULT;
    m->foc_temp_comp=MCCONF_FOC_TEMP_COMP_DEFAULT;
    m->foc_temp_comp_base_temp=MCCONF_FOC_TEMP_COMP_BASE_TEMP_DEFAULT;
    m->foc_offsets_cal_mode=MCCONF_FOC_OFFSETS_CAL_MODE_DEFAULT;
    m->foc_openloop_rpm=clampf(get_auto_at(w,201),10.0f,MOTOR_DEFAULT_MAX_ERPM);
    {int32_t q=205;m->foc_openloop_rpm_low=clampf(vesc_buf_get_float16(w,1000.0f,&q),0.0f,MOTOR_DEFAULT_MAX_ERPM);}
    /* foc_d_gain_scale_start/max_mod are present in the VESC-6.00 wire image,
       but this backend does not implement that controller feature. The SET
       ownership gate keeps those bytes immutable instead of silently ignoring
       a user change. */
    {int32_t q=211;m->foc_sl_openloop_hyst=clampf(vesc_buf_get_float16(w,100.0f,&q),0.0f,100.0f);}
    {int32_t q=213;m->foc_sl_openloop_time_lock=clampf(vesc_buf_get_float16(w,100.0f,&q),0.0f,20.0f);}
    {int32_t q=215;m->foc_sl_openloop_time_ramp=clampf(vesc_buf_get_float16(w,100.0f,&q),0.01f,20.0f);}
    {int32_t q=217;m->foc_sl_openloop_time=clampf(vesc_buf_get_float16(w,100.0f,&q),0.01f,20.0f);}
    {int32_t q=219;m->foc_sl_openloop_boost_q=clampf(vesc_buf_get_float16(w,100.0f,&q),0.0f,FOC_MAX_CURRENT_A);}
    {int32_t q=221;m->foc_sl_openloop_max_q=clampf(vesc_buf_get_float16(w,100.0f,&q),0.1f,FOC_MAX_CURRENT_A);}
    m->foc_hall_interp_erpm=clampf(get_auto_at(w,231),0.0f,MOTOR_DEFAULT_MAX_ERPM);
    m->foc_hall_interp_erpm_u32=(uint32_t)lrintf(m->foc_hall_interp_erpm);
    m->foc_sl_erpm=clampf(get_auto_at(w,235),10.0f,MOTOR_DEFAULT_MAX_ERPM);
    /* Private startup-validity threshold. It must not consume a VESC 6.00
       wire field, so derive it conservatively from the configured handover. */
    m->foc_sl_erpm_start=clampf(MCCONF_FOC_SL_ERPM_START_DEFAULT,10.0f,m->foc_sl_erpm);
    m->speed_pid.kp=clampf(get_auto_at(w,330),0.0f,1000.0f);m->speed_pid.ki=clampf(get_auto_at(w,334),0.0f,1000.0f);m->speed_pid.kd=clampf(get_auto_at(w,338),0.0f,1000.0f);
    {int32_t q=342;m->speed_kd_filter=clampf(vesc_buf_get_float16(w,10000.0f,&q),0.0f,1.0f);}
    m->speed_pid_min_erpm=clampf(get_auto_at(w,344),0.0f,MOTOR_DEFAULT_MAX_ERPM);
    m->speed_pid_allow_braking=w[348U]!=0U;
    m->speed_pid_ramp_erpms_s=clampf(get_auto_at(w,349),0.0f,1000000.0f);
    m->speed_pid_source=S_PID_SPEED_SRC_PLL; /* VESC6 wire has no speed-source field. */
    m->position_pid.kp=clampf(get_auto_at(w,353),0.0f,1000.0f);m->position_pid.ki=clampf(get_auto_at(w,357),0.0f,1000.0f);m->position_pid.kd=clampf(get_auto_at(w,361),0.0f,1000.0f);
    m->position_kd_proc=clampf(get_auto_at(w,365),0.0f,1000.0f);
    {int32_t q=369;m->position_kd_filter=clampf(vesc_buf_get_float16(w,10000.0f,&q),0.0f,1.0f);}
    m->position_ang_div=clampf(get_auto_at(w,371),0.01f,1000.0f);
    {int32_t q=375;m->position_gain_dec_angle=clampf(vesc_buf_get_float16(w,10.0f,&q),0.0f,3600.0f);}
    m->position_offset_deg=clampf(get_auto_at(w,377),-36000.0f,36000.0f);
    m->cc_min_current=clampf(get_auto_at(w,383),0.0f,FOC_MAX_CURRENT_A);
    m->pole_pairs=pp; m->invert_direction=w[VESC6_MC_OFF_M_INVERT_DIRECTION]!=0U;
    m->si_gear_ratio=clampf(si_gear_ratio,0.01f,1000.0f);
    m->si_wheel_diameter=clampf(si_wheel_diameter,0.001f,10.0f);
    m->si_battery_type=w[VESC6_MC_OFF_SI_BATTERY_TYPE];
    m->si_battery_cells=w[VESC6_MC_OFF_SI_BATTERY_CELLS];
    m->si_battery_ah=clampf(si_battery_ah,0.0f,10000.0f);
    m->si_motor_nl_current=clampf(si_motor_nl_current,0.0f,FOC_MAX_CURRENT_A);
    for(unsigned k=0;k<8;k++){m->foc_hall_table[k]=w[223+k]; if(w[223+k]==255U){m->hall_table[k]=-1;m->hall_angle_u16[k]=0;} else {m->hall_angle_u16[k]=(uint16_t)(((uint32_t)w[223+k]*65536U)/200U);m->hall_table[k]=(int8_t)(((uint32_t)w[223+k]*6U)/200U);}}
    if(id==MOTOR_LEFT){
        const bool enc=(w[VESC6_MC_OFF_FOC_SENSOR_MODE]==VESC_FOC_SENSOR_ENCODER);
        const uint32_t cpr=get_u32_at(w,VESC6_MC_OFF_M_ENCODER_COUNTS);
        if (enc && (cpr < 4U || cpr > 65535U)) return false;
        if (cpr >= 4U && cpr <= 65535U) m->encoder.cpr=cpr;

        const bool encoder_new_inverted=w[VESC6_MC_OFF_FOC_ENCODER_INVERTED]!=0U;
        float off=get_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_OFFSET);
        while (off < 0.0f) off += 360.0f;
        while (off >= 360.0f) off -= 360.0f;
        const uint16_t encoder_new_offset_u16=(uint16_t)lrintf(off*(65536.0f/360.0f));
        float ratio=get_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_RATIO);
        if(!isfinite(ratio)||ratio<=0.0f||ratio>1000.0f)return false;
        uint64_t rq=(uint64_t)llrintf(ratio*65536.0f);
        uint64_t step=(rq<<16)/m->encoder.cpr;
        if(rq==0U||rq>0xFFFFFFFFULL||step>0xFFFFFFFFULL)return false;

        const bool encoder_hw_changed = enc && (!encoder_was_active || encoder_old_cpr != m->encoder.cpr);
        const bool encoder_phase_changed = enc &&
            (encoder_old_inverted != encoder_new_inverted ||
             encoder_old_offset_u16 != encoder_new_offset_u16 ||
             fabsf(encoder_old_ratio - ratio) > 1.0e-6f);

        m->encoder.inverted=encoder_new_inverted;
        m->encoder.elec_offset_u16=encoder_new_offset_u16;
        m->encoder.electrical_ratio=ratio;
        m->encoder.electrical_ratio_q16=(uint32_t)rq;
        m->encoder.phase_per_count_q16=(uint32_t)step;
        /* VESC 6.00 wire encoder enum is 1. Map it to the internal A/B-only
           runtime strategy used by this board without changing the wire ABI. */
        if (enc && w[VESC6_MC_OFF_M_SENSOR_PORT_MODE] != VESC_SENSOR_PORT_ABI) return false;
        m->foc_sensor_mode = enc ? FOC_SENSOR_MODE_ENCODER_AB : (mc_foc_sensor_mode)w[VESC6_MC_OFF_FOC_SENSOR_MODE];
        m->sensor_request_mode=enc?SENSOR_MODE_ENCODER:SENSOR_MODE_HALL;
        if(enc) {
            if (encoder_hw_changed) {
                /* Switching to ABI or changing CPR genuinely changes the TIM4
                   hardware coordinate and therefore requires re-init. */
                (void)encoder_init(m);
            } else {
                /* Keep TIM4 turns/extended/session-zero untouched for ordinary
                   full-image VESC Tool writes. */
                m->sensor_mode=SENSOR_MODE_ENCODER;
            }
            if (encoder_hw_changed || encoder_phase_changed) {
                m->encoder.synced=false;
                m->encoder.motion_proved=false;
                m->encoder.sync_active=false;
                m->encoder.speed_sample_valid=false;
                m->using_encoder=false;
            }
        } else {
            encoder_deinit(m);
            motor_hw_configure_sensor(m,SENSOR_MODE_HALL);
            if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL) motor_hall_edge_isr(m);
        }
    } else {
        if (w[152] == VESC_FOC_SENSOR_ENCODER) return false;
        m->foc_sensor_mode = (mc_foc_sensor_mode)w[152];
        m->sensor_request_mode=SENSOR_MODE_HALL;
        motor_hw_configure_sensor(m,SENSOR_MODE_HALL);
        if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL) motor_hall_edge_isr(m);
    }
    if (old_pole_pairs != m->pole_pairs || old_invert_direction != m->invert_direction ||
        old_foc_sensor_mode != m->foc_sensor_mode ||
        (id == MOTOR_LEFT && encoder_old_cpr != m->encoder.cpr)) {
        m->stats.tachometer_source_valid=false;
    }

    /* R/L/flux, decoupling, filter and observer-offset changes alter fixed
       coefficients immediately. SET_MCCONF already enforces motor-stopped. */
#ifndef VESC_CONFIG_UNIT_TEST
    foc_precalc_values(m);
    foc_observer_reset(m, m->observer_phase_u16);
#endif
    return true;
}

static bool app_adc_wire_valid(const uint8_t *w) {
    const uint8_t ctrl = w[VESC6_APP_OFF_ADC_CTRL_TYPE];
    if (ctrl != ADC_CTRL_TYPE_NONE && ctrl != ADC_CTRL_TYPE_CURRENT &&
        ctrl != ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC &&
        ctrl != ADC_CTRL_TYPE_DUTY && ctrl != ADC_CTRL_TYPE_PID) return false;

    const float hyst = get_auto_at(w, VESC6_APP_OFF_ADC_HYST);
    const float vs = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_START, 1000.0f);
    const float ve = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_END, 1000.0f);
    const float vmin = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_MIN, 1000.0f);
    const float vmax = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_MAX, 1000.0f);
    const float vc = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_CENTER, 1000.0f);
    const float v2s = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE2_START, 1000.0f);
    const float v2e = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE2_END, 1000.0f);
    const float expa = get_auto_at(w, VESC6_APP_OFF_ADC_THROTTLE_EXP);
    const float expb = get_auto_at(w, VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE);
    const float rpos = get_auto_at(w, VESC6_APP_OFF_ADC_RAMP_TIME_POS);
    const float rneg = get_auto_at(w, VESC6_APP_OFF_ADC_RAMP_TIME_NEG);
    const float tcd = get_auto_at(w, VESC6_APP_OFF_ADC_TC_MAX_DIFF);
    const uint16_t rate = get_u16_at(w, VESC6_APP_OFF_ADC_UPDATE_RATE_HZ);

    if (!isfinite(hyst) || !isfinite(expa) || !isfinite(expb) ||
        !isfinite(rpos) || !isfinite(rneg) || !isfinite(tcd)) return false;
    if (hyst < 0.0f || hyst > 0.30f) return false;
    if (vmin < 0.0f || vmax > 3.30f || vmin > vmax) return false;
    if (vs < vmin || ve > vmax || ve - vs < 0.05f) return false;
    if (vc < 0.0f || vc > 3.30f || v2s < 0.0f || v2e > 3.30f || v2e - v2s < 0.05f) return false;
    if (w[VESC6_APP_OFF_ADC_USE_FILTER] > 1U ||
        w[VESC6_APP_OFF_ADC_SAFE_START] > SAFE_START_NO_FAULT ||
        w[VESC6_APP_OFF_ADC_VOLTAGE_INVERTED] > 1U ||
        w[VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED] > 1U ||
        w[VESC6_APP_OFF_ADC_MULTI_ESC] > 1U || w[VESC6_APP_OFF_ADC_TC] > 1U) return false;
    /* No external reverse/cruise buttons and no traction-control backend on
       this reduced board. Reject rather than fake support. */
    if (w[VESC6_APP_OFF_ADC_BUTTONS] != 0U || w[VESC6_APP_OFF_ADC_TC] != 0U || fabsf(tcd) > 1.0e-6f) return false;
    if (w[VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE] > THR_EXP_POLY) return false;
    if (expa < -1.0f || expa > 1.0f || expb < -1.0f || expb > 1.0f) return false;
    if (rpos < 0.0f || rpos > 20.0f || rneg < 0.0f || rneg > 20.0f) return false;
    if (rate < 10U || rate > 1000U) return false;
    return true;
}

static bool apply_app(const uint8_t *w) {
    if(!sig_ok(w,VESC6_APPCONF_SIGNATURE))return false;
    uint32_t timeout=get_u32_at(w,VESC6_APP_OFF_TIMEOUT_MSEC);
    float brake=get_auto_at(w,VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT);
    if(timeout>600000U||!isfinite(brake))return false;

    if(w[VESC6_APP_OFF_CONTROLLER_ID]!=VESC_CONTROLLER_ID_LEFT)return false;
    const uint8_t app=w[VESC6_APP_OFF_APP_TO_USE];
    if(app!=VESC_APP_NONE && app!=VESC_APP_ADC && app!=VESC_APP_UART && app!=VESC_APP_ADC_UART)return false;
    if(!app_adc_wire_valid(w))return false;

    /* USART3 PB10/PB11 is permanent management transport even when APP_ADC is
       selected, so its VESC-6 baud field must describe the real hardware. */
    if(get_u32_at(w,VESC6_APP_OFF_UART_BAUD)!=VESC_UART_BAUD)return false;

    timeout_configure(timeout,brake);
    app_notify_configuration_changed();
    return true;
}

/* Portable range validation mirroring VESC commands_apply_mcconf_hw_limits
   (the portion that needs no board-specific HW_LIM_* macros). This port
   forces foc_overmod_factor and foc_sl_erpm_start to fixed defaults in
   apply_mc(), so clamping them here is a harmless no-op; the three
   l_*_scale / l_erpm_start fields ARE carried on the VESC-6.00 wire image
   and are clamped into s_mc_active[] by the call site below. The hardware
   limit clamping half of the VESC function is covered by the apply_mc()
   preflight, which rejects anything outside the F103 envelope. */
void vesc_config_apply_mcconf_hw_limits(mc_configuration *mcconf) {
    if (mcconf == NULL) return;
    utils_truncate_number(&mcconf->l_current_max_scale, 0.0f, 1.0f);
    utils_truncate_number(&mcconf->l_current_min_scale, 0.0f, 1.0f);
    utils_truncate_number(&mcconf->l_erpm_start, 0.0f, 1.0f);
    utils_truncate_number(&mcconf->foc_overmod_factor, 1.0f, 1.5f);
    utils_truncate_number_abs(&mcconf->foc_sl_erpm_start, mcconf->foc_sl_erpm * 0.9f);
}

bool vesc_config_set_mc_wire(motor_id_t id,const uint8_t *wire,uint16_t len,bool store){
    vesc_config_init_defaults(); if(!wire||len!=VESC6_MCCONF_WIRE_SIZE||!sig_ok(wire,VESC6_MCCONF_SIGNATURE))return false;
    if(id!=MOTOR_LEFT && id!=MOTOR_RIGHT) return false;
    if(!unsupported_mc_bytes_unchanged(id,wire)) return false;
    /* Same-value writes are idempotent. In particular, do not revoke the
       LEFT no-index encoder alignment just because VESC Tool writes back the
       configuration it has just read. */
    if(memcmp(wire,s_mc_active[id],VESC6_MCCONF_WIRE_SIZE)==0){return !store||conf_general_store_mc_wire_persistent(id,s_mc_active[id]);}
    MotorRuntime *m=motor_get(id); if(m->pwm_enabled||m->detect.busy)return false; motor_stop(m);
    memcpy(s_rollback_mc[id],s_mc_active[id],sizeof(s_rollback_mc[id]));memcpy(s_mc_active[id],wire,len);
    /* Portable range validation (VESC commands_apply_mcconf_hw_limits, the
       HW_LIM_*,-free part). The wire image carries l_current_max_scale /
       l_current_min_scale / l_erpm_start as floats; decode, clamp the 5
       fields, then write the wire-backed ones back so apply_mc() sees the
       truncated values. foc_overmod_factor / foc_sl_erpm_start are
       forced to defaults later in apply_mc(), so their clamp is cosmetic. */
    {
        mc_configuration mc_trunc;
        mcconf_decode_wire(s_mc_active[id], &mc_trunc);
        vesc_config_apply_mcconf_hw_limits(&mc_trunc);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_CURRENT_MAX_SCALE, mc_trunc.l_current_max_scale, 10000.0f);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_CURRENT_MIN_SCALE, mc_trunc.l_current_min_scale, 10000.0f);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_ERPM_START,        mc_trunc.l_erpm_start,        10000.0f);
    }
    if(!apply_mc(id,s_mc_active[id])){memcpy(s_mc_active[id],s_rollback_mc[id],sizeof(s_rollback_mc[id]));(void)apply_mc(id,s_rollback_mc[id]);return false;}
    /* Keep the accepted VESC Tool wire image byte-exact. Preflight guarantees
       that every writable value is executable by this backend, so GET_MCCONF
       round-trips what SET_MCCONF ACKed without hidden runtime clamping. */
    if(store&&!conf_general_store_mc_wire_persistent(id,s_mc_active[id])){memcpy(s_mc_active[id],s_rollback_mc[id],sizeof(s_rollback_mc[id]));(void)apply_mc(id,s_rollback_mc[id]);return false;} return true;
}
bool vesc_config_set_app_wire(const uint8_t *wire,uint16_t len,bool store){
    vesc_config_init_defaults();if(!wire||len!=VESC6_APPCONF_WIRE_SIZE||!sig_ok(wire,VESC6_APPCONF_SIGNATURE))return false;
    if(!unsupported_app_bytes_unchanged(wire)) return false;
    if(memcmp(wire,s_app_active,VESC6_APPCONF_WIRE_SIZE)==0){return !store||conf_general_store_app_wire_persistent(s_app_active);}
    /* Changing throttle calibration/control type while either bridge is live can
       create an instantaneous command discontinuity. Require both local motors
       OFF, exactly as MCCONF writes already do for motor-critical parameters. */
    MotorRuntime *ml=motor_get(MOTOR_LEFT), *mr=motor_get(MOTOR_RIGHT);
    if((ml && (ml->pwm_enabled || ml->detect.busy)) ||
       (mr && (mr->pwm_enabled || mr->detect.busy))) return false;
    memcpy(s_rollback_app,s_app_active,sizeof(s_rollback_app));memcpy(s_app_active,wire,len);
    if(!apply_app(s_app_active)){memcpy(s_app_active,s_rollback_app,sizeof(s_rollback_app));(void)apply_app(s_rollback_app);return false;}
    if(store&&!conf_general_store_app_wire_persistent(s_app_active)){memcpy(s_app_active,s_rollback_app,sizeof(s_rollback_app));(void)apply_app(s_rollback_app);return false;}return true;
}

void vesc_config_sync_motor_runtime(motor_id_t id){
    vesc_config_init_defaults();
    MotorRuntime *m=motor_get(id);
    uint8_t *w=s_mc_active[id];

    /* IMPORTANT OWNERSHIP RULE
     * ------------------------
     * s_mc_active[] is the exact VESC-6.00 wire image and therefore the
     * source of truth for GET_MCCONF and flash persistence. Accepted writable
     * values are preflighted against the F103 envelope; defensive runtime
     * clamps must therefore never become a second hidden configuration. This
     * function is only called after a detection or an
     * explicit sensor-selection operation, so update only fields that those
     * operations genuinely own/change.
     *
     * In particular preserve current/input-current limits, VIN/battery
     * limits, duty/ERPM limits, SI wheel/battery data and every unsupported
     * VESC UI field byte-for-byte. */

    /* Detect-apply owns FOC motor parameters/current gains. */
    put_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KP,m->current_kp);
    put_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KI,m->current_ki);
    put_auto_at(w,161,m->foc_motor_l);
    put_auto_at(w,165,m->foc_motor_ld_lq_diff);
    put_auto_at(w,169,m->foc_motor_r);
    put_auto_at(w,173,m->foc_motor_flux_linkage);

    /* COMM_DETECT_APPLY_ALL_FOC can also update these sensorless thresholds. */
    put_auto_at(w,201,m->foc_openloop_rpm);
    put_f16_at(w,205,m->foc_openloop_rpm_low,1000.0f);
    put_auto_at(w,VESC6_MC_OFF_FOC_SL_ERPM,m->foc_sl_erpm);

    /* Hall/encoder detect or explicit sensor selection owns sensor fields. */
    for(unsigned k=0;k<8;k++) w[VESC6_MC_OFF_FOC_HALL_TABLE+k]=m->foc_hall_table[k];
    put_auto_at(w,VESC6_MC_OFF_FOC_HALL_INTERP_ERPM,m->foc_hall_interp_erpm);
    if(id==MOTOR_LEFT){
        w[VESC6_MC_OFF_FOC_ENCODER_INVERTED]=m->encoder.inverted?1U:0U;
        put_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_OFFSET,
                    (float)m->encoder.elec_offset_u16*360.0f/65536.0f);
        put_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_RATIO,m->encoder.electrical_ratio);
        put_u32_at(w,VESC6_MC_OFF_M_ENCODER_COUNTS,m->encoder.cpr);
        bool enc=(m->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER_AB ||
                  m->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER ||
                  m->sensor_request_mode==SENSOR_MODE_ENCODER);
        w[VESC6_MC_OFF_FOC_SENSOR_MODE]=enc?VESC_FOC_SENSOR_ENCODER:(uint8_t)m->foc_sensor_mode;
        w[VESC6_MC_OFF_M_SENSOR_PORT_MODE]=enc?VESC_SENSOR_PORT_ABI:VESC_SENSOR_PORT_HALL;
    } else {
        w[VESC6_MC_OFF_FOC_SENSOR_MODE]=(uint8_t)m->foc_sensor_mode;
        w[VESC6_MC_OFF_M_SENSOR_PORT_MODE]=VESC_SENSOR_PORT_HALL;
    }
}

void vesc_config_sync_detect_all_runtime(motor_id_t id){
    vesc_config_init_defaults();
    MotorRuntime *m=motor_get(id);
    if(!m)return;
    /* Detect-All owns the current limits derived from max_power_loss and the
       requested input-current limits. Keep this separate from the generic
       sensor/RL sync so a Hall-only detect cannot rewrite unrelated limits. */
    uint8_t *w=s_mc_active[id];
    put_auto_at(w,VESC6_MC_OFF_L_CURRENT_MAX,m->current_max_a);
    put_auto_at(w,VESC6_MC_OFF_L_CURRENT_MIN,m->current_min_a);
    put_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MAX,m->input_current_max_a);
    put_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MIN,m->input_current_min_a);
    vesc_config_sync_motor_runtime(id);
}

static bool commit_runtime_owned_fields(motor_id_t id, bool detect_all){
    vesc_config_init_defaults();
    if(id!=MOTOR_LEFT && id!=MOTOR_RIGHT)return false;
    MotorRuntime *m=motor_get(id);
    if(!m || m->pwm_enabled || m->detect.busy)return false;
    memcpy(s_rollback_mc[id],s_mc_active[id],sizeof(s_rollback_mc[id]));
    if(detect_all)vesc_config_sync_detect_all_runtime(id);
    else vesc_config_sync_motor_runtime(id);
    /* Validate the exact candidate that will be exposed through GET_MCCONF. */
    if(!apply_mc(id,s_mc_active[id]) || !conf_general_store_mc_wire_persistent(id,s_mc_active[id])){
        memcpy(s_mc_active[id],s_rollback_mc[id],sizeof(s_rollback_mc[id]));
        (void)apply_mc(id,s_rollback_mc[id]);
        return false;
    }
    return true;
}

bool vesc_config_commit_motor_runtime(motor_id_t id){
    return commit_runtime_owned_fields(id,false);
}

bool vesc_config_commit_detect_all_runtime(motor_id_t id){
    return commit_runtime_owned_fields(id,true);
}

bool vesc_config_commit_detect_all_runtime_dual(void){
    vesc_config_init_defaults();
    MotorRuntime *l=motor_get(MOTOR_LEFT);
    MotorRuntime *r=motor_get(MOTOR_RIGHT);
    if(!l||!r||l->pwm_enabled||r->pwm_enabled||l->detect.busy||r->detect.busy)return false;

    /* s_mc_active[] is still the last committed wire image while the blocking
       detection worker mutates MotorRuntime. Snapshot it once, synthesize both
       detected candidates, validate/apply both, then write one transactional
       flash record. This gives VESC Tool Detect-All true all-or-nothing
       semantics even though motor-2 is local rather than a physical CAN node. */
    memcpy(s_rollback_mc[MOTOR_LEFT],s_mc_active[MOTOR_LEFT],sizeof(s_rollback_mc[MOTOR_LEFT]));
    memcpy(s_rollback_mc[MOTOR_RIGHT],s_mc_active[MOTOR_RIGHT],sizeof(s_rollback_mc[MOTOR_RIGHT]));
    vesc_config_sync_detect_all_runtime(MOTOR_LEFT);
    vesc_config_sync_detect_all_runtime(MOTOR_RIGHT);

    bool ok=apply_mc(MOTOR_LEFT,s_mc_active[MOTOR_LEFT]) &&
            apply_mc(MOTOR_RIGHT,s_mc_active[MOTOR_RIGHT]) &&
            conf_general_store_all();
    if(!ok){
        memcpy(s_mc_active[MOTOR_LEFT],s_rollback_mc[MOTOR_LEFT],sizeof(s_rollback_mc[MOTOR_LEFT]));
        memcpy(s_mc_active[MOTOR_RIGHT],s_rollback_mc[MOTOR_RIGHT],sizeof(s_rollback_mc[MOTOR_RIGHT]));
        (void)apply_mc(MOTOR_LEFT,s_rollback_mc[MOTOR_LEFT]);
        (void)apply_mc(MOTOR_RIGHT,s_rollback_mc[MOTOR_RIGHT]);
    }
    return ok;
}

bool vesc_config_reapply_active_mc(motor_id_t id){
    vesc_config_init_defaults();
    if(id!=MOTOR_LEFT && id!=MOTOR_RIGHT)return false;
    MotorRuntime *m=motor_get(id);
    if(!m)return false;
    motor_stop(m);
    return apply_mc(id,s_mc_active[id]);
}

void vesc_config_export_wire(uint8_t l[VESC6_MCCONF_WIRE_SIZE],uint8_t r[VESC6_MCCONF_WIRE_SIZE],uint8_t a[VESC6_APPCONF_WIRE_SIZE]){vesc_config_init_defaults();memcpy(l,s_mc_active[0],VESC6_MCCONF_WIRE_SIZE);memcpy(r,s_mc_active[1],VESC6_MCCONF_WIRE_SIZE);memcpy(a,s_app_active,VESC6_APPCONF_WIRE_SIZE);}
bool vesc_config_import_wire(const uint8_t l[VESC6_MCCONF_WIRE_SIZE],const uint8_t r[VESC6_MCCONF_WIRE_SIZE],const uint8_t a[VESC6_APPCONF_WIRE_SIZE]){
    vesc_config_init_defaults();
    if(!l||!r||!a||!sig_ok(l,VESC6_MCCONF_SIGNATURE)||!sig_ok(r,VESC6_MCCONF_SIGNATURE)||!sig_ok(a,VESC6_APPCONF_SIGNATURE))return false;
    /* Flash must not be a back door around SET ownership. A record produced by
       an older build may contain UI-only/unsupported values that this build
       cannot execute. Reject the whole record and keep compiled safe defaults
       rather than loading bytes that would be displayed as active but ignored. */
    if(!unsupported_mc_bytes_unchanged(MOTOR_LEFT,l) ||
       !unsupported_mc_bytes_unchanged(MOTOR_RIGHT,r) ||
       !unsupported_app_bytes_unchanged(a)) return false;
    memcpy(s_rollback_mc[MOTOR_LEFT],s_mc_active[MOTOR_LEFT],sizeof(s_rollback_mc[MOTOR_LEFT]));
    memcpy(s_rollback_mc[MOTOR_RIGHT],s_mc_active[MOTOR_RIGHT],sizeof(s_rollback_mc[MOTOR_RIGHT]));
    memcpy(s_rollback_app,s_app_active,sizeof(s_rollback_app));
    memcpy(s_mc_active[MOTOR_LEFT],l,VESC6_MCCONF_WIRE_SIZE);
    memcpy(s_mc_active[MOTOR_RIGHT],r,VESC6_MCCONF_WIRE_SIZE);
    memcpy(s_app_active,a,VESC6_APPCONF_WIRE_SIZE);
    /* Batch-2 used 1000/1000 as placeholder observer defaults before the
       VESC gain_slow semantics were implemented. Preserve every other MCCONF
       field on upgrade, but migrate exactly that legacy pair to the Batch-3
       VESC-style defaults instead of rejecting the whole flash record. */
    for (unsigned mi = 0U; mi < 2U; mi++) {
        float legacy_gain = get_auto_at(s_mc_active[mi], 177U);
        float legacy_slow = get_auto_at(s_mc_active[mi], 181U);
        if (fabsf(legacy_gain - 1000.0f) < 0.01f &&
            fabsf(legacy_slow - 1000.0f) < 0.01f) {
            put_auto_at(s_mc_active[mi], 177U, MCCONF_FOC_OBSERVER_GAIN_DEFAULT);
            put_auto_at(s_mc_active[mi], 181U, MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT);
        }
    }
    if(!apply_mc(MOTOR_LEFT,s_mc_active[MOTOR_LEFT])||
       !apply_mc(MOTOR_RIGHT,s_mc_active[MOTOR_RIGHT])||
       !apply_app(s_app_active)){
        memcpy(s_mc_active[MOTOR_LEFT],s_rollback_mc[MOTOR_LEFT],sizeof(s_rollback_mc[MOTOR_LEFT]));
        memcpy(s_mc_active[MOTOR_RIGHT],s_rollback_mc[MOTOR_RIGHT],sizeof(s_rollback_mc[MOTOR_RIGHT]));
        memcpy(s_app_active,s_rollback_app,sizeof(s_rollback_app));
        (void)apply_mc(MOTOR_LEFT,s_rollback_mc[MOTOR_LEFT]);
        (void)apply_mc(MOTOR_RIGHT,s_rollback_mc[MOTOR_RIGHT]);
        (void)apply_app(s_rollback_app);
        return false;
    }
    /* Persistent wire image remains the source of truth. Accepted writable
       values already passed the same runtime envelope validation used by SET. */
    return true;
}

bool vesc_config_apply_defaults(void) {
    vesc_config_init_defaults();
    if (!s_layout_ok) {
        return false;
    }
    return vesc_config_import_wire(s_mc_factory[MOTOR_LEFT],
            s_mc_factory[MOTOR_RIGHT], s_app_factory);
}

/* ==================== Canonical confgenerator front-end ====================
 * The 481/493-byte VESC-6 image remains the source of truth. These functions
 * provide upstream-style typed access without duplicating persistent state. */
static void mcconf_decode_wire(const uint8_t *w, mc_configuration *c) {
    memset(c, 0, sizeof(*c));
    c->pwm_mode=(mc_pwm_mode)w[4]; c->comm_mode=(mc_comm_mode)w[5];
    c->motor_type=(mc_motor_type)w[6]; c->sensor_mode=(mc_sensor_mode)w[7];
    c->l_current_max=get_auto_at(w,VESC6_MC_OFF_L_CURRENT_MAX);
    c->l_current_min=get_auto_at(w,VESC6_MC_OFF_L_CURRENT_MIN);
    c->l_in_current_max=get_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MAX);
    c->l_in_current_min=get_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MIN);
    c->l_abs_current_max=get_auto_at(w,VESC6_MC_OFF_L_ABS_CURRENT_MAX);
    c->l_min_erpm=get_auto_at(w,VESC6_MC_OFF_L_MIN_ERPM);
    c->l_max_erpm=get_auto_at(w,VESC6_MC_OFF_L_MAX_ERPM);
    c->l_erpm_start=get_f16_at(w,VESC6_MC_OFF_L_ERPM_START,10000.0f);
    c->l_min_vin=get_auto_at(w,VESC6_MC_OFF_L_MIN_VIN);
    c->l_max_vin=get_auto_at(w,VESC6_MC_OFF_L_MAX_VIN);
    c->l_battery_cut_start=get_auto_at(w,VESC6_MC_OFF_L_BAT_CUT_START);
    c->l_battery_cut_end=get_auto_at(w,VESC6_MC_OFF_L_BAT_CUT_END);
    c->l_slow_abs_current=w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT]!=0U;
    c->l_temp_fet_start=get_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_START,10.0f);
    c->l_temp_fet_end=get_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_END,10.0f);
    c->l_temp_motor_start=get_f16_at(w,VESC6_MC_OFF_L_TEMP_MOTOR_START,10.0f);
    c->l_temp_motor_end=get_f16_at(w,VESC6_MC_OFF_L_TEMP_MOTOR_END,10.0f);
    c->l_temp_accel_dec=get_f16_at(w,VESC6_MC_OFF_L_TEMP_ACCEL_DEC,10000.0f);
    c->l_additional_faults=MCCONF_L_ADDITIONAL_FAULTS_DEFAULT;
    c->foc_short_ls_on_zero_duty=MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT;
    c->l_min_duty=get_f16_at(w,VESC6_MC_OFF_L_MIN_DUTY,10000.0f);
    c->l_max_duty=get_f16_at(w,VESC6_MC_OFF_L_MAX_DUTY,10000.0f);
    c->l_watt_max=get_auto_at(w,VESC6_MC_OFF_L_WATT_MAX);
    c->l_watt_min=get_auto_at(w,VESC6_MC_OFF_L_WATT_MIN);
    c->l_current_max_scale=get_f16_at(w,VESC6_MC_OFF_L_CURRENT_MAX_SCALE,10000.0f);
    c->l_current_min_scale=get_f16_at(w,VESC6_MC_OFF_L_CURRENT_MIN_SCALE,10000.0f);
    c->l_duty_start=get_f16_at(w,VESC6_MC_OFF_L_DUTY_START,10000.0f);
    /* Later-VESC/runtime-only fields are explicit defaults, never fabricated
       from unrelated VESC6 bytes. */
    c->l_in_current_map_start=MCCONF_L_IN_CURRENT_MAP_START_DEFAULT; c->l_in_current_map_filter=MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT;
    c->l_battery_regen_cut_start=c->l_max_vin-MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    c->l_battery_regen_cut_end=c->l_max_vin-MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    c->lo_current_max=c->l_current_max; c->lo_current_min=c->l_current_min;
    c->lo_in_current_max=c->l_in_current_max; c->lo_in_current_min=c->l_in_current_min;

    for(unsigned k=0;k<8;k++) c->hall_table[k]=(int8_t)w[115U+k];
    c->hall_sl_erpm=get_auto_at(w,123U);
    c->foc_current_kp=get_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KP);
    c->foc_current_ki=get_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KI);
    c->foc_f_zv=get_auto_at(w,VESC6_MC_OFF_FOC_F_ZV);
    c->foc_dt_us=get_auto_at(w,VESC6_MC_OFF_FOC_DT_US);
    c->foc_encoder_inverted=w[VESC6_MC_OFF_FOC_ENCODER_INVERTED]!=0U;
    c->foc_encoder_offset=get_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_OFFSET);
    c->foc_encoder_ratio=get_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_RATIO);
    c->foc_sensor_mode=(mc_foc_sensor_mode)w[VESC6_MC_OFF_FOC_SENSOR_MODE];
    if(c->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER && w[VESC6_MC_OFF_M_SENSOR_PORT_MODE]==VESC_SENSOR_PORT_ABI)
        c->foc_sensor_mode=FOC_SENSOR_MODE_ENCODER_AB;
    c->foc_pll_kp=get_auto_at(w,153U); c->foc_pll_ki=get_auto_at(w,157U);
    c->foc_motor_l=get_auto_at(w,161U); c->foc_motor_ld_lq_diff=get_auto_at(w,165U);
    c->foc_motor_r=get_auto_at(w,169U); c->foc_motor_flux_linkage=get_auto_at(w,173U);
    c->foc_observer_gain=get_auto_at(w,177U); c->foc_observer_gain_slow=get_auto_at(w,181U);
    c->foc_observer_offset=get_f16_at(w,VESC6_MC_OFF_FOC_OBSERVER_OFFSET,1000.0f);
    c->foc_duty_dowmramp_kp=get_auto_at(w,VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP);
    c->foc_duty_dowmramp_ki=get_auto_at(w,VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI);
    c->foc_start_curr_dec=get_f16_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC,10000.0f);
    c->foc_start_curr_dec_rpm=get_auto_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC_RPM);
    c->foc_openloop_rpm=get_auto_at(w,201U); c->foc_openloop_rpm_low=get_f16_at(w,205U,1000.0f);
    c->foc_sl_openloop_hyst=get_f16_at(w,211U,100.0f);
    c->foc_sl_openloop_time_lock=get_f16_at(w,213U,100.0f);
    c->foc_sl_openloop_time_ramp=get_f16_at(w,215U,100.0f);
    c->foc_sl_openloop_time=get_f16_at(w,217U,100.0f);
    c->foc_sl_openloop_boost_q=get_f16_at(w,219U,100.0f);
    c->foc_sl_openloop_max_q=get_f16_at(w,221U,100.0f);
    for(unsigned k=0;k<8;k++) c->foc_hall_table[k]=w[VESC6_MC_OFF_FOC_HALL_TABLE+k];
    c->foc_hall_interp_erpm=get_auto_at(w,VESC6_MC_OFF_FOC_HALL_INTERP_ERPM);
    c->foc_sl_erpm=get_auto_at(w,VESC6_MC_OFF_FOC_SL_ERPM);
    c->foc_sl_erpm_start=MCCONF_FOC_SL_ERPM_START_DEFAULT;
    c->foc_sample_v0_v7=w[VESC6_MC_OFF_FOC_SAMPLE_V0_V7]!=0U;
    c->foc_sample_high_current=w[VESC6_MC_OFF_FOC_SAMPLE_HIGH_CURRENT]!=0U;
    c->foc_speed_source=(FOC_SPEED_SRC)w[VESC6_MC_OFF_FOC_SPEED_SOURCE];
    c->foc_sat_comp_mode=(SAT_COMP_MODE)w[VESC6_MC_OFF_FOC_SAT_COMP_MODE];
    c->foc_sat_comp=get_f16_at(w,VESC6_MC_OFF_FOC_SAT_COMP,1000.0f);
    c->foc_current_filter_const=get_f16_at(w,VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST,10000.0f);
    c->foc_cc_decoupling=(mc_foc_cc_decoupling_mode)w[VESC6_MC_OFF_FOC_CC_DECOUPLING];
    c->foc_observer_type=(mc_foc_observer_type)w[VESC6_MC_OFF_FOC_OBSERVER_TYPE];
    c->foc_mtpa_mode=(MTPA_MODE)w[VESC6_MC_OFF_FOC_MTPA_MODE];
    c->foc_fw_current_max=get_auto_at(w,VESC6_MC_OFF_FOC_FW_CURRENT_MAX);
    c->foc_fw_duty_start=get_f16_at(w,VESC6_MC_OFF_FOC_FW_DUTY_START,10000.0f);
    c->foc_fw_ramp_time=get_f16_at(w,VESC6_MC_OFF_FOC_FW_RAMP_TIME,1000.0f);
    c->foc_fw_q_current_factor=get_f16_at(w,VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR,10000.0f);
    c->foc_fw_backoff=MCCONF_FOC_FW_BACKOFF_DEFAULT;
    c->foc_mag_vd_max=MCCONF_FOC_MAG_VD_MAX_DEFAULT;
    c->foc_overmod_factor=MCCONF_FOC_OVERMOD_FACTOR_DEFAULT;
    c->foc_temp_comp=MCCONF_FOC_TEMP_COMP_DEFAULT;
    c->foc_temp_comp_base_temp=MCCONF_FOC_TEMP_COMP_BASE_TEMP_DEFAULT;
    c->foc_offsets_cal_mode=MCCONF_FOC_OFFSETS_CAL_MODE_DEFAULT;

    c->s_pid_kp=get_auto_at(w,VESC6_MC_OFF_S_PID_KP); c->s_pid_ki=get_auto_at(w,VESC6_MC_OFF_S_PID_KI);
    c->s_pid_kd=get_auto_at(w,VESC6_MC_OFF_S_PID_KD); c->s_pid_kd_filter=get_f16_at(w,342U,10000.0f);
    c->s_pid_min_erpm=get_auto_at(w,344U); c->s_pid_allow_braking=w[348U]!=0U; c->s_pid_ramp_erpms_s=get_auto_at(w,349U);
    c->s_pid_speed_source=S_PID_SPEED_SRC_PLL; /* not present in VESC6 wire */
    c->p_pid_kp=get_auto_at(w,VESC6_MC_OFF_P_PID_KP); c->p_pid_ki=get_auto_at(w,VESC6_MC_OFF_P_PID_KI);
    c->p_pid_kd=get_auto_at(w,VESC6_MC_OFF_P_PID_KD); c->p_pid_kd_proc=get_auto_at(w,365U);
    c->p_pid_kd_filter=get_f16_at(w,369U,10000.0f); c->p_pid_ang_div=get_auto_at(w,371U);
    c->p_pid_gain_dec_angle=get_f16_at(w,375U,10.0f); c->p_pid_offset=get_auto_at(w,377U);
    c->cc_startup_boost_duty=get_f16_at(w,381U,10000.0f); c->cc_min_current=get_auto_at(w,383U);
    c->cc_gain=get_auto_at(w,387U); c->cc_ramp_step_max=get_f16_at(w,391U,10000.0f);
    c->m_encoder_counts=get_u32_at(w,VESC6_MC_OFF_M_ENCODER_COUNTS);
    c->m_sensor_port_mode=(sensor_port_mode)w[VESC6_MC_OFF_M_SENSOR_PORT_MODE];
    c->m_invert_direction=w[VESC6_MC_OFF_M_INVERT_DIRECTION]!=0U;
    c->si_motor_poles=w[VESC6_MC_OFF_SI_MOTOR_POLES]; c->si_gear_ratio=get_auto_at(w,VESC6_MC_OFF_SI_GEAR_RATIO);
    c->si_wheel_diameter=get_auto_at(w,VESC6_MC_OFF_SI_WHEEL_DIAMETER); c->si_battery_type=w[VESC6_MC_OFF_SI_BATTERY_TYPE];
    c->si_battery_cells=w[VESC6_MC_OFF_SI_BATTERY_CELLS]; c->si_battery_ah=get_auto_at(w,VESC6_MC_OFF_SI_BATTERY_AH);
    c->si_motor_nl_current=get_auto_at(w,VESC6_MC_OFF_SI_MOTOR_NL_CURRENT);
}

static void mcconf_patch_wire(uint8_t *w,const mc_configuration *c) {
    w[4]=(uint8_t)c->pwm_mode; w[5]=(uint8_t)c->comm_mode; w[6]=(uint8_t)c->motor_type; w[7]=(uint8_t)c->sensor_mode;
    put_auto_at(w,VESC6_MC_OFF_L_CURRENT_MAX,c->l_current_max); put_auto_at(w,VESC6_MC_OFF_L_CURRENT_MIN,c->l_current_min);
    put_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MAX,c->l_in_current_max); put_auto_at(w,VESC6_MC_OFF_L_IN_CURRENT_MIN,c->l_in_current_min);
    put_auto_at(w,VESC6_MC_OFF_L_ABS_CURRENT_MAX,c->l_abs_current_max); put_auto_at(w,VESC6_MC_OFF_L_MIN_ERPM,c->l_min_erpm);
    put_auto_at(w,VESC6_MC_OFF_L_MAX_ERPM,c->l_max_erpm); put_f16_at(w,VESC6_MC_OFF_L_ERPM_START,c->l_erpm_start,10000.0f);
    put_auto_at(w,VESC6_MC_OFF_L_MIN_VIN,c->l_min_vin); put_auto_at(w,VESC6_MC_OFF_L_MAX_VIN,c->l_max_vin);
    put_auto_at(w,VESC6_MC_OFF_L_BAT_CUT_START,c->l_battery_cut_start); put_auto_at(w,VESC6_MC_OFF_L_BAT_CUT_END,c->l_battery_cut_end); w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT]=c->l_slow_abs_current?1U:0U;
    put_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_START,c->l_temp_fet_start,10.0f); put_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_END,c->l_temp_fet_end,10.0f);
    put_f16_at(w,VESC6_MC_OFF_L_TEMP_MOTOR_START,c->l_temp_motor_start,10.0f); put_f16_at(w,VESC6_MC_OFF_L_TEMP_MOTOR_END,c->l_temp_motor_end,10.0f);
    put_f16_at(w,VESC6_MC_OFF_L_TEMP_ACCEL_DEC,c->l_temp_accel_dec,10000.0f);
    put_f16_at(w,VESC6_MC_OFF_L_MIN_DUTY,c->l_min_duty,10000.0f); put_f16_at(w,VESC6_MC_OFF_L_MAX_DUTY,c->l_max_duty,10000.0f);
    put_auto_at(w,VESC6_MC_OFF_L_WATT_MAX,c->l_watt_max); put_auto_at(w,VESC6_MC_OFF_L_WATT_MIN,c->l_watt_min);
    put_f16_at(w,VESC6_MC_OFF_L_CURRENT_MAX_SCALE,c->l_current_max_scale,10000.0f); put_f16_at(w,VESC6_MC_OFF_L_CURRENT_MIN_SCALE,c->l_current_min_scale,10000.0f);
    put_f16_at(w,VESC6_MC_OFF_L_DUTY_START,c->l_duty_start,10000.0f);
    put_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KP,c->foc_current_kp); put_auto_at(w,VESC6_MC_OFF_FOC_CURRENT_KI,c->foc_current_ki);
    put_auto_at(w,VESC6_MC_OFF_FOC_F_ZV,c->foc_f_zv); put_auto_at(w,VESC6_MC_OFF_FOC_DT_US,c->foc_dt_us);
    w[VESC6_MC_OFF_FOC_ENCODER_INVERTED]=c->foc_encoder_inverted?1U:0U; put_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_OFFSET,c->foc_encoder_offset);
    put_auto_at(w,VESC6_MC_OFF_FOC_ENCODER_RATIO,c->foc_encoder_ratio);
    w[VESC6_MC_OFF_FOC_SENSOR_MODE]=(uint8_t)(c->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER_AB?FOC_SENSOR_MODE_ENCODER:c->foc_sensor_mode);
    put_auto_at(w,153U,c->foc_pll_kp); put_auto_at(w,157U,c->foc_pll_ki); put_auto_at(w,161U,c->foc_motor_l);
    put_auto_at(w,165U,c->foc_motor_ld_lq_diff); put_auto_at(w,169U,c->foc_motor_r); put_auto_at(w,173U,c->foc_motor_flux_linkage);
    put_auto_at(w,177U,c->foc_observer_gain); put_auto_at(w,181U,c->foc_observer_gain_slow);
    put_f16_at(w,VESC6_MC_OFF_FOC_OBSERVER_OFFSET,c->foc_observer_offset,1000.0f);
    put_auto_at(w,VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP,c->foc_duty_dowmramp_kp); put_auto_at(w,VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI,c->foc_duty_dowmramp_ki);
    put_f16_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC,c->foc_start_curr_dec,10000.0f);
    put_auto_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC_RPM,c->foc_start_curr_dec_rpm);
    put_auto_at(w,201U,c->foc_openloop_rpm); put_f16_at(w,205U,c->foc_openloop_rpm_low,1000.0f);
    /* Keep typed serialization faithful even for fields whose runtime backend
       is intentionally disabled. SET_MCCONF ownership validation will reject
       such changes instead of silently accepting them. */
    put_f16_at(w,211U,c->foc_sl_openloop_hyst,100.0f);
    put_f16_at(w,213U,c->foc_sl_openloop_time_lock,100.0f); put_f16_at(w,215U,c->foc_sl_openloop_time_ramp,100.0f);
    put_f16_at(w,217U,c->foc_sl_openloop_time,100.0f); put_f16_at(w,219U,c->foc_sl_openloop_boost_q,100.0f); put_f16_at(w,221U,c->foc_sl_openloop_max_q,100.0f);
    for(unsigned k=0;k<8;k++)w[VESC6_MC_OFF_FOC_HALL_TABLE+k]=c->foc_hall_table[k];
    put_auto_at(w,VESC6_MC_OFF_FOC_HALL_INTERP_ERPM,c->foc_hall_interp_erpm); put_auto_at(w,VESC6_MC_OFF_FOC_SL_ERPM,c->foc_sl_erpm);
    w[VESC6_MC_OFF_FOC_SAMPLE_V0_V7]=c->foc_sample_v0_v7?1U:0U; w[VESC6_MC_OFF_FOC_SAMPLE_HIGH_CURRENT]=c->foc_sample_high_current?1U:0U;
    w[VESC6_MC_OFF_FOC_SAT_COMP_MODE]=(uint8_t)c->foc_sat_comp_mode; put_f16_at(w,VESC6_MC_OFF_FOC_SAT_COMP,c->foc_sat_comp,1000.0f);
    put_f16_at(w,VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST,c->foc_current_filter_const,10000.0f); w[VESC6_MC_OFF_FOC_CC_DECOUPLING]=(uint8_t)c->foc_cc_decoupling;
    w[VESC6_MC_OFF_FOC_OBSERVER_TYPE]=(uint8_t)c->foc_observer_type; w[VESC6_MC_OFF_FOC_MTPA_MODE]=(uint8_t)c->foc_mtpa_mode;
    put_auto_at(w,VESC6_MC_OFF_FOC_FW_CURRENT_MAX,c->foc_fw_current_max); put_f16_at(w,VESC6_MC_OFF_FOC_FW_DUTY_START,c->foc_fw_duty_start,10000.0f);
    put_f16_at(w,VESC6_MC_OFF_FOC_FW_RAMP_TIME,c->foc_fw_ramp_time,1000.0f); put_f16_at(w,VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR,c->foc_fw_q_current_factor,10000.0f);
    w[VESC6_MC_OFF_FOC_SPEED_SOURCE]=(uint8_t)c->foc_speed_source;
    put_auto_at(w,VESC6_MC_OFF_S_PID_KP,c->s_pid_kp); put_auto_at(w,VESC6_MC_OFF_S_PID_KI,c->s_pid_ki); put_auto_at(w,VESC6_MC_OFF_S_PID_KD,c->s_pid_kd);
    put_f16_at(w,342U,c->s_pid_kd_filter,10000.0f); put_auto_at(w,344U,c->s_pid_min_erpm); w[348U]=c->s_pid_allow_braking?1U:0U; put_auto_at(w,349U,c->s_pid_ramp_erpms_s);
    put_auto_at(w,VESC6_MC_OFF_P_PID_KP,c->p_pid_kp); put_auto_at(w,VESC6_MC_OFF_P_PID_KI,c->p_pid_ki); put_auto_at(w,VESC6_MC_OFF_P_PID_KD,c->p_pid_kd);
    put_auto_at(w,365U,c->p_pid_kd_proc); put_f16_at(w,369U,c->p_pid_kd_filter,10000.0f); put_auto_at(w,371U,c->p_pid_ang_div);
    put_f16_at(w,375U,c->p_pid_gain_dec_angle,10.0f); put_auto_at(w,377U,c->p_pid_offset);
    put_f16_at(w,381U,c->cc_startup_boost_duty,10000.0f); put_auto_at(w,383U,c->cc_min_current);
    put_auto_at(w,387U,c->cc_gain); put_f16_at(w,391U,c->cc_ramp_step_max,10000.0f);
    put_u32_at(w,VESC6_MC_OFF_M_ENCODER_COUNTS,c->m_encoder_counts); w[VESC6_MC_OFF_M_SENSOR_PORT_MODE]=(uint8_t)c->m_sensor_port_mode;
    w[VESC6_MC_OFF_M_INVERT_DIRECTION]=c->m_invert_direction?1U:0U; w[VESC6_MC_OFF_SI_MOTOR_POLES]=c->si_motor_poles;
    put_auto_at(w,VESC6_MC_OFF_SI_GEAR_RATIO,c->si_gear_ratio); put_auto_at(w,VESC6_MC_OFF_SI_WHEEL_DIAMETER,c->si_wheel_diameter);
    w[VESC6_MC_OFF_SI_BATTERY_TYPE]=c->si_battery_type; w[VESC6_MC_OFF_SI_BATTERY_CELLS]=c->si_battery_cells;
    put_auto_at(w,VESC6_MC_OFF_SI_BATTERY_AH,c->si_battery_ah);
    put_auto_at(w,VESC6_MC_OFF_SI_MOTOR_NL_CURRENT,c->si_motor_nl_current);
}

bool confgenerator_deserialize_mcconf(const uint8_t *buffer,mc_configuration *conf){
    if(!buffer||!conf||!sig_ok(buffer,VESC6_MCCONF_SIGNATURE)) return false;
    mcconf_decode_wire(buffer,conf);
    return true;
}
int32_t confgenerator_serialize_mcconf_motor(uint8_t *buffer,const mc_configuration *conf,motor_id_t id){
    if(!buffer||!conf||(id!=MOTOR_LEFT&&id!=MOTOR_RIGHT))return -1;
    /* VESC6 has no s_pid_speed_source field. Refuse to persist a runtime-only
       FAST/FASTER selection instead of silently serializing it as PLL. */
    if(conf->s_pid_speed_source!=S_PID_SPEED_SRC_PLL)return -1;
    if(fabsf(conf->l_in_current_map_start-MCCONF_L_IN_CURRENT_MAP_START_DEFAULT)>1.0e-6f ||
       fabsf(conf->l_in_current_map_filter-MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT)>1.0e-6f)return -1;
    if(conf->l_additional_faults!=MCCONF_L_ADDITIONAL_FAULTS_DEFAULT)return -1;
    if(conf->foc_short_ls_on_zero_duty!=MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT)return -1;
    vesc_config_init_defaults();
    memcpy(buffer,s_mc_active[id],VESC6_MCCONF_WIRE_SIZE); mcconf_patch_wire(buffer,conf); return (int32_t)VESC6_MCCONF_WIRE_SIZE;
}
int32_t confgenerator_serialize_mcconf(uint8_t *buffer,const mc_configuration *conf){
    motor_id_t id=mc_interface_get_motor_thread()==2?MOTOR_RIGHT:MOTOR_LEFT;return confgenerator_serialize_mcconf_motor(buffer,conf,id);
}
void confgenerator_set_defaults_mcconf(mc_configuration *conf){
    if(!conf) return;
    vesc_config_init_defaults();
    mcconf_decode_wire(s_mc_factory[MOTOR_LEFT],conf);
}

bool confgenerator_deserialize_appconf(const uint8_t *buffer,app_configuration *conf){
    if(!buffer||!conf||!sig_ok(buffer,VESC6_APPCONF_SIGNATURE)) return false;
    memset(conf,0,sizeof(*conf));
    conf->controller_id=buffer[VESC6_APP_OFF_CONTROLLER_ID];
    conf->timeout_msec=get_u32_at(buffer,VESC6_APP_OFF_TIMEOUT_MSEC);
    conf->timeout_brake_current=get_auto_at(buffer,VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT);
    conf->permanent_uart_enabled=true;
    conf->app_to_use=(app_use)buffer[VESC6_APP_OFF_APP_TO_USE];
    adc_config *a=&conf->app_adc_conf;
    a->ctrl_type=(adc_control_type)buffer[VESC6_APP_OFF_ADC_CTRL_TYPE];
    a->hyst=get_auto_at(buffer,VESC6_APP_OFF_ADC_HYST);
    a->voltage_start=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_START,1000.0f);
    a->voltage_end=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_END,1000.0f);
    a->voltage_min=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_MIN,1000.0f);
    a->voltage_max=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_MAX,1000.0f);
    a->voltage_center=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_CENTER,1000.0f);
    a->voltage2_start=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE2_START,1000.0f);
    a->voltage2_end=get_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE2_END,1000.0f);
    a->use_filter=buffer[VESC6_APP_OFF_ADC_USE_FILTER]!=0U;
    a->safe_start=(SAFE_START_MODE)buffer[VESC6_APP_OFF_ADC_SAFE_START];
    a->buttons=buffer[VESC6_APP_OFF_ADC_BUTTONS];
    a->voltage_inverted=buffer[VESC6_APP_OFF_ADC_VOLTAGE_INVERTED]!=0U;
    a->voltage2_inverted=buffer[VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED]!=0U;
    a->throttle_exp=get_auto_at(buffer,VESC6_APP_OFF_ADC_THROTTLE_EXP);
    a->throttle_exp_brake=get_auto_at(buffer,VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE);
    a->throttle_exp_mode=(thr_exp_mode)buffer[VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE];
    a->ramp_time_pos=get_auto_at(buffer,VESC6_APP_OFF_ADC_RAMP_TIME_POS);
    a->ramp_time_neg=get_auto_at(buffer,VESC6_APP_OFF_ADC_RAMP_TIME_NEG);
    a->multi_esc=buffer[VESC6_APP_OFF_ADC_MULTI_ESC]!=0U;
    a->tc=buffer[VESC6_APP_OFF_ADC_TC]!=0U;
    a->tc_max_diff=get_auto_at(buffer,VESC6_APP_OFF_ADC_TC_MAX_DIFF);
    a->update_rate_hz=get_u16_at(buffer,VESC6_APP_OFF_ADC_UPDATE_RATE_HZ);
    conf->app_uart_baudrate=get_u32_at(buffer,VESC6_APP_OFF_UART_BAUD);
    return true;
}
int32_t confgenerator_serialize_appconf(uint8_t *buffer,const app_configuration *conf){
    if(!buffer||!conf) return -1;
    vesc_config_init_defaults();
    memcpy(buffer,s_app_active,VESC6_APPCONF_WIRE_SIZE);
    buffer[VESC6_APP_OFF_CONTROLLER_ID]=conf->controller_id;
    put_u32_at(buffer,VESC6_APP_OFF_TIMEOUT_MSEC,conf->timeout_msec);
    put_auto_at(buffer,VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT,conf->timeout_brake_current);
    buffer[VESC6_APP_OFF_APP_TO_USE]=(uint8_t)conf->app_to_use;
    const adc_config *a=&conf->app_adc_conf;
    buffer[VESC6_APP_OFF_ADC_CTRL_TYPE]=(uint8_t)a->ctrl_type;
    put_auto_at(buffer,VESC6_APP_OFF_ADC_HYST,a->hyst);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_START,a->voltage_start,1000.0f);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_END,a->voltage_end,1000.0f);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_MIN,a->voltage_min,1000.0f);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_MAX,a->voltage_max,1000.0f);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE_CENTER,a->voltage_center,1000.0f);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE2_START,a->voltage2_start,1000.0f);
    put_f16_at(buffer,VESC6_APP_OFF_ADC_VOLTAGE2_END,a->voltage2_end,1000.0f);
    buffer[VESC6_APP_OFF_ADC_USE_FILTER]=a->use_filter?1U:0U;
    buffer[VESC6_APP_OFF_ADC_SAFE_START]=(uint8_t)a->safe_start;
    buffer[VESC6_APP_OFF_ADC_BUTTONS]=a->buttons;
    buffer[VESC6_APP_OFF_ADC_VOLTAGE_INVERTED]=a->voltage_inverted?1U:0U;
    buffer[VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED]=a->voltage2_inverted?1U:0U;
    put_auto_at(buffer,VESC6_APP_OFF_ADC_THROTTLE_EXP,a->throttle_exp);
    put_auto_at(buffer,VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE,a->throttle_exp_brake);
    buffer[VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE]=(uint8_t)a->throttle_exp_mode;
    put_auto_at(buffer,VESC6_APP_OFF_ADC_RAMP_TIME_POS,a->ramp_time_pos);
    put_auto_at(buffer,VESC6_APP_OFF_ADC_RAMP_TIME_NEG,a->ramp_time_neg);
    buffer[VESC6_APP_OFF_ADC_MULTI_ESC]=a->multi_esc?1U:0U;
    buffer[VESC6_APP_OFF_ADC_TC]=a->tc?1U:0U;
    put_auto_at(buffer,VESC6_APP_OFF_ADC_TC_MAX_DIFF,a->tc_max_diff);
    put_u16_at(buffer,VESC6_APP_OFF_ADC_UPDATE_RATE_HZ,a->update_rate_hz);
    put_u32_at(buffer,VESC6_APP_OFF_UART_BAUD,conf->app_uart_baudrate);
    return (int32_t)VESC6_APPCONF_WIRE_SIZE;
}

void confgenerator_set_defaults_appconf(app_configuration *conf){
    if(!conf) return;
    vesc_config_init_defaults();
    (void)confgenerator_deserialize_appconf(s_app_factory,conf);
}
