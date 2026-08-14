#include "motor_control.h"
#include "motor_hw.h"
#include "foc_math.h"
#include "foc_control.h"
#include "app_config.h"
#include "board_pins.h"
#include "sensor_detect.h"
#include <string.h>
#include <math.h>
#include "cmsis_os2.h"

MotorRuntime g_motor_left;
MotorRuntime g_motor_right;
static volatile uint32_t s_pending_fault_mask = 0U;

static int32_t amp_to_current_q15(float a) {
    float q = (a / FOC_CURRENT_Q_BASE_A) * 32768.0f;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int32_t)q;
}

void motor_set_foc_targets(MotorRuntime *m, float id_a, float iq_a) {
    id_a = foc_clampf(id_a, -FOC_MAX_CURRENT_A, FOC_MAX_CURRENT_A);
    iq_a = foc_clampf(iq_a, -FOC_MAX_CURRENT_A, FOC_MAX_CURRENT_A);
    m->id_target = id_a;
    m->iq_target = iq_a;
    m->id_target_q15 = amp_to_current_q15(id_a);
    m->iq_target_q15 = amp_to_current_q15(iq_a);
}

static void init_hall_defaults(MotorRuntime *m) {
    const int8_t table[8] = HALL_TABLE_DEFAULT;
    memcpy(m->hall_table, table, sizeof(table));
    for (unsigned i = 0; i < 8U; i++) {
        if (table[i] >= 0) {
            m->foc_hall_table[i] = (uint8_t)(((unsigned)table[i] * 200U) / 6U);
            m->hall_angle_u16[i] = (uint16_t)((unsigned)table[i] * (65536U / 6U));
        } else {
            m->foc_hall_table[i] = 255U;
            m->hall_angle_u16[i] = 0U;
        }
    }
    m->hall.sector = -1;
    m->hall.direction = 1;
}

static void motor_defaults(MotorRuntime *m, motor_id_t id) {
    memset(m, 0, sizeof(*m));
    m->id = id;
    m->pwm_tim = (id == MOTOR_LEFT) ? LEFT_TIM : RIGHT_TIM;
    m->pole_pairs = (id == MOTOR_LEFT) ? LEFT_POLE_PAIRS : RIGHT_POLE_PAIRS;
    m->sensor_mode = SENSOR_MODE_HALL; /* safe GPIO boot mode; AUTO request handled by timer_thread */
    m->sensor_request_mode = (id == MOTOR_LEFT) ? LEFT_SENSOR_BOOT_MODE : RIGHT_SENSOR_BOOT_MODE;
    m->control_mode = MOTOR_CTRL_OFF;
    m->fault = MOTOR_FAULT_NONE;
    m->current_kp = (id == MOTOR_LEFT) ? LEFT_FOC_KP : RIGHT_FOC_KP;
    m->current_ki = (id == MOTOR_LEFT) ? LEFT_FOC_KI : RIGHT_FOC_KI;
    m->current_scale = (id == MOTOR_LEFT) ? LEFT_CURRENT_A_PER_COUNT : RIGHT_CURRENT_A_PER_COUNT;
    m->dc_current_scale = (id == MOTOR_LEFT) ? LEFT_DC_CURRENT_A_PER_COUNT : RIGHT_DC_CURRENT_A_PER_COUNT;

    m->current_scale_q16 = (int32_t)((m->current_scale / FOC_CURRENT_Q_BASE_A) * 32768.0f * 65536.0f);
    m->current_kp_q15 = (int32_t)((m->current_kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    m->current_ki_dt_q15 = (int32_t)((m->current_ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    m->duty_u_q15 = m->duty_v_q15 = m->duty_w_q15 = FOC_Q15_HALF;
    m->speed_pid.kp = SPEED_PID_KP; m->speed_pid.ki = SPEED_PID_KI; m->speed_pid.kd = SPEED_PID_KD;
    init_hall_defaults(m);

    if (id == MOTOR_LEFT) {
        m->encoder.cpr = LEFT_ENCODER_CPR;
        m->encoder.inverted = LEFT_ENCODER_INVERTED_DEFAULT ? true : false;
        m->encoder.elec_offset_u16 = foc_deg_to_u16(LEFT_ENCODER_ELEC_OFFSET_DEG_DEFAULT);
        m->encoder.phase_per_count_q16 = (uint32_t)((((uint64_t)m->pole_pairs * 65536ULL) << 16) / m->encoder.cpr);
        m->hall_offset_u16 = foc_deg_to_u16(LEFT_HALL_ELEC_OFFSET_DEG_DEFAULT);
    } else {
        m->hall_offset_u16 = foc_deg_to_u16(RIGHT_HALL_ELEC_OFFSET_DEG_DEFAULT);
    }
}

void motor_control_init(void) {
    motor_defaults(&g_motor_left, MOTOR_LEFT);
    motor_defaults(&g_motor_right, MOTOR_RIGHT);
    foc_math_init();
    foc_control_init();
    motor_hw_configure_sensor(&g_motor_left, SENSOR_MODE_HALL);
    motor_hw_configure_sensor(&g_motor_right, SENSOR_MODE_HALL);
    motor_hall_edge_isr(&g_motor_left);
    motor_hall_edge_isr(&g_motor_right);
}

MotorRuntime *motor_get(motor_id_t id) { return (id == MOTOR_RIGHT) ? &g_motor_right : &g_motor_left; }

void motor_touch_command(MotorRuntime *m) {
    m->last_command_tick = osKernelGetTickCount();
    m->command_active = true;
    m->timeout_active = false;
}

void motor_keepalive(MotorRuntime *m) {
    if (m == NULL) return;
    m->last_command_tick = osKernelGetTickCount();
    m->timeout_active = false;
}
void motor_set_current(MotorRuntime *m, float amp) { m->current_command_a=foc_clampf(amp,-FOC_MAX_CURRENT_A,FOC_MAX_CURRENT_A); m->control_mode=MOTOR_CTRL_CURRENT; motor_touch_command(m); }
void motor_set_brake_current(MotorRuntime *m, float amp) { m->brake_current_a=fabsf(foc_clampf(amp,-FOC_MAX_CURRENT_A,FOC_MAX_CURRENT_A)); m->control_mode=MOTOR_CTRL_BRAKE_CURRENT; motor_touch_command(m); }
void motor_set_speed(MotorRuntime *m, float erpm) { m->speed_target_erpm=foc_clampf(erpm,-SPEED_PID_MAX_ERPM,SPEED_PID_MAX_ERPM); m->control_mode=MOTOR_CTRL_SPEED; motor_touch_command(m); }
void motor_set_position(MotorRuntime *m, float deg) { m->position_target_deg=foc_wrap_deg(deg); m->control_mode=MOTOR_CTRL_POSITION; motor_touch_command(m); }
void motor_set_duty_approx(MotorRuntime *m, float duty) { m->duty_command=foc_clampf(duty,-0.95f,0.95f); m->control_mode=MOTOR_CTRL_DUTY_APPROX; motor_touch_command(m); }

void motor_stop(MotorRuntime *m) {
    if (m == NULL) return;
    m->control_mode=MOTOR_CTRL_OFF; motor_set_foc_targets(m,0.0f,0.0f); m->command_active=false;
    m->speed_pid.integrator=0.0f; m->speed_pid.prev_error=0.0f;
    m->detect_force_angle=false;
    motor_hw_set_pwm_enabled(m,false);
}

void motor_clear_fault(MotorRuntime *m) {
    motor_stop(m); m->fault=MOTOR_FAULT_NONE; m->vd_int=m->vq_int=0.0f; m->vd_int_q15=m->vq_int_q15=0;
}

void motor_raise_fault_from_task(MotorRuntime *m, motor_fault_t fault) { if (m->fault==MOTOR_FAULT_NONE) m->fault=fault; motor_stop(m); }
void motor_request_fault_from_isr(MotorRuntime *m, motor_fault_t fault) {
    if (m->fault==MOTOR_FAULT_NONE) m->fault=fault;
    m->pwm_tim->BDTR &= ~TIM_BDTR_MOE; m->pwm_enabled=false; s_pending_fault_mask |= (1UL << (uint32_t)m->id);
}
uint32_t motor_take_pending_fault_mask(void) {
    uint32_t primask=__get_PRIMASK(); __disable_irq(); uint32_t mask=s_pending_fault_mask; s_pending_fault_mask=0U; if(!primask)__enable_irq(); return mask;
}

bool motor_select_sensor_mode(MotorRuntime *m, uint8_t mode) {
    if (m == NULL || mode > SENSOR_MODE_ENCODER) return false;
    motor_stop(m);
    if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER) return false;
    if (mode == SENSOR_MODE_AUTO) {
        return sensor_detect_request(m, SENSOR_MODE_AUTO);
    }
    m->sensor_request_mode = mode;
    motor_hw_configure_sensor(m, mode);
    if (mode == SENSOR_MODE_HALL) motor_hall_edge_isr(m);
    return true;
}

int32_t motor_encoder_extended_count(MotorRuntime *m) {
    if (m == NULL || m->id != MOTOR_LEFT) return 0;
    uint32_t primask=__get_PRIMASK(); __disable_irq(); int32_t turns=m->encoder.turns; uint16_t cnt=motor_hw_encoder_cnt(); if(!primask)__enable_irq();
    return turns*(int32_t)m->encoder.cpr + (int32_t)cnt;
}

static void update_encoder_speed_position(MotorRuntime *m) {
    if (m->id != MOTOR_LEFT || m->sensor_mode != SENSOR_MODE_ENCODER) return;
    static int32_t prev_ext=0;
    int32_t ext=motor_encoder_extended_count(m); m->encoder.extended_count=ext;
    int32_t d=ext-prev_ext; prev_ext=ext;
    m->mech_rpm=((float)d*60.0f*1000.0f)/(float)m->encoder.cpr;
    m->erpm=m->mech_rpm*(float)m->pole_pairs;
    m->position_deg=((float)ext*360.0f)/(float)m->encoder.cpr;
}

static void update_hall_speed_position(MotorRuntime *m) {
    if (m->sensor_mode != SENSOR_MODE_HALL || !m->hall.valid || m->hall.period_cycles==0U) return;
    uint32_t age=DWT->CYCCNT-m->hall.edge_cycle;
    if (age > (CPU_CLOCK_HZ/5U)) { m->erpm=0.0f; m->mech_rpm=0.0f; return; }
    float edge_hz=(float)CPU_CLOCK_HZ/(float)m->hall.period_cycles;
    m->erpm=(float)m->hall.direction*edge_hz*10.0f;
    m->mech_rpm=m->erpm/(float)m->pole_pairs;
    m->position_deg=((float)m->hall.edge_count*60.0f)/(float)m->pole_pairs;
}

static float lp(float oldv, float newv, float a) { return oldv + a*(newv-oldv); }

void motor_slow_update_1khz(MotorRuntime *m, uint32_t now_ms) {
    m->vbus=((float)m->vbus_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->ia=((float)m->ia_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->ib=((float)m->ib_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->ic=((float)m->ic_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->id_meas=((float)m->id_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->iq_meas=((float)m->iq_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->id_filter=((float)m->id_filter_q15*FOC_CURRENT_Q_BASE_A)/32768.0f; m->iq_filter=((float)m->iq_filter_q15*FOC_CURRENT_Q_BASE_A)/32768.0f;
    m->vd=((float)m->vd_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f; m->vq=((float)m->vq_q15*FOC_VOLTAGE_Q_BASE_V)/32768.0f;
    m->vd_filter=lp(m->vd_filter,m->vd,FOC_CURRENT_FILTER_CONST); m->vq_filter=lp(m->vq_filter,m->vq,FOC_CURRENT_FILTER_CONST);
    m->duty_u=(float)m->duty_u_q15/32768.0f; m->duty_v=(float)m->duty_v_q15/32768.0f; m->duty_w=(float)m->duty_w_q15/32768.0f;
    float a=fabsf(m->duty_u-0.5f), b=fabsf(m->duty_v-0.5f), c=fabsf(m->duty_w-0.5f); m->duty_now=2.0f*fmaxf(a,fmaxf(b,c)); if(m->iq_target<0.0f)m->duty_now=-m->duty_now;
    float dc=((float)((int32_t)m->dc_current_raw-m->dc_current_offset_counts))*m->dc_current_scale;
    m->dc_current_a=dc; m->dc_current_filter=lp(m->dc_current_filter,dc,DC_CURRENT_FILTER_CONST); m->input_current=m->dc_current_filter;
    m->vbus_filter=lp(m->vbus_filter,m->vbus,VBUS_FILTER_CONST);
    float imag=sqrtf(m->id_filter*m->id_filter + m->iq_filter*m->iq_filter); m->motor_current=((m->vq_filter*m->iq_filter)>=0.0f)?imag:-imag;

    if (m->sensor_mode==SENSOR_MODE_ENCODER) update_encoder_speed_position(m); else if(m->sensor_mode==SENSOR_MODE_HALL) update_hall_speed_position(m);
    m->erpm_int = (int32_t)m->erpm;
    m->rotor_elec_deg=((float)motor_sensor_electrical_phase_u16(m)*360.0f)/65536.0f;

    if (m->command_active && !m->detect.busy &&
        (uint32_t)(now_ms - m->last_command_tick) > MOTOR_COMMAND_TIMEOUT_MS) {
        /* VESC-like communication timeout: release the motor instead of
           latching a permanent fault. The timeout state is reported through
           COMM_GET_VALUES_SELECTIVE bit 21. */
        m->timeout_active = true;
        motor_stop(m);
        return;
    }
    if (m->fault!=MOTOR_FAULT_NONE) { motor_hw_set_pwm_enabled(m,false); return; }

    bool wants_pwm = m->detect.busy || (m->command_active && m->control_mode!=MOTOR_CTRL_OFF);
    if (foc_calibration_valid() && wants_pwm && m->vbus_filter>=VBUS_MIN_RUN_V && m->vbus_filter<=VBUS_MAX_RUN_V) motor_hw_set_pwm_enabled(m,true);
    else if (!m->detect.busy && !m->command_active) motor_hw_set_pwm_enabled(m,false);
}

static float speed_pid_step(MotorRuntime *m,float target_erpm) {
    const float dt=0.001f; float err=target_erpm-m->erpm; m->speed_pid.integrator+=m->speed_pid.ki*err*dt;
    m->speed_pid.integrator=foc_clampf(m->speed_pid.integrator,-FOC_MAX_CURRENT_A,FOC_MAX_CURRENT_A);
    float deriv=(err-m->speed_pid.prev_error)/dt; m->speed_pid.prev_error=err;
    return foc_clampf(m->speed_pid.kp*err+m->speed_pid.integrator+m->speed_pid.kd*deriv,-FOC_MAX_CURRENT_A,FOC_MAX_CURRENT_A);
}

void motor_pid_update_1khz(MotorRuntime *m) {
    if (m->detect.busy) return;
    if (m->fault!=MOTOR_FAULT_NONE || !m->command_active) { motor_set_foc_targets(m,0.0f,0.0f); return; }
    float iq=0.0f;
    switch(m->control_mode) {
        case MOTOR_CTRL_CURRENT: iq=m->current_command_a; break;
        case MOTOR_CTRL_BRAKE_CURRENT: iq=((m->erpm>5.0f)?-1.0f:((m->erpm<-5.0f)?1.0f:0.0f))*m->brake_current_a; break;
        case MOTOR_CTRL_SPEED: iq=speed_pid_step(m,m->speed_target_erpm); break;
        case MOTOR_CTRL_POSITION: { float now=foc_wrap_deg(m->position_deg); float err=m->position_target_deg-now; while(err>180.0f)err-=360.0f; while(err<-180.0f)err+=360.0f; float cmd=foc_clampf(err*POSITION_PID_KP_ERPM_PER_DEG,-POSITION_PID_MAX_ERPM,POSITION_PID_MAX_ERPM); iq=speed_pid_step(m,cmd); } break;
        case MOTOR_CTRL_DUTY_APPROX: iq=foc_clampf(m->duty_command*FOC_MAX_CURRENT_A,-FOC_MAX_CURRENT_A,FOC_MAX_CURRENT_A); break;
        default: iq=0.0f; break;
    }
    motor_set_foc_targets(m,0.0f,iq);
}
