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
    if (m == NULL) return;
    float hi = (m->current_max_a > 0.0f) ? m->current_max_a : FOC_MAX_CURRENT_A;
    float lo = (m->current_min_a < 0.0f) ? m->current_min_a : -hi;
    id_a = foc_clampf(id_a, lo, hi);
    iq_a = foc_clampf(iq_a, lo, hi);
    m->id_target = id_a;
    m->iq_target = iq_a;
    m->id_target_q15 = amp_to_current_q15(id_a);
    m->iq_target_q15 = amp_to_current_q15(iq_a);
}

void motor_set_current_pi_gains(MotorRuntime *m, float kp, float ki) {
    if (m == NULL) return;
    if (!isfinite(kp) || kp < 0.000001f) kp = 0.000001f;
    if (!isfinite(ki) || ki < 0.0f) ki = 0.0f;
    m->current_kp = kp;
    m->current_ki = ki;
    m->current_kp_q16 = (int32_t)((kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->current_ki_dt_q16 = (int32_t)((ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    /* A gain change is a control discontinuity. Reset only the fast current
       integrators; outer-loop state is left to the caller. */
    m->vd_int_q31 = 0; m->vq_int_q31 = 0;
    m->vd_int_q15 = 0; m->vq_int_q15 = 0;
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
    m->current_kp_q16 = (int32_t)((m->current_kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->current_ki_dt_q16 = (int32_t)((m->current_ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 65536.0f);
    m->duty_u_q15 = m->duty_v_q15 = m->duty_w_q15 = FOC_Q15_HALF;
    m->current_max_a = FOC_MAX_CURRENT_A; m->current_min_a = -FOC_MAX_CURRENT_A;
    m->abs_current_max_a = FOC_ABS_CURRENT_TRIP_A;
    m->abs_current_trip_q15 = amp_to_current_q15(FOC_ABS_CURRENT_TRIP_A);
    m->max_erpm = MOTOR_DEFAULT_MAX_ERPM; m->min_erpm = MOTOR_DEFAULT_MIN_ERPM;
    m->max_duty = MOTOR_DEFAULT_MAX_DUTY; m->min_duty = MOTOR_DEFAULT_MIN_DUTY;
    m->speed_pid.kp = SPEED_PID_KP; m->speed_pid.ki = SPEED_PID_KI; m->speed_pid.kd = SPEED_PID_KD;
    m->speed_kd_filter = SPEED_PID_D_FILTER;
    m->position_pid.kp = POSITION_PID_KP_CURRENT_PER_DEG;
    m->position_pid.ki = POSITION_PID_KI_CURRENT_PER_DEG_S;
    m->position_pid.kd = POSITION_PID_KD_CURRENT_PER_DEGPS;
    m->position_kd_filter = POSITION_PID_D_FILTER;
    m->duty_kp = DUTY_PID_KP_CURRENT_PER_DUTY;
    m->duty_ki = DUTY_PID_KI_CURRENT_PER_DUTY_S;
    init_hall_defaults(m);

    if (id == MOTOR_LEFT) {
        m->encoder.cpr = LEFT_ENCODER_CPR;
        m->encoder.inverted = LEFT_ENCODER_INVERTED_DEFAULT ? true : false;
        m->encoder.elec_offset_u16 = foc_deg_to_u16(LEFT_ENCODER_ELEC_OFFSET_DEG_DEFAULT);
        m->encoder.electrical_ratio = (float)m->pole_pairs;
        m->encoder.electrical_ratio_q16 = (uint32_t)lrintf(m->encoder.electrical_ratio * 65536.0f);
        m->encoder.phase_per_count_q16 = (uint32_t)(((uint64_t)m->encoder.electrical_ratio_q16 << 16) / m->encoder.cpr);
        m->encoder.synced = false;
        m->encoder.motion_proved = false;
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
static void enter_control_mode(MotorRuntime *m, motor_control_mode_t mode) {
    if (m->control_mode != mode) {
        m->speed_pid.integrator=0.0f; m->speed_pid.prev_error=0.0f;
        m->position_pid.integrator=0.0f; m->position_pid.prev_error=0.0f;
        m->duty_pid.integrator=0.0f; m->duty_pid.prev_error=0.0f;
        m->position_derivative_filtered=0.0f;
    }
    if (mode != MOTOR_CTRL_HANDBRAKE && mode != MOTOR_CTRL_DETECT) {
        m->detect_force_angle=false;
    }
    m->control_mode=mode;
}

void motor_set_current(MotorRuntime *m, float amp) {
    if (m == NULL) return;
    m->current_command_a=foc_clampf(amp,m->current_min_a,m->current_max_a);
    enter_control_mode(m,MOTOR_CTRL_CURRENT); motor_touch_command(m);
}
void motor_set_brake_current(MotorRuntime *m, float amp) {
    if (m == NULL) return;
    float lim=fmaxf(fabsf(m->current_min_a),fabsf(m->current_max_a));
    m->brake_current_a=fabsf(foc_clampf(amp,-lim,lim));
    enter_control_mode(m,MOTOR_CTRL_BRAKE_CURRENT); motor_touch_command(m);
}
void motor_set_current_rel(MotorRuntime *m, float rel) {
    if (m == NULL) return;
    rel = foc_clampf(rel, -1.0f, 1.0f);
    motor_set_current(m, rel >= 0.0f ? rel*m->current_max_a : (-rel)*m->current_min_a);
}
void motor_set_handbrake(MotorRuntime *m, float amp) {
    if (m == NULL) return;
    float lim=fmaxf(fabsf(m->current_min_a),fabsf(m->current_max_a));
    m->handbrake_current_a = fabsf(foc_clampf(amp, -lim, lim));
    /* Reduced-port equivalent of FOC handbrake: hold a stationary field at
     * the electrical phase present when the command is received. */
    m->detect_phase_u16 = motor_sensor_electrical_phase_u16(m);
    m->detect_force_angle = true;
    enter_control_mode(m,MOTOR_CTRL_HANDBRAKE);
    motor_touch_command(m);
}
void motor_set_speed(MotorRuntime *m, float erpm) {
    if (m == NULL) return;
    m->speed_target_erpm=foc_clampf(erpm,m->min_erpm,m->max_erpm);
    enter_control_mode(m,MOTOR_CTRL_SPEED); motor_touch_command(m);
}
void motor_set_position(MotorRuntime *m, float deg) {
    if (m == NULL) return;
    m->position_target_deg=foc_wrap_deg(deg);
    enter_control_mode(m,MOTOR_CTRL_POSITION); motor_touch_command(m);
}
void motor_set_duty(MotorRuntime *m, float duty) {
    if (m == NULL) return;
    m->duty_command=foc_clampf(duty,m->min_duty,m->max_duty);
    enter_control_mode(m,MOTOR_CTRL_DUTY); motor_touch_command(m);
}

void motor_stop(MotorRuntime *m) {
    if (m == NULL) return;
    m->control_mode=MOTOR_CTRL_OFF; motor_set_foc_targets(m,0.0f,0.0f); m->command_active=false;
    m->speed_pid.integrator=0.0f; m->speed_pid.prev_error=0.0f;
    m->position_pid.integrator=0.0f; m->position_pid.prev_error=0.0f;
    m->duty_pid.integrator=0.0f; m->duty_pid.prev_error=0.0f;
    m->position_derivative_filtered=0.0f;
    m->detect_force_angle=false;
    motor_hw_set_pwm_enabled(m,false);
}

void motor_clear_fault(MotorRuntime *m) {
    motor_stop(m); m->fault=MOTOR_FAULT_NONE; m->vd_int=m->vq_int=0.0f;
    m->vd_int_q31=m->vq_int_q31=0; m->vd_int_q15=m->vq_int_q15=0;
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
    if (m->id == MOTOR_LEFT && mode == SENSOR_MODE_ENCODER) {
        m->encoder.synced=false; m->encoder.motion_proved=false;
    }
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
    int32_t ext=motor_encoder_extended_count(m); m->encoder.extended_count=ext;
    if (!m->encoder.speed_sample_valid) {
        m->encoder.prev_extended_count=ext;
        m->encoder.speed_sample_valid=true;
        m->mech_rpm=0.0f; m->erpm=0.0f;
    }
    int32_t d=ext-m->encoder.prev_extended_count;
    m->encoder.prev_extended_count=ext;
    m->mech_rpm=((float)d*60.0f*1000.0f)/(float)m->encoder.cpr;
    m->erpm=m->mech_rpm*(float)m->pole_pairs;
    m->position_deg=((float)(ext-m->encoder.mechanical_zero_count)*360.0f)/(float)m->encoder.cpr;
    /* m_invert_direction changes the external VESC coordinate system as well
       as torque polarity. This keeps positive RPM/POS feedback consistent with
       a positive command after the physical motor direction is inverted. */
    if (m->invert_direction) { m->mech_rpm=-m->mech_rpm; m->erpm=-m->erpm; m->position_deg=-m->position_deg; }
}

static void update_hall_speed_position(MotorRuntime *m) {
    if (m->sensor_mode != SENSOR_MODE_HALL || !m->hall.valid || m->hall.period_cycles==0U) return;
    uint32_t age=DWT->CYCCNT-m->hall.edge_cycle;
    if (age > (CPU_CLOCK_HZ/5U)) { m->erpm=0.0f; m->mech_rpm=0.0f; return; }
    float edge_hz=(float)CPU_CLOCK_HZ/(float)m->hall.period_cycles;
    m->erpm=(float)m->hall.direction*edge_hz*10.0f;
    m->mech_rpm=m->erpm/(float)m->pole_pairs;
    m->position_deg=((float)m->hall.edge_count*60.0f)/(float)m->pole_pairs;
    if (m->invert_direction) { m->mech_rpm=-m->mech_rpm; m->erpm=-m->erpm; m->position_deg=-m->position_deg; }
}

void motor_rpm_update_1khz(MotorRuntime *m) {
    if (m == NULL) return;
    if (m->sensor_mode == SENSOR_MODE_ENCODER) {
        update_encoder_speed_position(m);
    } else if (m->sensor_mode == SENSOR_MODE_HALL) {
        update_hall_speed_position(m);
    } else {
        m->erpm = 0.0f;
        m->mech_rpm = 0.0f;
    }
    m->erpm_int = (int32_t)m->erpm;
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
    float dc=((float)(m->dc_current_offset_counts-(int32_t)m->dc_current_raw))*m->dc_current_scale;
    m->dc_current_a=dc; m->dc_current_filter=lp(m->dc_current_filter,dc,DC_CURRENT_FILTER_CONST); m->input_current=m->dc_current_filter;
    m->vbus_filter=lp(m->vbus_filter,m->vbus,VBUS_FILTER_CONST);
    float imag=sqrtf(m->id_filter*m->id_filter + m->iq_filter*m->iq_filter); m->motor_current=((m->vq_filter*m->iq_filter)>=0.0f)?imag:-imag;

    m->rotor_elec_deg=((float)motor_sensor_electrical_phase_u16(m)*360.0f)/65536.0f;

    (void)now_ms; /* command timeout is global and handled by timeout_thread. */
    if (m->fault!=MOTOR_FAULT_NONE) { motor_hw_set_pwm_enabled(m,false); return; }

    bool encoder_ready = !(m->id==MOTOR_LEFT && m->sensor_mode==SENSOR_MODE_ENCODER) || m->encoder.synced || m->detect.busy;
    bool wants_pwm = m->detect.busy || (m->command_active && m->control_mode!=MOTOR_CTRL_OFF);
    if (foc_calibration_valid() && encoder_ready && wants_pwm && m->vbus_filter>=VBUS_MIN_RUN_V && m->vbus_filter<=VBUS_MAX_RUN_V) motor_hw_set_pwm_enabled(m,true);
    else if (!m->detect.busy && !m->command_active) motor_hw_set_pwm_enabled(m,false);
}

static float configured_iq_limit(const MotorRuntime *m) {
    return fmaxf(fabsf(m->current_min_a), fabsf(m->current_max_a));
}

static float speed_pid_step(MotorRuntime *m,float target_erpm) {
    const float dt=0.001f;
    const float lim=configured_iq_limit(m);
    float err=target_erpm-m->erpm;
    m->speed_pid.integrator+=m->speed_pid.ki*err*dt;
    m->speed_pid.integrator=foc_clampf(m->speed_pid.integrator,-lim,lim);
    float raw_d=(err-m->speed_pid.prev_error)/dt; m->speed_pid.prev_error=err;
    /* Reuse position_derivative_filtered only for position. Speed derivative
       is intentionally inexpensive and filtered by a bounded first order term. */
    static float speed_d_left=0.0f, speed_d_right=0.0f;
    float *df=(m->id==MOTOR_LEFT)?&speed_d_left:&speed_d_right;
    *df += foc_clampf(m->speed_kd_filter,0.0f,1.0f)*(raw_d-*df);
    return foc_clampf(m->speed_pid.kp*err+m->speed_pid.integrator+m->speed_pid.kd*(*df),-lim,lim);
}

static float position_pid_step(MotorRuntime *m) {
    const float dt=0.001f;
    const float lim=configured_iq_limit(m);
    float now=foc_wrap_deg(m->position_deg);
    float err=m->position_target_deg-now;
    while(err>180.0f)err-=360.0f;
    while(err<-180.0f)err+=360.0f;
    m->position_pid.integrator += m->position_pid.ki*err*dt;
    m->position_pid.integrator=foc_clampf(m->position_pid.integrator,-lim,lim);
    float d=(err-m->position_pid.prev_error)/dt; m->position_pid.prev_error=err;
    m->position_derivative_filtered += foc_clampf(m->position_kd_filter,0.0f,1.0f)*(d-m->position_derivative_filtered);
    return foc_clampf(m->position_pid.kp*err + m->position_pid.integrator +
                      m->position_pid.kd*m->position_derivative_filtered,-lim,lim);
}

static float duty_pid_step(MotorRuntime *m) {
    const float dt=0.001f;
    const float lim=configured_iq_limit(m);
    float err=m->duty_command-m->duty_now;
    m->duty_pid.integrator += m->duty_ki*err*dt;
    m->duty_pid.integrator=foc_clampf(m->duty_pid.integrator,-lim,lim);
    return foc_clampf(m->duty_kp*err+m->duty_pid.integrator,-lim,lim);
}

void motor_pid_update_1khz(MotorRuntime *m) {
    if (m==NULL || m->detect.busy) return;
    if (m->fault!=MOTOR_FAULT_NONE || !m->command_active) { motor_set_foc_targets(m,0.0f,0.0f); return; }
    /* Incremental AB is a valid FOC phase source only after this boot's
       alignment. Hall remains usable immediately when its state is valid. */
    if (m->id==MOTOR_LEFT && m->sensor_mode==SENSOR_MODE_ENCODER && !m->encoder.synced) {
        motor_set_foc_targets(m,0.0f,0.0f); return;
    }
    float iq=0.0f;
    switch(m->control_mode) {
        case MOTOR_CTRL_CURRENT: iq=m->current_command_a; break;
        case MOTOR_CTRL_BRAKE_CURRENT:
            iq=((m->erpm>5.0f)?-1.0f:((m->erpm<-5.0f)?1.0f:0.0f))*m->brake_current_a;
            break;
        case MOTOR_CTRL_SPEED: iq=speed_pid_step(m,m->speed_target_erpm); break;
        case MOTOR_CTRL_POSITION: iq=position_pid_step(m); break;
        case MOTOR_CTRL_DUTY: iq=duty_pid_step(m); break;
        case MOTOR_CTRL_HANDBRAKE:
            motor_set_foc_targets(m, m->handbrake_current_a, 0.0f); return;
        default: iq=0.0f; break;
    }
    if (m->invert_direction) iq=-iq;
    motor_set_foc_targets(m,0.0f,iq);
}
