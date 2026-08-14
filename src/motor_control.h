#pragma once
#include "motor_types.h"

extern MotorRuntime g_motor_left;
extern MotorRuntime g_motor_right;

void motor_control_init(void);
MotorRuntime *motor_get(motor_id_t id);
void motor_set_current(MotorRuntime *m, float amp);
void motor_set_brake_current(MotorRuntime *m, float amp);
void motor_set_handbrake(MotorRuntime *m, float amp);
void motor_set_current_rel(MotorRuntime *m, float rel);
void motor_set_speed(MotorRuntime *m, float erpm);
void motor_set_position(MotorRuntime *m, float deg);
void motor_set_duty(MotorRuntime *m, float duty);
void motor_stop(MotorRuntime *m);
void motor_clear_fault(MotorRuntime *m);
void motor_touch_command(MotorRuntime *m);
void motor_keepalive(MotorRuntime *m);
void motor_slow_update_1khz(MotorRuntime *m, uint32_t now_ms);
void motor_rpm_update_1khz(MotorRuntime *m);
void motor_pid_update_1khz(MotorRuntime *m);
void motor_raise_fault_from_task(MotorRuntime *m, motor_fault_t fault);
void motor_request_fault_from_isr(MotorRuntime *m, motor_fault_t fault);
uint32_t motor_take_pending_fault_mask(void);

bool motor_select_sensor_mode(MotorRuntime *m, uint8_t mode);
void motor_set_foc_targets(MotorRuntime *m, float id_a, float iq_a);
void motor_set_current_pi_gains(MotorRuntime *m, float kp, float ki);
int32_t motor_encoder_extended_count(MotorRuntime *m);
