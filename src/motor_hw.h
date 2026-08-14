#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "motor_types.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern volatile uint32_t g_adc_dual_dma[4];

void motor_hw_init(void);
void motor_hw_start_sampling(void);
void motor_hw_set_pwm_enabled(MotorRuntime *m, bool enabled);
void motor_hw_set_pwm_duty(MotorRuntime *m, float du, float dv, float dw);
void motor_hw_set_pwm_q15(MotorRuntime *m, uint16_t du_q15, uint16_t dv_q15, uint16_t dw_q15);
void motor_hw_gate_global(bool enable);
uint8_t motor_hw_read_hall_raw(motor_id_t id);
uint16_t motor_hw_encoder_cnt(void);
void motor_hw_encoder_reset(void);
void motor_hw_configure_sensor(MotorRuntime *m, uint8_t mode);
void motor_hw_led(bool on);
void motor_hw_buzzer(bool on);
void motor_hw_uart_tx(const uint8_t *data, uint16_t len);
void motor_hw_uart_start_rx_irq(void);
void motor_hw_emergency_all_off(void);
