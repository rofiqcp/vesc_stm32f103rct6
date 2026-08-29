#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "datatypes.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern volatile uint32_t g_adc_dual_dma[6];
extern volatile uint16_t g_adc3_vbus_dma[2];

/* ADC3 DMA2_CH5 uses a two-halfword circular buffer. CNDTR=1 means slot 0
 * has just been written; CNDTR=2 means the second transfer completed and the
 * circular counter reloaded, so slot 1 is newest. Called only from the hard
 * ADC1/DMA1 current ISR, after ADC3 has had enough time to complete DCLINK. */
static inline uint16_t motor_hw_vbus_raw_from_isr(uint16_t *dma_cndtr) {
    const uint16_t rem = (uint16_t)DMA2_Channel5->CNDTR;
    __DMB();
    const uint16_t raw = (rem == 1U) ? g_adc3_vbus_dma[0] : g_adc3_vbus_dma[1];
    if (dma_cndtr) *dma_cndtr = rem;
    return raw;
}

void motor_hw_init(void);
void motor_hw_start_sampling(void);
enum {
    HW_SAMPLING_CONTRACT_TIM1_MODE     = 1UL << 0,
    HW_SAMPLING_CONTRACT_TIM8_MODE     = 1UL << 1,
    HW_SAMPLING_CONTRACT_TIM8_TRGO     = 1UL << 2,
    HW_SAMPLING_CONTRACT_TIM8_RCR      = 1UL << 3,
    HW_SAMPLING_CONTRACT_ADC1_LEN      = 1UL << 4,
    HW_SAMPLING_CONTRACT_ADC2_LEN      = 1UL << 5,
    HW_SAMPLING_CONTRACT_DMA1_MODE     = 1UL << 6,
    HW_SAMPLING_CONTRACT_TIM1_TRGO     = 1UL << 7,
    HW_SAMPLING_CONTRACT_TIM8_SLAVE    = 1UL << 8,
    HW_SAMPLING_CONTRACT_ADC_DUALMODE  = 1UL << 9,
    HW_SAMPLING_CONTRACT_ADC1_TRIGGER  = 1UL << 10,
    HW_SAMPLING_CONTRACT_ADC_CHANNELS  = 1UL << 11,
    HW_SAMPLING_CONTRACT_DMA1_TRANSFER = 1UL << 12,
    HW_SAMPLING_CONTRACT_ADC3_MODE     = 1UL << 13,
    HW_SAMPLING_CONTRACT_DMA2_MODE     = 1UL << 14,
    HW_SAMPLING_CONTRACT_DMA2_TRANSFER = 1UL << 15
};
uint32_t motor_hw_sampling_contract_flags(void);
bool motor_hw_sampling_contract_valid(void);
void motor_hw_set_pwm_enabled(MotorRuntime *m, bool enabled);
void motor_hw_service_pwm_enable_from_isr(MotorRuntime *m);
void motor_hw_set_pwm_duty(MotorRuntime *m, float du, float dv, float dw);
void motor_hw_set_pwm_q15(MotorRuntime *m, uint16_t du_q15, uint16_t dv_q15, uint16_t dw_q15);
void motor_hw_restore_foc_outputs(MotorRuntime *m);
void motor_hw_set_low_side_brake(MotorRuntime *m, bool enable);
void motor_hw_break_irq_handler(TIM_TypeDef *tim);
void motor_hw_pvd_irq_handler(void);
uint32_t motor_hw_powerstage_fault_flags(void);
bool motor_hw_powerstage_fault_latched(void);
bool motor_hw_pvd_low(void);
bool motor_hw_clear_recoverable_powerstage_faults(void);
uint8_t motor_hw_read_hall_raw(motor_id_t id);
uint16_t motor_hw_encoder_cnt(void);
bool motor_hw_board_temperature_c(float *temp_c);
void motor_hw_capture_app_adc_from_isr(void);
void motor_hw_set_adc_phase_offset_ticks(uint16_t ticks);
uint16_t motor_hw_get_adc_phase_offset_ticks(void);
bool motor_hw_get_app_adc_raw(uint16_t *pa2_raw, uint16_t *pa3_raw);
void motor_hw_encoder_set_count(MotorRuntime *m, uint16_t count);
void motor_hw_configure_sensor(MotorRuntime *m, uint8_t mode);
void motor_hw_led(bool on);
void motor_hw_emergency_all_off(void);

/* Board status/power-latch service. Kept in hwconf because these functions are
 * tied to the hoverboard PCB, not to the generic motor-control interface. */
void hw_status_early_init(void);
void hw_status_timer_init(void);
bool hw_status_init(void);
void hw_status_tone_start(uint16_t hz);
void hw_status_tone_start_for(uint16_t hz, uint32_t duration_ms);
void hw_status_tone_stop(void);
/* Replays the same non-blocking ~3.21 s TIM3 power-on melody. The command
 * layer only exposes this while both bridges are stopped. */
void hw_status_startup_melody_replay(void);
bool hw_status_tone_is_running(void);
bool hw_status_tone_level(void);
void hw_status_power_hold(bool on);
bool hw_status_power_is_held(void);
void hw_status_service_10ms(uint32_t now_ms);
void hw_status_tim3_irq_handler(void);
void hw_status_early_fatal_loop(void) __attribute__((noreturn));
extern volatile uint8_t g_vesc_buzzer_running;
extern volatile uint16_t g_vesc_buzzer_hz;
extern volatile uint32_t g_vesc_buzzer_remaining;
extern volatile uint8_t g_vesc_startup_melody_active;
extern volatile uint8_t g_vesc_startup_melody_index;
