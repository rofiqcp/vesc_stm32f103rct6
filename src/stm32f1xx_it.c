#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hwconf/hw.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "comm/commands.h"
#include "applications/app_uartcomm.h"
#include "applications/appconf_default.h"
#include "hwconf/hw_hoverboard.h"

extern void xPortSysTickHandler(void);

void SysTick_Handler(void) {
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void PVD_IRQHandler(void) {
    motor_hw_pvd_irq_handler();
}

void TIM1_BRK_IRQHandler(void) {
    motor_hw_break_irq_handler(TIM1);
}

void TIM8_BRK_IRQHandler(void) {
    motor_hw_break_irq_handler(TIM8);
}

void DMA2_Channel4_5_IRQHandler(void) {
    const uint32_t isr = DMA2->ISR;
    if ((isr & DMA_ISR_TEIF5) != 0U) {
        DMA2->IFCR = DMA_IFCR_CGIF5;
        motor_hw_emergency_all_off();
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        return;
    }

    /* ADC3 HT/TC IRQs are intentionally disabled. Clear any unexpected
       channel-5 flags defensively so a stale bootloader/debug setup cannot
       create an interrupt storm. */
    if ((isr & (DMA_ISR_HTIF5 | DMA_ISR_TCIF5)) != 0U) {
        DMA2->IFCR = DMA_IFCR_CGIF5;
    }
}

void DMA1_Channel1_IRQHandler(void) {
    uint32_t isr = DMA1->ISR;

    if ((isr & DMA_ISR_TEIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CGIF1;
        motor_hw_emergency_all_off();
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        return;
    }

    /* Fast path: the first three dual ranks are all six stock-board current
       channels (DC L/R, LEFT A/B, RIGHT B/C). Half-transfer is therefore the
       earliest coherent entry for both 16-kHz current loops. No kernel/UART/
       formatted-output/flash/blocking HAL calls are permitted here. APP ADC
       (PA2/PA3 at rank 4) is NOT touched in the ISR — it is read from the
       DMA buffer directly in app_adc_service_1khz() at 1 kHz, matching the
       VESC standard where application ADC sampling lives in the app thread. */
    if ((isr & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        foc_adc_dma_isr(g_adc_dual_dma);
    }
}

void TIM2_IRQHandler(void) {
    /* This port does not use TIM2 for ADC timing. Defensive clear only in case a
       stale bootloader/debug configuration left a TIM2 flag pending. */
    TIM2->SR = 0U;
}

void DMA1_Channel2_IRQHandler(void) {
    /* SmartESC-compatible USART3 TX DMA ISR -> STM32 HAL DMA state machine. */
    app_uartcomm_dma_tx_irq_handler();
}

void DMA1_Channel3_IRQHandler(void) {
    /* SmartESC-compatible USART3 RX circular DMA ISR -> HAL_DMA_IRQHandler. */
    app_uartcomm_dma_rx_irq_handler();
}

void TIM3_IRQHandler(void) {
    hw_status_tim3_irq_handler();
}

void EXTI9_5_IRQHandler(void) {
    uint32_t mask = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
    uint32_t pending = EXTI->PR & mask;
    if (pending != 0U) {
        EXTI->PR = pending;
        if (g_motor_left.sensor_mode == SENSOR_MODE_HALL) {
            motor_hall_edge_isr(&g_motor_left);
        }
    }
}

void EXTI15_10_IRQHandler(void) {
    uint32_t mask = RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN;
    uint32_t pending = EXTI->PR & mask;
    if (pending != 0U) {
        EXTI->PR = pending;
        if (g_motor_right.sensor_mode == SENSOR_MODE_HALL) {
            motor_hall_edge_isr(&g_motor_right);
        }
    }
}

void TIM4_IRQHandler(void) {
    if ((TIM4->SR & TIM_SR_UIF) != 0U) {
        TIM4->SR &= ~TIM_SR_UIF;
        if (g_motor_left.sensor_mode == SENSOR_MODE_ENCODER) {
            if ((TIM4->CR1 & TIM_CR1_DIR) != 0U) {
                g_motor_left.encoder.turns--;
            } else {
                g_motor_left.encoder.turns++;
            }
        }
    }
}

static void fatal_exception(void) {
    motor_hw_emergency_all_off();
    __disable_irq();
    while (1) { }
}

void HardFault_Handler(void) { fatal_exception(); }
void MemManage_Handler(void) { fatal_exception(); }
void BusFault_Handler(void)  { fatal_exception(); }
void UsageFault_Handler(void){ fatal_exception(); }
