#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "foc_control.h"
#include "vesc_comm.h"
#include "vesc_uart.h"
#include "app_config.h"
#include "board_pins.h"
#include "status_io.h"

extern void xPortSysTickHandler(void);

void SysTick_Handler(void) {
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void DMA1_Channel1_IRQHandler(void) {
    static bool shed_next = false;
    uint32_t isr = DMA1->ISR;

    if ((isr & DMA_ISR_TEIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CGIF1;
        motor_hw_emergency_all_off();
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        return;
    }

    /* The hard-real-time FOC loop executes at DMA half-transfer. No RTOS,
       UART, printf or blocking HAL call is allowed in this path. */
    if ((isr & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1;

        if (shed_next) {
            shed_next = false;
            foc_adc_dma_quick_guard_isr(g_adc_dual_dma);
            return;
        }

        foc_adc_dma_isr(g_adc_dual_dma);

        /* If another half-transfer arrived while the full dual-motor FOC was
         * still executing, the next immediate IRQ is intentionally shortened.
         * This mirrors the liveness relief that fixed no-connect starvation in
         * the known-working hoverboard VESC lineage. */
        if ((DMA1->ISR & DMA_ISR_HTIF1) != 0U) {
            shed_next = true;
        }
    }
}

void DMA1_Channel2_IRQHandler(void) {
    /* USART3 TX DMA completion. Priority 0 is safe because this handler never
     * calls FreeRTOS/CMSIS-RTOS2. */
    vesc_uart_tx_dma_irq_handler();
}

void DMA1_Channel3_IRQHandler(void) {
    /* RX DMA runs circular with its NVIC line disabled; defensive only. */
    vesc_uart_rx_dma_irq_handler();
}

void USART3_IRQHandler(void) {
    /* Normal VESC transport does not use USART3 IRQ. */
    vesc_uart_usart_defensive_irq_handler();
}

void TIM3_IRQHandler(void) {
    status_io_tim3_irq_handler();
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
