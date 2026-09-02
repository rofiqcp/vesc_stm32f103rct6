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

// Fungsi xPortSysTickHandler: menangani x port sys tick handler pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
extern void xPortSysTickHandler(void);

// Fungsi SysTick_Handler: menangani interrupt perangkat keras terkait dengan jalur sesingkat mungkin dan
// menjaga state keselamatan firmware.
void SysTick_Handler(void) {
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

// Fungsi PVD_IRQHandler: menjalankan operasi pvd irqhandler sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void PVD_IRQHandler(void) {
    motor_hw_pvd_irq_handler();
}

// Fungsi TIM1_BRK_IRQHandler: menangani interrupt perangkat keras terkait dengan jalur sesingkat mungkin dan
// menjaga state keselamatan firmware.
void TIM1_BRK_IRQHandler(void) {
    motor_hw_break_irq_handler(TIM1);
}

// Fungsi TIM8_BRK_IRQHandler: menangani interrupt perangkat keras terkait dengan jalur sesingkat mungkin dan
// menjaga state keselamatan firmware.
void TIM8_BRK_IRQHandler(void) {
    motor_hw_break_irq_handler(TIM8);
}

// Fungsi DMA2_Channel4_5_IRQHandler: menjalankan operasi dma2 channel4 5 irqhandler sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void DMA2_Channel4_5_IRQHandler(void) {
    /* Run31 tidak memakai ADC3/DMA2 untuk motor. Clear flag defensif saja jika
       bootloader/debugger meninggalkan status lama; jangan membuat fault palsu. */
    DMA2->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
}

// Fungsi DMA1_Channel1_IRQHandler: menangani interrupt perangkat keras terkait dengan jalur sesingkat mungkin
// dan menjaga state keselamatan firmware.
void DMA1_Channel1_IRQHandler(void) {
    /* V15-style TC-only ADC path. No VESC configuration, observer or floating
       point work is performed before a complete DMA batch is available. */
    const uint32_t start_cycle = DWT->CYCCNT;
    const uint32_t isr = DMA1->ISR;

    if ((isr & DMA_ISR_TEIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CGIF1;
        motor_hw_emergency_all_off();
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        mcpwm_foc_finish_irq_timing(start_cycle);
        return;
    }

    if ((isr & DMA_ISR_TCIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        motor_hw_capture_app_adc_from_isr();
        const uint32_t base = ADC_WORDS_PER_FRAME * (ADC_DMA_BATCH_FRAMES - 1U);
        foc_adc_dma_isr_timed(&g_adc_dual_dma[base], start_cycle);
        return;
    }

    /* Defensive stale IRQ clear; normal operation reaches only TC or TE. */
    DMA1->IFCR = DMA_IFCR_CGIF1;
    mcpwm_foc_finish_irq_timing(start_cycle);
}

// Fungsi TIM2_IRQHandler: menjalankan operasi tim2 irqhandler sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void TIM2_IRQHandler(void) {
    /* This port does not use TIM2 for ADC timing. Defensive clear only in case a
       stale bootloader/debug configuration left a TIM2 flag pending. */
    TIM2->SR = 0U;
}

// Fungsi DMA1_Channel2_IRQHandler: menjalankan operasi dma1 channel2 irqhandler sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void DMA1_Channel2_IRQHandler(void) {
    /* SmartESC-compatible USART3 TX DMA ISR -> STM32 HAL DMA state machine. */
    app_uartcomm_dma_tx_irq_handler();
}

// Fungsi DMA1_Channel3_IRQHandler: menjalankan operasi dma1 channel3 irqhandler sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void DMA1_Channel3_IRQHandler(void) {
    /* SmartESC-compatible USART3 RX circular DMA ISR -> HAL_DMA_IRQHandler. */
    app_uartcomm_dma_rx_irq_handler();
}

// Fungsi TIM3_IRQHandler: menjalankan operasi tim3 irqhandler sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void TIM3_IRQHandler(void) {
    hw_status_tim3_irq_handler();
}

// Fungsi EXTI9_5_IRQHandler: menjalankan operasi exti9 5 irqhandler sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void EXTI9_5_IRQHandler(void) {
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mask = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
    // Variabel pending: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t pending = EXTI->PR & mask;
    if (pending != 0U) {
        EXTI->PR = pending;
        if (g_motor_left.sensor_mode == SENSOR_MODE_HALL) {
            motor_hall_edge_isr(&g_motor_left);
        }
    }
}

// Fungsi EXTI15_10_IRQHandler: menjalankan operasi exti15 10 irqhandler sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void EXTI15_10_IRQHandler(void) {
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mask = RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN;
    // Variabel pending: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t pending = EXTI->PR & mask;
    if (pending != 0U) {
        EXTI->PR = pending;
        if (g_motor_right.sensor_mode == SENSOR_MODE_HALL) {
            motor_hall_edge_isr(&g_motor_right);
        }
    }
}

// Fungsi TIM4_IRQHandler: menjalankan operasi tim4 irqhandler sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void TIM4_IRQHandler(void) {
    if ((TIM4->SR & TIM_SR_UIF) != 0U) {
        TIM4->SR &= ~TIM_SR_UIF;
        if (g_motor_left.sensor_mode == SENSOR_MODE_ENCODER) {
            if ((TIM4->CR1 & TIM_CR1_DIR) != 0U) {
                g_motor_left.encoder.turns--;
            }
            else {
                g_motor_left.encoder.turns++;
            }
        }
    }
}

// Fungsi fatal_exception: menjalankan operasi fatal exception sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void fatal_exception(void) {
    motor_hw_emergency_all_off();
    __disable_irq();
    while (1) {
    }
}

// Fungsi HardFault_Handler: menangani interrupt perangkat keras terkait dengan jalur sesingkat mungkin dan
// menjaga state keselamatan firmware.
void HardFault_Handler(void) {
    fatal_exception();
}
// Fungsi MemManage_Handler: menangani mem manage handler pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
void MemManage_Handler(void) {
    fatal_exception();
}
// Fungsi BusFault_Handler: menangani bus fault handler pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
void BusFault_Handler(void) {
    fatal_exception();
}
// Fungsi UsageFault_Handler: menangani usage fault handler pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
void UsageFault_Handler(void) {
    fatal_exception();
}
