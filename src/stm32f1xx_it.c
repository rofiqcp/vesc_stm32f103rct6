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
    // Variabel isr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t isr = DMA2->ISR;
    if ((isr & DMA_ISR_TEIF5) != 0U) {
        DMA2->IFCR = DMA_IFCR_CGIF5;
        motor_hw_emergency_all_off();
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        return;
    }

    /* HT/TC interrupt ADC3 selalu dinonaktifkan pada start sampling. Flag
       HTIF5/TCIF5 sengaja dibiarkan latched di sini karena ISR FOC memakai
       kedua flag itu sebagai bukti hardware bahwa sampel DCLINK baru benar-
       benar ditransfer. Handler bersama ini hanya menangani transfer error. */
}

// Fungsi DMA1_Channel1_IRQHandler: menangani interrupt perangkat keras terkait dengan jalur sesingkat mungkin
// dan menjaga state keselamatan firmware.
void DMA1_Channel1_IRQHandler(void) {
    // Variabel irq_start_cycle: timestamp paling awal untuk budget end-to-end IRQ DMA dan FOC.
    const uint32_t irq_start_cycle = DWT->CYCCNT;
    // Variabel isr: snapshot flag DMA1 sebelum flag apa pun dibersihkan.
    const uint32_t isr = DMA1->ISR;

    if ((isr & DMA_ISR_TEIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CGIF1;
        motor_hw_emergency_all_off();
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        mcpwm_foc_finish_irq_timing(irq_start_cycle);
        return;
    }

    /* Rank 1-3 dual-ADC berisi enam kanal arus stock-board. Half-transfer
       menjadi titik masuk paling awal yang sudah koheren untuk kedua loop FOC
       16 kHz. ISR ini tidak memanggil kernel, UART, flash, printf, atau HAL yang
       blocking; APP ADC PA2/PA3 rank-4 diproses di task 1 kHz. */
    if ((isr & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        /* Rank 1..3 frame sekarang sudah koheren. Rank 4/5 dari frame
         * sebelumnya masih stabil pada titik ini: latch PA2/PA3+temperature
         * dengan beberapa load/store sebelum masuk FOC. */
        motor_hw_capture_app_adc_from_isr();
        foc_adc_dma_isr_timed(g_adc_dual_dma, irq_start_cycle);
        return;
    }

    /* Secara normal IRQ hanya muncul karena HT atau TE yang diaktifkan. Jika
       ada IRQ liar/stale, tetap tutup statistik agar tidak ada jalur keluar
       yang luput dari pencatatan budget cycle. */
    mcpwm_foc_finish_irq_timing(irq_start_cycle);
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
