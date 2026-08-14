#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "motor_tasks.h"
#include "foc_control.h"
#include "telemetry.h"
#include "debug_sample.h"
#include "vesc_comm.h"

static void SystemClock_Config(void);
static void dwt_init(void);
static void early_fatal(void);

int main(void) {
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    SystemClock_Config();
    dwt_init();

    motor_hw_init();
    motor_control_init();
    telemetry_init();
    debug_sample_init();

    if (osKernelInitialize() != osOK) {
        motor_hw_emergency_all_off();
        while (1) { }
    }

    /* Reserve communication resources first so COMM_FW_VERSION remains
       available even if later motor-task allocation is under memory pressure.
       Communication: packet_process + blocking. USART3 RX/TX bytes are moved
       by RXNE/TXE/TC IRQs through software rings; there is no UART DMA thread.
       Fast FOC remains in the DMA1 Channel1 ISR. */
    vesc_comm_task_init();
    motor_threads_init();

    /* Start synchronized PWM counters + dual ADC DMA. Both TIM1/TIM8 MOE
       remain disabled while the mandatory startup current-zero calibration
       runs in DMA1_Channel1_IRQHandler(). */
    motor_hw_start_sampling();

    if (osKernelStart() != osOK) {
        motor_hw_emergency_all_off();
    }

    while (1) { }
}

static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.HSIState = RCC_HSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        early_fatal();
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        early_fatal();
    }
}

static void early_fatal(void) {
    __disable_irq();
    while (1) { }
}
