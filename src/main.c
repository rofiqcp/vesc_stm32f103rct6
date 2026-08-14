#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include "motor_hw.h"
#include "motor_control.h"
#include "motor_tasks.h"
#include "foc_control.h"
#include "telemetry.h"
#include "debug_sample.h"
#include "vesc_comm.h"
#include "config_store.h"
#include "vesc_config.h"
#include "status_io.h"

static void SystemClock_Config(void);
static void dwt_init(void);
static void early_fatal(void);
static void motor_boot_thread(void *argument);

int main(void) {
    HAL_Init();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* PB2 LED + PA4 buzzer + PA5 power-hold are alive before PLL/motor init. */
    status_io_early_gpio_init();

    SystemClock_Config();
    dwt_init();
    status_io_tone_timer_init();

    /* Handshake-first boot architecture.
     *
     * Upstream app_uartcomm is an independent communication subsystem. Do not
     * make COMM_FW_VERSION depend on ADC/PWM/FOC initialization succeeding.
     * Initialize the RTOS and VESC UART/packet resources first, then perform
     * all motor hardware startup from a lower-priority boot thread after the
     * scheduler is already running. */
    if (osKernelInitialize() != osOK) {
        early_fatal();
    }

    /* LED and buzzer threads start independently of motor/ADC startup. */
    if (!status_threads_init()) {
        early_fatal();
    }

    if (!vesc_comm_task_init()) {
        early_fatal();
    }

    const osThreadAttr_t boot_attr = {
        .name = "motor_boot_thread",
        .priority = osPriorityBelowNormal,
        .stack_size = 2048U
    };
    if (osThreadNew(motor_boot_thread, NULL, &boot_attr) == NULL) {
        early_fatal();
    }

    if (osKernelStart() != osOK) {
        early_fatal();
    }

    while (1) { }
}

static void motor_boot_thread(void *argument) {
    (void)argument;

    /* packet_process_thread is Normal while this boot helper is BelowNormal, so a
     * FW_VERSION request preempts motor initialization. Any motor HAL failure stays confined to
     * this thread and must not globally disable USART interrupts. */
    motor_hw_init();
    motor_control_init();

    /* V11 configuration is version-isolated from the experimental V9 record.
     * Build the exact VESC-6.00 wire defaults, then import only a V11 CRC-valid
     * record. Failure simply keeps compiled safe defaults; communication is
     * already alive in the higher-priority packet thread. */
    vesc_config_init_defaults();
    bool loaded_cfg = config_store_load_apply();
    if (!loaded_cfg) {
        /* GET_MCCONF can legally arrive before this boot thread runs. In that
           case vesc_config_init_defaults() intentionally built a safe wire
           image without reading zeroed MotorRuntime. Now publish the actual
           initialized runtime defaults before motor control is enabled. */
        vesc_config_sync_motor_runtime(MOTOR_LEFT);
        vesc_config_sync_motor_runtime(MOTOR_RIGHT);
    }

    telemetry_init();
    debug_sample_init();
    motor_threads_init();

    /* Start synchronized PWM counters + dual ADC DMA only after communication
     * is alive. TIM1/TIM8 MOE remain off through current-zero calibration. */
    motor_hw_start_sampling();

    vesc_comm_set_motor_ready(true);
    osThreadExit();
}

static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* Hoverboard mainboard reference clock: internal HSI / 2 * 16 = 64 MHz.
     * Do not depend on an external HSE crystal that may not exist on the board. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
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

    SystemCoreClockUpdate();
}

static void early_fatal(void) {
    /* Keep interrupts/tick alive so PB2/PA4 visibly report an early failure. */
    status_io_early_fatal_loop();
}
