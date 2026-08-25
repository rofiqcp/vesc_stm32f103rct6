#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include "hwconf/hw.h"
#include "motor/mc_interface.h"
#include "motor/mc_interface_sample.h"
#include "motor/mcpwm_foc.h"
#include "telemetry.h"
#include "comm/commands.h"
#include "conf_general.h"
#include "confgenerator.h"
#include "timeout.h"

static void SystemClock_Config(void);
static void dwt_init(void);
static void early_fatal(void);
static void motor_boot_thread(void *argument);

int main(void) {
    HAL_Init();
    timeout_capture_reset_reason();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* PB2 LED + PA4 buzzer + PA5 power-hold are alive before PLL/motor init. */
    hw_status_early_init();

    SystemClock_Config();
    dwt_init();
    hw_status_timer_init();

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

    /* Satu status thread non-blocking (LED+buzzer) start independen dari motor/ADC. */
    if (!hw_status_init()) {
        early_fatal();
    }

    commands_init();
    if (!commands_is_initialized()) {
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

    /* uartcomm proc berprioritas lebih tinggi daripada boot helper, sehingga
     * FW_VERSION dapat mem-preempt inisialisasi motor. Any motor HAL failure stays confined to
     * this thread and must not globally disable USART interrupts. */
    motor_hw_init();
    mc_interface_init(false);

    /* Konfigurasi VESC 6.00 dipisahkan dari record eksperimen lama.
     * Build the exact VESC-6.00 wire defaults, then import only a CRC-valid
     * transactional record. Failure simply keeps compiled safe defaults; communication is
     * already alive in the higher-priority packet thread. */
    vesc_config_init_defaults();
    if (!vesc_config_apply_defaults()) {
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
        vesc_comm_set_config_ready(true);
        osThreadExit();
    }
    bool loaded_cfg = conf_general_init();
    if (!loaded_cfg) {
        /* Virgin erased flash is a valid first boot and uses compiled safe
           defaults. Non-blank flash with no valid transactional record is
           corruption: keep defaults for communication/repair but latch a
           configuration fault so neither bridge can enable until repaired and
           rebooted. */
        if (conf_general_boot_status() == CONF_BOOT_CORRUPT) {
            motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
            motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
        }
    }
    /* From this point VESC Tool may safely repair/write MC/APPCONF even when a
       later telemetry/control-task startup step fails. */
    vesc_comm_set_config_ready(true);

    if (!telemetry_init()) {
        /* Communication remains alive, but do not expose a motor-ready state
           without an atomic telemetry snapshot path. */
        osThreadExit();
    }
    mc_interface_sample_init();
    if (!mc_interface_start_threads()) {
        /* One missing control/stat/periodic thread is a partial controller,
           not a ready VESC. Keep UART handshakes alive and refuse motor use. */
        osThreadExit();
    }

    /* Start synchronized PWM counters + dual ADC DMA only after communication
     * is alive. TIM1/TIM8 MOE remain off through current-zero calibration. */
    motor_hw_start_sampling();
    if (!motor_hw_sampling_contract_valid()) {
        /* A timer/ADC/DMA contract mismatch can invalidate current feedback.
         * Leave USART3 alive for diagnostics/config repair, but never expose
         * the controller as motor-ready. */
        motor_hw_emergency_all_off();
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        osThreadExit();
    }
    timeout_watchdog_require_foc(true);

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
    hw_status_early_fatal_loop();
}
