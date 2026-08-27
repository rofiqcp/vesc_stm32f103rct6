#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hwconf/hw.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "applications/app_command.h"
#include "applications/app_adc.h"
#include "telemetry.h"
#include "comm/commands.h"
#include "conf_general.h"
#include "confgenerator.h"
#include "timeout.h"

static void SystemClock_Config(void);
static void dwt_init(void);
static void early_fatal(void);

/* VESC-standard worker names, with SmartESC-like priority distribution. */
#define VESC_PRIO_LOW       ((UBaseType_t)4U)
#define VESC_PRIO_NORMAL    ((UBaseType_t)5U)
#define VESC_PRIO_SAFETY    ((UBaseType_t)6U)

extern void packet_process_thread(void *argument);
extern void blocking_thread(void *argument);
extern void timer_thread(void *argument);
extern void sample_send_thread(void *argument);
extern void fault_stop_thread(void *argument);

static bool create_vesc_tasks(void) {
    TaskHandle_t packet = NULL, blocking = NULL;
    TaskHandle_t timer = NULL, sample = NULL, fault = NULL;

    if (xTaskCreate(fault_stop_thread, "fault_stop", 192U, NULL, VESC_PRIO_SAFETY, &fault) != pdPASS) return false;
    if (xTaskCreate(timer_thread, "timer", 256U, NULL, VESC_PRIO_NORMAL, &timer) != pdPASS) return false;
    if (xTaskCreate(packet_process_thread, "packet_process", 384U, NULL, VESC_PRIO_NORMAL, &packet) != pdPASS) return false;
    if (xTaskCreate(blocking_thread, "blocking", 768U, NULL, VESC_PRIO_NORMAL, &blocking) != pdPASS) return false;
    if (xTaskCreate(sample_send_thread, "sample_send", 192U, NULL, VESC_PRIO_LOW, &sample) != pdPASS) return false;

    vesc_comm_set_thread_ids(packet, blocking);
    mc_interface_set_thread_ids(timer, sample, fault);
    return true;
}

int main(void) {
    HAL_Init();
    timeout_capture_reset_reason();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    hw_status_early_init();
    SystemClock_Config();
    dwt_init();
    hw_status_timer_init();
    if (!hw_status_init()) early_fatal();

    /* Match SmartESC's boot model: hardware/configuration is initialized before
     * vTaskStartScheduler(), while all run-time work is owned by five tasks. */
    motor_hw_init();

    /* Initialize the VESC motor runtime before applying wire configuration.
     * The previous boot path read g_motor_left.foc_calibrate_on_boot
     * before motor_defaults(), which made the zero-initialized flag suppress
     * boot current calibration. The VESC6 wire image does not carry this
     * runtime-only field, so use its compiled default before FOC init. */
    foc_calibration_set_skip(!MCCONF_FOC_CALIBRATE_ON_BOOT_DEFAULT);
    mc_interface_init(false);

    vesc_config_init_defaults();
    if (!vesc_config_apply_defaults()) {
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
    }

    const bool loaded_cfg = conf_general_init();
    if (!loaded_cfg && conf_general_boot_status() == CONF_BOOT_CORRUPT) {
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
    }
    if (!telemetry_init()) early_fatal();
    mc_interface_sample_init();
    app_command_init();
    app_adc_init();
    if (!timeout_init()) early_fatal();

    /* Create all five tasks before resources that validate their handles. */
    if (!create_vesc_tasks()) early_fatal();
    commands_init();
    if (!commands_is_initialized()) early_fatal();
    vesc_comm_set_config_ready(true);
    if (!mc_interface_start_threads()) early_fatal();

    motor_hw_start_sampling();
    if (!motor_hw_sampling_contract_valid()) {
        motor_hw_emergency_all_off();
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        early_fatal();
    }
    timeout_watchdog_require_foc(true);
    vesc_comm_set_motor_ready(true);

    vTaskStartScheduler();
    early_fatal();
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
