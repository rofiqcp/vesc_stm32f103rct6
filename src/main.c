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

/* Debug-visible boot breadcrumbs. They cost 8 bytes of RAM and make OpenOCD
 * diagnosis deterministic when the UART cannot yet report a failure. */
volatile uint32_t g_vesc_boot_stage = 0U;
volatile uint32_t g_vesc_boot_error = 0U;
volatile uint32_t g_vesc_sampling_contract_flags = 0U;

/* Export task handles for OpenOCD/GDB commissioning. They are intentionally
 * globals (rather than function locals) so thread creation failures can be
 * diagnosed even before the scheduler starts. */
TaskHandle_t g_task_fault_stop = NULL;
TaskHandle_t g_task_timer = NULL;
TaskHandle_t g_task_packet_process = NULL;
TaskHandle_t g_task_blocking = NULL;
TaskHandle_t g_task_sample_send = NULL;


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
    if (xTaskCreate(fault_stop_thread, "fault_stop", 192U, NULL, VESC_PRIO_SAFETY, &g_task_fault_stop) != pdPASS) return false;
    if (xTaskCreate(timer_thread, "timer", 256U, NULL, VESC_PRIO_NORMAL, &g_task_timer) != pdPASS) return false;
    if (xTaskCreate(packet_process_thread, "packet_process", 384U, NULL, VESC_PRIO_NORMAL, &g_task_packet_process) != pdPASS) return false;
    if (xTaskCreate(blocking_thread, "blocking", 768U, NULL, VESC_PRIO_NORMAL, &g_task_blocking) != pdPASS) return false;
    if (xTaskCreate(sample_send_thread, "sample_send", 192U, NULL, VESC_PRIO_LOW, &g_task_sample_send) != pdPASS) return false;

    vesc_comm_set_thread_ids(g_task_packet_process, g_task_blocking);
    mc_interface_set_thread_ids(g_task_timer, g_task_sample_send, g_task_fault_stop);
    return true;
}

int main(void) {
    g_vesc_boot_stage = 1U;
    HAL_Init();
#if defined(VESC_DEBUG_BUILD) && defined(DBGMCU_CR_DBG_IWDG_STOP)
    /* If IWDG is deliberately enabled later, freeze it whenever Cortex-M3 is
     * halted by ST-Link/GDB. */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
#endif
    timeout_capture_reset_reason();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    g_vesc_boot_stage = 10U;
    hw_status_early_init();
    SystemClock_Config();
    dwt_init();
    hw_status_timer_init();
    if (!hw_status_init()) { g_vesc_boot_error = 11U; early_fatal(); }

    /* Create the five native-FreeRTOS task objects first, but do not start the
     * scheduler yet. This gives commands.c the packet/blocking task handles it
     * validates while keeping the exact five-application-task architecture. */
    g_vesc_boot_stage = 20U;
    if (!create_vesc_tasks()) { g_vesc_boot_error = 21U; early_fatal(); }

    /* Bring up the permanent USART3 VESC management transport BEFORE the motor
     * power-stage/ADC initialization. If motor hardware init fails, hw.c can
     * still poll this parser and answer COMM_FW_VERSION for diagnosis/recovery. */
    g_vesc_boot_stage = 30U;
    commands_init();
    if (!commands_is_initialized()) { g_vesc_boot_error = 31U; early_fatal(); }

    /* Match SmartESC's separation: communication is independent of the hard
     * motor-control startup, while FOC remains interrupt-driven once sampling
     * is enabled. */
    g_vesc_boot_stage = 40U;
    motor_hw_init();

    /* Initialize the VESC motor runtime before applying wire configuration. */
    g_vesc_boot_stage = 50U;
    foc_calibration_set_skip(!MCCONF_FOC_CALIBRATE_ON_BOOT_DEFAULT);
    mc_interface_init(false);

    vesc_config_init_defaults();
    g_vesc_boot_stage = 60U;
    if (!vesc_config_apply_defaults()) {
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
        g_vesc_boot_error = 61U;
    }

    const bool loaded_cfg = conf_general_init();
    if (!loaded_cfg && conf_general_boot_status() == CONF_BOOT_CORRUPT) {
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
    }
    g_vesc_boot_stage = 70U;
    if (!telemetry_init()) { g_vesc_boot_error = 71U; early_fatal(); }
    mc_interface_sample_init();
    app_command_init();
    app_adc_init();
    if (!timeout_init()) { g_vesc_boot_error = 72U; early_fatal(); }

    vesc_comm_set_config_ready(true);
    g_vesc_boot_stage = 80U;
    if (!mc_interface_start_threads()) { g_vesc_boot_error = 81U; early_fatal(); }

    g_vesc_boot_stage = 90U;
    motor_hw_start_sampling();
    g_vesc_sampling_contract_flags = motor_hw_sampling_contract_flags();
    if (g_vesc_sampling_contract_flags != 0U) {
        /* A sampling-contract failure is safety-critical for motor drive, but
         * it must NOT kill the VESC management link. Keep both bridges hard
         * off, latch ADC/DMA faults, inhibit motor commands and still start
         * FreeRTOS so packet_process can answer COMM_FW_VERSION/diagnostics. */
        motor_hw_emergency_all_off();
        motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_ADC_DMA);
        motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_ADC_DMA);
        g_vesc_boot_error = 91U;
        timeout_watchdog_require_foc(false);
        vesc_comm_set_motor_ready(false);
    } else {
        timeout_watchdog_require_foc(true);
        vesc_comm_set_motor_ready(true);
    }

    g_vesc_boot_stage = 100U;
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
