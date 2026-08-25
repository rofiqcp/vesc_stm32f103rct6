#pragma once

#include <stdint.h>
extern uint32_t SystemCoreClock;

/* CMSIS-RTOS2 wrapper configuration for STM32CubeF1 1.8.6.
 * freertos_os2.h includes CMSIS_device_header after FreeRTOS.h, therefore the
 * macro must expand to a quoted CMSIS device header name. */
#ifndef CMSIS_device_header
#define CMSIS_device_header                    "stm32f1xx.h"
#endif

/* We do not use osThreadEnumerate(). Disabling it avoids enabling the FreeRTOS
 * trace facility solely for that optional CMSIS-RTOS2 API. */
#define configUSE_OS2_THREAD_ENUMERATE          0

/* cmsis_os2.c from CubeF1 otherwise supplies its own SysTick_Handler. The
 * application provides a custom handler that advances both HAL and FreeRTOS. */
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 1

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    56
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
/* STM32F103RCT6 has exactly 48 KiB SRAM. 18 KiB is sufficient for the
 * seven application threads, idle/timer tasks, mutexes and the one-entry
 * blocking queue while leaving several KiB linker headroom. Do not grow
 * this without checking the final .map file. */
#define configTOTAL_HEAP_SIZE                   ((size_t)(18 * 1024))
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_TRACE_FACILITY                0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            256

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xSemaphoreGetMutexHolder         1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           1

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                         __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                         4
#endif
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
/* SysTick_Handler is intentionally provided by stm32f1xx_it.c so HAL tick and
   FreeRTOS tick both advance from the same 1 kHz interrupt. */

#define configASSERT(x) do { if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;){} } } while(0)
