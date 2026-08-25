#include "FreeRTOS.h"
#include "task.h"
#include "hwconf/hw.h"

void vApplicationMallocFailedHook(void) {
    motor_hw_emergency_all_off();
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    motor_hw_emergency_all_off();
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
