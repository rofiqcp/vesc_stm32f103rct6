#include "FreeRTOS.h"
#include "task.h"
#include "hwconf/hw.h"

// Fungsi vApplicationMallocFailedHook: menjalankan operasi v application malloc failed hook sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void vApplicationMallocFailedHook(void) {
    motor_hw_emergency_all_off();
    taskDISABLE_INTERRUPTS();
    for (;; ) {
    }
}

// Parameter xTask: handle, stack, atau state task FreeRTOS.
// Parameter pcTaskName: handle, stack, atau state task FreeRTOS.
// Fungsi vApplicationStackOverflowHook: menjalankan operasi v application stack overflow hook sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    motor_hw_emergency_all_off();
    taskDISABLE_INTERRUPTS();
    for (;; ) {
    }
}
