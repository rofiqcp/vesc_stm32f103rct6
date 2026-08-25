#ifndef TASK_H
#define TASK_H
#ifndef INC_FREERTOS_H
#error "include FreeRTOS.h must appear in source files before include task.h"
#endif
typedef void* TaskHandle_t;
static inline unsigned long uxTaskGetStackHighWaterMark(TaskHandle_t x){(void)x;return 100;}
static inline size_t xPortGetFreeHeapSize(void){return 4096;}
static inline size_t xPortGetMinimumEverFreeHeapSize(void){return 3000;}
#endif
