#ifndef CMSIS_OS2_H
#define CMSIS_OS2_H
#include <stdint.h>
typedef void *osThreadId_t;
static inline osThreadId_t osThreadGetId(void){return (osThreadId_t)1;}
static inline uint32_t osKernelGetTickCount(void){return 0U;}
static inline int32_t osDelay(uint32_t ms){(void)ms; return 0;}
#endif
