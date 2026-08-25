#ifndef CMSIS_OS2_H
#define CMSIS_OS2_H
#include <stdint.h>
#include <stddef.h>
typedef void *osThreadId_t; typedef void *osMutexId_t; typedef void *osMessageQueueId_t;
typedef int32_t osStatus_t; typedef int32_t osPriority_t;
#define osOK 0
#define osError (-1)
#define osWaitForever 0xFFFFFFFFU
#define osFlagsWaitAny 0U
#define osFlagsError 0x80000000U
#define osMutexPrioInherit 0x00000001U
#define osPriorityLow 8
#define osPriorityBelowNormal 16
#define osPriorityNormal 24
#define osPriorityAboveNormal 32
#define osPriorityHigh 40
typedef struct {const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size;} osMutexAttr_t;
typedef struct {const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size;} osMessageQueueAttr_t;
typedef struct {const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size; void *stack_mem; uint32_t stack_size; osPriority_t priority; uint32_t tz_module; uint32_t reserved;} osThreadAttr_t;
static inline osThreadId_t osThreadGetId(void){return (osThreadId_t)1;}
static inline uint32_t osThreadGetStackSpace(osThreadId_t id){(void)id;return 1024U;}
static inline uint32_t osKernelGetTickCount(void){return 0U;}
static inline osStatus_t osDelay(uint32_t ms){(void)ms; return osOK;}
static inline osStatus_t osThreadYield(void){return osOK;}
static inline uint32_t osThreadFlagsSet(osThreadId_t id,uint32_t f){(void)id;return f;}
static inline uint32_t osThreadFlagsWait(uint32_t f,uint32_t o,uint32_t t){(void)o;(void)t;return f;}
static inline osStatus_t osDelayUntil(uint32_t t){(void)t;return osOK;}
static inline osMutexId_t osMutexNew(const osMutexAttr_t*a){(void)a;return (void*)1;}
static inline osStatus_t osMutexAcquire(osMutexId_t m,uint32_t t){(void)m;(void)t;return osOK;}
static inline osStatus_t osMutexRelease(osMutexId_t m){(void)m;return osOK;}
static inline osMessageQueueId_t osMessageQueueNew(uint32_t a,uint32_t b,const osMessageQueueAttr_t*c){(void)a;(void)b;(void)c;return (void*)1;}
static inline osStatus_t osMessageQueuePut(osMessageQueueId_t q,const void*a,uint8_t p,uint32_t t){(void)q;(void)a;(void)p;(void)t;return osOK;}
static inline osStatus_t osMessageQueueGet(osMessageQueueId_t q,void*a,uint8_t*p,uint32_t t){(void)q;(void)a;(void)p;(void)t;return osError;}
static inline uint32_t osMessageQueueGetCount(osMessageQueueId_t q){(void)q;return 0;}
static inline osThreadId_t osThreadNew(void(*f)(void*),void*a,const osThreadAttr_t*b){(void)f;(void)a;(void)b;return (void*)1;}
#endif
