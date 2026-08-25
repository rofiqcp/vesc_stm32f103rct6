#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H
#include <stdint.h>
typedef struct { volatile uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RCR,CCR1,CCR2,CCR3,CCR4,BDTR,DCR,DMAR; } TIM_TypeDef;
typedef struct { void *Instance; void *DMA_Handle; } ADC_HandleTypeDef;
typedef struct { void *Instance; } DMA_HandleTypeDef;
typedef struct { volatile uint32_t CCR,CNDTR,CPAR,CMAR; } DMA_Channel_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { void *Instance; } IWDG_HandleTypeDef;

extern TIM_TypeDef _stub_tim1, _stub_tim8;
extern DMA_Channel_TypeDef _stub_dma2_ch5;
#define TIM1 (&_stub_tim1)
#define TIM8 (&_stub_tim8)
#define DMA2_Channel5 (&_stub_dma2_ch5)
typedef struct { volatile uint32_t CYCCNT; } DWT_Type;
extern DWT_Type _stub_dwt;
#define DWT (&_stub_dwt)
#define TIM_BDTR_MOE (1U << 15)
static inline uint32_t __get_PRIMASK(void){return 0U;}
static inline void __disable_irq(void){}
static inline void __enable_irq(void){}
static inline void __DMB(void){}
#endif
