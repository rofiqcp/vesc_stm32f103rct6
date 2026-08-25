#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H
#include <stdint.h>
#include <stddef.h>

typedef struct { volatile uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RCR,CCR1,CCR2,CCR3,CCR4,BDTR,DCR,DMAR; } TIM_TypeDef;
typedef struct { volatile uint32_t SR,CR1,CR2,SMPR1,SMPR2,JOFR1,JOFR2,JOFR3,JOFR4,HTR,LTR,SQR1,SQR2,SQR3,JSQR,JDR1,JDR2,JDR3,JDR4,DR; } ADC_TypeDef;
typedef struct { volatile uint32_t CCR,CNDTR,CPAR,CMAR; } DMA_Channel_TypeDef;
typedef struct { volatile uint32_t ISR,IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t CR,CFGR,CIR,APB2RSTR,APB1RSTR,AHBENR,APB2ENR,APB1ENR,BDCR,CSR; } RCC_TypeDef;
typedef struct { volatile uint32_t SR,DR,BRR,CR1,CR2,CR3,GTPR; } USART_TypeDef;
typedef struct { volatile uint32_t EVCR,MAPR,EXTICR[4],MAPR2; } AFIO_TypeDef;
typedef struct { volatile uint32_t CRL,CRH,IDR,ODR,BSRR,BRR,LCKR; } GPIO_TypeDef;

typedef struct { void *Instance; void *DMA_Handle; } ADC_HandleTypeDef;
typedef struct { void *Instance; } DMA_HandleTypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } IWDG_HandleTypeDef;
typedef struct { uint32_t Pin,Mode,Speed,Pull; } GPIO_InitTypeDef;
typedef struct { uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling; } UART_InitTypeDef;
typedef struct { USART_TypeDef *Instance; UART_InitTypeDef Init; } UART_HandleTypeDef;

extern TIM_TypeDef _stub_tim1, _stub_tim2, _stub_tim3, _stub_tim8;
extern ADC_TypeDef _stub_adc1,_stub_adc2,_stub_adc3;
extern DMA_TypeDef _stub_dma1,_stub_dma2;
extern RCC_TypeDef _stub_rcc;
extern DMA_Channel_TypeDef _stub_dma1_ch1,_stub_dma1_ch2,_stub_dma1_ch3,_stub_dma2_ch5;
extern USART_TypeDef _stub_usart3;
extern AFIO_TypeDef _stub_afio;
extern GPIO_TypeDef _stub_gpioa, _stub_gpiob;

#define TIM1 (&_stub_tim1)
#define TIM2 (&_stub_tim2)
#define TIM3 (&_stub_tim3)
#define TIM8 (&_stub_tim8)
#define ADC1 (&_stub_adc1)
#define ADC2 (&_stub_adc2)
#define ADC3 (&_stub_adc3)
#define DMA1 (&_stub_dma1)
#define DMA2 (&_stub_dma2)
#define DMA1_Channel1 (&_stub_dma1_ch1)
#define DMA1_Channel2 (&_stub_dma1_ch2)
#define DMA1_Channel3 (&_stub_dma1_ch3)
#define DMA2_Channel5 (&_stub_dma2_ch5)
#define RCC (&_stub_rcc)
#define USART3 (&_stub_usart3)
#define AFIO (&_stub_afio)
#define GPIOA (&_stub_gpioa)
#define GPIOB (&_stub_gpiob)

typedef struct { volatile uint32_t CYCCNT; } DWT_Type;
extern DWT_Type _stub_dwt;
#define DWT (&_stub_dwt)

#define TIM_BDTR_MOE (1U << 15)
#define TIM_CR1_DIR (1U<<4)
#define TIM_CR1_CEN (1U<<0)
#define TIM_DIER_UIE (1U<<0)
#define TIM_SR_UIF (1U<<0)
#define TIM_EGR_UG (1U<<0)
#define ADC_CR2_ADON (1U<<0)
#define DMA_CCR_EN (1U<<0)
#define DMA_CCR_MINC (1U<<7)
#define DMA_CCR_CIRC (1U<<5)
#define DMA_CCR_PL_1 (1U<<13)
#define DMA_CCR_HTIE (1U<<2)
#define DMA_CCR_TCIE (1U<<1)
#define DMA_CCR_TEIE (1U<<3)
#define DMA_CCR_DIR (1U<<4)
#define DMA_IFCR_CGIF2 (1U<<4)
#define DMA_IFCR_CGIF3 (1U<<8)
#define DMA_IFCR_CHTIF3 (1U<<10)
#define DMA_IFCR_CTCIF2 (1U<<5)
#define DMA_IFCR_CTCIF3 (1U<<9)
#define DMA_ISR_HTIF3 (1U<<10)
#define DMA_ISR_TCIF2 (1U<<5)
#define DMA_ISR_TCIF3 (1U<<9)
#define DMA_ISR_TEIF2 (1U<<7)
#define DMA_ISR_TEIF3 (1U<<11)

#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define GPIO_PIN_10 (1U<<10)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_MODE_OUTPUT_PP 0U
#define GPIO_MODE_AF_PP 1U
#define GPIO_MODE_INPUT 2U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_LOW 1U
#define GPIO_SPEED_FREQ_HIGH 3U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U

#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 3U
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 0U
#define USART_CR1_RXNEIE (1U<<5)
#define USART_CR1_TXEIE (1U<<7)
#define USART_CR1_TCIE (1U<<6)
#define USART_CR1_IDLEIE (1U<<4)
#define USART_CR1_PEIE (1U<<8)
#define USART_CR3_DMAR (1U<<6)
#define USART_CR3_DMAT (1U<<7)
#define USART_CR3_EIE (1U<<0)
#define USART_SR_ORE (1U<<3)
#define USART_SR_NE (1U<<2)
#define USART_SR_FE (1U<<1)
#define USART_SR_PE (1U<<0)
#define USART_SR_IDLE (1U<<4)
#define AFIO_MAPR_USART3_REMAP (3U<<4)

#define TIM3_IRQn 29
#define DMA1_Channel2_IRQn 12
#define DMA1_Channel3_IRQn 13
#define USART3_IRQn 39
#define HAL_OK 0
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)

static inline void NVIC_SystemReset(void){}
static inline uint32_t __get_PRIMASK(void){return 0U;}
static inline void __disable_irq(void){}
static inline void __enable_irq(void){}
static inline void __DMB(void){}
static inline void HAL_GPIO_Init(GPIO_TypeDef*p,GPIO_InitTypeDef*g){(void)p;(void)g;}
static inline void HAL_GPIO_WritePin(GPIO_TypeDef*p,uint32_t pin,uint32_t state){(void)p;(void)pin;(void)state;}
static inline void HAL_Delay(uint32_t ms){(void)ms;}
static inline int HAL_UART_Init(UART_HandleTypeDef*h){(void)h;return HAL_OK;}
static inline void HAL_NVIC_SetPriority(int irq,uint32_t p,uint32_t s){(void)irq;(void)p;(void)s;}
static inline void HAL_NVIC_EnableIRQ(int irq){(void)irq;}

#endif
