#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "applications/appconf_default.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "comm/commands.h"
#include "motor/foc_math.h"
#include "cmsis_os2.h"
#include <string.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
volatile uint32_t g_adc_dual_dma[6] __attribute__((aligned(4)));
volatile uint16_t g_adc3_vbus_dma[2] __attribute__((aligned(4)));
/* PA2/PA3 APP ADC values are captured from dual-ADC rank 6 at the next
 * half-transfer interrupt. This keeps application sampling outside the first
 * three current ranks and adds no extra 16-kHz interrupt. */
static volatile uint32_t s_app_adc_word = 0U;
static volatile uint32_t s_app_adc_seq = 0U;
static volatile uint8_t s_app_adc_ht_seen = 0U;

/* Power-stage faults are deliberately reset-latched. A software motor fault
 * may be cleared after its normal recovery policy, but PVD/BKIN events keep
 * MOE blocked until MCU reset because they indicate supply or external gate
 * integrity loss. */
#define POWERSTAGE_FAULT_PVD   (1UL << 0)
#define POWERSTAGE_FAULT_TIM1  (1UL << 1)
#define POWERSTAGE_FAULT_TIM8  (1UL << 2)
static volatile uint32_t s_powerstage_fault_flags = 0U;

#define TIM_CCMR1_OC1M_MASK_LOCAL (7UL << 4)
#define TIM_CCMR1_OC2M_MASK_LOCAL (7UL << 12)
#define TIM_CCMR2_OC3M_MASK_LOCAL (7UL << 4)
#define TIM_OCMODE_FORCED_INACTIVE_LOCAL (4UL)
#define TIM_OCMODE_PWM1_LOCAL            (6UL)
#define TIM_EGR_COMG_LOCAL               (1UL << 5)

static void Error_Handler_Local(void) {
    /* Motor-subsystem failure must not kill the UART communication stack.
     * main() starts motor hardware from a normal RTOS boot thread, so keeping
     * IRQs enabled lets packet_process_thread continue answering FW_VERSION. */
    motor_hw_emergency_all_off();
    for (;;) {
        osDelay(1000U);
    }
}

static uint32_t deadtime_to_dtg(uint32_t ns) {
    uint32_t ticks = (uint32_t)(((uint64_t)CPU_CLOCK_HZ * ns + 999999999ULL) / 1000000000ULL);
    if (ticks > 127U) ticks = 127U; /* baseline uses simple DTG region */
    return ticks;
}

static void init_gpio(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Keep SWD, release JTAG-only pins such as PB3/PB4/PB5. */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    GPIO_InitTypeDef g = {0};

    /* PWM high/low outputs */
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = LEFT_TIM_UH_PIN | LEFT_TIM_VH_PIN | LEFT_TIM_WH_PIN;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = LEFT_TIM_UL_PIN;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = LEFT_TIM_VL_PIN | LEFT_TIM_WL_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = RIGHT_TIM_UH_PIN | RIGHT_TIM_VH_PIN | RIGHT_TIM_WH_PIN;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = RIGHT_TIM_UL_PIN | RIGHT_TIM_VL_PIN | RIGHT_TIM_WL_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* Optional external hardware-break inputs. These pins are configured only
       when the matching backend is explicitly enabled for a validated PCB. */
#if HOVERBOARD_TIM1_BREAK_ENABLE
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = HOVERBOARD_TIM1_BREAK_ACTIVE_HIGH ? GPIO_PULLDOWN : GPIO_PULLUP;
    g.Pin = HOVERBOARD_TIM1_BKIN_PIN;
    HAL_GPIO_Init(HOVERBOARD_TIM1_BKIN_PORT, &g);
#endif
#if HOVERBOARD_TIM8_BREAK_ENABLE
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = HOVERBOARD_TIM8_BREAK_ACTIVE_HIGH ? GPIO_PULLDOWN : GPIO_PULLUP;
    g.Pin = HOVERBOARD_TIM8_BKIN_PIN;
    HAL_GPIO_Init(HOVERBOARD_TIM8_BKIN_PORT, &g);
#endif

    /* ADC analog inputs */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pin = LEFT_U_CUR_PIN | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &g); /* PA0 current, PA2 APP ADC1, PA3 APP ADC2 */
    g.Pin = LEFT_DC_CUR_PIN | LEFT_V_CUR_PIN | RIGHT_DC_CUR_PIN |
            RIGHT_U_CUR_PIN | RIGHT_V_CUR_PIN | DCLINK_PIN;
    HAL_GPIO_Init(GPIOC, &g);

    /* PB2 LED, PA4 buzzer and PA5 power-hold are initialized before the
       RTOS/motor subsystem by hw_status_early_init(). Do not reset them
       here; they are our boot-liveness indicators. */

    /* Button */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = BUTTON_PIN;
    HAL_GPIO_Init(BUTTON_PORT, &g);

    /* LEFT shared sensor pins start in safe Hall-input mode. Runtime sensor
       selection can reconfigure PB6/PB7 to TIM4 encoder without reboot. */
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_NOPULL;
    g.Pin = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* Right Hall PC10/11/12 */
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_NOPULL;
    g.Pin = RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN;
    HAL_GPIO_Init(GPIOC, &g);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void init_pwm_timer(TIM_HandleTypeDef *h, TIM_TypeDef *inst) {
    h->Instance = inst;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    h->Init.Period = PWM_TIMER_ARR;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.RepetitionCounter = 0;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(h) != HAL_OK) Error_Handler_Local();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = PWM_TIMER_ARR / 2U;
    /* Power stage polarity: top input HIGH=ON, bottom input LOW=ON.
       The timer still generates complementary CHx/CHxN with hardware deadtime. */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    /* MOE/OFF state must turn both MOSFETs off physically. */
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;   /* top pin LOW -> OFF */
    oc.OCNIdleState = TIM_OCNIDLESTATE_SET;  /* bottom pin HIGH -> OFF */
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler_Local();
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler_Local();
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, TIM_CHANNEL_3) != HAL_OK) Error_Handler_Local();

    TIM_BreakDeadTimeConfigTypeDef bd = {0};
    bd.OffStateRunMode = TIM_OSSR_ENABLE;
    bd.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bd.LockLevel = TIM_LOCKLEVEL_OFF;
    bd.DeadTime = deadtime_to_dtg(PWM_DEADTIME_NS);
    const bool break_enable = (inst == TIM1) ? (HOVERBOARD_TIM1_BREAK_ENABLE != 0)
                                            : (HOVERBOARD_TIM8_BREAK_ENABLE != 0);
    const bool break_high = (inst == TIM1) ? (HOVERBOARD_TIM1_BREAK_ACTIVE_HIGH != 0)
                                          : (HOVERBOARD_TIM8_BREAK_ACTIVE_HIGH != 0);
    bd.BreakState = break_enable ? TIM_BREAK_ENABLE : TIM_BREAK_DISABLE;
    bd.BreakPolarity = break_high ? TIM_BREAKPOLARITY_HIGH : TIM_BREAKPOLARITY_LOW;
    /* Never allow hardware automatic re-enable after a break event. */
    bd.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(h, &bd) != HAL_OK) Error_Handler_Local();

    /* CCR preload: the FOC ISR writes a coherent triplet that is latched by
       the next timer update event, avoiding mid-cycle waveform changes. */
    inst->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    inst->CCMR2 |= TIM_CCMR2_OC3PE;
    inst->CCR1 = PWM_TIMER_ARR / 2U;
    inst->CCR2 = PWM_TIMER_ARR / 2U;
    inst->CCR3 = PWM_TIMER_ARR / 2U;
    inst->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE;
    inst->BDTR &= ~TIM_BDTR_MOE;
}

static void init_timers(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Stock hoverboard timing, following the proven EFeru hardware port:
     *
     *   TIM1 RIGHT : center-aligned master, TRGO = ENABLE
     *   TIM8 LEFT  : gated from TIM1 ITR0, TRGO = UPDATE, RCR = 1
     *   ADC1/ADC2  : dual regular simultaneous, trigger = TIM8 TRGO
     *
     * TIM8 starts ADC_MOTOR_PHASE_OFFSET_TICKS ahead of TIM1. The offset is
     * exactly one phase-current conversion at the configured ADC divider, so
     * the LEFT and RIGHT shunts are sampled in their valid low-side windows.
     * RCR=1 yields one dual-ADC current frame per 16-kHz PWM period. */
    init_pwm_timer(&htim1, TIM1);
    init_pwm_timer(&htim8, TIM8);

    TIM1->CR2 |= TIM_CR2_CCPC;
    TIM8->CR2 |= TIM_CR2_CCPC;

    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_ENABLE;
    TIM1->SMCR &= ~TIM_SMCR_MSM;

    TIM8->SMCR = (TIM8->SMCR & ~(TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) |
                 TIM_TS_ITR0 | TIM_SLAVEMODE_GATED | TIM_SMCR_MSM;
    TIM8->CR2 = (TIM8->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_UPDATE;
    TIM8->RCR = 1U;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = LEFT_ENCODER_CPR - 1U;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    TIM_Encoder_InitTypeDef enc = {0};
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 6;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 6;
    if (HAL_TIM_Encoder_Init(&htim4, &enc) != HAL_OK) Error_Handler_Local();
    __HAL_TIM_DISABLE(&htim4);
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

static void cfg_adc_channel(ADC_HandleTypeDef *h, uint32_t ch, uint32_t rank,
                            uint32_t sample_time) {
    ADC_ChannelConfTypeDef c = {0};
    c.Channel = ch;
    c.Rank = rank;
    c.SamplingTime = sample_time;
    if (HAL_ADC_ConfigChannel(h, &c) != HAL_OK) Error_Handler_Local();
}

static void init_adc_dma(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* 64 MHz / 6 = 10.67 MHz ADC clock, within STM32F103 specification. */
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 6;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler_Local();
    /* STM32F103 high-density remap: ADC1 regular external trigger is TIM8 TRGO. */
    __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 6;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler_Local();

    /* ADC3 is dedicated to the stock PC2 DCLINK divider. On STM32F103xE the
       HAL maps ADC_EXTERNALTRIGCONV_T8_TRGO to ADC3 EXTSEL=100, so ADC3 gets
       the same TIM8 update edge as the dual current scan without the ADC1/2
       AFIO ETRGREG remap. One 28.5-cycle DCLINK conversion completes before
       the first three ADC1/ADC2 current ranks reach DMA half-transfer. */
    hadc3.Instance = ADC3;
    hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) Error_Handler_Local();

    /* ADC1 is the low halfword and ADC2 the high halfword of every 32-bit DMA
       word. Keep the first three dual ranks identical to the stock EFeru
       hardware schedule. DMA half-transfer then fires immediately after all
       six fast current channels are coherent:

         rank 1: ADC1 PC1 RIGHT DC | ADC2 PC0 LEFT DC   (1.5 cycles)
         rank 2: ADC1 PA0 LEFT  A  | ADC2 PC3 LEFT  B   (7.5 cycles)
         rank 3: ADC1 PC4 RIGHT B  | ADC2 PC5 RIGHT C   (7.5 cycles)

       Rank 4..5 are slow diagnostics; rank 6 is reserved for APP ADC PA2/PA3. */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5); /* RIGHT DC PC1 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5); /* LEFT  DC PC0 */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_0,  ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5); /* LEFT  A  PA0 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_13, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5); /* LEFT  B  PC3 */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5); /* RIGHT B  PC4 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_15, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5); /* RIGHT C  PC5 */
    /* Rank 4..6 stay after the DMA HT boundary. Ranks 4..5 are diagnostics;
       rank 6 is application ADC and never participates in FOC feedback.
       DCLINK is deliberately removed from ADC1/ADC2; ADC3 owns PC2 now. */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_28CYCLES_5); /* duplicate RIGHT DC */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_28CYCLES_5); /* duplicate LEFT DC */
    /* Rank 5 is after DMA half-transfer, so the long internal-temperature
       conversion cannot delay the 16-kHz current-control ISR. */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_239CYCLES_5); /* MCU/board temp */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_11, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_239CYCLES_5); /* match ADC1 rank5 simultaneous timing */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_2, ADC_REGULAR_RANK_6, ADC_SAMPLETIME_28CYCLES_5); /* APP ADC1 PA2 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_3, ADC_REGULAR_RANK_6, ADC_SAMPLETIME_28CYCLES_5); /* APP ADC2 PA3 */

    cfg_adc_channel(&hadc3, ADC_CHANNEL_12, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_28CYCLES_5); /* DCLINK PC2 */

    ADC_MultiModeTypeDef multi = {0};
    multi.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multi) != HAL_OK) Error_Handler_Local();

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler_Local();
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) Error_Handler_Local();
    if (HAL_ADCEx_Calibration_Start(&hadc3) != HAL_OK) Error_Handler_Local();

    /* STM32F1 internal temperature/Vref path enable. HAL versions normally
       set this when ADC_CHANNEL_TEMPSENSOR is configured; doing it explicitly
       keeps the hardware contract visible and robust across HAL revisions. */
#ifdef ADC_CR2_TSVREFE
    ADC1->CR2 |= ADC_CR2_TSVREFE;
#endif

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler_Local();
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0); /* NEVER call RTOS here */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* ADC3 regular conversions have their own DMA request on DMA2 Channel 5
       on STM32F103 high-density devices. A two-sample circular buffer makes
       CNDTR alternate 1/2 every PWM trigger, which lets the FOC ISR detect a
       stale Vbus DMA stream without enabling a 16-kHz ADC3 TC interrupt. */
    hdma_adc3.Instance = DMA2_Channel5;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) Error_Handler_Local();
    __HAL_LINKDMA(&hadc3, DMA_Handle, hdma_adc3);

    /* Transfer error is asynchronous safety-critical; HT/TC remain disabled. */
    HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
}


void motor_hw_init(void) {
    init_gpio();
    init_timers();
    init_adc_dma();
}

static void init_powerstage_safety(void) {
#if HOVERBOARD_PVD_ENABLE
    __HAL_RCC_PWR_CLK_ENABLE();
    /* PWR_CR: PVDE bit4, PLS bits7:5. Use the highest threshold band. */
    PWR->CR = (PWR->CR & ~(7UL << 5)) | HOVERBOARD_PVD_PLS_BITS | (1UL << 4);
    EXTI->IMR |= (1UL << 16);
    EXTI->RTSR |= (1UL << 16);
    EXTI->FTSR |= (1UL << 16);
    EXTI->PR = (1UL << 16);
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PVD_IRQn);
    /* If supply is already below threshold, latch before any MOE can arm. */
    if ((PWR->CSR & (1UL << 2)) != 0U) {
        s_powerstage_fault_flags |= POWERSTAGE_FAULT_PVD;
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_MCU_UNDER_VOLTAGE);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_MCU_UNDER_VOLTAGE);
    }
#endif

#if HOVERBOARD_TIM1_BREAK_ENABLE
    TIM1->SR &= ~TIM_SR_BIF;
    TIM1->DIER |= TIM_DIER_BIE;
    HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_IRQn);
#endif
#if HOVERBOARD_TIM8_BREAK_ENABLE
    TIM8->SR &= ~TIM_SR_BIF;
    TIM8->DIER |= TIM_DIER_BIE;
    HAL_NVIC_SetPriority(TIM8_BRK_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_IRQn);
#endif
}

void motor_hw_start_sampling(void) {
    memset((void *)g_adc_dual_dma, 0, sizeof(g_adc_dual_dma));
    memset((void *)g_adc3_vbus_dma, 0, sizeof(g_adc3_vbus_dma));

    /* Arm the independent ADC3 Vbus DMA first. TIM8 is still stopped, so no
       conversion can occur until the synchronized PWM start below. */
    if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_adc3_vbus_dma, 2U) != HAL_OK) {
        Error_Handler_Local();
    }
    __HAL_DMA_DISABLE_IT(&hdma_adc3, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_adc3, DMA_IT_TC);
    __HAL_DMA_ENABLE_IT(&hdma_adc3, DMA_IT_TE);

    /* STM32F1 dual-regular rule: ADC2 is armed as the slave; ADC1 is the
       external-triggered master. TIM8 TRGO starts both the dual current scan
       and the independent ADC3 DCLINK conversion once per PWM period. */
    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        (void)HAL_ADC_Stop_DMA(&hadc3);
        Error_Handler_Local();
    }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)g_adc_dual_dma, 6U) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc2);
        (void)HAL_ADC_Stop_DMA(&hadc3);
        Error_Handler_Local();
    }

    /* With six dual ranks, HT occurs after rank 3. All first three ranks are
       current channels, so this follows the upstream VESC HT condition. TC
       remains disabled; voltage/auxiliary ranks finish in the background. */
    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);

    init_powerstage_safety();

    __disable_irq();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CNT = 0U;
    TIM8->CNT = (uint16_t)ADC_MOTOR_PHASE_OFFSET_TICKS;
    TIM8->RCR = 1U;
    TIM1->SR = 0U;
    TIM8->SR = 0U;

    /* In gated mode TIM8 must have CEN set before the TIM1 master gate goes
       high. TIM1 then starts both PWM timebases while the programmed counter
       offset is preserved. */
    TIM8->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
    __enable_irq();
}

static uint8_t adc_regular_rank_channel(const ADC_TypeDef *adc, uint8_t rank) {
    if (adc == NULL || rank == 0U || rank > 16U) return 0xFFU;
    uint32_t reg;
    uint32_t shift;
    if (rank <= 6U) {
        reg = adc->SQR3;
        shift = (uint32_t)(rank - 1U) * 5U;
    } else if (rank <= 12U) {
        reg = adc->SQR2;
        shift = (uint32_t)(rank - 7U) * 5U;
    } else {
        reg = adc->SQR1;
        shift = (uint32_t)(rank - 13U) * 5U;
    }
    return (uint8_t)((reg >> shift) & 0x1FU);
}

uint32_t motor_hw_sampling_contract_flags(void) {
    uint32_t flags = 0U;
    const uint32_t cms1 = TIM1->CR1 & TIM_CR1_CMS;
    const uint32_t cms8 = TIM8->CR1 & TIM_CR1_CMS;
    if (cms1 != TIM_COUNTERMODE_CENTERALIGNED1 || TIM1->ARR != PWM_TIMER_ARR) {
        flags |= HW_SAMPLING_CONTRACT_TIM1_MODE;
    }
    if (cms8 != TIM_COUNTERMODE_CENTERALIGNED1 || TIM8->ARR != PWM_TIMER_ARR) {
        flags |= HW_SAMPLING_CONTRACT_TIM8_MODE;
    }
    if ((TIM1->CR2 & TIM_CR2_MMS) != TIM_TRGO_ENABLE) {
        flags |= HW_SAMPLING_CONTRACT_TIM1_TRGO;
    }
    if ((TIM8->CR2 & TIM_CR2_MMS) != TIM_TRGO_UPDATE) {
        flags |= HW_SAMPLING_CONTRACT_TIM8_TRGO;
    }
    const uint32_t tim8_slave_expected = TIM_TS_ITR0 | TIM_SLAVEMODE_GATED | TIM_SMCR_MSM;
    if ((TIM8->SMCR & (TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) != tim8_slave_expected) {
        flags |= HW_SAMPLING_CONTRACT_TIM8_SLAVE;
    }
    if (TIM8->RCR != 1U) {
        flags |= HW_SAMPLING_CONTRACT_TIM8_RCR;
    }

    /* SQR1.L stores conversion-count minus one. The first three ranks are the
     * hard-current boundary; rank six is APP ADC and must stay after HT. */
    if (((ADC1->SQR1 & ADC_SQR1_L) >> 20) != 5U) {
        flags |= HW_SAMPLING_CONTRACT_ADC1_LEN;
    }
    if (((ADC2->SQR1 & ADC_SQR1_L) >> 20) != 5U) {
        flags |= HW_SAMPLING_CONTRACT_ADC2_LEN;
    }
    if ((ADC1->CR1 & ADC_CR1_DUALMOD) != ADC_DUALMODE_REGSIMULT) {
        flags |= HW_SAMPLING_CONTRACT_ADC_DUALMODE;
    }
    if ((ADC1->CR2 & (ADC_CR2_EXTTRIG | ADC_CR2_DMA)) != (ADC_CR2_EXTTRIG | ADC_CR2_DMA)) {
        flags |= HW_SAMPLING_CONTRACT_ADC1_TRIGGER;
    }
    if (adc_regular_rank_channel(ADC1, 1U) != 11U ||
        adc_regular_rank_channel(ADC2, 1U) != 10U ||
        adc_regular_rank_channel(ADC1, 2U) != 0U  ||
        adc_regular_rank_channel(ADC2, 2U) != 13U ||
        adc_regular_rank_channel(ADC1, 3U) != 14U ||
        adc_regular_rank_channel(ADC2, 3U) != 15U ||
        adc_regular_rank_channel(ADC1, 6U) != 2U  ||
        adc_regular_rank_channel(ADC2, 6U) != 3U) {
        flags |= HW_SAMPLING_CONTRACT_ADC_CHANNELS;
    }

    const uint32_t dma1_required = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_0 | DMA_CCR_PL_1 |
                                   DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
    if ((DMA1_Channel1->CCR & dma1_required) != dma1_required) {
        flags |= HW_SAMPLING_CONTRACT_DMA1_MODE;
    }
    uint32_t dma1_count = DMA1_Channel1->CNDTR;
    if ((DMA1_Channel1->CCR & DMA_CCR_EN) == 0U || dma1_count == 0U || dma1_count > 6U) {
        flags |= HW_SAMPLING_CONTRACT_DMA1_TRANSFER;
    }

    if (((ADC3->SQR1 & ADC_SQR1_L) >> 20) != 0U ||
        adc_regular_rank_channel(ADC3, 1U) != 12U ||
        (ADC3->CR2 & (ADC_CR2_EXTTRIG | ADC_CR2_DMA)) != (ADC_CR2_EXTTRIG | ADC_CR2_DMA)) {
        flags |= HW_SAMPLING_CONTRACT_ADC3_MODE;
    }
    const uint32_t dma2_required = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_0 | DMA_CCR_PL_1 |
                                   DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0;
    if ((DMA2_Channel5->CCR & dma2_required) != dma2_required) {
        flags |= HW_SAMPLING_CONTRACT_DMA2_MODE;
    }
    uint32_t dma2_count = DMA2_Channel5->CNDTR;
    if ((DMA2_Channel5->CCR & DMA_CCR_EN) == 0U || dma2_count == 0U || dma2_count > 2U) {
        flags |= HW_SAMPLING_CONTRACT_DMA2_TRANSFER;
    }
    return flags;
}

bool motor_hw_sampling_contract_valid(void) {
    return motor_hw_sampling_contract_flags() == 0U;
}

void motor_hw_set_pwm_enabled(MotorRuntime *m, bool enabled) {
    if (m == NULL) return;
    if (enabled) {
        if (s_powerstage_fault_flags != 0U) {
            m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
            m->pwm_enabled = false;
            m->pwm_enable_pending_events = 0U;
            return;
        }
        if (!m->pwm_enabled && m->pwm_enable_pending_events == 0U) {
            /* Stage an exact zero-vector triplet first. MOE remains OFF until the
               hard ADC schedule has observed enough timer update events to guarantee
               the preload is active in hardware. Guard against re-entrant calls from
               the calibration 200 Hz task that would reset the countdown before
               the 16 kHz ISR has finished arming. */
            motor_hw_restore_foc_outputs(m);
            motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
            m->pwm_enable_blank_cycles = 0U;
            if (m->pwm_enable_pending_events == 0U) {
                m->pwm_enable_pending_events = PWM_ENABLE_PRELOAD_EVENTS;
            }
            m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
        }
    } else {
        m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
        m->pwm_enabled = false;
        m->pwm_enable_blank_cycles = 0U;
        m->pwm_enable_pending_events = 0U;
        if (m->full_brake_active) motor_hw_restore_foc_outputs(m);
    }
}

void motor_hw_service_pwm_enable_from_isr(MotorRuntime *m) {
    if (m == NULL || m->pwm_enabled || m->pwm_enable_pending_events == 0U) return;
    /* Hardware power-stage faults (PVD/BKIN) always block MOE. A normal
     * software fault blocks MOE only after calibration has finished (i.e. the
     * normal running state). During calibration, even a stale fault (e.g.
     * FLASH_CONFIG from a blank flash) must not prevent the safe 50% zero-
     * vector driven stage from arming MOE so offset calibration can finish. */
    if (s_powerstage_fault_flags != 0U ||
        (m->fault != MOTOR_FAULT_NONE && !foc_calibration_in_progress())) {
        m->pwm_enable_pending_events = 0U;
        m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
        return;
    }

    /* Keep refreshing the preload while waiting. Every call corresponds to a
       completed fast ADC frame, i.e. one complete 16-kHz PWM schedule. */
    motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
    if (--m->pwm_enable_pending_events != 0U) return;

    m->pwm_enable_blank_cycles = PWM_ENABLE_BLANK_CYCLES;
    m->pwm_tim->BDTR |= TIM_BDTR_MOE;
    m->pwm_enabled = true;
}


static void motor_hw_set_oc_mode_triplet(TIM_TypeDef *tim, uint32_t mode) {
    uint32_t ccmr1 = tim->CCMR1;
    uint32_t ccmr2 = tim->CCMR2;
    ccmr1 &= ~(TIM_CCMR1_OC1M_MASK_LOCAL | TIM_CCMR1_OC2M_MASK_LOCAL);
    ccmr2 &= ~TIM_CCMR2_OC3M_MASK_LOCAL;
    ccmr1 |= (mode << 4) | (mode << 12);
    ccmr2 |= (mode << 4);
    tim->CCMR1 = ccmr1;
    tim->CCMR2 = ccmr2;
}

void motor_hw_restore_foc_outputs(MotorRuntime *m) {
    if (m == NULL || m->pwm_tim == NULL) return;
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                          TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    motor_hw_set_oc_mode_triplet(m->pwm_tim, TIM_OCMODE_PWM1_LOCAL);
    m->pwm_tim->CCER |= mask;
    m->pwm_tim->EGR = TIM_EGR_COMG_LOCAL;
    m->full_brake_active = false;
}

void motor_hw_set_low_side_brake(MotorRuntime *m, bool enable) {
    if (m == NULL || m->pwm_tim == NULL) return;
    if (!enable) {
        if (m->full_brake_active) motor_hw_restore_foc_outputs(m);
        return;
    }
    if (m->full_brake_active) return;
    if (!m->pwm_enabled || m->fault != MOTOR_FAULT_NONE || s_powerstage_fault_flags != 0U) return;

    /* With this board's configured polarity, forced-inactive drives CHx low
       (high-side OFF) while the complementary active-low CHxN is asserted
       (low-side ON). This mirrors VESC full_brake_hw. The feature is disabled
       by default because continuous low-side gate-drive must be bench-tested. */
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                          TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    motor_hw_set_oc_mode_triplet(m->pwm_tim, TIM_OCMODE_FORCED_INACTIVE_LOCAL);
    m->pwm_tim->CCER |= mask;
    m->pwm_tim->EGR = TIM_EGR_COMG_LOCAL;
    m->full_brake_active = true;
}

uint32_t motor_hw_powerstage_fault_flags(void) { return s_powerstage_fault_flags; }
bool motor_hw_powerstage_fault_latched(void) { return s_powerstage_fault_flags != 0U; }
bool motor_hw_pvd_low(void) {
#if HOVERBOARD_PVD_ENABLE
    return (PWR->CSR & (1UL << 2)) != 0U;
#else
    return false;
#endif
}

bool motor_hw_clear_recoverable_powerstage_faults(void) {
    /* Calibration is a stopped, zero-vector operation. Clear only a stale
     * software latch when the underlying hardware condition is absent now.
     * Never clear while PVD is currently low or a timer break flag is set. */
#if HOVERBOARD_PVD_ENABLE
    if ((PWR->CSR & (1UL << 2)) != 0U) return false;
#endif
#if HOVERBOARD_TIM1_BREAK_ENABLE
    if ((TIM1->SR & TIM_SR_BIF) != 0U) return false;
#endif
#if HOVERBOARD_TIM8_BREAK_ENABLE
    if ((TIM8->SR & TIM_SR_BIF) != 0U) return false;
#endif
    s_powerstage_fault_flags = 0U;
    return true;
}

void motor_hw_pvd_irq_handler(void) {
#if HOVERBOARD_PVD_ENABLE
    EXTI->PR = (1UL << 16);
    if ((PWR->CSR & (1UL << 2)) != 0U) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM8->BDTR &= ~TIM_BDTR_MOE;
        s_powerstage_fault_flags |= POWERSTAGE_FAULT_PVD;
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_MCU_UNDER_VOLTAGE);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_MCU_UNDER_VOLTAGE);
    }
#endif
}

void motor_hw_break_irq_handler(TIM_TypeDef *tim) {
    if (tim == NULL || (tim->SR & TIM_SR_BIF) == 0U) return;
    tim->SR &= ~TIM_SR_BIF;
    /* Hardware already cleared MOE asynchronously. Explicitly clear both
       bridges as the dual-motor board shares one supply/power-stage domain. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    if (tim == TIM1) s_powerstage_fault_flags |= POWERSTAGE_FAULT_TIM1;
    else if (tim == TIM8) s_powerstage_fault_flags |= POWERSTAGE_FAULT_TIM8;
    motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_BREAK);
    motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_BREAK);
}

void motor_hw_set_pwm_q15(MotorRuntime *m, uint16_t du_q15, uint16_t dv_q15, uint16_t dw_q15) {
    /* The 10..90% current-sampling window is enforced as a vector operation
       in foc_svm_q15(). Do not clip U/V/W independently here because that
       rotates/distorts the alpha/beta voltage vector. Keep only the absolute
       timer-domain guard for corrupt/non-FOC callers. */
    if (du_q15 > 32767U) du_q15 = 32767U;
    if (dv_q15 > 32767U) dv_q15 = 32767U;
    if (dw_q15 > 32767U) dw_q15 = 32767U;

    uint32_t cu = ((uint32_t)du_q15 * PWM_TIMER_ARR) >> 15;
    uint32_t cv = ((uint32_t)dv_q15 * PWM_TIMER_ARR) >> 15;
    uint32_t cw = ((uint32_t)dw_q15 * PWM_TIMER_ARR) >> 15;

    /* All three CCRs are preload-enabled. UDIS prevents a software update from
       splitting this triplet; the next hardware update latches them together. */
    m->pwm_tim->CR1 |= TIM_CR1_UDIS;
    m->pwm_tim->CCR1 = cu;
    m->pwm_tim->CCR2 = cv;
    m->pwm_tim->CCR3 = cw;
    m->pwm_tim->CR1 &= ~TIM_CR1_UDIS;
}

void motor_hw_set_pwm_duty(MotorRuntime *m, float du, float dv, float dw) {
    /* This helper is not the FOC current-control path. Bound only to the timer
       domain; callers that need the current-sampling window must use SVM. */
    if (du < 0.0f) du = 0.0f;
    if (du > 0.999969f) du = 0.999969f;
    if (dv < 0.0f) dv = 0.0f;
    if (dv > 0.999969f) dv = 0.999969f;
    if (dw < 0.0f) dw = 0.0f;
    if (dw > 0.999969f) dw = 0.999969f;
    motor_hw_set_pwm_q15(m,
        (uint16_t)(du * 32768.0f),
        (uint16_t)(dv * 32768.0f),
        (uint16_t)(dw * 32768.0f));
}

uint8_t motor_hw_read_hall_raw(motor_id_t id) {
    /* Stock hoverboard Hall inputs are active-low. This function is called
       from the hard FOC path, so use one IDR snapshot per GPIO port rather
       than three HAL_GPIO_ReadPin calls. */
    uint32_t idr;
    uint8_t u, v, w;
    if (id == MOTOR_LEFT) {
        idr = GPIOB->IDR;
        u = (idr & LEFT_HALL_U_PIN) ? 0U : 1U;
        v = (idr & LEFT_HALL_V_PIN) ? 0U : 1U;
        w = (idr & LEFT_HALL_W_PIN) ? 0U : 1U;
    } else {
        idr = GPIOC->IDR;
        u = (idr & RIGHT_HALL_U_PIN) ? 0U : 1U;
        v = (idr & RIGHT_HALL_V_PIN) ? 0U : 1U;
        w = (idr & RIGHT_HALL_W_PIN) ? 0U : 1U;
    }
    return (uint8_t)(u | (v << 1) | (w << 2));
}

bool motor_hw_board_temperature_c(float *temp_c) {
    if (temp_c == NULL) return false;

    /* ADC1 is the low halfword in dual mode. Rank 5 maps to DMA word 4.
       Read once so a DMA refresh cannot split raw and conversion. */
    const uint32_t dual = g_adc_dual_dma[4];
    const uint16_t raw = (uint16_t)(dual & 0xFFFFU);
    if (raw < HOVERBOARD_MCU_TEMP_ADC_MIN_VALID ||
        raw > HOVERBOARD_MCU_TEMP_ADC_MAX_VALID) return false;

    const float vsense = ((float)raw * HOVERBOARD_ADC_VDDA_NOMINAL_V) / 4095.0f;
    const float t = 25.0f +
        (HOVERBOARD_MCU_TEMP_V25_V - vsense) / HOVERBOARD_MCU_TEMP_AVG_SLOPE_V_PER_C;
    if (t < HOVERBOARD_MCU_TEMP_MIN_VALID_C || t > HOVERBOARD_MCU_TEMP_MAX_VALID_C) return false;
    *temp_c = t;
    return true;
}

uint16_t motor_hw_encoder_cnt(void) {
    return (uint16_t)TIM4->CNT;
}

void motor_hw_encoder_set_count(MotorRuntime *m, uint16_t count) {
    if (!m || m->id != MOTOR_LEFT || m->encoder.cpr < 4U) return;
    count = (uint16_t)(count % m->encoder.cpr);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM4->CNT = count;
    TIM4->SR &= ~TIM_SR_UIF;
    m->encoder.turns = 0;
    m->encoder.extended_count = (int32_t)count;
    m->encoder.prev_extended_count = (int32_t)count;
    m->encoder.speed_sample_valid = false;
    if (!primask) __enable_irq();
}

void motor_hw_configure_sensor(MotorRuntime *m, uint8_t mode) {
    if (m == NULL) return;

    /* Sensor mux changes are never permitted while power PWM is enabled. */
    motor_hw_set_pwm_enabled(m, false);

    if (m->id == MOTOR_RIGHT) {
        /* This PCB has only right Hall inputs. AUTO resolves to Hall. */
        m->sensor_mode = SENSOR_MODE_HALL;
        uint32_t pending = EXTI->PR & (RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN);
        if (pending != 0U) EXTI->PR = pending;
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        return;
    }

    /* LEFT PB6/PB7 are shared by Hall and TIM4 encoder. */
    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    EXTI->IMR &= ~(LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN);
    EXTI->PR = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
    HAL_GPIO_DeInit(GPIOB, LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN);

    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_NOPULL;

    if (mode == SENSOR_MODE_ENCODER) {
        g.Mode = GPIO_MODE_INPUT;
        g.Pin = LEFT_ENCODER_A_PIN | LEFT_ENCODER_B_PIN;
        HAL_GPIO_Init(GPIOB, &g);
        g.Pin = LEFT_HALL_U_PIN;
        HAL_GPIO_Init(GPIOB, &g);

        uint32_t cpr=m->encoder.cpr; if (cpr<4U) cpr=4U; if (cpr>65535U) cpr=65535U;
        TIM4->ARR=cpr-1U;
        __HAL_TIM_SET_COUNTER(&htim4, 0U);
        m->encoder.turns=0; m->encoder.extended_count=0;
        m->encoder.prev_extended_count=0; m->encoder.speed_sample_valid=false;
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
        if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler_Local();
        m->sensor_mode = SENSOR_MODE_ENCODER;
    } else {
        g.Mode = GPIO_MODE_IT_RISING_FALLING;
        g.Pin = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
        HAL_GPIO_Init(GPIOB, &g);
        EXTI->PR = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        m->sensor_mode = SENSOR_MODE_HALL;
    }
}

void motor_hw_led(bool on) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_hw_emergency_all_off(void) {
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    g_motor_left.pwm_enabled = false;
    g_motor_right.pwm_enabled = false;
    g_motor_left.pwm_enable_blank_cycles = 0U;
    g_motor_right.pwm_enable_blank_cycles = 0U;
    g_motor_left.pwm_enable_pending_events = 0U;
    g_motor_right.pwm_enable_pending_events = 0U;
}


void motor_hw_capture_app_adc_from_isr(void) {
    /* On the first HT event rank 6 has never been converted yet. From the
     * second event onward slot 5 is the complete previous PWM frame and stays
     * stable until the slow half of the current frame reaches rank 6. */
    if (s_app_adc_ht_seen != 0U) {
        __DMB();
        s_app_adc_word = g_adc_dual_dma[5];
        __DMB();
        s_app_adc_seq++;
    } else {
        s_app_adc_ht_seen = 1U;
    }
}

bool motor_hw_get_app_adc_raw(uint16_t *pa2_raw, uint16_t *pa3_raw) {
    if (pa2_raw == NULL || pa3_raw == NULL || s_app_adc_seq == 0U) return false;
    uint32_t a, b, word;
    do {
        a = s_app_adc_seq;
        __DMB();
        word = s_app_adc_word;
        __DMB();
        b = s_app_adc_seq;
    } while (a != b);
    *pa2_raw = (uint16_t)(word & 0xFFFFU);
    *pa3_raw = (uint16_t)(word >> 16);
    return true;
}
