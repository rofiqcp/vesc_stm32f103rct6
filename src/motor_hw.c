#include "motor_hw.h"
#include "board_pins.h"
#include "app_config.h"
#include "motor_control.h"
#include "foc_control.h"
#include "vesc_comm.h"
#include "foc_math.h"
#include "cmsis_os2.h"
#include <string.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
volatile uint32_t g_adc_dual_dma[4] __attribute__((aligned(4)));

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

    /* ADC analog inputs */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pin = LEFT_U_CUR_PIN;
    HAL_GPIO_Init(LEFT_U_CUR_PORT, &g);
    g.Pin = LEFT_DC_CUR_PIN | LEFT_V_CUR_PIN | RIGHT_DC_CUR_PIN |
            RIGHT_U_CUR_PIN | RIGHT_V_CUR_PIN | DCLINK_PIN;
    HAL_GPIO_Init(GPIOC, &g);

    /* LED, buzzer, OFF */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &g);
    g.Pin = BUZZER_PIN | OFF_PIN;
    HAL_GPIO_Init(GPIOA, &g);
    motor_hw_led(false);
    motor_hw_buzzer(false);
    motor_hw_gate_global(false);

    /* Button */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    g.Pin = BUTTON_PIN;
    HAL_GPIO_Init(BUTTON_PORT, &g);

    /* LEFT shared sensor pins start in safe Hall-input mode. Runtime sensor
       selection can reconfigure PB6/PB7 to TIM4 encoder without reboot. */
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
    g.Pin = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* Right Hall PC10/11/12 */
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
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
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
    bd.BreakState = TIM_BREAK_DISABLE;
    bd.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bd.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(h, &bd) != HAL_OK) Error_Handler_Local();

    /* CCR preload: the FOC ISR writes a coherent triplet that is latched by
       the next timer update event, avoiding mid-cycle waveform changes. */
    inst->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    inst->CCMR2 |= TIM_CCMR2_OC3PE;
    inst->CR1   |= TIM_CR1_ARPE;
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
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    init_pwm_timer(&htim1, TIM1);
    init_pwm_timer(&htim8, TIM8);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (2U * PWM_TIMER_ARR) - 1U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK) Error_Handler_Local();
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_TIMING;
    oc.Pulse = PWM_TIMER_ARR + ADC_SAMPLE_DELAY_TICKS;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler_Local();
    TIM2->CCR2 = PWM_TIMER_ARR + ADC_SAMPLE_DELAY_TICKS;
    TIM2->CCER |= TIM_CCER_CC2E;

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
    enc.IC1Filter = 4;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 4;
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
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 72 MHz / 6 = 12 MHz ADC clock, within STM32F103 specification. */
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler_Local();

    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 4;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler_Local();

    /* ADC1 lower halfword, ADC2 upper halfword for each rank. */
    /* Keep the four phase-current channels first and fast. At 12 MHz ADC,
       1.5 + 12.5 = 14 ADC clocks per rank (~1.17 us). Rank 2 sample-and-hold
       therefore still occurs inside the conservative zero-vector window. */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_0,  ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5); /* LEFT U PA0 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_13, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5); /* LEFT V PC3 */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_1CYCLE_5); /* RIGHT U PC4 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_15, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_1CYCLE_5); /* RIGHT V PC5 */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_10, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_1CYCLE_5); /* LEFT DC PC0 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_11, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_1CYCLE_5); /* RIGHT DC PC1 */
    /* DC-link can use a longer aperture because it is not used for current
       reconstruction at this exact instant. */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_12, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_28CYCLES_5); /* DCLINK PC2 */
    cfg_adc_channel(&hadc2, ADC_CHANNEL_11, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_28CYCLES_5); /* duplicate RIGHT DC */

    ADC_MultiModeTypeDef multi = {0};
    multi.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multi) != HAL_OK) Error_Handler_Local();

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler_Local();
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) Error_Handler_Local();

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
}


void motor_hw_init(void) {
    init_gpio();
    init_timers();
    init_adc_dma();
}

void motor_hw_start_sampling(void) {
    memset((void *)g_adc_dual_dma, 0, sizeof(g_adc_dual_dma));
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)g_adc_dual_dma, 4U) != HAL_OK) {
        Error_Handler_Local();
    }
    /* VESC-style latency reduction: FOC starts as soon as ranks 1..2 are in
       RAM. Transfer-complete IRQ is unnecessary; ranks 3..4 remain available
       from the previous cycle when the next half-transfer arrives. */
    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);

    __disable_irq();
    TIM1->CNT = 0U;
    TIM8->CNT = 0U;
    TIM2->CNT = 0U;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;
    __enable_irq();
}

void motor_hw_set_pwm_enabled(MotorRuntime *m, bool enabled) {
    if (enabled) {
        if (!m->pwm_enabled) {
            motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
            motor_hw_gate_global(true);
            m->pwm_tim->BDTR |= TIM_BDTR_MOE;
            m->pwm_enabled = true;
        }
    } else {
        m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
        m->pwm_enabled = false;
    }
}

void motor_hw_set_pwm_q15(MotorRuntime *m, uint16_t du_q15, uint16_t dv_q15, uint16_t dw_q15) {
    if (du_q15 < PWM_MIN_DUTY_Q15) du_q15 = PWM_MIN_DUTY_Q15;
    if (du_q15 > PWM_MAX_DUTY_Q15) du_q15 = PWM_MAX_DUTY_Q15;
    if (dv_q15 < PWM_MIN_DUTY_Q15) dv_q15 = PWM_MIN_DUTY_Q15;
    if (dv_q15 > PWM_MAX_DUTY_Q15) dv_q15 = PWM_MAX_DUTY_Q15;
    if (dw_q15 < PWM_MIN_DUTY_Q15) dw_q15 = PWM_MIN_DUTY_Q15;
    if (dw_q15 > PWM_MAX_DUTY_Q15) dw_q15 = PWM_MAX_DUTY_Q15;

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
    if (du < PWM_MIN_DUTY) du = PWM_MIN_DUTY;
    if (du > PWM_MAX_DUTY) du = PWM_MAX_DUTY;
    if (dv < PWM_MIN_DUTY) dv = PWM_MIN_DUTY;
    if (dv > PWM_MAX_DUTY) dv = PWM_MAX_DUTY;
    if (dw < PWM_MIN_DUTY) dw = PWM_MIN_DUTY;
    if (dw > PWM_MAX_DUTY) dw = PWM_MAX_DUTY;
    motor_hw_set_pwm_q15(m,
        (uint16_t)(du * 32768.0f),
        (uint16_t)(dv * 32768.0f),
        (uint16_t)(dw * 32768.0f));
}

void motor_hw_gate_global(bool enable) {
#if OFF_PIN_ACTIVE_HIGH
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

uint8_t motor_hw_read_hall_raw(motor_id_t id) {
    uint8_t u, v, w;
    if (id == MOTOR_LEFT) {
        u = HAL_GPIO_ReadPin(LEFT_HALL_U_PORT, LEFT_HALL_U_PIN) ? 1U : 0U;
        v = HAL_GPIO_ReadPin(LEFT_HALL_V_PORT, LEFT_HALL_V_PIN) ? 1U : 0U;
        w = HAL_GPIO_ReadPin(LEFT_HALL_W_PORT, LEFT_HALL_W_PIN) ? 1U : 0U;
    } else {
        u = HAL_GPIO_ReadPin(RIGHT_HALL_U_PORT, RIGHT_HALL_U_PIN) ? 1U : 0U;
        v = HAL_GPIO_ReadPin(RIGHT_HALL_V_PORT, RIGHT_HALL_V_PIN) ? 1U : 0U;
        w = HAL_GPIO_ReadPin(RIGHT_HALL_W_PORT, RIGHT_HALL_W_PIN) ? 1U : 0U;
    }
    return (uint8_t)(u | (v << 1) | (w << 2));
}

uint16_t motor_hw_encoder_cnt(void) {
    return (uint16_t)TIM4->CNT;
}

void motor_hw_encoder_reset(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM4->CNT = 0U;
    g_motor_left.encoder.turns = 0;
    g_motor_left.encoder.extended_count = 0;
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
    g.Pull = GPIO_PULLUP;

    if (mode == SENSOR_MODE_ENCODER) {
        g.Mode = GPIO_MODE_INPUT;
        g.Pin = LEFT_ENCODER_A_PIN | LEFT_ENCODER_B_PIN;
        HAL_GPIO_Init(GPIOB, &g);
        g.Pin = LEFT_HALL_U_PIN;
        HAL_GPIO_Init(GPIOB, &g);

        __HAL_TIM_SET_COUNTER(&htim4, 0U);
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

void motor_hw_buzzer(bool on) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void motor_hw_emergency_all_off(void) {
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    g_motor_left.pwm_enabled = false;
    g_motor_right.pwm_enabled = false;
    motor_hw_gate_global(false);
}
