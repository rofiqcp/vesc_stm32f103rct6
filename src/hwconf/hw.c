#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "applications/appconf_default.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "comm/commands.h"
#include "motor/foc_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

// Variabel hadc1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
ADC_HandleTypeDef hadc1;
// Variabel hadc2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
ADC_HandleTypeDef hadc2;
// Variabel hdma_adc1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
DMA_HandleTypeDef hdma_adc1;
// Variabel htim1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
TIM_HandleTypeDef htim1;
// Variabel htim8: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
TIM_HandleTypeDef htim8;
// Variabel htim2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
TIM_HandleTypeDef htim2;
// Variabel htim4: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
TIM_HandleTypeDef htim4;
/* Run35: ADC sequence is the proven hoverboard V15 sequence: five dual-ADC
 * words per PWM trigger. Three complete frames are stored by DMA before a TC
 * interrupt, so ADC sampling stays at 16 kHz while the complete Run31 FOC
 * pipeline executes at 5.333 kHz with a 12,000-cycle CPU slot. */
volatile uint32_t g_adc_dual_dma[ADC_DMA_BATCH_WORDS] __attribute__((aligned(4)));
/* PA2 is ADC2 rank-4, PA3 is ADC2 rank-5 and internal temperature is ADC1
 * rank-5 exactly as V15. Auxiliary values are copied from the newest frame. */
static volatile uint32_t s_app_adc_word = 0U;
// Variabel s_app_adc_seq: nilai atau state ADC pada jalur pengukuran.
static volatile uint32_t s_app_adc_seq = 0U;
static volatile uint32_t s_temp_adc_word = 0U;
static volatile uint32_t s_temp_adc_seq = 0U;
/* Runtime-tunable TIM8/ADC phase offset for shunt sampling validation. */
// Variabel s_adc_phase_offset_ticks: nilai atau state ADC pada jalur pengukuran.
static volatile uint16_t s_adc_phase_offset_ticks = (uint16_t)ADC_MOTOR_PHASE_OFFSET_TICKS;

// Parameter ticks: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_set_adc_phase_offset_ticks: mengatur motor hw set adc phase offset ticks setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void motor_hw_set_adc_phase_offset_ticks(uint16_t ticks) {
    s_adc_phase_offset_ticks = ticks;
}

// Fungsi motor_hw_get_adc_phase_offset_ticks: membaca motor hw get adc phase offset ticks tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
uint16_t motor_hw_get_adc_phase_offset_ticks(void) {
    return s_adc_phase_offset_ticks;
}

/* Power-stage faults are deliberately reset-latched. A software motor fault
 * may be cleared after its normal recovery policy, but PVD/BKIN events keep
 * MOE blocked until MCU reset because they indicate supply or external gate
 * integrity loss. */
#define POWERSTAGE_FAULT_PVD   (1UL << 0)
#define POWERSTAGE_FAULT_TIM1  (1UL << 1)
#define POWERSTAGE_FAULT_TIM8  (1UL << 2)
// Variabel s_powerstage_fault_flags: status atau data gangguan untuk sistem proteksi.
static volatile uint32_t s_powerstage_fault_flags = 0U;

#define TIM_CCMR1_OC1M_MASK_LOCAL (7UL << 4)
#define TIM_CCMR1_OC2M_MASK_LOCAL (7UL << 12)
#define TIM_CCMR2_OC3M_MASK_LOCAL (7UL << 4)
#define TIM_OCMODE_FORCED_INACTIVE_LOCAL (4UL)
#define TIM_OCMODE_PWM1_LOCAL            (6UL)
#define TIM_EGR_COMG_LOCAL               (1UL << 5)

// Fungsi Error_Handler_Local: menangani error handler local pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
static void Error_Handler_Local(void) {
    /* Motor-subsystem failure must never make the controller disappear from
     * VESC Tool. The management UART is initialized before motor_hw_init().
     * Before the scheduler starts, service the same packet parser directly;
     * after scheduler start the packet_process task owns it. */
    motor_hw_emergency_all_off();
    for (;; ) {
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            vTaskDelay(pdMS_TO_TICKS(100U));
        }
        else {
            (void)vesc_comm_poll_once();
            HAL_Delay(1U);
        }
    }
}

// Parameter ns: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi deadtime_to_dtg: menjalankan operasi deadtime to dtg sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static uint32_t deadtime_to_dtg(uint32_t ns) {
    // Variabel ticks: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t ticks = (uint32_t)(((uint64_t)CPU_CLOCK_HZ * ns + 999999999ULL) / 1000000000ULL);
    if (ticks > 127U)
        ticks = 127U; /* baseline uses simple DTG region */
    return ticks;
}

/* STM32F1 AFIO->MAPR has a trap: SWJ_CFG readback is not safe to feed back
 * through generic read-modify-write remap macros. Some STM32CubeF1 remap
 * helpers OR the SWJ field while enabling an unrelated remap, which can turn
 * JTAG-off/SWD-on into JTAG-off/SWD-off. That makes ST-Link appear to require
 * connect-under-reset after the application starts.
 *
 * Keep every MAPR field used by this firmware in ONE explicit write. We may
 * read the other ordinary remap bits, but the SWJ bits are always masked out
 * and replaced with the known-safe value 0b010 = JTAG disabled, SWD enabled.
 * USART3 remap is explicitly cleared so the VESC link remains PB10/PB11.
 * ADC1 ETRGREG remap is explicitly set so TIM8_TRGO triggers the regular FOC
 * scan on STM32F103xE. Do not add __HAL_AFIO_REMAP_* calls elsewhere. */
#define AFIO_SWJ_CFG_MASK_LOCAL        (0x7UL << 24)
#define AFIO_SWJ_JTAG_OFF_SWD_ON_LOCAL (0x2UL << 24)

// Fungsi afio_apply_vesc_mapr_once: menjalankan operasi afio apply vesc mapr once sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static void afio_apply_vesc_mapr_once(void) {
    // Variabel mapr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mapr = AFIO->MAPR;

    mapr &= ~AFIO_SWJ_CFG_MASK_LOCAL;
#ifdef AFIO_MAPR_USART3_REMAP
    mapr &= ~AFIO_MAPR_USART3_REMAP;
#endif
#ifdef AFIO_MAPR_ADC1_ETRGREG_REMAP
    mapr |= AFIO_MAPR_ADC1_ETRGREG_REMAP;
#endif
    mapr |= AFIO_SWJ_JTAG_OFF_SWD_ON_LOCAL;

    AFIO->MAPR = mapr;
    __DSB();
    __ISB();
}

// Fungsi init_gpio: menginisialisasi init gpio sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
static void init_gpio(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Release JTAG PB3/PB4/PB5 but preserve SWD PA13/PA14, and establish all
     * AFIO MAPR remaps required by this firmware in a single safe write. */
    afio_apply_vesc_mapr_once();

    // Variabel g: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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

// Parameter h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inst: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi init_pwm_timer: menginisialisasi init pwm timer sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
static void init_pwm_timer(TIM_HandleTypeDef *h, TIM_TypeDef *inst) {
    h->Instance = inst;
    h->Init.Prescaler = 0;
    h->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    h->Init.Period = PWM_TIMER_ARR;
    h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    h->Init.RepetitionCounter = 0;
    h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(h) != HAL_OK)
        Error_Handler_Local();

    // Variabel oc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TIM_OC_InitTypeDef oc = {
        0
    }
    ;
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = PWM_TIMER_ARR / 2U;
    /* Power stage polarity: top input HIGH=ON, bottom input LOW=ON.
       The timer still generates complementary CHx/CHxN with hardware deadtime. */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    /* MOE/OFF state must turn both MOSFETs off physically. */
    oc.OCIdleState = TIM_OCIDLESTATE_RESET; /* top pin LOW -> OFF */
    oc.OCNIdleState = TIM_OCNIDLESTATE_SET; /* bottom pin HIGH -> OFF */
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler_Local();
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler_Local();
    if (HAL_TIM_PWM_ConfigChannel(h, &oc, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler_Local();

    // Variabel bd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TIM_BreakDeadTimeConfigTypeDef bd = {
        0
    }
    ;
    bd.OffStateRunMode = TIM_OSSR_ENABLE;
    bd.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bd.LockLevel = TIM_LOCKLEVEL_OFF;
    bd.DeadTime = deadtime_to_dtg(PWM_DEADTIME_NS);
    // Variabel break_enable: penanda untuk mengaktifkan atau menonaktifkan fitur.
    const bool break_enable = (inst == TIM1) ? (HOVERBOARD_TIM1_BREAK_ENABLE != 0)
                                             : (HOVERBOARD_TIM8_BREAK_ENABLE != 0);
    // Variabel break_high: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool break_high = (inst == TIM1) ? (HOVERBOARD_TIM1_BREAK_ACTIVE_HIGH != 0)
                                           : (HOVERBOARD_TIM8_BREAK_ACTIVE_HIGH != 0);
    bd.BreakState = break_enable ? TIM_BREAK_ENABLE : TIM_BREAK_DISABLE;
    bd.BreakPolarity = break_high ? TIM_BREAKPOLARITY_HIGH : TIM_BREAKPOLARITY_LOW;
    /* Never allow hardware automatic re-enable after a break event. */
    bd.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(h, &bd) != HAL_OK)
        Error_Handler_Local();

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

// Fungsi init_timers: menginisialisasi init timers sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
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
    // Variabel enc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    TIM_Encoder_InitTypeDef enc = {
        0
    }
    ;
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 6;
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 6;
    if (HAL_TIM_Encoder_Init(&htim4, &enc) != HAL_OK)
        Error_Handler_Local();
    __HAL_TIM_DISABLE(&htim4);
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

// Parameter h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ch: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter rank: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sample_time: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi cfg_adc_channel: menjalankan operasi cfg adc channel sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void cfg_adc_channel(ADC_HandleTypeDef *h, uint32_t ch, uint32_t rank,
                            uint32_t sample_time) {
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    ADC_ChannelConfTypeDef c = {
        0
    }
    ;
    c.Channel = ch;
    c.Rank = rank;
    c.SamplingTime = sample_time;
    if (HAL_ADC_ConfigChannel(h, &c) != HAL_OK)
        Error_Handler_Local();
}

// Fungsi init_adc_dma: menginisialisasi init adc dma sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
static void init_adc_dma(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 64 MHz / 6 = 10.67 MHz ADC clock, within STM32F103 specification. */
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 5;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler_Local();
    /* ADC1 ETRGREG -> TIM8_TRGO was established by afio_apply_vesc_mapr_once().
     * Do NOT use the generic HAL AFIO remap macro here: on STM32F1 its MAPR
     * read-modify-write can corrupt SWJ_CFG and disable SWD. */

    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 5;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
        Error_Handler_Local();

    /* Tidak ada ADC3 runtime pada Run31. PC2/VBAT berada di ADC1 rank-4,
       sama dengan firmware hoverboard V13 yang telah membuktikan motor dapat
       berputar. Ini menghapus trigger-route kedua dan DMA2 dari jalur safety. */

    /* Exact hoverboard V15 dual-ADC rank order (ADC1=low16, ADC2=high16):
         rank 1: ADC1 PC1 RIGHT DC | ADC2 PC0 LEFT DC   (1.5 cycles)
         rank 2: ADC1 PA0 LEFT  A  | ADC2 PC3 LEFT  B   (7.5 cycles)
         rank 3: ADC1 PC4 RIGHT B  | ADC2 PC5 RIGHT C   (7.5 cycles)
         rank 4: ADC1 PC2 VBAT     | ADC2 PA2 APP1      (7.5 cycles)
         rank 5: ADC1 TEMP         | ADC2 PA3 APP2      (239.5/7.5)
       There is no rank-6 filler and no ADC3/DMA2 path. */
    cfg_adc_channel(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5);
    cfg_adc_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5);
    cfg_adc_channel(&hadc1, ADC_CHANNEL_0,  ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5);
    cfg_adc_channel(&hadc2, ADC_CHANNEL_13, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5);
    cfg_adc_channel(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5);
    cfg_adc_channel(&hadc2, ADC_CHANNEL_15, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5);
    cfg_adc_channel(&hadc1, ADC_CHANNEL_12, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_7CYCLES_5);
    cfg_adc_channel(&hadc2, ADC_CHANNEL_2,  ADC_REGULAR_RANK_4, ADC_SAMPLETIME_7CYCLES_5);
    cfg_adc_channel(&hadc1, ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_239CYCLES_5);
    cfg_adc_channel(&hadc2, ADC_CHANNEL_3,  ADC_REGULAR_RANK_5, ADC_SAMPLETIME_7CYCLES_5);
    // Variabel multi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    ADC_MultiModeTypeDef multi = {
        0
    }
    ;
    multi.Mode = ADC_DUALMODE_REGSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multi) != HAL_OK)
        Error_Handler_Local();

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
        Error_Handler_Local();
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
        Error_Handler_Local();

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
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        Error_Handler_Local();
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0); /* NEVER call RTOS here */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* DMA2/ADC3 sengaja tidak dikonfigurasi: jalur referensi hanya memakai
       DMA1 Channel1 untuk dual ADC1/ADC2. */
}


// Fungsi motor_hw_init: menginisialisasi motor hw init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void motor_hw_init(void) {
    init_gpio();
    init_timers();
    init_adc_dma();
}

// Fungsi init_powerstage_safety: menginisialisasi init powerstage safety sehingga resource, konfigurasi awal,
// dan state modul siap digunakan dengan aman.
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

// Fungsi motor_hw_start_sampling: memulai motor hw start sampling setelah prasyarat hardware, konfigurasi, dan
// state keselamatan terpenuhi.
void motor_hw_start_sampling(void) {
    memset((void *)g_adc_dual_dma, 0, sizeof(g_adc_dual_dma));
    s_app_adc_word = 0U;
    s_app_adc_seq = 0U;
    s_temp_adc_word = 0U;
    s_temp_adc_seq = 0U;

    /* Same master/slave start as hoverboard V15. ADC still converts all five
       ranks every PWM period. The only scheduling difference is DMA length:
       3 x 5 words are accumulated before TC, giving the complete Run31 FOC
       pipeline a 12,000-cycle execution window. */
    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        Error_Handler_Local();
    }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)g_adc_dual_dma, ADC_DMA_BATCH_WORDS) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc2);
        Error_Handler_Local();
    }

    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);

    init_powerstage_safety();

    __disable_irq();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CNT = 0U;
    TIM8->CNT = s_adc_phase_offset_ticks;
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

// Parameter adc: peripheral ADC yang register sampling-time-nya akan dibaca.
// Parameter channel: nomor kanal ADC 0..17 yang akan diperiksa.
// Fungsi adc_sample_time_code: membaca kode SMP tiga-bit langsung dari SMPR1/SMPR2 untuk audit timing runtime.
static uint8_t adc_sample_time_code(const ADC_TypeDef *adc, uint8_t channel) {
    if (adc == NULL || channel > 17U)
        return 0xFFU;
    if (channel <= 9U)
        return (uint8_t)((adc->SMPR2 >> ((uint32_t)channel * 3U)) & 0x7U);
    return (uint8_t)((adc->SMPR1 >> ((uint32_t)(channel - 10U) * 3U)) & 0x7U);
}

// Parameter adc: nilai atau state ADC pada jalur pengukuran arus/tegangan.
// Parameter rank: nomor rank regular sequence ADC yang akan dibaca dari register SQR.
// Fungsi adc_regular_rank_channel: membaca nomor kanal pada rank regular ADC untuk audit mapping runtime.
static uint8_t adc_regular_rank_channel(const ADC_TypeDef *adc, uint8_t rank) {
    if (adc == NULL || rank == 0U || rank > 16U)
        return 0xFFU;
    // Variabel reg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t reg;
    // Variabel shift: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t shift;
    if (rank <= 6U) {
        reg = adc->SQR3;
        shift = (uint32_t)(rank - 1U) * 5U;
    }
    else if (rank <= 12U) {
        reg = adc->SQR2;
        shift = (uint32_t)(rank - 7U) * 5U;
    }
    else {
        reg = adc->SQR1;
        shift = (uint32_t)(rank - 13U) * 5U;
    }
    return (uint8_t)((reg >> shift) & 0x1FU);
}

// Fungsi motor_hw_sampling_contract_flags: menjalankan operasi motor hw sampling contract flags sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t motor_hw_sampling_contract_flags(void) {
    // Variabel flags: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t flags = 0U;
    // Variabel cms1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t cms1 = TIM1->CR1 & TIM_CR1_CMS;
    // Variabel cms8: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
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
    // Variabel tim8_slave_expected: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t tim8_slave_expected = TIM_TS_ITR0 | TIM_SLAVEMODE_GATED | TIM_SMCR_MSM;
    if ((TIM8->SMCR & (TIM_SMCR_TS | TIM_SMCR_SMS | TIM_SMCR_MSM)) != tim8_slave_expected) {
        flags |= HW_SAMPLING_CONTRACT_TIM8_SLAVE;
    }
    if (TIM8->RCR != 1U) {
        flags |= HW_SAMPLING_CONTRACT_TIM8_RCR;
    }

    /* V15 uses exactly five regular ranks: SQR1.L = 4. */
    if (((ADC1->SQR1 & ADC_SQR1_L) >> 20) != 4U) {
        flags |= HW_SAMPLING_CONTRACT_ADC1_LEN;
    }
    if (((ADC2->SQR1 & ADC_SQR1_L) >> 20) != 4U) {
        flags |= HW_SAMPLING_CONTRACT_ADC2_LEN;
    }
    if ((ADC1->CR1 & ADC_CR1_DUALMOD) != ADC_DUALMODE_REGSIMULT) {
        flags |= HW_SAMPLING_CONTRACT_ADC_DUALMODE;
    }
    /* ADCPRE=10b berarti PCLK2/6. Clock firmware nyata 64 MHz, sehingga
     * ADC berjalan 10.67 MHz dan tetap di bawah batas 14 MHz STM32F103. */
    if ((RCC->CFGR & (3UL << 14)) != (2UL << 14)) {
        flags |= HW_SAMPLING_CONTRACT_ADC_CLOCK;
    }
    if ((ADC1->CR2 & (ADC_CR2_EXTTRIG | ADC_CR2_DMA)) != (ADC_CR2_EXTTRIG | ADC_CR2_DMA)) {
        flags |= HW_SAMPLING_CONTRACT_ADC1_TRIGGER;
    }
    /* EXTSEL dan AFIO remap sama-sama menentukan sumber trigger regular
     * pada STM32F1. Keduanya harus menunjuk TIM8_TRGO agar sampling tidak
     * bergeser ke event timer lain setelah register corruption/re-init. */
    if ((ADC1->CR2 & ADC_CR2_EXTSEL) != ADC_EXTERNALTRIGCONV_T8_TRGO
#ifdef AFIO_MAPR_ADC1_ETRGREG_REMAP
        || (AFIO->MAPR & AFIO_MAPR_ADC1_ETRGREG_REMAP) == 0U
#endif
       ) {
        flags |= HW_SAMPLING_CONTRACT_TRIGGER_ROUTE;
    }
    if (adc_regular_rank_channel(ADC1, 1U) != 11U ||
        adc_regular_rank_channel(ADC2, 1U) != 10U ||
        adc_regular_rank_channel(ADC1, 2U) != 0U ||
        adc_regular_rank_channel(ADC2, 2U) != 13U ||
        adc_regular_rank_channel(ADC1, 3U) != 14U ||
        adc_regular_rank_channel(ADC2, 3U) != 15U ||
        /* Auxiliary mengikuti reference: PC2/PA2 pada rank-4 dan
         * temperature/PA3 pada rank-5. */
        adc_regular_rank_channel(ADC1, 4U) != 12U ||
        adc_regular_rank_channel(ADC2, 4U) != 2U ||
        adc_regular_rank_channel(ADC1, 5U) != 16U ||
        adc_regular_rank_channel(ADC2, 5U) != 3U) {
        flags |= HW_SAMPLING_CONTRACT_ADC_CHANNELS;
    }

    /* Kode SMP STM32F1: 0=1.5, 1=7.5, 3=28.5, 7=239.5 cycle.
     * Tiga rank current harus tetap cepat; auxiliary mengikuti reference. */
    if (adc_sample_time_code(ADC1, 11U) != 0U ||
        adc_sample_time_code(ADC2, 10U) != 0U ||
        adc_sample_time_code(ADC1, 0U) != 1U ||
        adc_sample_time_code(ADC2, 13U) != 1U ||
        adc_sample_time_code(ADC1, 14U) != 1U ||
        adc_sample_time_code(ADC2, 15U) != 1U ||
        adc_sample_time_code(ADC1, 12U) != 1U ||
        adc_sample_time_code(ADC2, 2U) != 1U ||
        adc_sample_time_code(ADC1, 16U) != 7U ||
        adc_sample_time_code(ADC2, 3U) != 1U) {
        flags |= HW_SAMPLING_CONTRACT_ADC_SAMPLE_TIMES;
    }

    // Variabel dma1_required: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t dma1_required = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_0 | DMA_CCR_PL_1 |
                                   DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1;
    if ((DMA1_Channel1->CCR & dma1_required) != dma1_required) {
        flags |= HW_SAMPLING_CONTRACT_DMA1_MODE;
    }
    /* Run35 follows the V15 TC-only IRQ model; three frames are batched. */
    if ((DMA1_Channel1->CCR & DMA_CCR_TCIE) == 0U ||
        (DMA1_Channel1->CCR & DMA_CCR_HTIE) != 0U) {
        flags |= HW_SAMPLING_CONTRACT_DMA1_IRQ_MODE;
    }
    // Variabel dma1_count: pencacah kejadian atau sampel.
    uint32_t dma1_count = DMA1_Channel1->CNDTR;
    if ((DMA1_Channel1->CCR & DMA_CCR_EN) == 0U || dma1_count == 0U || dma1_count > ADC_DMA_BATCH_WORDS) {
        flags |= HW_SAMPLING_CONTRACT_DMA1_TRANSFER;
    }

    /* ADC3/DMA2 bukan bagian kontrak Run31. Bit legacy 13..15/19 tetap
       didefinisikan untuk kompatibilitas decoder tetapi tidak pernah diset. */
    return flags;
}

// Fungsi motor_hw_sampling_drive_flags: membaca subset hard-critical dari kontrak sampling untuk gate motor.
uint32_t motor_hw_sampling_drive_flags(void) {
    return motor_hw_sampling_contract_flags() & HW_SAMPLING_CONTRACT_DRIVE_CRITICAL_MASK;
}

// Fungsi motor_hw_sampling_contract_valid: menjalankan operasi motor hw sampling contract valid sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
bool motor_hw_sampling_contract_valid(void) {
    return motor_hw_sampling_contract_flags() == 0U;
}

// Fungsi motor_hw_sampling_drive_valid: memastikan bit hard-critical timer/ADC/DMA valid untuk mengizinkan drive.
bool motor_hw_sampling_drive_valid(void) {
    return motor_hw_sampling_drive_flags() == 0U;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter enabled: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi motor_hw_set_pwm_enabled: mengatur motor hw set pwm enabled setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_hw_set_pwm_enabled(MotorRuntime *m, bool enabled) {
    if (m == NULL || m->pwm_tim == NULL)
        return;
    if (enabled) {
        if (s_powerstage_fault_flags != 0U && !foc_calibration_in_progress()) {
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
    }
    else {
        m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
        m->pwm_enabled = false;
        m->pwm_enable_blank_cycles = 0U;
        m->pwm_enable_pending_events = 0U;
        if (m->full_brake_active)
            motor_hw_restore_foc_outputs(m);
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_service_pwm_enable_from_isr: menangani motor hw service pwm enable from isr pada konteks
// interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
void motor_hw_service_pwm_enable_from_isr(MotorRuntime *m) {
    if (m == NULL || m->pwm_enabled || m->pwm_enable_pending_events == 0U)
        return;
    /* Hardware power-stage faults (PVD/BKIN) block MOE in the normal running
     * state. During calibration, however, the bridges are driven with a safe
     * 50% zero-vector and no torque is produced, so a latched/stale power-stage
     * fault (e.g. a PVD glitch coupled in by the switching edges, or a
     * FLASH_CONFIG fault from blank flash) must NOT prevent MOE from arming.
     * The comment at the top of this function already states this intent; the
     * original check below re-cleared MOE on every ISR once a fault latched,
     * which wedged the driven calibration stage (MOE confirmed in WARMUP, then
     * dropped in DRIVEN, wait-events expired, FAIL). Only a real *software*
     * motor fault (not calibration-in-progress) may block MOE while calibrating. */
    if ((s_powerstage_fault_flags != 0U && !foc_calibration_in_progress()) ||
        (m->fault != MOTOR_FAULT_NONE && !foc_calibration_in_progress())) {
        m->pwm_enable_pending_events = 0U;
        m->pwm_tim->BDTR &= ~TIM_BDTR_MOE;
        return;
    }

    /* Keep refreshing the preload while waiting. Every call corresponds to a
       completed fast ADC frame, i.e. one complete 16-kHz PWM schedule. */
    motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
    if (--m->pwm_enable_pending_events != 0U)
        return;

    m->pwm_enable_blank_cycles = PWM_ENABLE_BLANK_CYCLES;
    m->pwm_tim->BDTR |= TIM_BDTR_MOE;
    m->pwm_enabled = true;
}


// Parameter tim: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi motor_hw_set_oc_mode_triplet: mengatur motor hw set oc mode triplet setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
static void motor_hw_set_oc_mode_triplet(TIM_TypeDef *tim, uint32_t mode) {
    // Variabel ccmr1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t ccmr1 = tim->CCMR1;
    // Variabel ccmr2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t ccmr2 = tim->CCMR2;
    ccmr1 &= ~(TIM_CCMR1_OC1M_MASK_LOCAL | TIM_CCMR1_OC2M_MASK_LOCAL);
    ccmr2 &= ~TIM_CCMR2_OC3M_MASK_LOCAL;
    ccmr1 |= (mode << 4) | (mode << 12);
    ccmr2 |= (mode << 4);
    tim->CCMR1 = ccmr1;
    tim->CCMR2 = ccmr2;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_restore_foc_outputs: menjalankan bagian motor hw restore foc outputs pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void motor_hw_restore_foc_outputs(MotorRuntime *m) {
    if (m == NULL || m->pwm_tim == NULL)
        return;
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                          TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    motor_hw_set_oc_mode_triplet(m->pwm_tim, TIM_OCMODE_PWM1_LOCAL);
    m->pwm_tim->CCER |= mask;
    m->pwm_tim->EGR = TIM_EGR_COMG_LOCAL;
    m->full_brake_active = false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter enable: penanda untuk mengaktifkan atau menonaktifkan fitur.
// Fungsi motor_hw_set_low_side_brake: mengatur motor hw set low side brake setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_hw_set_low_side_brake(MotorRuntime *m, bool enable) {
    if (m == NULL || m->pwm_tim == NULL)
        return;
    if (!enable) {
        if (m->full_brake_active)
            motor_hw_restore_foc_outputs(m);
        return;
    }
    if (m->full_brake_active)
        return;
    if (!m->pwm_enabled || m->fault != MOTOR_FAULT_NONE || s_powerstage_fault_flags != 0U)
        return;

    /* With this board's configured polarity, forced-inactive drives CHx low
       (high-side OFF) while the complementary active-low CHxN is asserted
       (low-side ON). This mirrors VESC full_brake_hw. The feature is disabled
       by default because continuous low-side gate-drive must be bench-tested. */
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                          TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    motor_hw_set_oc_mode_triplet(m->pwm_tim, TIM_OCMODE_FORCED_INACTIVE_LOCAL);
    m->pwm_tim->CCER |= mask;
    m->pwm_tim->EGR = TIM_EGR_COMG_LOCAL;
    m->full_brake_active = true;
}

// Fungsi motor_hw_powerstage_fault_flags: menangani motor hw powerstage fault flags dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
uint32_t motor_hw_powerstage_fault_flags(void) {
    return s_powerstage_fault_flags;
}
// Fungsi motor_hw_powerstage_fault_latched: menangani motor hw powerstage fault latched dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
bool motor_hw_powerstage_fault_latched(void) {
    return s_powerstage_fault_flags != 0U;
}
// Fungsi motor_hw_pvd_low: menjalankan operasi motor hw pvd low sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool motor_hw_pvd_low(void) {
#if HOVERBOARD_PVD_ENABLE
    return (PWR->CSR & (1UL << 2)) != 0U;
#else
    return false;
#endif
}

// Fungsi motor_hw_clear_recoverable_powerstage_faults: mereset motor hw clear recoverable powerstage faults ke
// kondisi awal yang aman tanpa meninggalkan state lama yang tidak konsisten.
bool motor_hw_clear_recoverable_powerstage_faults(void) {
    /* Calibration is a stopped, zero-vector operation. Clear only a stale
     * software latch when the underlying hardware condition is absent now.
     * Never clear while PVD is currently low or a timer break flag is set. */
#if HOVERBOARD_PVD_ENABLE
    if ((PWR->CSR & (1UL << 2)) != 0U)
        return false;
#endif
#if HOVERBOARD_TIM1_BREAK_ENABLE
    if ((TIM1->SR & TIM_SR_BIF) != 0U)
        return false;
#endif
#if HOVERBOARD_TIM8_BREAK_ENABLE
    if ((TIM8->SR & TIM_SR_BIF) != 0U)
        return false;
#endif
    s_powerstage_fault_flags = 0U;
    return true;
}

// Fungsi motor_hw_pvd_irq_handler: menangani motor hw pvd irq handler pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
void motor_hw_pvd_irq_handler(void) {
#if HOVERBOARD_PVD_ENABLE
    EXTI->PR = (1UL << 16);
    /* During current-offset calibration the bridges are driven with a safe
     * 50% zero-vector and produce no phase current, so a PVD glitch coupled in
     * by the switching edges must not latch a fault or clear MOE. Suppress the
     * PVD reaction entirely while calibration is in progress; the normal
     * running state still gets the full under-voltage protection. */
    if (foc_calibration_in_progress())
        return;
    if ((PWR->CSR & (1UL << 2)) != 0U) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM8->BDTR &= ~TIM_BDTR_MOE;
        s_powerstage_fault_flags |= POWERSTAGE_FAULT_PVD;
        motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_MCU_UNDER_VOLTAGE);
        motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_MCU_UNDER_VOLTAGE);
    }
#endif
}

// Parameter tim: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_break_irq_handler: menangani motor hw break irq handler pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void motor_hw_break_irq_handler(TIM_TypeDef *tim) {
    if (tim == NULL || (tim->SR & TIM_SR_BIF) == 0U)
        return;
    tim->SR &= ~TIM_SR_BIF;
    /* During current-offset calibration the bridges run a safe 50% zero-vector,
     * so a spurious break event must not latch a fault or clear MOE. Suppress
     * the break reaction while calibration is in progress. */
    if (foc_calibration_in_progress())
        return;
    /* Hardware already cleared MOE asynchronously. Explicitly clear both
       bridges as the dual-motor board shares one supply/power-stage domain. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
    if (tim == TIM1)
        s_powerstage_fault_flags |= POWERSTAGE_FAULT_TIM1;
    else if (tim == TIM8) {
        s_powerstage_fault_flags |= POWERSTAGE_FAULT_TIM8;
    }
    motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_BREAK);
    motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_BREAK);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter du_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter dv_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter dw_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi motor_hw_set_pwm_q15: mengatur motor hw set pwm q15 setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_hw_set_pwm_q15(MotorRuntime *m, uint16_t du_q15, uint16_t dv_q15, uint16_t dw_q15) {
    /* The 10..90% current-sampling window is enforced as a vector operation
       in foc_svm_q15(). Do not clip U/V/W independently here because that
       rotates/distorts the alpha/beta voltage vector. Keep only the absolute
       timer-domain guard for corrupt/non-FOC callers. */
    if (du_q15 > 32767U)
        du_q15 = 32767U;
    if (dv_q15 > 32767U)
        dv_q15 = 32767U;
    if (dw_q15 > 32767U)
        dw_q15 = 32767U;

    // Variabel cu: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t cu = ((uint32_t)du_q15 * PWM_TIMER_ARR) >> 15;
    // Variabel cv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t cv = ((uint32_t)dv_q15 * PWM_TIMER_ARR) >> 15;
    // Variabel cw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t cw = ((uint32_t)dw_q15 * PWM_TIMER_ARR) >> 15;

    /* All three CCRs are preload-enabled. UDIS prevents a software update from
       splitting this triplet; the next hardware update latches them together. */
    m->pwm_tim->CR1 |= TIM_CR1_UDIS;
    m->pwm_tim->CCR1 = cu;
    m->pwm_tim->CCR2 = cv;
    m->pwm_tim->CCR3 = cw;
    m->pwm_tim->CR1 &= ~TIM_CR1_UDIS;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter du: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dv: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dw: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_set_pwm_duty: mengatur motor hw set pwm duty setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_hw_set_pwm_duty(MotorRuntime *m, float du, float dv, float dw) {
    /* This helper is not the FOC current-control path. Bound only to the timer
       domain; callers that need the current-sampling window must use SVM. */
    if (du < 0.0f)
        du = 0.0f;
    if (du > 0.999969f)
        du = 0.999969f;
    if (dv < 0.0f)
        dv = 0.0f;
    if (dv > 0.999969f)
        dv = 0.999969f;
    if (dw < 0.0f)
        dw = 0.0f;
    if (dw > 0.999969f)
        dw = 0.999969f;
    motor_hw_set_pwm_q15(m,
        (uint16_t)(du * 32768.0f),
        (uint16_t)(dv * 32768.0f),
        (uint16_t)(dw * 32768.0f));
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi motor_hw_read_hall_raw: menjalankan operasi motor hw read hall raw sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
uint8_t motor_hw_read_hall_raw(motor_id_t id) {
    /* Stock hoverboard Hall inputs are active-low. This function is called
       from the hard FOC path, so use one IDR snapshot per GPIO port rather
       than three HAL_GPIO_ReadPin calls. */
    // Variabel idr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t idr;
    // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t u, v, w;
    if (id == MOTOR_LEFT) {
        idr = GPIOB->IDR;
        u = (idr & LEFT_HALL_U_PIN) ? 0U : 1U;
        v = (idr & LEFT_HALL_V_PIN) ? 0U : 1U;
        w = (idr & LEFT_HALL_W_PIN) ? 0U : 1U;
    }
    else {
        idr = GPIOC->IDR;
        u = (idr & RIGHT_HALL_U_PIN) ? 0U : 1U;
        v = (idr & RIGHT_HALL_V_PIN) ? 0U : 1U;
        w = (idr & RIGHT_HALL_W_PIN) ? 0U : 1U;
    }
    /* Samakan packing dengan firmware hoverboard referensi dan estimator
       generated: U adalah bit-2, V bit-1, W bit-0. Urutan bit ini penting
       karena tabel Hall mengindeks raw state secara langsung. */
    return (uint8_t)((u << 2) | (v << 1) | w);
}

// Parameter temp_c: temperatur atau nilai sementara sesuai konteks modul.
// Fungsi motor_hw_board_temperature_c: menjalankan operasi motor hw board temperature c sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
bool motor_hw_board_temperature_c(float *temp_c) {
    if (temp_c == NULL)
        return false;

    /* ADC1 adalah low-halfword. Gunakan snapshot rank-5 dari frame sebelumnya
       yang dilatch di HT agar task tidak membaca slot saat DMA sedang menulis. */
    if (s_temp_adc_seq == 0U)
        return false;
    // Variabel dual: snapshot temperatur yang koheren terhadap satu frame PWM.
    const uint32_t dual = s_temp_adc_word;
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    const uint16_t raw = (uint16_t)(dual & 0xFFFFU);
    if (raw < HOVERBOARD_MCU_TEMP_ADC_MIN_VALID ||
        raw > HOVERBOARD_MCU_TEMP_ADC_MAX_VALID)
        return false;

    // Variabel vsense: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float vsense = ((float)raw * HOVERBOARD_ADC_VDDA_NOMINAL_V) / 4095.0f;
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float t = 25.0f +
        (HOVERBOARD_MCU_TEMP_V25_V - vsense) / HOVERBOARD_MCU_TEMP_AVG_SLOPE_V_PER_C;
    if (t < HOVERBOARD_MCU_TEMP_MIN_VALID_C || t > HOVERBOARD_MCU_TEMP_MAX_VALID_C)
        return false;
    *temp_c = t;
    return true;
}

// Fungsi motor_hw_encoder_cnt: menjalankan operasi motor hw encoder cnt sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
uint16_t motor_hw_encoder_cnt(void) {
    return (uint16_t)TIM4->CNT;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Fungsi motor_hw_encoder_set_count: mengatur motor hw encoder set count setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_hw_encoder_set_count(MotorRuntime *m, uint16_t count) {
    if (!m || m->id != MOTOR_LEFT || m->encoder.cpr < 4U)
        return;
    count = (uint16_t)(count % m->encoder.cpr);
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM4->CNT = count;
    TIM4->SR &= ~TIM_SR_UIF;
    m->encoder.turns = 0;
    m->encoder.extended_count = (int32_t)count;
    m->encoder.prev_extended_count = (int32_t)count;
    m->encoder.speed_sample_valid = false;
    if (!primask)
        __enable_irq();
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi motor_hw_configure_sensor: menjalankan operasi motor hw configure sensor sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void motor_hw_configure_sensor(MotorRuntime *m, uint8_t mode) {
    if (m == NULL)
        return;

    /* Sensor mux changes are never permitted while power PWM is enabled. */
    motor_hw_set_pwm_enabled(m, false);

    if (m->id == MOTOR_RIGHT) {
        // Variabel hall_mask: mask pin Hall untuk memutus jalur interrupt saat sensorless.
        const uint32_t hall_mask = RIGHT_HALL_U_PIN | RIGHT_HALL_V_PIN | RIGHT_HALL_W_PIN;
        EXTI->IMR &= ~hall_mask;
        EXTI->PR = hall_mask;

        if (mode == SENSOR_MODE_NO_SENSOR) {
            /* Sensorless murni tidak memerlukan edge Hall. Biarkan pin sebagai
               input pasif tanpa EXTI agar noise kabel kosong tidak menambah ISR. */
            HAL_GPIO_DeInit(GPIOC, hall_mask);
            GPIO_InitTypeDef g = {0};
            g.Mode = GPIO_MODE_INPUT;
            g.Pull = GPIO_NOPULL;
            g.Pin = hall_mask;
            HAL_GPIO_Init(GPIOC, &g);
            m->sensor_mode = SENSOR_MODE_NO_SENSOR;
            return;
        }

        /* PCB RIGHT tidak memiliki encoder; mode selain SENSORLESS memakai Hall. */
        GPIO_InitTypeDef g = {0};
        g.Mode = GPIO_MODE_IT_RISING_FALLING;
        g.Pull = GPIO_NOPULL;
        g.Pin = hall_mask;
        HAL_GPIO_Init(GPIOC, &g);
        EXTI->PR = hall_mask;
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        m->sensor_mode = SENSOR_MODE_HALL;
        return;
    }

    /* LEFT PB6/PB7 are shared by Hall and TIM4 encoder. */
    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    EXTI->IMR &= ~(LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN);
    EXTI->PR = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
    HAL_GPIO_DeInit(GPIOB, LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN);

    // Variabel g: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_NOPULL;

    if (mode == SENSOR_MODE_NO_SENSOR) {
        /* LEFT sensorless juga melepas TIM4 dan EXTI Hall. PB5/PB6/PB7 tetap
           input pasif sehingga firmware tidak bergantung pada sensor eksternal. */
        g.Mode = GPIO_MODE_INPUT;
        g.Pin = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
        HAL_GPIO_Init(GPIOB, &g);
        m->sensor_mode = SENSOR_MODE_NO_SENSOR;
    }
    else if (mode == SENSOR_MODE_ENCODER) {
        g.Mode = GPIO_MODE_INPUT;
        g.Pin = LEFT_ENCODER_A_PIN | LEFT_ENCODER_B_PIN;
        HAL_GPIO_Init(GPIOB, &g);
        g.Pin = LEFT_HALL_U_PIN;
        HAL_GPIO_Init(GPIOB, &g);

        // Variabel cpr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t cpr = m->encoder.cpr;
        if (cpr < 4U)
            cpr = 4U;
        if (cpr > 65535U)
            cpr = 65535U;
        TIM4->ARR = cpr-1U;
        __HAL_TIM_SET_COUNTER(&htim4, 0U);
        m->encoder.turns = 0;
        m->encoder.extended_count = 0;
        m->encoder.prev_extended_count = 0;
        m->encoder.speed_sample_valid = false;
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
        if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
            Error_Handler_Local();
        m->sensor_mode = SENSOR_MODE_ENCODER;
    }
    else {
        g.Mode = GPIO_MODE_IT_RISING_FALLING;
        g.Pin = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
        HAL_GPIO_Init(GPIOB, &g);
        EXTI->PR = LEFT_HALL_U_PIN | LEFT_HALL_V_PIN | LEFT_HALL_W_PIN;
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        m->sensor_mode = SENSOR_MODE_HALL;
    }
}

// Parameter on: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_led: menjalankan operasi motor hw led sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
void motor_hw_led(bool on) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Fungsi motor_hw_emergency_all_off: menjalankan operasi motor hw emergency all off sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
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


// Fungsi motor_hw_capture_app_adc_from_isr: menangani motor hw capture app adc from isr pada konteks interrupt
// dengan pekerjaan minimum agar timing FOC tetap deterministik.
void motor_hw_capture_app_adc_from_isr(void) {
    /* TC occurs after three complete V15-format frames. Read rank 4/5 from
       the newest frame; DMA is circular and will start frame 0 again only
       after this completed batch. */
    const uint32_t base = ADC_WORDS_PER_FRAME * (ADC_DMA_BATCH_FRAMES - 1U);
    __DMB();
    s_app_adc_word = g_adc_dual_dma[base + 3U]; /* PC2/VBAT | ADC2 PA2 */
    s_temp_adc_word = g_adc_dual_dma[base + 4U]; /* TEMP | ADC2 PA3 */
    __DMB();
    s_app_adc_seq++;
    s_temp_adc_seq++;
}


// Parameter pa2_raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter pa3_raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi motor_hw_get_app_adc_raw: membaca motor hw get app adc raw tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
bool motor_hw_get_app_adc_raw(uint16_t *pa2_raw, uint16_t *pa3_raw) {
    if (pa2_raw == NULL || pa3_raw == NULL || s_app_adc_seq == 0U)
        return false;
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel word: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t a, b, rank4_word, rank5_word;
    do {
        a = s_app_adc_seq;
        __DMB();
        rank4_word = s_app_adc_word;
        rank5_word = s_temp_adc_word;
        __DMB();
        b = s_app_adc_seq;
    } while (a != b);
    /* Dual-mode DMA: ADC1 berada di low16 dan ADC2 di high16. Sesuai
     * reference, PA2=ADC2 rank-4 dan PA3=ADC2 rank-5. */
    *pa2_raw = (uint16_t)(rank4_word >> 16);
    *pa3_raw = (uint16_t)(rank5_word >> 16);
    return true;
}
