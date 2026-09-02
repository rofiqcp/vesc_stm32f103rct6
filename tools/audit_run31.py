#!/usr/bin/env python3
from pathlib import Path
import re, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]

def text(rel): return (ROOT/rel).read_text(encoding='utf-8',errors='replace')
checks=[]
def ck(name, ok):
    checks.append((name,bool(ok)))
    print(('PASS  ' if ok else 'FAIL  ')+name)

r=subprocess.run([sys.executable,str(ROOT/'tools/audit_run30.py')],cwd=ROOT,text=True,capture_output=True)
print('=== RUN30 BASELINE ===')
print(r.stdout.rstrip())
ck('Run30 baseline audit remains green', r.returncode==0 and '41/41 PASS' in r.stdout)

pio=text('platformio.ini')
app=text('src/applications/appconf_default.h')
hw=text('src/hwconf/hw.c')
hwh=text('src/hwconf/hw.h')
hwboard=text('src/hwconf/hw_hoverboard.h')
it=text('src/stm32f1xx_it.c')
foc=text('src/motor/mcpwm_foc.c')
cmd=text('src/comm/commands.c')
gen=text('src/confgenerator.c')
dbg=text('tools/debug.py')
main=text('src/main.c')

print('\n=== RUN31 REALTIME BUILD ===')
ck('default hardware environment is optimized release', 'default_envs = stm32f103rc' in pio and 'default_envs = stm32f103rc_debug' not in pio)
ck('release image uses O3', '-O3' in pio and '-DVESC_REALTIME_OPTIMIZED=1' in pio)
# Look only inside debug section for optimization contract.
dbgsec=pio.split('[env:stm32f103rc_debug]',1)[1] if '[env:stm32f103rc_debug]' in pio else ''
ck('debug-symbol image also forces O3', '-O3' in dbgsec and '-Og' in dbgsec and 'build_unflags' in dbgsec)
ck('firmware CPU clock contract is 64 MHz', '#define CPU_CLOCK_HZ                    64000000UL' in app)
ck('system clock is HSI PLL x16 = 64 MHz', 'RCC_PLL_MUL16' in main and 'RCC_PLLSOURCE_HSI_DIV2' in main)
ck('16-kHz FOC slot is derived from CPU clock', 'FOC_ISR_SLOT_CYCLES             (CPU_CLOCK_HZ / FOC_ISR_EVENT_HZ)' in app)

print('\n=== RUN31 ADC / PWM / ISR ===')
ck('ADC3 and DMA2 are absent from active hardware globals', 'ADC_HandleTypeDef hadc3' not in hw and 'DMA_HandleTypeDef hdma_adc3' not in hw and 'g_adc3_vbus_dma' not in hw)
ck('public ADC DMA frame is six words', 'g_adc_dual_dma[6]' in hw and 'g_adc_dual_dma[6]' in hwh)
ck('ADC1 and ADC2 each configure six regular ranks', hw.count('Init.NbrOfConversion = 6;') >= 2)
for token,label in [
 ('ADC_CHANNEL_11, ADC_REGULAR_RANK_1','ADC1 PC1 RIGHT DC rank1'),
 ('ADC_CHANNEL_10, ADC_REGULAR_RANK_1','ADC2 PC0 LEFT DC rank1'),
 ('ADC_CHANNEL_0, ADC_REGULAR_RANK_2','ADC1 PA0 LEFT A rank2'),
 ('ADC_CHANNEL_13, ADC_REGULAR_RANK_2','ADC2 PC3 LEFT B rank2'),
 ('ADC_CHANNEL_14, ADC_REGULAR_RANK_3','ADC1 PC4 RIGHT B rank3'),
 ('ADC_CHANNEL_15, ADC_REGULAR_RANK_3','ADC2 PC5 RIGHT C rank3'),
 ('ADC_CHANNEL_12, ADC_REGULAR_RANK_4','ADC1 PC2 VBUS rank4'),
 ('ADC_CHANNEL_2, ADC_REGULAR_RANK_4','ADC2 PA2 APP rank4'),
 ('ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_5','ADC1 TEMP rank5'),
 ('ADC_CHANNEL_3, ADC_REGULAR_RANK_5','ADC2 PA3 APP rank5'),
]: ck(label, token in hw)
ck('rank6 filler exists only to align DMA half-transfer', 'ADC_CHANNEL_12, ADC_REGULAR_RANK_6' in hw and 'ADC_CHANNEL_3, ADC_REGULAR_RANK_6' in hw and 'Filler hanya' in hw)
ck('ADC1 master trigger remains TIM8_TRGO', 'hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;' in hw)
ck('ADC1 regular trigger AFIO remap is forced safely', 'AFIO_MAPR_ADC1_ETRGREG_REMAP' in hw and 'AFIO_SWJ_JTAG_OFF_SWD_ON_LOCAL' in hw)
ck('ADC clock stays /6 at 64 MHz', '__HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);' in hw and '#define ADC_CLOCK_DIV                   6UL' in app)
ck('DMA length is exactly six words', 'HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)g_adc_dual_dma, 6U)' in hw)
ck('DMA ISR uses half-transfer after current rank3', '__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);' in hw and '__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);' in hw and 'DMA_ISR_HTIF1' in it and 'DMA_IFCR_CHTIF1' in it)
ck('Vbus comes from ADC1 rank4 without ADC3 freshness fault', 'uint16_t vraw = low16(adc_words[3]);' in foc and 'DMA2_Channel5->' not in foc)
ck('sampling contract expects 6 ranks + HT IRQ', '!= 5U' in hw and 'DMA_CCR_HTIE' in hw and 'dma1_count > 6U' in hw)
ck('trigger-route audit only checks ADC1 TIM8 remap', 'HW_SAMPLING_CONTRACT_TRIGGER_ROUTE' in hw and 'ADC3->' not in hw)
ck('legacy ADC3 diagnostic slots are zero-only', 'legacy adc3_enabled' in cmd and 'ADC3->' not in cmd and 'DMA2_Channel5->' not in cmd)

print('\n=== RUN31 POWER-STAGE / PIN PARITY WITH RUNNING HOVERBOARD ===')
ck('PWM frequency remains 16 kHz and ARR resolves to 2000 at 64 MHz',
   '#define PWM_FREQUENCY_HZ                16000UL' in app and
   '#define PWM_TIMER_ARR                   (CPU_CLOCK_HZ / (2UL * PWM_FREQUENCY_HZ))' in app)
ck('deadtime is 750 ns = 48 timer ticks at 64 MHz, matching reference DEAD_TIME=48',
   '#define PWM_DEADTIME_NS                 750UL' in app and 'deadtime_to_dtg(PWM_DEADTIME_NS)' in hw)
ck('high-side active-high and low-side active-low contract retained',
   '#define HIGH_SIDE_ACTIVE_HIGH           1' in app and '#define LOW_SIDE_ACTIVE_LOW             1' in app)
ck('advanced-timer output polarity matches running reference',
   'oc.OCPolarity = TIM_OCPOLARITY_HIGH;' in hw and 'oc.OCNPolarity = TIM_OCNPOLARITY_LOW;' in hw)
ck('MOE-off idle levels physically switch both FET inputs off',
   'oc.OCIdleState = TIM_OCIDLESTATE_RESET' in hw and 'oc.OCNIdleState = TIM_OCNIDLESTATE_SET' in hw)
ck('all three complementary PWM channel pairs are enabled',
   all(tok in hw for tok in ('TIM_CCER_CC1E | TIM_CCER_CC1NE','TIM_CCER_CC2E | TIM_CCER_CC2NE','TIM_CCER_CC3E | TIM_CCER_CC3NE')))
ck('hardware break remains disabled by default like running reference',
   '#define HOVERBOARD_TIM1_BREAK_ENABLE          0' in hwboard and '#define HOVERBOARD_TIM8_BREAK_ENABLE          0' in hwboard)
ck('LEFT TIM8 phase pins match running hoverboard reference',
   all(tok in hwboard for tok in ('LEFT_TIM_UH_PIN GPIO_PIN_6','LEFT_TIM_UH_PORT GPIOC','LEFT_TIM_UL_PIN GPIO_PIN_7','LEFT_TIM_UL_PORT GPIOA',
                                  'LEFT_TIM_VH_PIN GPIO_PIN_7','LEFT_TIM_VH_PORT GPIOC','LEFT_TIM_VL_PIN GPIO_PIN_0','LEFT_TIM_VL_PORT GPIOB',
                                  'LEFT_TIM_WH_PIN GPIO_PIN_8','LEFT_TIM_WH_PORT GPIOC','LEFT_TIM_WL_PIN GPIO_PIN_1','LEFT_TIM_WL_PORT GPIOB')))
ck('RIGHT TIM1 phase pins match running hoverboard reference',
   all(tok in hwboard for tok in ('RIGHT_TIM_UH_PIN GPIO_PIN_8','RIGHT_TIM_UH_PORT GPIOA','RIGHT_TIM_UL_PIN GPIO_PIN_13','RIGHT_TIM_UL_PORT GPIOB',
                                  'RIGHT_TIM_VH_PIN GPIO_PIN_9','RIGHT_TIM_VH_PORT GPIOA','RIGHT_TIM_VL_PIN GPIO_PIN_14','RIGHT_TIM_VL_PORT GPIOB',
                                  'RIGHT_TIM_WH_PIN GPIO_PIN_10','RIGHT_TIM_WH_PORT GPIOA','RIGHT_TIM_WL_PIN GPIO_PIN_15','RIGHT_TIM_WL_PORT GPIOB')))
ck('current-sense and DCLINK GPIO pins match running hoverboard reference',
   all(tok in hwboard for tok in ('LEFT_DC_CUR_PIN GPIO_PIN_0','LEFT_DC_CUR_PORT GPIOC','LEFT_U_CUR_PIN GPIO_PIN_0','LEFT_U_CUR_PORT GPIOA',
                                  'LEFT_V_CUR_PIN GPIO_PIN_3','LEFT_V_CUR_PORT GPIOC','RIGHT_DC_CUR_PIN GPIO_PIN_1','RIGHT_DC_CUR_PORT GPIOC',
                                  'RIGHT_U_CUR_PIN GPIO_PIN_4','RIGHT_U_CUR_PORT GPIOC','RIGHT_V_CUR_PIN GPIO_PIN_5','RIGHT_V_CUR_PORT GPIOC',
                                  'DCLINK_PIN GPIO_PIN_2','DCLINK_PORT GPIOC')))
ck('Hall GPIO pins match running hoverboard reference',
   all(tok in hwboard for tok in ('LEFT_HALL_U_PIN GPIO_PIN_5','LEFT_HALL_V_PIN GPIO_PIN_6','LEFT_HALL_W_PIN GPIO_PIN_7',
                                  'RIGHT_HALL_U_PIN GPIO_PIN_10','RIGHT_HALL_V_PIN GPIO_PIN_11','RIGHT_HALL_W_PIN GPIO_PIN_12')))

print('\n=== RUN31 HFI / VESC TOOL ===')
ck('HFI legacy values below VESC minimum are normalized', 'get_u16_at(w, VESC6_MC_OFF_FOC_HFI_START_SAMPLES) < 5U' in gen and 'put_u16_at(w, VESC6_MC_OFF_FOC_HFI_START_SAMPLES, 5U);' in gen)
ck('factory HFI start samples matches VESC 6.00 default 5', 'vesc_buf_append_u16(b, 5U, &i);' in gen)
ck('factory HFI voltage max matches VESC 6.00 default 6 V', 'append_float16_field(b, 6.0f, 10, &i);' in gen)
ck('factory HFI transition ERPM matches VESC 6.00 default 3000', 'append_float32_auto_field(b, 3000.0f, &i);' in gen)
ck('persistent import normalizes both motor MCCONF images', gen.count('normalize_vesc6_hfi_ui_fields(s_mc_active[') >= 3)
ck('debug VESC Tool check explicitly rejects HFI <5', 'hfi_start_samples < 5' in dbg and 'VESC 6.00 default=5' in dbg)

print('\n=== RUN31 NATIVE FAULT DIAGNOSTICS ===')
ck('debug exposes fault-detail command', 'def cmd_fault_detail' in dbg and '"fault-detail"' in dbg)
ck('debug decodes native ADC_DMA and FOC_ISR_OVERRUN separately', 'ADC_DMA' in dbg and 'FOC_ISR_OVERRUN' in dbg and 'native_fault' in dbg)
ck('motor-test prints native detail on any VESC fault', '_print_fault_detail(link,args.motor,"fault during motor-test")' in dbg)
ck('firmware maps both private realtime faults to standard VESC DRV', 'MOTOR_FAULT_ADC_DMA' in text('src/motor/mc_interface.c') and 'MOTOR_FAULT_FOC_ISR_OVERRUN' in text('src/motor/mc_interface.c') and 'return FAULT_CODE_DRV;' in text('src/motor/mc_interface.c'))
ck('ISR records end-to-end cycle max and faults only after repeated overrun', 's_isr_total_max_cycles' in foc and 's_overrun_consecutive' in foc and '>= 8U' in foc)

print('\n=== RUN31 DETECT-ALL / COMMAND ACCEPTANCE ===')
ck('standard Detect-All FOC worker is active', 'case COMM_DETECT_APPLY_ALL_FOC:' in cmd and 'detect_apply_all_one_runtime' in cmd)
ck('Detect-All measures R and L before flux', 'measure_res_ind_ex(m, current_a, 200' in foc and 'mcpwm_foc_measure_flux_linkage_motor(m,' in foc)
ck('Detect-All applies measured R L Ld-Lq and flux to runtime', 'm->foc_motor_r = r;' in foc and 'm->foc_motor_l = l;' in foc and 'm->foc_motor_ld_lq_diff = ldq;' in foc and 'm->foc_motor_flux_linkage = flux;' in foc)
ck('Detect-All tries physical sensor then sensorless fallback', 'mcpwm_foc_hall_detect_motor' in cmd and 'apply_sensorless_result(m);' in cmd and 'validate_sensorless_runtime' in cmd)
ck('detect_can=true commissions both local motors atomically', 'detect_can && m->id == MOTOR_LEFT' in cmd and 'vesc_config_commit_detect_all_runtime_dual()' in cmd)
ck('debug Detect-All requires positive R L flux', 'parameter_fail' in dbg and "foc_motor_flux_linkage_wb" in dbg)
acc=dbg[dbg.index('def cmd_drive_acceptance'):dbg.index('# Fungsi _measured_rate_hz:')]
ck('drive acceptance runs Detect-All before motion matrix', acc.index('cmd_detect_all_foc') < acc.index("'current'"))
for token in ('cmd_hall_commission',"'duty'","'current'","'current-rel'","'rpm'",'cmd_brake_test','cmd_position_test',"'handbrake'",'cmd_config_save'):
    ck(f'drive acceptance retains {token}', token in acc)

# Hygiene catches the exact accidental out-of-bounds regression seen while developing Run31.
print('\n=== RUN31 HYGIENE ===')
ck('no out-of-bounds five-word declaration with six-word diagnostic loop', 'g_adc_dual_dma[5]' not in hw and 'for (uint8_t k = 0U; k < 6U; k++)\n        put_u32(p, &i, g_adc_dual_dma[k]);' in cmd)
ck('no active ADC3/DMA2 references anywhere except defensive IRQ/legacy names', not any(x in (hw+foc+cmd) for x in ('ADC3->','DMA2_Channel5->','HAL_ADC_Start_DMA(&hadc3')))
ck('no tabs in project C/H/Python', all('\t' not in p.read_text(encoding='utf-8',errors='ignore') for p in list((ROOT/'src').rglob('*.c'))+list((ROOT/'src').rglob('*.h'))+list((ROOT/'tools').glob('*.py'))))

failed=[n for n,ok in checks if not ok]
print(f"\nSUMMARY: {len(checks)-len(failed)}/{len(checks)} PASS")
if failed:
    for n in failed: print(' -',n)
    raise SystemExit(1)
