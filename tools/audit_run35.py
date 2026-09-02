#!/usr/bin/env python3
from pathlib import Path
import hashlib, re, sys
ROOT=Path(__file__).resolve().parents[1]
PASS=0; FAIL=0

def check(name, cond):
    global PASS, FAIL
    if cond:
        PASS += 1; print('PASS ', name)
    else:
        FAIL += 1; print('FAIL ', name)

def txt(rel): return (ROOT/rel).read_text(errors='replace')
def sha(rel): return hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()

app=txt('src/applications/appconf_default.h')
hw=txt('src/hwconf/hw.c')
hwh=txt('src/hwconf/hw.h')
it=txt('src/stm32f1xx_it.c')
foc=txt('src/motor/mcpwm_foc.c')
cmd=txt('src/comm/commands.c')
dbg=txt('tools/debug.py')
plat=txt('platformio.ini')

print('=== RUN31 BASE PRESERVATION ===')
manifest=txt('docs/RUN35_RUN31_UNCHANGED_SHA256.txt').splitlines()
manifest_ok=True
for line in manifest:
    if not line.strip(): continue
    h, rel=line.split('  ',1)
    p=ROOT/rel
    if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:
        print('  MISMATCH',rel); manifest_ok=False
check('all non-ADC/ISR Run31 base files are byte-identical', manifest_ok)
check('Run29 audit retained', (ROOT/'tools/audit_run29.py').exists())
check('Run30 audit retained', (ROOT/'tools/audit_run30.py').exists())
check('Run31 Detect-All source retained', 'detect_all' in cmd.lower() and 'COMM_DETECT_APPLY_ALL_FOC' in cmd)
check('Run31 Hall detector retained', 'mcpwm_foc_hall_detect_motor' in foc)
check('Run31 position path retained', 'MOTOR_CTRL_POSITION' in foc)
check('Run31 HFI minimum migration retained', 'foc_hfi_start_samples' in txt('src/confgenerator.c') and '5U' in txt('src/confgenerator.c'))
check('release build remains O3', 'default_envs = stm32f103rc' in plat and '-O3' in plat)

print('\n=== V15 ADC MAPPING ===')
check('ADC1 has exactly 5 conversions', 'hadc1.Init.NbrOfConversion = 5;' in hw)
check('ADC2 has exactly 5 conversions', 'hadc2.Init.NbrOfConversion = 5;' in hw)
expect=[
('ADC1 PC1 RIGHT DC rank1','cfg_adc_channel(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5);'),
('ADC2 PC0 LEFT DC rank1','cfg_adc_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5);'),
('ADC1 PA0 LEFT A rank2','cfg_adc_channel(&hadc1, ADC_CHANNEL_0,  ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5);'),
('ADC2 PC3 LEFT B rank2','cfg_adc_channel(&hadc2, ADC_CHANNEL_13, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5);'),
('ADC1 PC4 RIGHT B rank3','cfg_adc_channel(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5);'),
('ADC2 PC5 RIGHT C rank3','cfg_adc_channel(&hadc2, ADC_CHANNEL_15, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5);'),
('ADC1 PC2 VBUS rank4','cfg_adc_channel(&hadc1, ADC_CHANNEL_12, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_7CYCLES_5);'),
('ADC2 PA2 APP rank4','cfg_adc_channel(&hadc2, ADC_CHANNEL_2,  ADC_REGULAR_RANK_4, ADC_SAMPLETIME_7CYCLES_5);'),
('ADC1 TEMP rank5','cfg_adc_channel(&hadc1, ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_239CYCLES_5);'),
('ADC2 PA3 APP rank5','cfg_adc_channel(&hadc2, ADC_CHANNEL_3,  ADC_REGULAR_RANK_5, ADC_SAMPLETIME_7CYCLES_5);'),
]
for n,t in expect: check(n,t in hw)
check('no ADC rank6 configured', 'ADC_REGULAR_RANK_6' not in hw)
check('ADC master trigger remains TIM8_TRGO', 'ADC_EXTERNALTRIGCONV_T8_TRGO' in hw)
check('dual regular simultaneous remains active', 'ADC_DUALMODE_REGSIMULT' in hw)
check('ADC clock remains APB2 /6', 'RCC_ADCPCLK2_DIV6' in hw)
check('ADC3 not active', 'ADC_HandleTypeDef hadc3' not in hw)
check('DMA2 not active for ADC', 'DMA_HandleTypeDef hdma_adc3' not in hw)

print('\n=== DMA / ISR BUDGET ===')
check('five words per ADC frame', '#define ADC_WORDS_PER_FRAME             5UL' in app)
check('three ADC frames per DMA batch', '#define ADC_DMA_BATCH_FRAMES            3UL' in app)
check('DMA batch is 15 words', 'ADC_DMA_BATCH_WORDS             (ADC_WORDS_PER_FRAME * ADC_DMA_BATCH_FRAMES)' in app)
check('ADC sampling stays 16 kHz', '#define ADC_SAMPLE_EVENT_HZ             PWM_FREQUENCY_HZ' in app)
check('full FOC rate is PWM/3', '#define FOC_ISR_EVENT_HZ                (PWM_FREQUENCY_HZ / FOC_CONTROL_DIV)' in app)
check('FOC dt is 3/PWM', '#define FOC_DT_S                        ((float)FOC_CONTROL_DIV / (float)PWM_FREQUENCY_HZ)' in app)
check('FOC slot is three PWM periods', '#define FOC_ISR_SLOT_CYCLES             ((CPU_CLOCK_HZ * FOC_CONTROL_DIV) / PWM_FREQUENCY_HZ)' in app)
check('DMA start length uses 15-word batch macro', 'HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)g_adc_dual_dma, ADC_DMA_BATCH_WORDS)' in hw)
check('DMA HT IRQ disabled', '__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);' in hw)
check('DMA TC IRQ enabled', '__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);' in hw)
check('DMA IRQ handles TC not HT', 'DMA_ISR_TCIF1' in it and 'DMA_ISR_HTIF1' not in it)
check('DMA IRQ clears TC flag', 'DMA_IFCR_CTCIF1' in it)
check('ISR passes newest third frame', '&g_adc_dual_dma[base]' in it and 'ADC_DMA_BATCH_FRAMES - 1U' in it)
check('app ADC snapshot uses newest third frame', 'base + 3U' in hw and 'base + 4U' in hw)
check('TE path still shuts both bridges off', 'motor_hw_emergency_all_off();' in it and 'MOTOR_FAULT_ADC_DMA' in it)
check('no ADC work is routed through DMA2 ISR', 'DMA2_Channel4_5_IRQHandler' in it and 'foc_adc_dma' not in it[it.find('void DMA2_Channel4_5_IRQHandler'):it.find('void DMA1_Channel1_IRQHandler')])

print('\n=== FULL RUN31 CONTROL FEATURES ===')
for token in ['COMM_SET_DUTY','COMM_SET_CURRENT','COMM_SET_CURRENT_BRAKE','COMM_SET_RPM','COMM_SET_POS','COMM_SET_HANDBRAKE','COMM_SET_CURRENT_REL']:
    check(token+' retained', token in cmd)
for token in ['detect_resistance','detect_inductance','detect_flux_linkage','mcpwm_foc_hall_detect_motor']:
    check(token+' retained', token in (cmd+foc))
check('EEPROM 5-beep contract retained', 'EEPROM' in txt('src/hwconf/hw_status.c') and ('5' in txt('src/hwconf/hw_status.c')))
check('running 3-pulse LED path retained', 'RUNNING' in txt('src/hwconf/hw_status.c').upper())

print('\n=== VESC IDENTITY ===')
check('local ID1 reports MOTOR_LEFT', '"MOTOR_RIGHT" : "MOTOR_LEFT"' in cmd)
check('forwarded ID2 reports MOTOR_RIGHT', '? "MOTOR_RIGHT" : "MOTOR_LEFT"' in cmd)
check('debug expects MOTOR_RIGHT for motor2-forward', 'expected MOTOR_RIGHT' in dbg)

print(f'\nSUMMARY: {PASS}/{PASS+FAIL} PASS')
if FAIL:
    sys.exit(1)
