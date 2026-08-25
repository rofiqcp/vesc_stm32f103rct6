#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,sys
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

hw=read('src/hwconf/hw.c'); hwh=read('src/hwconf/hw.h')
foc=read('src/motor/mcpwm_foc.c'); irq=read('src/stm32f1xx_it.c')
conf=read('src/applications/appconf_default.h'); cmd=read('src/comm/commands.c')
dbg=read('tools/debug.py'); pio=read('platformio.ini')

# Target and ABI/build policy remain pinned.
ok('-DSTM32F103xE' in pio and 'genericSTM32F103RC' in pio,
   'target remains STM32F103RCT6/high-density')
ok(all(x in pio for x in ['platform = ststm32','board = genericSTM32F103RC',
                           'framework = stm32cube','board_build.ldscript = src/stm32f103rc_flash.ld']),
   'PlatformIO keeps the STM32F103RC/Cube target and linker script')
ok('board_build.f_cpu' not in pio,
   'firmware-owned 64-MHz HSI clock is not overridden by PlatformIO')

# Current acquisition contract remains unchanged.
fast_expected=[
 'cfg_adc_channel(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5)',
 'cfg_adc_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5)',
 'cfg_adc_channel(&hadc1, ADC_CHANNEL_0,  ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5)',
 'cfg_adc_channel(&hadc2, ADC_CHANNEL_13, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5)',
 'cfg_adc_channel(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5)',
 'cfg_adc_channel(&hadc2, ADC_CHANNEL_15, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5)']
ok(all(x in hw for x in fast_expected), 'all six phase/DC current conversions in ADC1/2 ranks 1-3 are unchanged')
ok('__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);' in hw and '__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);' in hw,
   'hard FOC remains driven by DMA1 Channel1 HT after current rank 3')

# ADC3/DMA2 dedicated Vbus backend.
ok(all(x in hw for x in ['__HAL_RCC_ADC3_CLK_ENABLE()','__HAL_RCC_DMA2_CLK_ENABLE()',
                         'hadc3.Instance = ADC3','hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO',
                         'cfg_adc_channel(&hadc3, ADC_CHANNEL_12, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_28CYCLES_5)']),
   'ADC3 owns PC2/DCLINK and is synchronized directly from TIM8_TRGO')
ok('hdma_adc3.Instance = DMA2_Channel5' in hw and
   'DMA_PDATAALIGN_HALFWORD' in hw and 'DMA_MDATAALIGN_HALFWORD' in hw and 'DMA_CIRCULAR' in hw,
   'ADC3 uses dedicated DMA2 Channel5 two-byte-width circular transfers')
ok('HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_adc3_vbus_dma, 2U)' in hw,
   'ADC3 Vbus uses a two-sample circular buffer for transfer-progress observability')
ok('__HAL_DMA_DISABLE_IT(&hdma_adc3, DMA_IT_HT);' in hw and
   '__HAL_DMA_DISABLE_IT(&hdma_adc3, DMA_IT_TC);' in hw and
   '__HAL_DMA_ENABLE_IT(&hdma_adc3, DMA_IT_TE);' in hw,
   'ADC3 avoids 16-kHz HT/TC IRQ load while retaining transfer-error IRQ safety')
ok('DMA2_Channel4_5_IRQHandler' in irq and 'DMA_ISR_TEIF5' in irq and
   'MOTOR_FAULT_ADC_DMA' in irq and 'motor_hw_emergency_all_off()' in irq,
   'DMA2 Channel5 transfer errors hard-disable both bridges')

# DCLINK was removed from the dual scan, while temperature stays after HT.
dual_dclink=re.findall(r'cfg_adc_channel\(&hadc[12],\s*ADC_CHANNEL_12[^;]+;',hw)
ok(len(dual_dclink)==0, 'ADC1/ADC2 no longer waste slow ranks on DCLINK')
ok('ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_239CYCLES_5' in hw,
   'Part-1 MCU/board temperature remains safely after the current HT boundary')

# Timing proof at 64MHz / 6 ADC clock.
adc_clk=64_000_000/6
adc3_cycles=28.5+12.5
current_ht_cycles=(1.5+12.5)+(7.5+12.5)+(7.5+12.5)
adc3_us=adc3_cycles/adc_clk*1e6
ht_us=current_ht_cycles/adc_clk*1e6
margin_us=ht_us-adc3_us
ok(adc3_cycles < current_ht_cycles and margin_us > 1.0,
   f'ADC3 DCLINK conversion nominally completes before current HT ({adc3_us:.3f} us vs {ht_us:.3f} us, margin {margin_us:.3f} us)')

# Freshness guard. CNDTR should alternate 1/2 in a 2-item circular DMA.
ok('FOC_VBUS_DMA_STALE_FAULT_SAMPLES 3U' in conf and
   's_vbus_dma_prev_cndtr' in foc and 's_vbus_dma_stale_count' in foc,
   'Vbus DMA freshness is supervised across PWM frames')
ok('motor_hw_vbus_raw_from_isr(&vbus_dma_cndtr)' in foc and
   's_vbus_dma_stale_count >= FOC_VBUS_DMA_STALE_FAULT_SAMPLES' in foc and
   foc.count('MOTOR_FAULT_ADC_DMA') >= 2,
   'three consecutive stale ADC3 DMA frames force the existing ADC/DMA fault path')
ok('const uint16_t raw = (rem == 1U) ? g_adc3_vbus_dma[0] : g_adc3_vbus_dma[1];' in hwh,
   'latest ADC3 buffer slot is selected from DMA2 CNDTR without enabling a TC ISR')

# No accidental DMA2 peripheral collision inside this reduced firmware.
all_src='\n'.join(p.read_text(errors='ignore') for p in (ROOT/'src').rglob('*.c'))
claims=re.findall(r'\.Instance\s*=\s*(DMA2_Channel[1-5])',all_src)
ok(claims == ['DMA2_Channel5'], 'DMA2 Channel5 is the only DMA2 channel claimed by a peripheral handle')

# Debug visibility is appended, not standard ABI mutation.
ok('p[i++] = 16U; /* calibration diagnostic revision */' in cmd and
   'foc_vbus_dma_stale_events()' in cmd and 'DMA2_Channel5->CNDTR' in cmd,
   'custom calibration diagnostics expose ADC3/DMA2 Vbus health as revision 16')
ok('cal_diag_revision",0) >= 16' in dbg and 'ADC3 DCLINK PATH' in dbg,
   'host debug parser understands revision-16 ADC3/DMA2 diagnostics')

print('ALL BATCH 9 PART-2 ADC3/VBUS REGRESSIONS: PASS')
