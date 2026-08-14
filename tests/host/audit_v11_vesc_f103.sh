#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
fail(){ echo "audit_v11_vesc_f103: FAIL: $*" >&2; exit 1; }
need(){ grep -qE "$1" "$2" || fail "missing '$1' in $2"; }
absent(){ if grep -qE "$1" "$2"; then fail "forbidden '$1' in $2"; fi; }

# 1. Proven V8 transport is immutable.
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256 >/dev/null || fail "V8 UART/packet transport hash changed"

# 2. 64-MHz hoverboard clock and 16-kHz center-aligned PWM.
need '#define CPU_CLOCK_HZ[[:space:]]+64000000UL' src/app_config.h
need '#define PWM_FREQUENCY_HZ[[:space:]]+16000UL' src/app_config.h
need 'RCC_PLLSOURCE_HSI_DIV2' src/main.c
need 'RCC_PLL_MUL16' src/main.c
need 'TIM_COUNTERMODE_CENTERALIGNED1' src/motor_hw.c
need 'PWM_TIMER_ARR' src/motor_hw.c

# 3. Current VESC dual-motor timer topology: TIM1 master -> TIM8 -> TIM2 reset/CC2.
need 'TIM_TRGO_ENABLE' src/motor_hw.c
need 'TIM_SMCR_MSM' src/motor_hw.c
need 'TIM_TS_ITR0.*TIM_SLAVEMODE_TRIGGER|TIM_TS_ITR0' src/motor_hw.c
need 'TIM_TRGO_UPDATE' src/motor_hw.c
need 'TIM_TS_ITR1.*TIM_SLAVEMODE_RESET|TIM_TS_ITR1' src/motor_hw.c
need 'TIM2->CCR2 = VESC_CURRENT_SAMP_OFFSET_TICKS / 2U' src/motor_hw.c
need 'TIM_IT_CC2' src/motor_hw.c
need 'TIM1->EGR = TIM_EGR_COMG' src/stm32f1xx_it.c
need 'TIM8->EGR = TIM_EGR_COMG' src/stm32f1xx_it.c

# 4. F103 ADC1 master + ADC2 slave, dual regular simultaneous, TIM2_CC2, DMA1_CH1 circular.
need 'ADC_EXTERNALTRIGCONV_T2_CC2' src/motor_hw.c
need 'ADC_DUALMODE_REGSIMULT' src/motor_hw.c
need 'hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START' src/motor_hw.c
need 'NbrOfConversion = 6' src/motor_hw.c
need 'HAL_ADC_Start\(&hadc2\)' src/motor_hw.c
need 'HAL_ADCEx_MultiModeStart_DMA\(&hadc1.*6U' src/motor_hw.c
need 'DMA1_Channel1' src/motor_hw.c
need 'DMA_CIRCULAR' src/motor_hw.c
need 'DMA_PRIORITY_VERY_HIGH' src/motor_hw.c
need 'DMA_IT_HT' src/motor_hw.c
need 'DMA_IT_TC' src/motor_hw.c
# First three paired ranks must be current-only so HT is safe.
need 'ADC_CHANNEL_0,.*ADC_REGULAR_RANK_1' src/motor_hw.c
need 'ADC_CHANNEL_13,.*ADC_REGULAR_RANK_1' src/motor_hw.c
need 'ADC_CHANNEL_14,.*ADC_REGULAR_RANK_2' src/motor_hw.c
need 'ADC_CHANNEL_15,.*ADC_REGULAR_RANK_2' src/motor_hw.c
need 'ADC_CHANNEL_10,.*ADC_REGULAR_RANK_3' src/motor_hw.c
need 'ADC_CHANNEL_11,.*ADC_REGULAR_RANK_3' src/motor_hw.c
need 'DMA_ISR_HTIF1' src/stm32f1xx_it.c
need 'foc_adc_dma_isr\(g_adc_dual_dma\)' src/stm32f1xx_it.c

# 5. Hard ISRs must stay kernel-free / transport-free.
awk '/void DMA1_Channel1_IRQHandler/{p=1} p{print} /^}/{if(p){exit}}' src/stm32f1xx_it.c > /tmp/v11_dma1_isr.txt
absent 'osDelay|osThread|osMessage|vesc_comm_send|HAL_UART|printf|malloc|FLASH' /tmp/v11_dma1_isr.txt
awk '/void TIM2_IRQHandler/{p=1} p{print} /^}/{if(p){exit}}' src/stm32f1xx_it.c > /tmp/v11_tim2_isr.txt
absent 'osDelay|osThread|osMessage|vesc_comm_send|HAL_UART|printf|malloc|FLASH' /tmp/v11_tim2_isr.txt

# 6. VESC V0/V7 dual motor alternation: exactly one full motor per ADC event.
need 'sample_left = \(TIM1->CR1 & TIM_CR1_DIR\) == 0U' src/foc_control.c
need 'foc_one_motor_isr\(active,l_u,l_v,l_dc' src/foc_control.c
need 'foc_one_motor_isr\(active,r_u,r_v,r_dc' src/foc_control.c
need 'FOC_ISR_SLOT_CYCLES' src/foc_control.c
absent 'shed_next|quick_guard' src/foc_control.c

# 7. Forced phase has priority over Hall/AB feedback for sensor detection.
python3 - <<'PY'
from pathlib import Path
s=Path('src/foc_control.c').read_text()
a=s.index('uint16_t motor_sensor_electrical_phase_u16')
b=s.index('if (m->sensor_mode == SENSOR_MODE_ENCODER)', a)
c=s.index('if (m->detect_force_angle)', a)
assert c < b, 'forced detect angle must be checked before sensor feedback'
PY
need 'm->detect_force_angle=true' src/sensor_detect.c
need 'motor_set_foc_targets\(m,d->drive_current_a,0.0f\)' src/sensor_detect.c

# 8. VESC6 config sizes/signatures and safe early GET_MCCONF path.
need 'VESC6_MCCONF_WIRE_SIZE[[:space:]]+481U' src/vesc_config.h
need 'VESC6_APPCONF_WIRE_SIZE[[:space:]]+493U' src/vesc_config.h
need 'VESC6_MCCONF_SIGNATURE' src/vesc_config.c
need 'VESC6_APPCONF_SIGNATURE' src/vesc_config.c
need 'runtime_mc_ready' src/vesc_config.c
need 'COMM_GET_MCCONF' src/vesc_comm.c
need 'COMM_GET_APPCONF' src/vesc_comm.c

# 9. Standard control command scalings and local/virtual route retained.
need 'COMM_SET_DUTY' src/vesc_comm.c
need '100000.0f' src/vesc_comm.c
need 'COMM_SET_CURRENT' src/vesc_comm.c
need '1000.0f' src/vesc_comm.c
need 'COMM_SET_RPM' src/vesc_comm.c
need 'COMM_SET_POS' src/vesc_comm.c
need 'COMM_FORWARD_CAN' src/vesc_comm.c
need 'VESC_VIRTUAL_CAN_RIGHT_ID' src/vesc_comm.c

# 10. Full Detect-All must not fabricate success before physical R/L/flux routines exist.
need 'COMM_DETECT_APPLY_ALL_FOC' src/vesc_comm.c
need 'result = -10' src/vesc_comm.c
need 'full VESC Detect All requires validated R/L/flux measurement' src/vesc_comm.c

# 11. No physical CAN stack was reintroduced.
if grep -R -nE 'HAL_CAN_|CAN1->|CAN2->|comm_can_' src --include='*.c' --include='*.h' >/tmp/v11_can_hits.txt; then
    fail "physical CAN symbols reintroduced: $(head -3 /tmp/v11_can_hits.txt | tr '\n' ' ')"
fi

# 12. PB2 status should not be driven by UART packet traffic anymore.
absent 'recent_packet|packet.*double.flash|communication.*double.flash' src/motor_tasks.c

echo 'audit_v11_vesc_f103: PASS'
echo '  V8 UART DMA transport frozen'
echo '  64MHz + TIM1/TIM8/TIM2 VESC timing topology present'
echo '  ADC1/ADC2 F103 multimode + SW-start slave/pre-enable + DMA1_CH1 HT present'
echo '  FOC/ADC/TIM2 ISRs are kernel-free'
echo '  V0/V7 one-motor-per-event + forced-angle detect present'
echo '  VESC6 config/control/virtual-right checks present'
