#!/usr/bin/env python3
from pathlib import Path
import re, subprocess, sys
ROOT = Path(__file__).resolve().parents[1]

def txt(rel): return (ROOT/rel).read_text(errors='replace')
def req(cond, msg):
    if not cond:
        raise AssertionError(msg)

def body(src, name):
    m=re.search(r'\b%s\s*\([^;]*?\)\s*\{'%re.escape(name),src,re.S)
    req(m is not None, f'{name} not found')
    i=m.end(); depth=1
    while i < len(src) and depth:
        if src[i]=='{': depth+=1
        elif src[i]=='}': depth-=1
        i+=1
    return src[m.start():i]

hw=txt('src/hwconf/hw.c'); hwh=txt('src/hwconf/hw.h')
foc=txt('src/motor/mcpwm_foc.c'); foch=txt('src/motor/mcpwm_foc.h')
uart=txt('src/applications/app_uartcomm.c'); uarth=txt('src/applications/app_uartcomm.h')
mt=txt('src/motor/mc_interface_tasks.c'); mth=txt('src/motor/mc_interface.h')
cmd=txt('src/comm/commands.c'); cmdh=txt('src/comm/commands.h')
term=txt('src/terminal.c'); conf=txt('src/confgenerator.c')

# Power-stage contract must remain exactly as the proven hoverboard target.
req('TIM_OCPOLARITY_HIGH' in hw, 'high-side active-high contract missing')
req('TIM_OCNPOLARITY_LOW' in hw, 'low-side active-low contract missing')
req('TIM8' in hw and 'TIM1' in hw, 'dual PWM timers missing')

# Unsupported modern VESC sample modes must remain explicitly rejected until HW-qualified.
req('foc_sample_v0_v7' in conf and 'foc_sample_high_current' in conf,
    'VESC6 sample-mode compatibility fields missing')
req(re.search(r'sample_v0_v7\s*\|\|\s*sample_high_current', conf) is not None,
    'unsafe V0/V7/high-current modes are no longer rejected')

# Runtime sampling contract and boot gate.
for token in ['motor_hw_sampling_contract_flags','motor_hw_sampling_contract_valid',
              'HW_SAMPLING_CONTRACT_TIM1_MODE','HW_SAMPLING_CONTRACT_TIM1_TRGO',
              'HW_SAMPLING_CONTRACT_TIM8_TRGO','HW_SAMPLING_CONTRACT_TIM8_SLAVE',
              'HW_SAMPLING_CONTRACT_ADC_DUALMODE','HW_SAMPLING_CONTRACT_ADC_CHANNELS',
              'HW_SAMPLING_CONTRACT_DMA1_MODE','HW_SAMPLING_CONTRACT_ADC3_MODE',
              'HW_SAMPLING_CONTRACT_DMA2_MODE']:
    req(token in hwh+hw, f'sampling-contract token missing: {token}')
main=txt('src/main.c')
req('motor_hw_sampling_contract_valid()' in main and 'MOTOR_FAULT_ADC_DMA' in main,
    'boot does not gate readiness on sampling contract')
req('DMA_CCR_PL_0' in hw and 'DMA_CCR_PL_1' in hw, 'DMA priority contract not explicit')

# Complete dual-motor ISR timing/jitter instrumentation.
for token in ['foc_isr_total_max_cycles','foc_isr_near_deadline_count',
              'foc_isr_period_min_cycles','foc_isr_period_max_cycles']:
    req(token in foc and token in foch, f'FOC timing API missing: {token}')
req('s_isr_near_deadline_count' in foc and 'FOC_ISR_SLOT_CYCLES' in foc,
    'near-deadline ISR monitor missing')

# UART TX must be true non-blocking backpressure and queue reduced for SRAM.
req(re.search(r'#define\s+VESC_UART_TX_QUEUE_DEPTH\s+4U', uarth) is not None,
    'UART TX queue is not reduced to 4')
wb=body(uart,'app_uartcomm_write_raw_class')
req('osMutexAcquire(s_tx_mutex, 0U)' in wb, 'UART TX mutex is still blocking')
req('osDelay' not in wb and '250U' not in wb, 'UART TX still waits when queue is full')
req('tx_queue_high_water' in uarth+uart and 'tx_queue_busy_drops' in uarth+uart and 'tx_low_priority_drops' in uarth+uart,
    'UART backpressure diagnostics missing')
req('app_uartcomm_write_raw_low_priority' in uart+uarth and 'VESC_UART_TX_QUEUE_DEPTH - 2U' in uart,
    'UART low-priority traffic does not reserve response headroom')
req('vesc_comm_send_payload_low_priority' in cmd and 'vesc_comm_send_sample_buffer' in cmd,
    'sample/periodic telemetry is not classified as low priority')

# Heap and stack high-water monitoring.
req('xPortGetMinimumEverFreeHeapSize' in mt, 'minimum-ever heap monitor missing')
req('uxTaskGetStackHighWaterMark' in mt and ('osThreadGetStackSpace' in cmd or 'uxTaskGetStackHighWaterMark' in cmd),
    'stack high-water monitoring incomplete')
for token in ['mc_interface_resource_stats_t','mc_interface_get_resource_stats',
              'vesc_comm_resource_stats_t','vesc_comm_get_resource_stats']:
    req(token in mth+mt+cmdh+cmd, f'resource API missing: {token}')
req('resources' in term and 'timing' in term, 'resource/timing terminal commands missing')

# Diagnostic transport hardening.
req('uint8_t p[224]' not in body(cmd,'vesc_comm_reply_diag'), 'COMM_DIAG still uses hand-sized stack buffer')
req('payload_begin()' in body(cmd,'vesc_comm_reply_diag') and 'payload_end(i)' in body(cmd,'vesc_comm_reply_diag'),
    'COMM_DIAG does not use shared bounded payload scratch')
req(re.search(r'p\[i\+\+\]\s*=\s*14',cmd) is not None, 'COMM_DIAG revision is not 14')
for token in ['sampling_contract_flags','isr_total_max_cycles','heap_min_ever',
              'tx_queue_high_water','tx_queue_busy_drops','tx_low_priority_drops']:
    req(token in cmd+cmdh, f'diagnostic/resource field missing: {token}')
req('vesc-f103-hoverboard-v33-vesc-layout' in cmd,
    'V33 VESC-layout firmware identity missing')

# Excluded subsystems should not reappear as runtime source dependencies.
for forbidden in ['#include "bms',' #include "imu','#include "nrf','#include "lispif',
                  '#include "lzo','#include "comm_can','#include "ledpwm']:
    req(forbidden.strip() not in '\n'.join(txt(str(p.relative_to(ROOT))) for p in (ROOT/'src').rglob('*.[ch]')),
        f'forbidden runtime dependency found: {forbidden.strip()}')


# Dead compatibility stubs removed: startup/fault melody is owned by StatusIO buzzer.
for dead in ['mcpwm_foc_set_audio_sample_table','mcpwm_foc_get_audio_sample_table',
             'mcpwm_foc_play_audio_samples','mcpwm_foc_tim_sample_int_handler',
             'mc_interface_adc_inj_int_handler']:
    allsrc='\n'.join(txt(str(p.relative_to(ROOT))) for p in (ROOT/'src').rglob('*.[ch]'))
    req(dead not in allsrc, f'dead compatibility API remains: {dead}')

# Debug parser self-test must understand rev14.
dbg=ROOT/'tools/debug.py'
res=subprocess.run([sys.executable,str(dbg),'--self-test'],cwd=ROOT,text=True,capture_output=True)
req(res.returncode==0, 'debug.py self-test failed: '+res.stdout+res.stderr)
req('COMM_DIAG-v14' in res.stdout, 'debug.py does not report COMM_DIAG-v14')

print('[PASS] Stage3 production hardening static/runtime-contract test')
print('[PASS] safe sampling modes retained; no fake V0/V7/high-current support')
print('[PASS] ISR timing/jitter + RTOS resource diagnostics present')
print('[PASS] UART TX non-blocking backpressure and reduced queue verified')
