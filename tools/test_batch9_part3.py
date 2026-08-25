#!/usr/bin/env python3
from pathlib import Path
import hashlib, subprocess, tempfile, sys
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

hw=read('src/hwconf/hw.c'); hwh=read('src/hwconf/hw_hoverboard.h')
irq=read('src/stm32f1xx_it.c'); foc=read('src/motor/mcpwm_foc.c')
dt=read('src/datatypes.h'); mc=read('src/motor/mc_interface.c')
mt=read('src/motor_tasks.c'); cg=read('src/confgenerator.c')
defs=read('src/motor/mcconf_default.h'); term=read('src/terminal.c')
pio=read('platformio.ini')

ok('-DSTM32F103xE' in pio and 'genericSTM32F103RC' in pio,
   'target remains STM32F103RCT6/high-density')
ok(all(x in pio for x in ['platform = ststm32','board = genericSTM32F103RC',
                           'framework = stm32cube','board_build.ldscript = src/stm32f103rc_flash.ld']),
   'PlatformIO keeps the STM32F103RC/Cube target and linker script')
ok('board_build.f_cpu' not in pio,
   'firmware-owned 64-MHz HSI clock is not overridden by PlatformIO')

# PVD: internal, default enabled, highest threshold, IRQ-latched and MOE-gated.
ok('HOVERBOARD_PVD_ENABLE                 1' in hwh and 'HOVERBOARD_PVD_PLS_BITS               (7U << 5)' in hwh,
   'internal STM32 PVD is enabled by default at the highest threshold band')
ok('PWR->CR = (PWR->CR & ~(7UL << 5)) | HOVERBOARD_PVD_PLS_BITS | (1UL << 4)' in hw and
   'EXTI->IMR |= (1UL << 16)' in hw and 'HAL_NVIC_EnableIRQ(PVD_IRQn)' in hw,
   'PVD is routed through EXTI16/NVIC without RTOS work in the IRQ')
ok('void PVD_IRQHandler(void)' in irq and 'motor_hw_pvd_irq_handler()' in irq,
   'PVD vector delegates to the hard power-stage shutdown handler')
ok('POWERSTAGE_FAULT_PVD' in hw and 'MOTOR_FAULT_MCU_UNDER_VOLTAGE' in hw and
   'TIM1->BDTR &= ~TIM_BDTR_MOE' in hw and 'TIM8->BDTR &= ~TIM_BDTR_MOE' in hw,
   'PVD low supply clears both MOEs and reset-latches an MCU-undervoltage fault')
ok('s_powerstage_fault_flags != 0U' in hw and 'motor_hw_powerstage_fault_latched()' in mc,
   'power-stage latch blocks PWM re-enable and cannot be hidden by clear-fault')

# BKIN backend is real but stock-board safe default remains disabled.
ok('HOVERBOARD_TIM1_BREAK_ENABLE          0' in hwh and 'HOVERBOARD_TIM8_BREAK_ENABLE          0' in hwh,
   'TIM1/TIM8 external BKIN backends remain disabled by default pending PCB validation')
ok('HOVERBOARD_TIM1_BKIN_PIN              GPIO_PIN_12' in hwh and
   'HOVERBOARD_TIM8_BKIN_PIN              GPIO_PIN_6' in hwh,
   'optional BKIN pins use STM32F103 default TIM1 PB12 and TIM8 PA6 mappings')
ok('bd.BreakState = break_enable ? TIM_BREAK_ENABLE : TIM_BREAK_DISABLE' in hw and
   'bd.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE' in hw,
   'advanced timers implement conditional hardware Break with no automatic re-enable')
ok('TIM1_BRK_IRQHandler' in irq and 'TIM8_BRK_IRQHandler' in irq and
   'motor_hw_break_irq_handler(TIM1)' in irq and 'motor_hw_break_irq_handler(TIM8)' in irq,
   'TIM1/TIM8 Break IRQs latch/report the asynchronous hardware shutdown')
ok('MOTOR_FAULT_BREAK' in dt and 'FAULT_CODE_BRK' in mc and 'FAULT_CODE_MCU_UNDER_VOLTAGE' in mc,
   'new local power-stage faults map explicitly to canonical VESC fault codes')
ok('MOTOR_FAULT_BREAK, MOTOR_FAULT_MCU_UNDER_VOLTAGE' in mt,
   'status/fault diagnostics prioritize the new reset-latched power-stage faults')

# VESC-style low-side zero-vector brake backend, safely runtime-only for VESC6.
ok('MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT false' in defs,
   'foc_short_ls_on_zero_duty defaults OFF on unvalidated hoverboard gate hardware')
ok('bool foc_short_ls_on_zero_duty;' in dt and 'volatile bool full_brake_active;' in dt,
   'typed configuration and runtime track zero-vector low-side brake state')
ok('const bool exact_zero_vector = (du == dv) && (dv == dw);' in foc and
   'motor_hw_set_low_side_brake(m, true)' in foc,
   'hard FOC selects static low-side brake only for an exact zero vector')
ok('TIM_OCMODE_FORCED_INACTIVE_LOCAL' in hw and 'TIM_EGR_COMG_LOCAL' in hw and
   'motor_hw_set_oc_mode_triplet' in hw,
   'low-side brake uses preload/COM-safe forced-inactive timer modes')
ok('motor_hw_restore_foc_outputs' in hw and 'TIM_OCMODE_PWM1_LOCAL' in hw and
   'm->full_brake_active = false' in hw,
   'leaving full brake restores all channels to PWM1 coherently')
ok('if(conf->foc_short_ls_on_zero_duty!=MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT)return -1;' in cg,
   'VESC6 serializer refuses to fake-persist the newer short-ls field')
ok('c->foc_short_ls_on_zero_duty=MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT' in cg,
   'VESC6 deserialization gives the unsupported wire field an explicit safe default')
ok('if(m->full_brake_active)return MC_STATE_FULL_BRAKE' in mc,
   'public motor state reports VESC-style FULL_BRAKE when static low-side braking is active')

# Batch-2 acquisition contract must still be intact.
ok('hadc3.Instance = ADC3' in hw and 'hdma_adc3.Instance = DMA2_Channel5' in hw and
   '__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT)' in hw,
   'ADC3/DMA2 Vbus path and DMA1-HT FOC trigger remain intact')
ok('powerstage' in term and 'motor_hw_powerstage_fault_flags()' in term,
   'terminal exposes non-destructive power-stage latch/PVD/BKIN/short-ls diagnostics')

# Strict host compile of changed portable units.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for unit,extra in [('src/motor/mc_interface.c',[]),('src/motor/mcpwm_foc.c',['-DDMA1_Channel1=DMA2_Channel5']),('src/terminal.c',[])]:
    cp=subprocess.run(base+extra+incs+['-fsyntax-only',unit],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True,f'{unit} passes strict host syntax/warning check')

with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'b9p3_roundtrip'
    cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections',*incs,
         'tools/test_batch9_part3_roundtrip.c','src/confgenerator.c','src/util/buffer.c',
         '-Wl,--gc-sections','-lm','-o',str(exe)]
    cp=subprocess.run(cmd,cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    rp=subprocess.run([str(exe)],cwd=ROOT,text=True,capture_output=True)
    if rp.returncode:
        print(rp.stdout,rp.stderr); sys.exit(rp.returncode)
    ok('ALL BATCH 9 PART-3 CONFIG ROUNDTRIPS: PASS' in rp.stdout,
       'short-ls runtime field defaults safely and cannot be fake-persisted into VESC6')

print('ALL BATCH 9 PART-3 POWER-STAGE REGRESSIONS: PASS')
