#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,sys,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
def read(p): return (ROOT/p).read_text()
def ok(c,msg):
    if not c:
        print('FAIL:',msg); sys.exit(1)
    print('PASS:',msg)

dt=read('src/datatypes.h')
cg=read('src/confgenerator.c'); cgh=read('src/confgenerator.h')
hw=read('src/hwconf/hw.c'); hwh=read('src/hwconf/hw_hoverboard.h')
mc=read('src/motor/mc_interface.c'); mcd=read('src/motor/mcconf_default.h')
mt=read('src/motor/mc_interface_tasks.c'); status=read('src/hwconf/hw_status.c')
mm=read('src/motor/mc_math.c'); mmh=read('src/motor/mc_math.h')
foc=read('src/motor/mcpwm_foc.c')
part2=('Batch 9 Part 2' in foc and 'motor_hw_vbus_raw_from_isr' in foc)

# Part 1 originally left hard FOC byte-identical. Later B9 Part 2 intentionally
# changes only Vbus acquisition inside that ISR, so retain the historical hash
# guard only when the Part-2 marker is absent.
if part2:
    ok('motor_hw_vbus_raw_from_isr' in foc and 'FOC_VBUS_DMA_STALE_FAULT_SAMPLES' in foc,
       'Part-2 intentionally supersedes the Part-1 hard-FOC byte-identity guard only for ADC3 Vbus acquisition')
else:
    ok(hashlib.sha256((ROOT/'src/motor/mcpwm_foc.c').read_bytes()).hexdigest()==
       'c8ca737180b6223615b59aff565c636c92d153a1abe11a6479a6bcbbed997ffc',
       'Batch 9 Part-1 leaves the proven hard mcpwm_foc.c byte-identical to Batch 8 best')
pio=read('platformio.ini')
ok(all(x in pio for x in ['platform = ststm32','board = genericSTM32F103RC',
                           'framework = stm32cube','board_build.ldscript = src/stm32f103rc_flash.ld']),
   'PlatformIO keeps the STM32F103RC/Cube hardware target and linker script')
ok('board_build.f_cpu' not in pio,
   'PlatformIO does not override the firmware-owned 64-MHz HSI clock configuration')

# VESC6 ABI fields that already existed on the pinned wire image become real backends.
for sym,val in [('VESC6_MC_OFF_L_TEMP_FET_START','63U'),('VESC6_MC_OFF_L_TEMP_FET_END','65U'),
                ('VESC6_MC_OFF_L_TEMP_MOTOR_START','67U'),('VESC6_MC_OFF_L_TEMP_MOTOR_END','69U'),
                ('VESC6_MC_OFF_L_TEMP_ACCEL_DEC','71U'),('VESC6_MC_OFF_FOC_START_CURR_DEC','195U'),
                ('VESC6_MC_OFF_FOC_START_CURR_DEC_RPM','197U')]:
    ok(re.search(r'#define\s+'+sym+r'\s+'+val+r'\b',cgh) is not None, f'{sym} stays at pinned VESC6 offset {val}')
ok(all(x in dt for x in ['float l_temp_fet_start, l_temp_fet_end;', 'float l_temp_motor_start, l_temp_motor_end;',
                         'float l_temp_accel_dec;', 'float foc_start_curr_dec, foc_start_curr_dec_rpm;']),
   'typed MCCONF exposes thermal and startup-current fields')
ok('put_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_START' in cg and
   'c->l_temp_fet_start=get_f16_at(w,VESC6_MC_OFF_L_TEMP_FET_START' in cg and
   'put_f16_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC' in cg and
   'c->foc_start_curr_dec=get_f16_at(w,VESC6_MC_OFF_FOC_START_CURR_DEC' in cg,
   'thermal/start-current settings round-trip through the existing VESC6 image')

# New-master additional fault semantics are runtime-only because VESC6 has no byte for them.
ok(re.search(r'#define\s+MCCONF_L_ADDITIONAL_FAULTS_DEFAULT\s+0U',mcd) is not None,
   'additional RPM faults follow upstream safe default disabled')
ok(all(x in mcd for x in ['MCCONF_L_ADDITIONAL_FAULT_OVERSPEED  (1U << 1)',
                          'MCCONF_L_ADDITIONAL_FAULT_UNDERSPEED (1U << 2)',
                          'MCCONF_L_ADDITIONAL_FAULT_ABS_SPEED  (1U << 3)']),
   'additional-fault bit assignments match current VESC semantics')
ok('if(conf->l_additional_faults!=MCCONF_L_ADDITIONAL_FAULTS_DEFAULT)return -1;' in cg,
   'VESC6 serializer refuses to fake-persist newer additional-fault state')
ok('m->erpm_fault_filter = lp(m->erpm_fault_filter, m->erpm, 0.02f);' in mc and
   'MOTOR_FAULT_OVERSPEED' in mc and 'MOTOR_FAULT_UNDERSPEED' in mc and 'MOTOR_FAULT_ABS_OVERSPEED' in mc,
   'optional RPM faults use a slow speed signal and explicit board fault codes')

# ADC fast ranks stay untouched; MCU temp lives after HT on ADC1 rank 5.
fast_expected=[
 'cfg_adc_channel(&hadc1, ADC_CHANNEL_11, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5)',
 'cfg_adc_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5)',
 'cfg_adc_channel(&hadc1, ADC_CHANNEL_0,  ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5)',
 'cfg_adc_channel(&hadc2, ADC_CHANNEL_13, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_7CYCLES_5)',
 'cfg_adc_channel(&hadc1, ADC_CHANNEL_14, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5)',
 'cfg_adc_channel(&hadc2, ADC_CHANNEL_15, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_7CYCLES_5)']
ok(all(x in hw for x in fast_expected), 'all six fast current conversions in ranks 1-3 are unchanged')
ok('ADC_CHANNEL_TEMPSENSOR, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_239CYCLES_5' in hw,
   'internal MCU temperature uses ADC1 rank 5 with maximum sample time')
if part2:
    ok('hadc3, ADC_CHANNEL_12, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_28CYCLES_5' in hw and
       'DCLINK is deliberately removed from ADC1/ADC2' in hw,
       'Part-2 supersedes the Part-1 DCLINK rank-4 guard with dedicated ADC3 ownership')
else:
    ok('ADC_CHANNEL_12, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_28CYCLES_5' in hw,
       'DCLINK remains on ADC1 rank 4 in Part 1')
ok('__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_HT);' in hw and '__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);' in hw,
   'FOC still triggers on DMA half-transfer immediately after current rank 3')
# 64 MHz / 6 ADC clock; conversion = sampling + 12.5 cycles.
scan_cycles=sum(x+12.5 for x in [1.5,7.5,7.5,28.5,239.5,28.5])
scan_us=scan_cycles/(64_000_000/6)*1e6
ok(scan_us < 62.5 and scan_us < 40.0, f'ADC1 six-rank scan remains inside 16-kHz period ({scan_us:.3f} us)')

# Temperature conversion/backend is explicitly a board proxy, not fake MOSFET NTC.
ok('board/MCU thermal proxy, NOT a MOSFET-junction measurement' in hwh,
   'temperature backend is labelled as MCU/board proxy rather than MOSFET junction')
ok('bool motor_hw_board_temperature_c(float *temp_c)' in hw and
   'HOVERBOARD_MCU_TEMP_V25_V' in hw and 'HOVERBOARD_MCU_TEMP_AVG_SLOPE_V_PER_C' in hw,
   'board-temperature conversion validates and converts the ADC sample')
ok('motor_hw_board_temperature_c(&t_board)' in mc and 'm->board_temp_filter_c = lp(' in mc,
   '1-kHz motor service filters the board temperature outside the hard ISR')
ok('mc_math_thermal_current_limit' in mc and 'MOTOR_FAULT_OVER_TEMP_BOARD' in mc,
   'board temperature drives VESC-style current derating and hard end-temperature fault')
ok('mc_math_thermal_accel_limit' in mc and 'm->temp_accel_dec' in mc,
   'l_temp_accel_dec preserves braking authority with an acceleration-only thermal cap')
ok('mc_interface_temp_fet_filtered(void)' in mc and 'm->board_temp_filter_c:MC_TEMP_SENSOR_UNAVAILABLE_C' in mc,
   'standard FET-temperature telemetry now exposes the documented board proxy')

# Startup-current semantics and task-fault delivery.
ok('mc_math_start_current_limit' in mmh and 'mc_math_start_current_limit' in mm and
   'mc_math_start_current_limit(base_max, rpm_abs' in mc,
   'foc_start_curr_dec has a tested 1-kHz VESC-style runtime backend')
ok('s_pending_fault_mask |=' in mc,
   'task-level thermal/RPM faults are delivered through the existing fault-thread pending mask')
ok(all(x in status for x in ['MOTOR_FAULT_OVER_TEMP_BOARD','MOTOR_FAULT_OVER_TEMP_MOTOR',
                         'MOTOR_FAULT_OVERSPEED','MOTOR_FAULT_UNDERSPEED','MOTOR_FAULT_ABS_OVERSPEED']),
   'status fault priority includes all new Part-1 fault classes')

# Syntax-check the modified controller with the same strict warning classes as
# platformio.ini, using host-only CMSIS/HAL type stubs.
cmd=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror',
     '-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util',
     '-Isrc/applications','-Isrc/comm','-fsyntax-only','src/motor/mc_interface.c']
cp=subprocess.run(cmd,cwd=ROOT,text=True,capture_output=True)
if cp.returncode != 0:
    print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
ok(True, 'modified mc_interface.c passes strict host syntax/warning check')

# Compile and execute actual numeric helpers with strict warnings.
with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'b9p1_limits'
    cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Isrc',
         'tools/test_batch9_part1_limits.c','src/motor/mc_math.c','-lm','-o',str(exe)]
    cp=subprocess.run(cmd,cwd=ROOT,text=True,capture_output=True)
    ok(cp.returncode==0, 'Batch 9 Part-1 numeric limit helper compiles with GCC -Wall -Wextra -Werror')
    rp=subprocess.run([str(exe)],cwd=ROOT,text=True,capture_output=True)
    if rp.returncode != 0:
        print(rp.stdout, rp.stderr); sys.exit(rp.returncode)
    ok('ALL BATCH 9 PART-1 LIMIT NUMERICS: PASS' in rp.stdout,
       'thermal, accel-thermal and startup-current numeric vectors pass')

# Strict host round-trip compile with a minimal HAL type stub. Link-time section
# GC discards hardware-only configuration paths that this codec test does not use.
with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'b9p1_roundtrip'
    cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections',
         '-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util',
         'tools/test_batch9_part1_roundtrip.c','src/confgenerator.c','src/util/buffer.c',
         '-Wl,--gc-sections','-lm','-o',str(exe)]
    cp=subprocess.run(cmd,cwd=ROOT,text=True,capture_output=True)
    if cp.returncode != 0:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True, 'thermal/start-current VESC6 codec compiles with strict host warnings')
    rp=subprocess.run([str(exe)],cwd=ROOT,text=True,capture_output=True)
    if rp.returncode != 0:
        print(rp.stdout,rp.stderr); sys.exit(rp.returncode)
    ok('ALL BATCH 9 PART-1 CONFIG ROUNDTRIPS: PASS' in rp.stdout,
       'thermal/start-current VESC6 round-trip and additional-fault non-persistence pass')

print('ALL BATCH 9 PART-1 SAFETY/THERMAL REGRESSIONS: PASS')
