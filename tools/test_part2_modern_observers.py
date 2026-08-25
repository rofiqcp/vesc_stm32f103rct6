import re
#!/usr/bin/env python3
from pathlib import Path
import re, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
def t(p): return (ROOT/p).read_text(errors='replace')
def ok(c,m):
    if not c:
        print('FAIL:',m); sys.exit(1)
    print('PASS:',m)

dt=t('src/datatypes.h'); fm=t('src/motor/foc_math.c'); cfg=t('src/confgenerator.c')
mc=t('src/motor/mc_interface.c'); foc=t('src/motor/mcpwm_foc.c')
timeout=t('src/timeout.c'); timeh=t('src/timeout.h'); term=t('src/terminal.c')
cmd=t('src/comm/commands.c'); hw=t('src/hwconf/hw.c'); dbg=t('tools/debug.py')

# All non-HFI VESC observer families are executable, not enum-only placeholders.
for name in ['FOC_OBSERVER_ORTEGA_ORIGINAL','FOC_OBSERVER_MXLEMMING','FOC_OBSERVER_ORTEGA_LAMBDA_COMP',
             'FOC_OBSERVER_MXLEMMING_LAMBDA_COMP','FOC_OBSERVER_MXV','FOC_OBSERVER_MXV_LAMBDA_COMP',
             'FOC_OBSERVER_MXV_LAMBDA_COMP_LIN']:
    ok(name in dt and ('case '+name) in fm, name+' has a fixed-point execution branch')
ok('observer_lambda_est_q30' in dt and 'observer_i_alpha_last_q15' in dt and 'observer_i_beta_last_q15' in dt,
   'observer runtime carries adaptive-lambda and previous-current state')
ok('David Molony' in fm and 'MESC' in fm, 'MXLEMMING attribution is preserved')
body=re.search(r'void foc_observer_update_fixed\(.*?\n\}',fm,re.S)
ok(body is not None,'fixed observer update body found')
for tok in ['sqrtf(','sinf(','cosf(','atan2f(','powf(','double ']:
    ok(tok not in body.group(0), 'hard observer update avoids '+tok.strip())
ok('lambda_adapt_nonlinear_q30' in fm and 'lambda_clamp_q30' in fm,
   'lambda-comp observers use bounded adaptive flux estimate')

# Observer and sat-comp choices are persistent through the existing VESC6 wire ABI.
ok('i == VESC6_MC_OFF_FOC_OBSERVER_TYPE' in cfg, 'observer type byte is runtime-mutable/persistent')
ok('observer_type > FOC_OBSERVER_MXV_LAMBDA_COMP_LIN' in cfg, 'wire validator accepts all implemented observer types')
ok('sat_comp_mode > SAT_COMP_LAMBDA_AND_FACTOR' in cfg, 'all implemented saturation compensation modes validate')
ok('c->foc_speed_source=(FOC_SPEED_SRC)w[VESC6_MC_OFF_FOC_SPEED_SOURCE]' in cfg and
   'w[VESC6_MC_OFF_FOC_SPEED_SOURCE]=(uint8_t)c->foc_speed_source' in cfg,
   'canonical VESC6 FOC speed-source byte 314 now round-trips and persists')

# Do not fake unsupported dynamic sampling on this shared ADC topology.
ok('sample_v0_v7 || sample_high_current' in cfg and 'return false' in cfg,
   'unsupported V0/V7/high-current sampling requests are explicitly rejected')
ok('sampling_window_clamp_count' in dt and 'sampling_margin_min_q15' in dt,
   'safe fixed sampling window is instrumented at runtime')
ok('PWM_MIN_DUTY_Q15' in foc and 'PWM_MAX_DUTY_Q15' in foc and 'sampling_window_clamp_count' in foc,
   'FOC ISR records approach to the hardware-qualified 10-90% sampling window')

# Typed and wire configuration paths agree on Part-1 sensor policy.
ok('const bool sensorless = c->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;' in mc and
   'if ((!sensorless && c->foc_sensor_mode != FOC_SENSOR_MODE_HALL) || encoder_ab' in mc,
   'typed config preserves sensorless and still rejects RIGHT physical encoder')
ok('m->foc_sensor_mode = encoder_ab ? FOC_SENSOR_MODE_ENCODER_AB : c->foc_sensor_mode;' in mc and
   'm->foc_sensor_mode = c->foc_sensor_mode;' in mc,
   'typed apply no longer silently forces sensorless to Hall')

# Watchdog diagnostics retain conservative IWDG timing but make missed owners observable.
ok('timeout_watchdog_unhealthy_mask' in timeh and 'timeout_watchdog_miss_count' in timeh,
   'watchdog exposes unhealthy-owner mask and miss counters')
ok('s_unhealthy_mask' in timeout and 's_hb_miss' in timeout,
   'watchdog records per-owner missed heartbeat windows')
ok('watchdog_unhealthy_mask' in dbg and 'sample_margin_left_q15' in dbg,
   'debug tool decodes Part-2 watchdog and sampling diagnostics')
ok(any(f'p[i++] = {n}U;' in cmd for n in range(10, 20)), 'custom diagnostic protocol is revision 10 or later')
ok(re.search(r'vesc-f103-hoverboard-v(?:2[8-9]|[3-9][0-9])-', cmd) is not None, 'Part-2 or later firmware identity is explicit')

# MXV 4..6 are later-than-VESC6 backend extensions. They remain selectable
# safely from VESC Tool's terminal while the advertised MCCONF ABI stays 6.00.
ok('observer [set N|sat N|speed N]' in term and 'set 0..6 | sat 0..3 | speed 0..1' in term,
   'terminal exposes safe observer/saturation/speed-source tuning commands')
ok('m->pwm_enabled || m->detect.busy' in term and 'conf_general_store_mc_configuration' in term,
   'terminal tuning requires a stopped motor and uses canonical validated transactional config path')
ok('value >= 0 && value <= 6' in term and 'value >= 0 && value <= 3' in term and
   'value >= 0 && value <= 1' in term,
   'terminal observer/saturation/speed-source ranges are bounded')

# Safety/hardware invariants remain untouched.
ok('oc.OCPolarity = TIM_OCPOLARITY_HIGH;' in hw, 'high-side PWM remains active HIGH')
ok('oc.OCNPolarity = TIM_OCNPOLARITY_LOW;' in hw, 'low-side complementary PWM remains active LOW')
combined='\n'.join([dt,fm,cfg,mc,foc,timeout,term,cmd,hw]).lower()
for token in ['#include "comm_can.h"','#include "imu/','#include "bms/','#include "bm_if','#include "nrf',
              '#include "ledpwm','#include "comm_usb','#include "qml','#include "lispif','#include "lzo']:
    ok(token not in combined,'excluded subsystem not introduced: '+token)

# Strict host syntax/warnings for all portable Part-2 touched units.
incs=['-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm']
base=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror']
for unit,extra in [('src/motor/foc_math.c',[]),('src/motor/mc_interface.c',[]),
                   ('src/motor/mcpwm_foc.c',['-DDMA1_Channel1=DMA2_Channel5']),('src/confgenerator.c',[]),
                   ('src/terminal.c',[])]:
    cp=subprocess.run(base+incs+extra+['-fsyntax-only',unit],cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
    ok(True,unit+' passes strict host syntax/warning check')
cp=subprocess.run([sys.executable,'tools/debug.py','--self-test'],cwd=ROOT,text=True,capture_output=True)
if cp.returncode:
    print(cp.stdout,cp.stderr); sys.exit(cp.returncode)
ok(('COMM_DIAG-v10' in cp.stdout) or ('COMM_DIAG-v13' in cp.stdout) or ('COMM_DIAG-v14' in cp.stdout),'debug.py self-test validates COMM_DIAG revision 10 or later')
print('ALL PART-2 MODERN-OBSERVER/HARDENING REGRESSIONS: PASS')
