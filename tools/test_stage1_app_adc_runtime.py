#!/usr/bin/env python3
from pathlib import Path
import subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
cmd=[
    'gcc','-std=c11','-O2','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror',
    '-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm',
    'tools/test_stage1_app_adc_runtime.c','src/applications/app_adc.c','src/applications/app_command.c','-lm',
    '-o','/tmp/vesc_stage1_app_adc_runtime'
]
cp=subprocess.run(cmd,cwd=ROOT,text=True,capture_output=True)
if cp.returncode:
    print(cp.stdout); print(cp.stderr); sys.exit(cp.returncode)
cp=subprocess.run(['/tmp/vesc_stage1_app_adc_runtime'],cwd=ROOT,text=True,capture_output=True)
print(cp.stdout,end='')
if cp.returncode:
    print(cp.stderr); sys.exit(cp.returncode)
print('PASS: real app_adc.c + app_command.c execute safe-start, brake, fault and source arbitration paths')
