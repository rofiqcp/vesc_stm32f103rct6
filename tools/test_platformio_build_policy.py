#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
text = (root / 'platformio.ini').read_text()

def fail(msg):
    print('FAIL:', msg)
    sys.exit(1)

def ok(msg):
    print('PASS:', msg)

if 'build_src_flags =' not in text:
    fail('build_src_flags missing')
ok('project-only build_src_flags section exists')

before, after = text.split('build_src_flags =', 1)
# No blanket Werror in the global build_flags block.
global_block = before.split('build_flags =', 1)[1] if 'build_flags =' in before else ''
if any(line.strip() == '-Werror' for line in global_block.splitlines()):
    fail('blanket -Werror still leaks into STM32Cube/FreeRTOS framework sources')
ok('blanket -Werror is absent from global framework build flags')

# Project source keeps useful diagnostics and only semantic hard failures.
for flag in ['-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2',
             '-Werror=return-type','-Werror=implicit-function-declaration']:
    if flag not in after:
        fail(f'missing project-source diagnostic flag {flag}')
ok('firmware src/ keeps strict warnings and critical semantic errors')

if any(line.strip() == '-Werror' for line in after.splitlines()):
    fail('blanket project -Werror should remain off for first full ARM build; strict host regressions cover core units')
ok('blanket project -Werror is intentionally disabled for portable vendor/toolchain compatibility')

for required in ['-DSTM32F103xE','-DUSE_HAL_DRIVER','-O3','-fno-math-errno']:
    if required not in global_block:
        fail(f'global functional build flag lost: {required}')
ok('target MCU/framework/optimization flags remain unchanged')

print('ALL PLATFORMIO BUILD-POLICY REGRESSIONS: PASS')
