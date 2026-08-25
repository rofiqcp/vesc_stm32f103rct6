#!/usr/bin/env python3
"""Optional host static-analysis gate for logic-heavy firmware translation units.

Uses Clang Static Analyzer and GCC -fanalyzer when available. Hardware-only
translation units are covered by the target PlatformIO build because lightweight
host stubs intentionally do not model the complete STM32 peripheral layer.
"""
from pathlib import Path
import re, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
UNITS = [
    'src/applications/app_adc.c',
    'src/applications/app_command.c',
    'src/applications/app_uartcomm.c',
    'src/comm/commands.c',
    'src/comm/packet.c',
    'src/confgenerator.c',
    'src/hwconf/hw_status.c',
    'src/motor/mc_interface_sample.c',
    'src/motor/mc_interface_tasks.c',
    'src/motor/mcpwm_foc.c',
    'src/motor/mc_interface.c',
    'src/telemetry.c',
    'src/terminal.c',
]
INCS = [f'-I{ROOT / "tools/compile_stubs"}', f'-I{ROOT / "src"}']
ran = 0

clang = shutil.which('clang')
if clang:
    ran += 1
    for rel in UNITS:
        cmd = [clang, '--analyze', '-std=gnu11', *INCS,
               '-Xanalyzer', '-analyzer-output=text', str(ROOT / rel)]
        r = subprocess.run(cmd, text=True, capture_output=True)
        out = (r.stdout + r.stderr).strip()
        if r.returncode or out:
            sys.stderr.write(f'[FAIL] clang analyzer {rel}\n{out}\n')
            raise SystemExit(1)
    print(f'[PASS] clang static analyzer: {len(UNITS)} critical units')

gcc = shutil.which('gcc')
if gcc:
    ran += 1
    for rel in UNITS:
        cmd = [gcc, '-std=gnu11', '-fsyntax-only', '-fanalyzer', '-Wall', '-Wextra',
               *INCS, str(ROOT / rel)]
        r = subprocess.run(cmd, text=True, capture_output=True)
        out = (r.stdout + r.stderr).strip()
        if r.returncode or out:
            sys.stderr.write(f'[FAIL] gcc -fanalyzer {rel}\n{out}\n')
            raise SystemExit(1)
    print(f'[PASS] gcc -fanalyzer: {len(UNITS)} critical units')

# Source hygiene relevant to memory safety and release cleanliness.
for p in sorted((ROOT / 'src').rglob('*')):
    if p.suffix not in {'.c', '.h'}:
        continue
    text = p.read_text(errors='ignore')
    for bad in ('TODO', 'FIXME', 'HACK'):
        if bad in text:
            raise SystemExit(f'[FAIL] {p.relative_to(ROOT)} contains release marker {bad}')
    for bad_name in ('strcpy', 'strcat', 'sprintf', 'gets'):
        if re.search(r'\b' + bad_name + r'\s*\(', text):
            raise SystemExit(f'[FAIL] {p.relative_to(ROOT)} contains unsafe libc call {bad_name}()')
print('[PASS] release source hygiene scan')

if ran == 0:
    print('[SKIP] no host static analyzer available')
print('[PASS] Stage3 v32 static-analysis contract')
