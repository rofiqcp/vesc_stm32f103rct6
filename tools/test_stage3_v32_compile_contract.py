#!/usr/bin/env python3
"""Host translation-unit compile contract for cross-module declaration errors.

This does NOT replace the ARM/PlatformIO target build. It deliberately compiles
logic-heavy translation units against tiny hardware/RTOS declaration stubs so
missing project headers, implicit declarations, return-type errors and many
copy/paste syntax regressions are caught before target flashing.
"""
from pathlib import Path
import shutil, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
COMPILERS=[c for c in (shutil.which('gcc'), shutil.which('clang')) if c]
if not COMPILERS:
    raise SystemExit('[SKIP] host C compiler unavailable')
incs=[f'-I{ROOT / "tools/compile_stubs"}', f'-I{ROOT / "src"}']
units=[
    'src/applications/app.c',
    'src/applications/app_adc.c',
    'src/applications/app_command.c',
    'src/applications/app_uartcomm.c',
    'src/comm/commands.c',
    'src/comm/packet.c',
    'src/confgenerator.c',
    'src/motor/mc_interface_sample.c',
    'src/encoder/enc_abi.c',
    'src/encoder/encoder.c',
    'src/encoder/encoder_cfg.c',
    'src/hwconf/hw_status.c',
    'src/motor/foc_math.c',
    'src/motor/mc_interface.c',
    'src/motor/mc_math.c',
    'src/motor/mcpwm_foc.c',
    'src/motor/mc_interface_tasks.c',
    'src/telemetry.c',
    'src/terminal.c',
    'src/util/buffer.c',
]
for compiler in COMPILERS:
    compiler_name=Path(compiler).name
    for rel in units:
        cmd=[compiler,'-std=gnu11','-fsyntax-only','-Wall','-Wextra','-Wshadow',
             '-Wdouble-promotion','-Wformat=2','-Werror',*incs,str(ROOT/rel)]
        r=subprocess.run(cmd,text=True,capture_output=True)
        if r.returncode:
            sys.stderr.write(f'[FAIL] {compiler_name} {rel}\n{r.stdout}{r.stderr}')
            raise SystemExit(1)
        print(f'[PASS] {compiler_name} TU compile: {rel}')

# FreeRTOS public kernel headers require FreeRTOS.h to appear first.
kernel_headers=("task.h","queue.h","semphr.h","timers.h","event_groups.h","stream_buffer.h")
for src in sorted((ROOT/"src").rglob("*")):
    if src.suffix not in {".c",".h"}:
        continue
    lines=src.read_text(errors="ignore").splitlines()
    include_lines=[(i+1,line.strip()) for i,line in enumerate(lines) if line.strip().startswith("#include")]
    free_idx=next((i for i,line in include_lines if "FreeRTOS.h" in line), None)
    for i,line in include_lines:
        if any((f'\"{h}\"' in line) or (f'<{h}>' in line) for h in kernel_headers):
            if free_idx is None or free_idx >= i:
                raise SystemExit(f'[FAIL] {src.relative_to(ROOT)}:{i}: {line} requires FreeRTOS.h first')
print('[PASS] FreeRTOS kernel-header include order')

print('[PASS] Stage3 v32 cross-module compile contract')
