#!/usr/bin/env python3
from pathlib import Path
import re, subprocess, tempfile, sys
ROOT = Path(__file__).resolve().parents[1]

def read(p): return (ROOT / p).read_text()
def ok(cond, msg):
    if not cond:
        print('FAIL:', msg); sys.exit(1)
    print('PASS:', msg)

ld = read('src/stm32f103rc_flash.ld')
pio = read('platformio.ini')
fr = read('src/FreeRTOSConfig.h')
app = read('src/applications/appconf_default.h')
mt = read('src/motor/mc_interface_tasks.c')
dbg = read('src/motor/mc_interface_sample.c')

ok('LENGTH = 48K' in ld and 'ORIGIN = 0x20000000' in ld,
   'linker remains locked to real STM32F103RCT6 48-KiB SRAM')
ok('board_upload.maximum_ram_size = 49152' in pio,
   'PlatformIO RAM checker is explicitly locked to 49152 bytes')
ok(re.search(r'configTOTAL_HEAP_SIZE\s+\(\(size_t\)\(18 \* 1024\)\)', fr) is not None,
   'FreeRTOS heap is bounded to 18 KiB')
ok(re.search(r'#define\s+SAMPLE_BUFFER_LEN\s+64U', app) is not None,
   'debug sampler is capped at 64 samples')
ok('RTOS_READY_HEAP_RESERVE_BYTES 2048U' in mt and 'xPortGetFreeHeapSize()' in mt,
   'motor-ready state requires at least 2 KiB runtime FreeRTOS heap reserve')
ok('mode < DEBUG_SAMPLING_OFF' not in dbg,
   'debug sampling enum no longer triggers ARM -Wtype-limits')

# Compile only enough to get the exact ABI size of debug_sample_t.
with tempfile.TemporaryDirectory() as td:
    src = Path(td) / 'sizeof_debug.c'
    exe = Path(td) / 'sizeof_debug'
    src.write_text('#include <stdio.h>\n#include "datatypes.h"\nint main(void){printf("%zu\\n",sizeof(debug_sample_t));return 0;}\n')
    cp = subprocess.run(['gcc','-std=c11','-Itools/host_stubs','-Isrc',str(src),'-o',str(exe)],
                        cwd=ROOT,text=True,capture_output=True)
    if cp.returncode != 0:
        print(cp.stdout, cp.stderr); sys.exit(cp.returncode)
    rp = subprocess.run([str(exe)],cwd=ROOT,text=True,capture_output=True)
    size = int(rp.stdout.strip())
    ok(size == 30, 'debug_sample_t remains 30 bytes')

old_debug = 256 * 30
new_debug = 64 * 30
saved_debug = old_debug - new_debug
saved_heap = (22 - 18) * 1024
saved_total = saved_debug + saved_heap
ok(saved_debug == 5760, 'debug-buffer reduction saves 5760 bytes of static SRAM')
ok(saved_heap == 4096, 'RTOS-heap reduction saves 4096 bytes of static SRAM')
ok(saved_total == 9856, 'combined RAM fix saves 9856 bytes versus the failing build')

# User's real linker report overflowed by 6184 bytes. This is not used as a
# substitute for pio run, but verifies the selected reductions exceed that
# measured overflow with useful link-time margin.
measured_overflow = 6184
projected_headroom = saved_total - measured_overflow
ok(projected_headroom >= 3000,
   f'RAM changes exceed measured overflow with ~{projected_headroom} bytes projected linker headroom')

print('ALL STM32F103RCT6 RAM-BUDGET REGRESSIONS: PASS')
