#!/usr/bin/env python3
from pathlib import Path
import subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as td:
    exe=Path(td)/'stage1_appconf'
    cmd=['gcc','-std=c11','-Wall','-Wextra','-Wshadow','-Wdouble-promotion','-Wformat=2','-Werror',
         '-ffunction-sections','-fdata-sections','-Itools/host_stubs','-Isrc','-Isrc/motor','-Isrc/hwconf','-Isrc/encoder','-Isrc/util','-Isrc/applications','-Isrc/comm',
         'tools/test_stage1_appconf_roundtrip.c','src/confgenerator.c','src/util/buffer.c','-Wl,--gc-sections','-lm','-o',str(exe)]
    cp=subprocess.run(cmd,cwd=ROOT,text=True,capture_output=True)
    if cp.returncode:
        print(cp.stdout);print(cp.stderr);sys.exit(cp.returncode)
    rp=subprocess.run([str(exe)],cwd=ROOT,text=True,capture_output=True)
    print(rp.stdout,end='')
    if rp.returncode:
        print(rp.stderr);sys.exit(rp.returncode)
    print('PASS: actual VESC6 493-byte APPCONF codec preserves PA2/PA3 settings and rejects live/unsupported writes')
