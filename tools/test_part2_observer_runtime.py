#!/usr/bin/env python3
from pathlib import Path
import shutil, subprocess, tempfile, sys

ROOT = Path(__file__).resolve().parents[1]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("gcc is required for Part-2 observer runtime regression")
incs = ["-Itools/host_stubs", "-Isrc", "-Isrc/motor", "-Isrc/hwconf", "-Isrc/encoder",
        "-Isrc/util", "-Isrc/applications", "-Isrc/comm"]
common = [gcc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
          "-ffunction-sections", "-fdata-sections"] + incs
with tempfile.TemporaryDirectory(prefix="vesc_obs_rt_") as td:
    td = Path(td)
    foc_o = td / "foc_math.o"
    test_o = td / "test.o"
    exe = td / "observer_runtime"
    for src, out in [("src/motor/foc_math.c", foc_o), ("tools/test_part2_observer_runtime.c", test_o)]:
        cp = subprocess.run(common + ["-c", src, "-o", str(out)], cwd=ROOT, text=True,
                            capture_output=True)
        if cp.returncode:
            print(cp.stdout, cp.stderr)
            raise SystemExit(cp.returncode)
    cp = subprocess.run([gcc, "-Wl,--gc-sections", str(test_o), str(foc_o), "-lm", "-o", str(exe)],
                        cwd=ROOT, text=True, capture_output=True)
    if cp.returncode:
        print(cp.stdout, cp.stderr)
        raise SystemExit(cp.returncode)
    cp = subprocess.run([str(exe)], cwd=ROOT, text=True, capture_output=True)
    print(cp.stdout, end="")
    if cp.stderr:
        print(cp.stderr, file=sys.stderr, end="")
    if cp.returncode:
        raise SystemExit(cp.returncode)
print("PASS: actual fixed-point C observer runtime tracks synthetic PMSM flux trajectories")
