# ARM/PlatformIO Build Fix — STM32CubeF1 Vendor Warning Isolation

Baseline: `vesc_stm32f103rct6_batch11_part2_commands_integrity_cleanup.zip`

## Reported target-build failure

The first real PlatformIO/ARM build reached STM32CubeF1 and stopped in the vendor file:

`Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c`

because `HAL_PCD_WKUP_IRQHandler(PCD_HandleTypeDef *hpcd)` has an unused parameter in STM32CubeF1 1.8.6. The project had a blanket `-Werror` in `build_flags`; PlatformIO applies `build_flags` globally, so this converted an upstream/vendor warning into a fatal error before project motor sources were reached.

## Fix

Compiler warning policy is now scope-separated:

- `build_flags`: target definitions, include paths and optimization only. These flags may be consumed by framework, FreeRTOS and project sources.
- `build_src_flags`: warning diagnostics applied only to files under the project `src/` tree.

Project sources retain:

- `-Wall`
- `-Wextra`
- `-Wshadow`
- `-Wdouble-promotion`
- `-Wformat=2`
- `-Werror=return-type`
- `-Werror=implicit-function-declaration`

A blanket `-Werror` is intentionally not used for the target build. This avoids tying a clean firmware build to warnings inside the pinned STM32CubeF1/FreeRTOS vendor packages. Core portable firmware translation units continue to be strict-host-compiled with `-Werror` by the existing regression suite.

No motor-control or runtime source was changed by this build fix.

## Recommended clean rebuild

From the project root:

```bash
pio run -t clean
pio run
pio run -t upload
```

Running `pio run` separately before `-t upload` makes any compile/link issue easier to identify and avoids attempting ST-Link upload until the ELF has linked successfully.

## Unrelated shell startup message

The message:

`bash: /media/sirobo/Data/BLDC/agv_ws/install/setup.bash: No such file or directory`

is unrelated to this firmware. It means a shell startup file is sourcing an old ROS/workspace `install/setup.bash` path that no longer exists. It does not cause the STM32CubeF1 compilation failure.

## Verification in this revision

Passed in the audit environment:

- `tools/verify_vesc_port.py`
- `tools/test_batch11_part1.py`
- `tools/test_batch11_part2.py`
- `tools/debug.py --self-test`
- `tools/test_platformio_build_policy.py`

A PlatformIO/ARM toolchain is not installed in the audit container, so the exact user-side GCC 7.2.1 target build cannot be executed here. The fix directly addresses the global-warning-policy failure shown by the real target log and preserves all functional source/regression invariants.
