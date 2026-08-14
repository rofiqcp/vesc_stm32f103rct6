# V12 Validation Status

Host-side validation completed:

```text
test_packet_v8: PASS
test_foc_math: PASS
test_vesc_buffer: PASS
test_vesc_config_layout: PASS
test_vesc_config_early_get: PASS
audit_v11_vesc_f103: PASS
audit_v12_calibration: PASS
Python protocol self-test: PASS
V12 host/audit tests: ALL PASS
```

The V8 transport SHA256 freeze is rechecked by V12 audit.

No ARM cross-build is claimed in this environment because PlatformIO / arm-none-eabi-gcc is not installed here. The target board must still be built and flashed on the user's STM32F103RCT6 toolchain.
