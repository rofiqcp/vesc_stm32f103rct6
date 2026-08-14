# First Power Test V6

1. Build only:

```bash
pio run -t clean
pio run
```

2. Flash with motor power/current limited and wheels unloaded:

```bash
pio run -t upload
```

3. Check handshake; do not command motor yet:

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0
python3 debug_vesc_f103.py comm-diag --port /dev/ttyUSB0
```

4. Verify virtual CAN topology:

```bash
python3 debug_vesc_f103.py can-scan --port /dev/ttyUSB0
```

Expected: node 2 is reported and forwarded FW_VERSION for RIGHT returns successfully.

5. Verify current calibration and passive telemetry:

```bash
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0
python3 debug_vesc_f103.py status --port /dev/ttyUSB0
```

6. Detect sensors at current-limited bench supply:

```bash
python3 debug_vesc_f103.py sensor-detect --motor 0 --mode auto --yes --port /dev/ttyUSB0
python3 debug_vesc_f103.py sensor-detect --motor 1 --mode hall --yes --port /dev/ttyUSB0
```

7. Save detected runtime configuration:

```bash
python3 debug_vesc_f103.py config-save --port /dev/ttyUSB0
python3 debug_vesc_f103.py config-status --port /dev/ttyUSB0
```

8. Capture samples before higher-current tests:

```bash
python3 debug_vesc_f103.py sample --motor 0 --count 64 --decimation 8 --port /dev/ttyUSB0
python3 debug_vesc_f103.py sample --motor 1 --count 64 --decimation 8 --port /dev/ttyUSB0
```

9. Start active tests at low current only after all passive checks pass.
