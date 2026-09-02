# Run31 — ADC/ISR + HFI + Detect-All Fix

Checkpoint ini menindaklanjuti fault hardware Run30 yang baru muncul setelah `SET_CURRENT` aktif. Current calibration sebelumnya sebenarnya PASS; Run31 menghilangkan ADC3/DMA2 VBUS freshness path, menjaga mapping lima rank ADC hoverboard V13, memakai boundary DMA half-transfer yang lebih cepat untuk dual-FOC VESC, memaksa build hardware `-O3`, dan memigrasikan `HFI Start Samples` legacy 2 menjadi nilai resmi VESC 6.00 yaitu 5.

Mulai dengan **clean optimized build**:

```bash
pio run -t clean
pio run -e stm32f103rc -t upload
```

Kemudian lakukan staged acceptance:

```bash
python3 tools/debug.py vesc-tool-check --motor 0
python3 tools/debug.py vesc-tool-check --motor 1
python3 tools/debug.py startup-check
python3 tools/debug.py calibrate --timeout 8
python3 tools/debug.py startup-check
python3 tools/debug.py sensor-select --motor 0 --mode sensorless
python3 tools/debug.py motor-test --motor 0 --mode current --value 0.5 --seconds 2 --yes
```

Jika fault muncul, jangan lanjut; ambil native diagnostic:

```bash
python3 tools/debug.py fault-detail --motor 0
python3 tools/debug.py fault-detail --motor 1
```

Setelah low-current proof PASS, ikuti:

- `docs/RUN31_HARDWARE_ACCEPTANCE_COMMANDS.txt`
- `docs/AUDIT_CHECKPOINT_RUN31_ADC_ISR_HFI_DETECT_ALL_20260901.md`

Audit offline (Run31 mencakup parity pin/PWM/polarity/deadtime terhadap firmware hoverboard V13 yang sudah berjalan):

```bash
python3 tools/debug.py --self-test
python3 tools/audit_run29.py
python3 tools/audit_run30.py
python3 tools/audit_run31.py
```
