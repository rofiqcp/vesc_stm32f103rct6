# Run30

Checkpoint ini melanjutkan Run29 dengan cross-check terhadap firmware hoverboard FOC yang sudah berjalan nyata, Hall detect berbasis forced electrical phase + Id, acceptance aktif seluruh command VESC utama, serta revisi indikator EEPROM menjadi 5 beep per transaksi save sukses.

Mulai dari:

```bash
pio run -t upload
python3 tools/debug.py startup-check
python3 tools/debug.py calibrate --timeout 8
python3 tools/debug.py drive-acceptance --yes
```

Dokumentasi lengkap:

- `docs/AUDIT_CHECKPOINT_RUN30_HOVERBOARD_REAL_MODE_VESC_ACCEPTANCE_20260901.md`
- `docs/RUN30_HARDWARE_ACCEPTANCE_COMMANDS.txt`
- `docs/HOVERBOARD_REFERENCE_HOST_VALIDATION_RUN30.txt`

Audit offline:

```bash
python3 tools/debug.py --self-test
python3 tools/audit_run29.py
python3 tools/audit_run30.py
```
