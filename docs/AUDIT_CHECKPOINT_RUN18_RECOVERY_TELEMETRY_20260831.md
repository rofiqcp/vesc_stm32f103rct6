# Audit Checkpoint Run 18 — Recovery, ISR, ADC, dan Telemetri

Tanggal audit: 31 Agustus 2026

## Provenance checkpoint

Runtime sempat kehilangan checkpoint Run16/Run17 secara fisik. Checkpoint fisik terbaru yang berhasil dipulihkan dari Library adalah `vesc_stm32f103rct6_checkpoint_run12_clean_format_20260830.zip`, dengan Run11 juga dipulihkan sebagai pembanding sebelum clean-format. Perbaikan Run13–Run17 yang sudah tercatat pada percakapan direkonstruksi secara terkontrol dan dikunci kembali dengan acceptance tests. Karena file bernama persis `hoverboard-firmware-hack-FOC_v3_DQ_SIGN_FIX(1).zip` tidak ditemukan pada Library/runtime, pembandingan hardware tambahan menggunakan reference fisik yang tersedia `hoverboard-firmware-hack-FOC_v3_VESC_HOVERBOARD_FINAL.zip`. Hal ini tidak dianggap sebagai verifikasi byte-identik terhadap DQ_SIGN_FIX yang hilang.

## Perbaikan dan temuan Run18

1. Regresi formatter Run12 pada operator shift-assignment dipulihkan (`>>=` dan `<<=`), kemudian dikunci dengan `tools/audit_run14_operator_integrity.py`.
2. ADC rank-6 spare dipertahankan pada 7.5 cycle. Sequence nominal enam rank sekitar 34.406 us dari periode PWM 62.5 us sehingga margin nominal sekitar 28.094 us. Rank 1–3 current, rank 4 APP ADC, rank 5 temperatur MCU 239.5 cycle, dan ADC3 DCLINK tetap dipertahankan.
3. Timestamp cycle ISR ditangkap dari awal `DMA1_Channel1_IRQHandler`. Jalur DMA transfer-error dan early-return kalibrasi menutup accounting timing melalui helper yang sama.
4. Sampel ke-2000 boot calibration hanya menyelesaikan offset dan tetap keluar dari ISR. FOC baru memakai frame ADC berikutnya sehingga satu frame tidak menjadi sampel kalibrasi sekaligus sampel kontrol.
5. `Id_ref` dan `Iq_ref` sekarang dibawa dari `foc_rt_snapshot_t` ke `motor_telemetry_t`, kemudian diekspor pada extended telemetry revision 8 dan parser `tools/debug.py`.
6. Duty realtime tidak lagi menggabungkan snapshot ISR dengan `m->duty_now` task-side. Nilai duty dihitung dari `duty_u_q15`, `duty_v_q15`, dan `duty_w_q15` pada snapshot seqlock yang sama; tanda mengikuti Vq.
7. `current_motor` tetap memakai konvensi VESC `SIGN(Vq*Iq) * |Idq|`, sedangkan `current_in` berasal dari DC-shunt snapshot.
8. Clean-format gate diperketat. Selain fungsi/control/early-return satu baris, `case/default` yang masih menempel dengan statement sekarang juga ditolak. Sisa `case ...: return ...` telah dipisah.
9. ABI MCCONF/APPCONF tetap dikunci pada 481/493 byte. Gate Run16 mempertahankan `foc_sensor_mode` offset 152, `foc_openloop_rpm` 201, `foc_openloop_rpm_low` 205, `foc_hfi_start_samples` 265, dan `foc_mtpa_mode` 303.
10. Tidak ada `pio run` pada audit ini.

## Pembandingan reference hardware yang tersedia

Reference fisik yang tersedia mempertahankan pola current calibration 2000 sample dengan seed 2000 dan filter `(raw + offset) / 2`, serta polaritas current `offset - raw`. Mapping enam channel utama juga konsisten: DCR/DCL, Left-A/Left-B, Right-B/Right-C. Reference menyatakan TIM1/TIM8 PWM/ADC-trigger tetap dipertahankan pada firmware yang sebelumnya tervalidasi.

`logs.zip` berisi 6 CSV dan 3639 baris. Audit log menghasilkan:

- maksimum `foc_isr_cycles` yang tercatat pada sampel: 3931 cycle;
- tidak ada sampel baris yang melebihi 4000 cycle;
- maksimum field historis `foc_isr_cycles_max`: 4007 cycle;
- karena historical max pernah 4007, reference hardware tidak diklaim sebagai zero-overrun absolut.

## Acceptance yang dijalankan

PASS pada audit/tes berikut sebelum checkpoint dibuat:

- Run12 clean-format (termasuk gate baru case/default body);
- Run13 ADC margin;
- Run14 operator integrity;
- Run15 host C syntax untuk unit portable;
- Run16 VESC 6.00 ABI;
- Run17 ISR/boot calibration;
- Run18 telemetry contract;
- Run8, Run9, dan Run11 static contracts;
- RTOS audit;
- communication audit;
- VESC protocol self-test CRC/framing/resync;
- pytest: 95/95 PASS;
- Python compileall;
- `bash -n` pada script shell.

Audit dokumentasi C/H sebelum cleanup `case/default` terakhir melaporkan 930 definisi fungsi, 591 prototype, 2090 parameter, dan 3078 deklarator variabel tanpa missing comment. Cleanup terakhir hanya memecah label switch/return dan tidak menambah/menghapus definisi, prototype, parameter, atau deklarator. Percobaan rerun penuh audit dokumentasi setelah cleanup terkena timeout harness, bukan assertion FAIL; karena itu hasil dokumentasi terakhir yang selesai tetap dicatat secara eksplisit dan tidak disamarkan sebagai rerun baru.

## File source utama yang berubah terhadap checkpoint fisik Run12

- `src/applications/app.c`
- `src/applications/app_command.c`
- `src/comm/commands.c`
- `src/comm/packet.c`
- `src/conf_general.c`
- `src/confgenerator.c`
- `src/datatypes.h`
- `src/hwconf/hw.c`
- `src/motor/foc_math.c`
- `src/motor/mc_interface.c`
- `src/motor/mcpwm_foc.c`
- `src/motor/mcpwm_foc.h`
- `src/stm32f1xx_it.c`
- `src/telemetry.c`

Sebagian file di atas berubah karena pemulihan clean-format/operator dari Run12 dan rekonstruksi gate Run13–Run17, bukan karena algoritma baru Run18 semuanya.

## Tool/audit yang ditambah atau diperbarui

- `tools/audit_reference_logs_cycle.py`
- `tools/audit_run12_format.py`
- `tools/audit_run13_adc_margin.py`
- `tools/audit_run14_operator_integrity.py`
- `tools/audit_run15_host_syntax.py`
- `tools/audit_run15_language_inventory.py`
- `tools/audit_run16_vesc600_abi.py`
- `tools/audit_run17_isr_bootcal.py`
- `tools/audit_run18_telemetry_contract.py`
- `tools/run_static_acceptance.py`
- `tools/debug.py`
- `tools/rtos_audit.py`

## Belum terverifikasi tanpa build/hardware

Checkpoint ini tidak mengklaim PASS hardware untuk:

- penggunaan Flash/RAM aktual setelah linker;
- worst-case ISR Run18 di STM32F103RCT6 nyata tetap <= 4000 cycle;
- RT Data 50 Hz tanpa dropout pada VESC Tool/perangkat nyata;
- sensorless start, reversal, dan transition observer/open-loop pada motor nyata;
- Hall nyata dan encoder AB LEFT-only;
- rotor sign/offset serta position control fisik;
- duty/current/speed/position/brake/handbrake di bawah beban;
- R/L/flux/Hall/encoder/Detect-All pada hardware;
- brownout/power-loss persistence flash;
- LED dan buzzer fisik;
- fault behavior under load.

Baseline resource pengguna tetap RAM 41912/49152 dan Flash 157276/253952. Run18 menambah dua `float` pada `motor_telemetry_t` dan beberapa kode/test statis, tetapi ukuran hasil link belum diverifikasi karena `pio run` sengaja tidak dijalankan.
