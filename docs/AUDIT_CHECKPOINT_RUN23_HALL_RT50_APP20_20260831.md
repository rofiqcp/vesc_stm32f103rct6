# Audit Checkpoint Run23 — Hall LEFT/RIGHT + Motor-2 + RT50/App20

Tanggal audit: 2026-08-31

## Ruang lingkup

Run ini memprioritaskan acceptance Hall dan sensorless yang diminta pengguna, dengan fokus khusus pada Hall detect sampai jalur running dapat diuji secara deterministik pada hardware, seluruh command standar VESC untuk motor RIGHT/controller ID 2, RT Data 50 Hz, dan App ADC 20 Hz melalui USART3 115200.

`pio run` tidak dijalankan. Karena tidak ada akses ke motor fisik pada runtime ini, PASS hardware tidak diklaim. Tool `hall-commission` dibuat agar pengguna dapat menjadikan gerakan nyata sebagai acceptance gate setelah flash.

## Temuan dan perbaikan Hall

1. Packing Hall disamakan dengan firmware hoverboard referensi terbaru: `(U << 2) | (V << 1) | W`. Pin LEFT PB5/PB6/PB7 dan RIGHT PC10/PC11/PC12 juga cocok dengan reference yang diberikan.
2. Hall detect diubah mengikuti pola VESC FOC: rotor dikunci dengan forced electrical angle, Id diramp 1 detik menuju arus detect, lalu fase diputar maju dan mundur 3 revolusi dengan langkah 1 derajat/5 ms sambil mengakumulasi Hall state.
3. Parameter current Hall detect benar-benar menjadi target Id; bukan fixed-duty yang mengabaikan argumen current.
4. Hasil tidak cukup hanya memiliki enam state. Tabel wajib mempunyai raw 0/7 invalid, raw 1..6 valid, membentuk transisi Gray-code satu-bit, dan spacing sektor masuk akal.
5. Tabel hasil detect diterapkan ke lookup fast-path ISR dan mode Hall runtime, lalu disimpan melalui konfigurasi transactional yang sudah ada.
6. `hall-commission` melakukan detect → apply → persist → `COMM_GET_MCCONF` readback → flash status check → uji `SET_CURRENT` positif dan negatif. Default FAIL jika salah satu arah tidak melewati 50 ERPM atau muncul fault. Command trace diambil sebelum command stop, sehingga `SET_CURRENT=0`/`SET_DUTY=0` tidak dapat menghasilkan false PASS.
7. Standard `COMM_DETECT_HALL_FOC` tetap didukung dengan layout reply VESC (8 byte table + result). Seperti upstream, command standar ini mengembalikan hasil detect; penerapan otomatis dilakukan oleh commissioning helper/custom workflow, bukan mengubah semantics command VESC standar.

## Motor RIGHT / controller ID 2

Motor RIGHT tetap menggunakan `COMM_FORWARD_CAN` local controller ID 2 untuk command VESC standar. Audit statis memastikan `SET_DUTY`, `SET_CURRENT`, `SET_CURRENT_BRAKE`, `SET_RPM`, `SET_POS`, `SET_HANDBRAKE`, dan `SET_CURRENT_REL` diteruskan ke instance `MotorRuntime` hasil seleksi thread motor-2, bukan ke LEFT.

## RT Data 50 Hz dan App Data 20 Hz

Firmware tidak melakukan push `COMM_GET_VALUES` spontan karena VESC standar menggunakan request/reply. Host melakukan polling:

- RT cepat 50 Hz: `COMM_GET_VALUES_SELECTIVE` dengan bit I_motor, I_in, Id, Iq, duty, ERPM, Vin, fault, position, controller ID, Vd, Vq, status. Urutan dan scale dicocokkan dengan `commands.c` VESC upstream.
- App ADC 20 Hz: `COMM_GET_DECODED_ADC`, menggunakan data PA2/PA3 nyata.
- Extended/raw diagnostics: rate lambat default 2 Hz agar USART3 115200 tidak dijenuhkan oleh paket custom besar.

`stream-all` mengukur rate aktual. Untuk run dengan durasi finite, return code non-zero jika RT atau APP turun di bawah 90% target. `speed-test` memiliki acceptance minimum default RT >=45 Hz dan APP >=18 Hz serta gagal jika timeout/fault terjadi.

## Data telemetry

Jalur VESC standard mempertahankan I_motor, I_in, Id, Iq, duty, ERPM, Vin, fault, position, controller ID, Vd, Vq, Ah/Wh/tachometer pada `COMM_GET_VALUES` full. RT selective memakai subset cepat di atas agar dua motor tetap realistis pada USART3 115200. Raw ADC/current calibration/Hall/observer/ISR/USART/config diagnostics tetap tersedia melalui `debug.py` pada slow diagnostic rate.

## Clean code / static audit

- `#if 0`: 0.
- obvious unreferenced static function: 0.
- tab pada C/H/Python: 0.
- trailing whitespace: 0.
- Python runtime: tepat dua file (`tools/vesc.py`, `tools/debug.py`).
- folder `vesc_tools/`: tidak ada.
- host syntax `foc_math.c` dengan `-Wall -Wextra -Werror`: PASS.
- `debug.py --self-test`: PASS, termasuk VESC frame/CRC/config, controller-ID2 forwarding, selective RT parser, APP ADC parser dan topology Hall.
- Run22 sensorless/Hall static contract: 30/30 PASS.
- Run23 Hall/RT/Motor2 static contract: 25/25 PASS.
- `py_compile` / `compileall`: PASS.
- shell `bash -n`: PASS.

## File source berubah dibanding Run21

- `src/applications/appconf_default.h`
- `src/comm/commands.c`
- `src/hwconf/hw.c`
- `src/motor/foc_math.c`
- `src/motor/mc_interface.c`
- `src/motor/mc_interface.h`
- `src/motor/mcpwm_foc.c`
- `tools/debug.py`
- `tools/vesc.py`
- `tools/README.md`

## Acceptance hardware yang harus dijalankan pengguna

Roda wajib diangkat dan area aman.

```bash
cd /media/sirobo/Data/BLDC/vesc_stm32f103rct6
python3 -m pip install -r tools/requirements.txt
python3 tools/debug.py hall-commission --motor both --yes
```

Command tersebut hanya PASS bila LEFT dan RIGHT berhasil detect table yang valid, MCCONF readback cocok, flash save status valid, command current diterima, dan kedua arah mencapai minimum 50 ERPM tanpa fault.

Uji rate nyata:

```bash
python3 tools/debug.py stream-all --seconds 10 --csv rt_app_50_20.csv
python3 tools/debug.py speed-test --motor 0 --erpm 300 --seconds 5 --yes --csv left_rate.csv
python3 tools/debug.py speed-test --motor 1 --erpm 300 --seconds 5 --yes --csv right_rate.csv
```

Uji command standar controller ID 2:

```bash
python3 tools/vesc.py current 1.5 --motor right --seconds 3 --yes
python3 tools/vesc.py duty 0.03 --motor right --seconds 3 --yes
python3 tools/vesc.py rpm 300 --motor right --seconds 5 --yes
python3 tools/vesc.py brake 1.0 --motor right --seconds 2 --yes
python3 tools/vesc.py handbrake 1.0 --motor right --seconds 2 --yes
python3 tools/vesc.py position 30 --motor right --seconds 3 --yes
```

## Belum diklaim PASS

- Build/Flash/RAM ukuran Run23 karena `pio run` dilarang pada run ini.
- Hall detect dan running aktual LEFT/RIGHT sampai pengguna menjalankan `hall-commission` pada hardware.
- RT 50 Hz / APP 20 Hz aktual sampai rate test pengguna dijalankan pada `/dev/ttyUSB0`.
- Torsi, arah mekanik, dan tuning current/speed/position pada beban nyata.
