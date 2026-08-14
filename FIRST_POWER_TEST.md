# FIRST POWER TEST

## A. Tanpa power motor / gate disabled

```bash
pio run -t clean
pio run
pio run -t upload
python3 debug_vesc_f103.py --self-test
python3 debug_vesc_f103.py info --port /dev/ttyUSB0
python3 debug_vesc_f103.py test-all --port /dev/ttyUSB0
```

Pastikan startup calibration `done=1 valid=1`.

## B. Verifikasi gate dengan oscilloscope

Sebelum MOSFET diberi bus penuh, pastikan:

```text
OFF/deadtime: High pin LOW, Low pin HIGH
High FET ON : High pin HIGH
Low  FET ON : Low pin LOW
```

Tidak boleh ada overlap gate HIGH-side dan low-side secara fisik.

## C. Verifikasi ADC zero

```bash
python3 debug_vesc_f103.py calibrate --port /dev/ttyUSB0
python3 debug_vesc_f103.py status --port /dev/ttyUSB0
```

Id/Iq/Imotor/Ibatt harus dekat nol saat tidak ada arus. Jika offset valid tetapi Ampere terlalu besar/kecil saat arus nyata diberikan, perbaiki A/count di `src/app_config.h`.

## D. Sensor auto-detect

Motor harus bebas bergerak dan current scaling sudah diverifikasi.

```bash
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 0 --mode auto --yes
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 1 --mode auto --yes
```

LEFT dapat jatuh ke Hall atau Encoder. Untuk memaksa encoder:

```bash
python3 debug_vesc_f103.py sensor-detect --port /dev/ttyUSB0 --motor 0 --mode encoder --yes
```

## E. Arus kecil

```bash
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode current --value 0.5 --seconds 1 --yes
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode current --value -0.5 --seconds 1 --yes
```

Pantau Id, Iq, Vd, Vq, Imotor, Ibatt, duty, ERPM, rotor angle dan fault.

## F. Baru RPM

```bash
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode rpm --value 300 --seconds 2 --yes
python3 debug_vesc_f103.py motor-test --port /dev/ttyUSB0 --motor 0 --mode rpm --value -300 --seconds 2 --yes
```

Jangan langsung menaikkan current/RPM sampai A/count, phase order, Hall/encoder angle dan ISR margin terbukti benar.
