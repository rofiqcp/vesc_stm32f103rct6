# Audit Checkpoint Run26 - Runtime Telemetry, ISR Calibration, dan Buzzer

## Acceptance hardware yang memicu run ini

Log Run25 menunjukkan RT Data hanya sekitar 4.7 Hz, App Data 0 Hz dengan timeout
berulang, dan historical ISR maximum 5582 cycle. Speed-test juga menolak karena
MCCONF masih memilih encoder/Hall dan sensor belum didetect.

## Perbaikan

1. `tools/debug.py`: `COMM_GET_DECODED_ADC` dikoreksi dari 31 menjadi ABI VESC
   6.00 yang benar, 32. ID 31 adalah `COMM_GET_DECODED_PPM`.
2. RT LEFT/RIGHT menggunakan dua request `COMM_GET_VALUES_SELECTIVE` standar
   VESC dalam satu slot dan membedakan reply memakai `controller_id` 1/2.
3. App Data board-level PA2/PA3 dipoll satu kali pada 20 Hz dengan command 32.
4. Diagnostik besar menjadi round-robin satu request per slow slot; CSV tidak
   di-flush setiap frame RT.
5. Kalibrasi hard ISR hanya mengupdate enam pasangan min/max dan counter.
   Finalisasi midpoint/validasi/commit offset dikerjakan task 1 kHz. Akumulator
   sum/square 64-bit dihapus dari ISR.
6. Power-on melody tetap aktif. Manual/boot calibration memiliki cue non-blocking.
   Save konfigurasi yang benar-benar sukses mengantrikan dua beep pendek.
   Fault VESC code N menggunakan N beep pendek. Running/detect tidak memainkan
   cue non-fault.
7. Fault recovery 1000 ms dari Run24 tetap dipertahankan untuk fault software
   yang memiliki bukti kondisi sehat; hard powerstage/flash fault tidak auto-clear.

## ADC/ISR yang dipertahankan

Current-critical dual ranks tetap mengikuti reference hoverboard:
- rank1 ADC1 PC1 RIGHT_DC | ADC2 PC0 LEFT_DC
- rank2 ADC1 PA0 LEFT_A | ADC2 PC3 LEFT_B
- rank3 ADC1 PC4 RIGHT_B | ADC2 PC5 RIGHT_C

DMA half-transfer terjadi setelah rank3 sehingga kedua loop FOC menerima enam
kanal arus koheren seawal mungkin. PA2/PA3 dan temperature berada setelah
boundary current. Polarity current tetap `offset - raw`, skala 0.020 A/count.

## Static/host verification

- `tools/debug.py --self-test`: PASS.
- Python compile: PASS.
- offline dual `GET_VALUES_SELECTIVE` controller-ID routing: PASS.
- `foc_math.c` GCC `-Wall -Wextra -Werror -fsyntax-only`: PASS.
- exactly two Python files: `tools/debug.py`, `tools/vesc.py`.
- `vesc_tools/`: absent.
- `#if 0`: 0.
- obvious unreferenced static functions: 0.
- tabs/trailing whitespace in project-owned C/H: 0.

`pio run` was intentionally not executed. ISR <4000 and actual 50/20 Hz remain
hardware acceptance items after user build/flash.
