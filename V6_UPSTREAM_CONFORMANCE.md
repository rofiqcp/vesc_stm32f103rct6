# V6 Upstream Conformance Audit

## Tujuan

V6 dibuat setelah audit ulang `vedderb/bldc` current master terhadap V5. Sasaran utamanya adalah **kesamaan semantik jalur VESC Tool**, bukan sekadar membuat fungsi/thread dengan nama yang mirip.

## 1. UART request/reply

```text
USB-UART TX
 -> PB11 USART3 RX
 -> RXNE IRQ
 -> RX software ring
 -> packet_process_thread
 -> packet decoder + CRC
 -> command dispatcher
 -> motor interface / blocking worker
 -> response payload
 -> packet encoder + CRC
 -> TX software ring
 -> TXE IRQ
 -> PB10 USART3 TX
 -> TC IRQ
 -> USB-UART RX
```

ISR USART3 hanya memindahkan byte/error/TC state. Parsing dan motor command tidak pernah dijalankan di ISR UART.

## 2. Dual motor

```text
UART command normal = LEFT local, controller ID 1

UART:
COMM_FORWARD_CAN
  target = 2
  inner = <command>
      |
      v
RIGHT motor context
      |
      v
inner reply langsung ke UART
```

Tidak ada HAL CAN/CAN1/physical `comm_can` route. `COMM_PING_CAN` hanya melaporkan virtual node 2.

## 3. Real-time data

Diimplementasikan dengan ordering/scaling VESC protocol subset yang relevan:

- GET_VALUES + selective
- GET_VALUES_SETUP + selective
- Id/Iq, motor/input current, duty, eRPM, Vbus, Ah/Wh, tachometer, fault, position, controller ID, Vd/Vq, timeout/status
- NTC field tetap berada pada wire layout tetapi bernilai 0 karena NTC subsystem sengaja dikeluarkan.

## 4. App data

`COMM_CUSTOM_APP_DATA` sekarang mempunyai callback registration API. Callback menerima payload **setelah command byte di-strip**, sama seperti pola app-data callback upstream. Jika callback user tidak diregister, diagnostic app-data V6 digunakan sebagai default.

## 5. Thread responsibility mapping

| V6 worker | Tanggung jawab |
|---|---|
| `adc_thread` | application ADC worker; tidak dipakai untuk FOC ADC zero calibration |
| `packet_process_thread` | RX ring -> packet parser -> non-blocking commands |
| `blocking_thread` | perintah detect/ping yang dapat memakan waktu |
| `timer_thread` | motor/interface housekeeping |
| `sample_send_thread` | mengirim sample setelah ISR selesai menangkap RAM |
| `fault_stop_thread` | deferred fault stop/status |
| `stat_thread` | statistics accumulation |
| `pid_thread` | outer speed/position controller |
| `rpm_thread` | port-specific Hall/ABI RPM mirror |
| `periodic_thread` | periodic telemetry/rotor position |
| `led_thread` | board LED status |
| `timeout_thread` | command timeout global kedua motor |
| `current_cal_thread` | worker tambahan khusus F103 untuk current-zero startup |

Current PI/SVPWM tetap hard-real-time di ADC DMA ISR dan tidak dipindahkan ke RTOS thread.

## 6. Hall / encoder detect

`COMM_DETECT_HALL_FOC` dan `COMM_DETECT_ENCODER` masuk `blocking_thread`, membaca requested detect current dari command, menjalankan forced-angle detection state machine, dan membalas dengan wire structure Hall/encoder detect. Right encoder ditolak karena hardware proyek hanya menyediakan encoder ABI pada LEFT.

## 7. Sample

Sample ISR hanya menulis buffer RAM. Pengiriman serial dilakukan `sample_send_thread`. Output V6 bukan lagi proprietary chunk `CUSTOM_SAMPLE_DATA`; output memakai `COMM_SAMPLE_PRINT` per sample dan float32-auto field layout.

## 8. Persistence

V6 menggunakan append-log flash emulation dengan CRC/sequence/commit-last. Ini mengganti single-record erase-every-save rancangan sementara. Fast ADC IRQ dihentikan selama F1 flash busy dan power stage sudah gated OFF.

## 9. Test yang tersedia

```bash
python3 debug_vesc_f103.py --self-test
bash tests/host/audit_v6_upstream.sh

gcc -std=c11 -O2 -Wall -Wextra -Werror -Isrc \
  tests/host/test_packet_v6.c src/vesc_packet.c -o /tmp/test_packet_v6
/tmp/test_packet_v6
```

Live hardware:

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0
python3 debug_vesc_f103.py can-scan --port /dev/ttyUSB0
python3 debug_vesc_f103.py status --port /dev/ttyUSB0
python3 debug_vesc_f103.py sample --motor 0 --count 64 --decimation 8 --port /dev/ttyUSB0
python3 debug_vesc_f103.py sample --motor 1 --count 64 --decimation 8 --port /dev/ttyUSB0
python3 debug_vesc_f103.py config-status --port /dev/ttyUSB0
```

## 10. Remaining conformance blockers

V6 intentionally reports these rather than pretending they are implemented:

- full confgenerator MCCONF/APPCONF binary schema;
- physical R/L/flux-linkage measurement and Detect All;
- complete triggered/fault sample prebuffer;
- HFI/sensorless FOC path;
- upstream hardware watchdog/thread monitor.

Therefore V6 should be called **upstream-semantics aligned core**, not a byte-identical build of `vedderb/bldc`.
