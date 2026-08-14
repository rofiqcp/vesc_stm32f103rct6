# V8 — Audit jalur VESC Tool

## Referensi kerja

Transport dibandingkan terhadap branch `v20` dari:

`https://github.com/rofiqcp/hoverboard_vesc`

Referensi tersebut digunakan karena pada hardware keluarga hoverboard jalurnya sudah terbukti dapat membuka VESC Tool. V8 mempertahankan command/FOC/thread/persistence milik project ini, tetapi mengganti lapisan fisik USART V7.

## Call chain V8

```text
VESC Tool
  -> USB-UART
  -> PB11 USART3 RX
  -> DMA1 Channel3 circular (1024 B)
  -> packet_process_thread polls CNDTR
  -> vesc_packet_process_byte
  -> CRC16 / frame validation
  -> process_payload
  -> process_payload_for_motor(LEFT)
  -> commands
  -> reply payload
  -> VESC packet encode
  -> TX queue (6 frame)
  -> DMA1 Channel2
  -> USART3 DR
  -> PB10 TX
  -> USB-UART
  -> VESC Tool
```

RIGHT tidak mempunyai UART tersendiri:

```text
COMM_FORWARD_CAN + ID 2 + inner command
  -> process_payload_for_motor(RIGHT)
  -> normal inner-command reply
```

## Handshake

Request VESC FW version tetap:

```text
02 01 00 00 00 03
```

Payload reply V8 dimulai:

```text
00 06 00 "HOVERBOARD_DUAL_FOC" 00 ...
```

Field FW_VERSION awal disusun dengan urutan VESC 6.00 yang sama dengan firmware pembanding yang sudah connect. Nama firmware V8 sendiri adalah `hoverboard-vesc6-rtos-v8`.

## CPU liveness

Pada 64 MHz dan FOC 16 kHz terdapat sekitar 4000 CPU cycle per update. V8 memperbaiki perhitungan deadline lama yang salah membagi dua lagi. Jika setelah satu full FOC ISR event DMA berikutnya sudah pending, satu ISR berikutnya melakukan short safety pass (hard current + voltage protection) dan menghindari Park/PI/SVPWM/debug work. Tujuannya memberi window CPU kepada scheduler dan packet thread tanpa meniadakan proteksi listrik cepat.

## Power latch

PA5 HIGH dipertahankan sebagai power-hold. TIM1/TIM8 MOE tetap menjadi mekanisme enable/disable bridge. Ini mencegah PA5 ditarik LOW saat motor di-enable, yang berbeda dari perilaku firmware hoverboard pembanding.
