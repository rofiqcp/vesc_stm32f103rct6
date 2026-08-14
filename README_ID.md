# V8 — Hoverboard-Proven VESC UART Transport + 64 MHz + Status Indicators

V8 dibuat khusus karena V7 masih menghasilkan pesan VESC Tool:

`Could not read firmware version`

Perubahan V8 sengaja membatalkan eksperimen RXNE/TXE V7 untuk jalur normal. Transport UART sekarang mengikuti arsitektur yang sudah terbukti dapat terhubung pada `rofiqcp/hoverboard_vesc`:

- STM32F103RCT6 clock: HSI/2 x16 = 64 MHz.
- USART3: PB10 TX, PB11 RX, 115200 8N1.
- RX: DMA1 Channel3 circular 1024 byte, parser membaca posisi melalui CNDTR.
- TX: DMA1 Channel2, queue 6 frame, satu encoded VESC frame per transfer DMA.
- USART3 IRQ tidak dipakai pada normal transport.
- DMA1 Channel3 IRQ tidak dipakai pada normal RX.
- Parser dan command dispatcher tetap berjalan di CMSIS-RTOS2 `packet_process_thread`.
- COMM_FW_VERSION memakai field order VESC 6.00 yang telah terbukti pada firmware pembanding.
- LEFT tetap local controller ID 1.
- RIGHT tetap virtual CAN controller ID 2 melalui COMM_FORWARD_CAN/COMM_PING_CAN.
- Tidak ada CAN hardware fisik.

## Indikator board

PB2 hanya digunakan sebagai status LED firmware:

- sesaat setelah reset/awal boot: LED ON;
- runtime normal tanpa traffic VESC: heartbeat 1 Hz;
- frame VESC valid diterima: double-flash per detik;
- fault motor: fast blink 5 Hz.

PA4 digunakan untuk buzzer pasif. Setelah scheduler hidup terdengar tiga nada naik. Fault kemudian memakai pola beep yang dijelaskan di `V8_LED_BUZZER_CODES.md`.

PA5 diperlakukan sebagai power-hold/OFF latch dan selalu dipertahankan HIGH selama firmware hidup. Enable bridge tidak lagi memakai PA5; TIM1/TIM8 MOE yang mengendalikan output bridge.

## Test pertama

Setelah upload, perhatikan urutan ini sebelum membuka VESC Tool:

1. PB2 langsung ON ketika firmware masuk main.
2. Terdengar tiga nada naik setelah RTOS scheduler berjalan.
3. PB2 mulai heartbeat.
4. Jalankan:

```bash
python3 debug_vesc_f103.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 10
```

Jika handshake valid, LED berubah menjadi pola double-flash karena frame VESC valid telah diterima.

Kemudian buka VESC Tool pada port yang sama, 115200 baud.


# V10 Communication Recovery

V10 deliberately freezes the complete V8 VESC Tool communication behavior, including dispatcher and FW_VERSION reply. V9 fixed-point motor-control code remains, but experimental full MCCONF/APPCONF command handling and V9 config import at boot are isolated until the VESC 6.00 serializer is ported byte-for-byte from the proven reference.
