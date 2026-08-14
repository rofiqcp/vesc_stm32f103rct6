# V5 Validation

## Fokus revisi V5

V5 mengganti total transport UART VESC pada V4 dari DMA menjadi model interrupt + software queue:

```text
USART3 PB11 RX -> RXNE IRQ -> RX software ring -> packet_process_thread
packet reply -> TX software ring -> TXE IRQ -> USART3 PB10 TX -> TC IRQ
```

Konfigurasi tetap:

```text
USART3
TX = PB10
RX = PB11
115200 baud
8 data bit, no parity, 1 stop bit
```

DMA UART RX/TX tidak digunakan. DMA1 Channel1 tetap dipakai khusus ADC/FOC fast loop.

## Yang sudah diuji di environment pembuatan

1. Host packet test:
   - VESC short-frame encode/decode
   - CRC16/XMODEM vector `123456789 = 0x31C3`
2. Host FOC math test:
   - sin/cos Q15 checkpoints
   - zero-vector SVM
   - duty range
3. Python debug utility:
   - `py_compile`
   - `--self-test`
   - VESC FW_VERSION payload parser 7.01
   - streaming frame parser
   - sample parser
4. Source audit V5:
   - USART3 PB11 RX menggunakan RXNE interrupt
   - USART3 PB10 TX menggunakan TXE interrupt
   - TC interrupt menandai akhir transmisi fisik
   - parser/CRC/command tidak dijalankan di ISR
   - tidak ada `uart_process_thread`
   - tidak ada `HAL_UART_Transmit_DMA`
   - tidak ada `HAL_UART_Receive_DMA`
   - tidak ada `HAL_UARTEx_ReceiveToIdle_DMA`
   - tidak ada DMA1 Channel2/Channel3 handler untuk USART3
   - tidak ada `HAL_UART_IRQHandler` pada jalur VESC
   - fast FOC tetap berada di DMA1 Channel1 IRQ
   - PWM high-side active HIGH dan complementary low-side active LOW tetap dipertahankan dari V4

Host command:

```bash
bash tests/host/run.sh
```

Expected:

```text
test_packet: PASS
test_foc_math: PASS
SELF-TEST PASS: CRC, framing, VESC FW_VERSION parser, streaming parser, sample parser
host tests: ALL PASS
```

## ARM build status

Environment pembuatan artifact ini tidak memiliki PlatformIO / `arm-none-eabi-gcc`. Upaya mengambil PlatformIO juga tidak dapat dilakukan karena runtime tidak memiliki akses jaringan. Karena itu V5 **tidak diklaim sudah cross-compile** di environment ini.

Verifikasi pada PC STM32/PlatformIO pengguna:

```bash
pio run -t clean
pio run
```

Kemudian upload:

```bash
pio run -t upload
```

## Audit UART DMA

Perintah audit:

```bash
grep -R -n -E \
  "HAL_UART_Transmit_DMA|HAL_UART_Receive_DMA|HAL_UARTEx_ReceiveToIdle_DMA|USART_CR3_DMAR|USART_CR3_DMAT|hdma_usart3_rx|hdma_usart3_tx|DMA1_Channel2_IRQHandler|DMA1_Channel3_IRQHandler|uart_process_thread|HAL_UART_IRQHandler" \
  src
```

Target V5:

```text
tidak ada output
```

## Batas implementasi yang tidak diubah dari V4

- Sensor Hall/encoder runtime selection/detection: ada.
- Startup zero-current offset calibration: ada.
- Full physical R/L/flux-linkage VESC Detect All: belum selesai; blocking command tetap dikarantina di `blocking_thread` dan mengembalikan failure/sentinel, bukan angka palsu.
- Exact A/count dan V/count: harus disesuaikan dengan PCB.
- MCCONF/APPCONF binary compatibility penuh VESC Tool: belum diklaim.
