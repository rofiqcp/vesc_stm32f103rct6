# V5 UART IRQ Changelog

V5 dibuat dari V4 dengan satu fokus arsitektur: komunikasi VESC USART3 mengikuti pola RXNE/TXE software queue, bukan DMA UART.

## Perubahan source

### File baru

```text
src/vesc_uart.c
src/vesc_uart.h
```

Driver ini menyediakan:

```text
RX ring  = 256 byte
TX ring  = 2048 byte
RX wake  = VESC_RX_AVAILABLE
TX end   = VESC_TX_COMPLETE
```

### `src/stm32f1xx_it.c`

`USART3_IRQHandler()` sekarang:

```text
RXNE -> baca DR -> vesc_uart_rx_isr_put()
TXE  -> vesc_uart_tx_isr_get() -> tulis DR
TC   -> vesc_uart_tx_complete_isr()
```

ISR tidak menjalankan parser, CRC atau command decoder.

### `src/vesc_comm.c`

Dihapus:

```text
uart_process_thread
RX DMA pump
TX DMA owner
RTOS TX message queue
HAL UART TX/RX callback
```

`packet_process_thread` sekarang langsung drain RX ring dan memanggil `vesc_packet_process_byte()`.

Reply packet di-encode oleh `vesc_packet_encode()` kemudian seluruh frame dimasukkan ke TX ring dengan `vesc_uart_write_raw()`.

### `src/motor_hw.c/.h`

Dihapus seluruh resource DMA USART3:

```text
hdma_usart3_rx
hdma_usart3_tx
DMA1 Channel3 RX
DMA1 Channel2 TX
```

HAL UART masih dipakai hanya untuk konfigurasi USART3 115200 8N1. Transport byte ditangani manual oleh IRQ.

### Yang tetap

```text
DMA1 Channel1 ADC/FOC
TIM1/TIM8 PWM complementary
High-side active HIGH
Low-side active LOW
High-side idle LOW
Low-side idle HIGH
```

## Handshake final

```text
VESC Tool
  -> USB-UART TX
  -> PB11 USART3 RX
  -> RXNE IRQ
  -> RX software ring
  -> packet_process_thread
  -> packet parser + CRC
  -> commands/process payload
  -> COMM_FW_VERSION reply
  -> packet encoder
  -> TX software ring
  -> TXE IRQ
  -> PB10 USART3 TX
  -> TC IRQ
  -> USB-UART RX
  -> VESC Tool
```
