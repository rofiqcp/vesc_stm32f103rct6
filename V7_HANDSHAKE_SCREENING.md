# V7 — VESC Tool Handshake Screening

## Tujuan

V7 memprioritaskan satu target sebelum fitur lain: **VESC Tool harus menerima `COMM_FW_VERSION` yang valid melalui USART3 PB10/PB11 @ 115200 8N1**.

Jika handshake ini belum berhasil, MCCONF, RT Data, sample, detect, dan virtual CAN belum relevan karena VESC Tool belum menganggap perangkat sebagai VESC.

## Request pertama yang harus masuk

VESC Tool mengirim payload satu byte:

```text
COMM_FW_VERSION = 0x00
```

Dengan framing VESC, wire request-nya adalah:

```text
02 01 00 00 00 03
```

Arti:

```text
02       start, length 8-bit
01       payload length = 1
00       COMM_FW_VERSION
00 00    CRC16(payload 00)
03       stop
```

V7 harus membalas frame valid yang payload-nya mulai:

```text
00 07 01 ...
│  │  │
│  │  └─ FW minor 1
│  └──── FW major 7
└─────── COMM_FW_VERSION
```

Karena payload FW_VERSION V7 panjangnya 53 byte (`0x35`), waveform TX MCU yang sehat akan diawali:

```text
PB10 -> USB-UART RX
02 35 00 07 01 46 31 30 33 ...
      │  │  │  └────────────── HW_NAME mulai "F103..."
      │  │  └───────────────── FW minor
      │  └──────────────────── FW major
      └─────────────────────── COMM_FW_VERSION
```

CRC akhir berubah mengikuti UUID STM32, jadi yang perlu dicocokkan di logic analyzer adalah prefix dan framingnya, bukan CRC contoh statis.

## Perbedaan penting V6 -> V7

### 1. Communication-first boot

V6:

```text
clock
 -> motor_hw_init
 -> ADC/PWM init
 -> motor control/config
 -> RTOS objects
 -> ADC DMA/FOC start
 -> osKernelStart
 -> packet thread akhirnya berjalan
```

Jika init motor/ADC gagal atau fast ISR mengganggu CPU sebelum scheduler aktif, firmware tidak pernah membalas FW_VERSION.

V7:

```text
clock
 -> osKernelInitialize
 -> VESC packet/thread resources
 -> USART3 direct init
 -> create motor_boot_thread
 -> osKernelStart
      |
      +-> packet_process_thread (Normal)
      |
      +-> motor_boot_thread (BelowNormal)
             -> motor_hw_init
             -> motor/config/thread init
             -> ADC DMA/FOC start
```

Jadi parser VESC hidup terlebih dahulu dan motor boot tidak boleh mematikan communication stack.

### 2. USART3 tidak lagi melalui HAL UART protocol handler

V7 mengonfigurasi register USART3 mengikuti struktur `serial_lld.c` upstream:

```text
BRR = PCLK1 / 115200
CR2 = LINEN | LBDIE
CR3 = EIE
CR1 = UE | PEIE | RXNEIE | TE | RE
```

USART3 IRQ priority pada 115200 = 7.

PB11 RX menggunakan pull-up.

Tidak ada UART DMA.

### 3. RX ISR mengikuti loop upstream

V6 membaca satu RX byte, kemudian ketika error flag aktif melakukan SR->DR read kedua.

V7:

```text
read CR1
read SR

while RXNE / ORE / NE / FE / PE:
    record error bila ada
    read DR SATU KALI
    jika RXNE -> masukkan byte ke RX ring
    read SR lagi

TXE -> TX ring -> DR
queue kosong -> TXEIE off + TCIE on
TC -> clear TCIE + clear TC flag
```

Ini mencegah byte berikutnya ikut termakan oleh pembacaan DR kedua saat error/overrun.

### 4. Parser canonical upstream

Karena max payload 512 byte:

- start `0x02` dipakai untuk payload <= 255;
- start `0x03` dipakai untuk payload 255..512;
- `0x03` dengan length <255 ditolak;
- start `0x04` tidak diterima untuk build max-payload 512.

### 5. FW_VERSION reply diperkecil ke envelope upstream

Reply V7 memakai:

```text
major = 7
minor = 1
HW_TYPE = VESC (0)
HW_NAME = F103RC_DUAL
FW_NAME = F103_RTOS2_V7
```

Total payload FW_VERSION = 53 byte, masih di bawah buffer 65 byte yang digunakan current upstream `commands.c` untuk reply FW version.

## Screening setelah flash

Jangan mulai dari VESC Tool dulu. Jalankan raw handshake:

```bash
python3 debug_vesc_f103.py handshake \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --attempts 5
```

### Hasil PASS

```text
TX: 02 01 00 00 00 03
RX raw: 02 ... 03
PASS: framing + CRC + COMM_FW_VERSION reply valid
```

Setelah ini barulah buka VESC Tool 115200.

### LEVEL-1: RX raw = `<no bytes>`

Artinya masalah berada sebelum packet decoder:

```text
USB-UART TX
 -> PB11 RX
 -> USART3 RXNE IRQ
 -> RX ring
 -> packet thread
```

Periksa:

1. USB-UART TX -> PB11.
2. USB-UART RX <- PB10.
3. GND adapter dan board tersambung.
4. Logic level 3.3 V.
5. Board benar-benar menjalankan firmware baru.
6. HSE/system clock berhasil boot.
7. USART3 IRQ benar-benar masuk.
8. Tidak ada pin remap USART3 aktif.

### LEVEL-2: ada RX raw tetapi tidak ada valid frame

Fokus ke:

1. clock/baud mismatch;
2. TXE/TC IRQ;
3. corrupted bytes;
4. framing/CRC;
5. noise/grounding.

Jalankan juga:

```bash
python3 debug_vesc_f103.py baud-scan --port /dev/ttyUSB0
```

Jika baud lain justru menjawab, system clock/BRR harus diperiksa.

## Setelah handshake berhasil

Verifikasi local motor LEFT:

```bash
python3 debug_vesc_f103.py info --port /dev/ttyUSB0
```

Lalu virtual motor RIGHT:

```bash
python3 debug_vesc_f103.py can-scan --port /dev/ttyUSB0
```

Target:

```text
Virtual CAN IDs: [2]
RIGHT FW via COMM_FORWARD_CAN: ...
```

LEFT tetap local controller ID 1. RIGHT tetap virtual CAN controller ID 2; tidak ada CAN peripheral hardware.

## Batas validasi

Host tests dapat memverifikasi framing, CRC, parser, source architecture, dan routing virtual CAN. Host tests tidak dapat membuktikan bahwa PB10/PB11 pada board fisik benar-benar mempunyai waveform 115200. Jika `handshake` tetap LEVEL-1 setelah V7, langkah berikutnya adalah mengukur PB11 dan PB10 dengan logic analyzer/oscilloscope saat tool mengirim `02 01 00 00 00 03`.
