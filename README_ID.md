# V9 — V8 Proven Communication + Full Control/Config/Encoder + Fixed-Point FOC

V9 mempertahankan transport V8 yang sudah berhasil terhubung ke VESC Tool. Empat file transport (`vesc_uart.c/.h`, `vesc_packet.c/.h`) dibekukan dengan SHA256 dan diuji pada setiap audit V9.

Fokus V9 adalah melengkapi lapisan di atas komunikasi:

- standard `SET_DUTY`, `SET_CURRENT`, `SET_CURRENT_BRAKE`, `SET_RPM`, `SET_POS`, `SET_HANDBRAKE`, `SET_CURRENT_REL`;
- LEFT encoder AB TIM4 PB6/PB7 dengan no-index per-boot alignment, direction/wrap/CPR/ratio/offset/invert;
- Motor Config VESC 6.00 penuh 481 byte per motor;
- App Config VESC 6.00 penuh 493 byte;
- persistence flash transactional 4 x 2 KiB;
- fixed-point FOC Q15/Q16.16 dengan Q31 current-integrator, flash sine LUT interpolation, cached Q30 Vbus reciprocal, integer SVM;
- LEFT local ID1 + RIGHT virtual CAN ID2 tetap.

Baca `V9_FEATURE_COMPLETION.md` untuk detail implementasi dan batasan.

## Test host

```bash
./tests/host/run_v9.sh
./tests/host/run_v8.sh
sha256sum -c audit/V8_TRANSPORT_FROZEN.sha256
```

## Build target

```bash
pio run -t clean
pio run
pio run -t upload
```

`platformio.ini` menggunakan linker script `stm32f103rc_v9.ld` agar 8 KiB terakhir flash tidak dapat ditimpa image firmware.
