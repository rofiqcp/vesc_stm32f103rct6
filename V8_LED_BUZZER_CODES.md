# V8 LED dan Buzzer Codes

## PB2 LED

| Kondisi | Pola |
|---|---|
| Awal boot sebelum RTOS | ON tetap |
| Program normal, belum ada VESC packet baru | 500 ms ON / 500 ms OFF |
| VESC packet valid dalam 1.5 s terakhir | double-flash: ON 100 ms, OFF 100 ms, ON 100 ms, lalu OFF |
| Salah satu motor fault | 100 ms ON / 100 ms OFF |

LED thread dibuat sebelum `motor_boot_thread`. Jadi kegagalan ADC/PWM/FOC tidak boleh menghilangkan indikator bahwa MCU + scheduler masih hidup.

## PA4 buzzer

Startup normal: tiga nada naik sekitar 900 Hz -> 1350 Hz -> 1900 Hz.

Fault diulang selama fault masih aktif:

| Fault | Beep | Pitch |
|---|---:|---:|
| command timeout internal | 1 | 1100 Hz |
| under-voltage | 2 | 900 Hz |
| over-voltage | 2 | 2600 Hz |
| absolute over-current | 3 | 3000 Hz |
| Hall invalid / sensor detect | 4 | 1700 Hz |
| current-offset / ADC DMA | 5 | 1200 Hz |
| FOC ISR extreme overrun | 6 | 2200 Hz |

Motor-fault buzzer tidak berada di FOC ISR. TIM3 hanya membentuk tone PA4; `buzzer_thread` mengatur urutan beep sehingga jalur kontrol motor tetap non-blocking.
