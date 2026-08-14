# V7 Handshake-First Changelog

V7 dibuat khusus setelah VESC Tool masih menampilkan `Could not read firmware version` pada V6.

Perubahan inti:

- communication stack dibuat hidup sebelum motor/ADC/FOC boot;
- USART3 dipindahkan dari HAL UART init menjadi direct-register init dengan semantics `serial_lld.c` VESC;
- USART3 priority 115200 diubah 6 -> 7;
- PB11 RX diberi pull-up;
- RX IRQ sekarang menguras RX/error dalam loop dan hanya membaca DR sekali per SR snapshot;
- TC flag dibersihkan eksplisit;
- parser length dibuat canonical seperti upstream `packet.c`;
- FW_VERSION response diperkecil menjadi 53-byte payload dan tetap menggunakan FW 7.1 / HW_TYPE_VESC;
- motor hardware failure tidak lagi memanggil global `__disable_irq()` sehingga VESC communication tetap dapat hidup;
- parser/resource/thread creation diperiksa return handle-nya;
- packet/blocking thread dipetakan ke Normal priority seperti upstream, motor boot helper berada BelowNormal;
- raw handshake tool sekarang melakukan beberapa attempt dan mengklasifikasikan LEVEL-1/LEVEL-2;
- LEFT local + RIGHT virtual CAN ID 2 tetap dipertahankan.
