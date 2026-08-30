# Sumber protokol

Implementasi ini ditulis ulang dalam Python berdasarkan kode firmware resmi VESC:

- `comm/packet.c`: framing paket, panjang, CRC, dan stop byte  
  https://github.com/vedderb/bldc/blob/master/comm/packet.c
- `util/crc.c`: CRC16 polynomial `0x1021`  
  https://github.com/vedderb/bldc/blob/master/util/crc.c
- `util/buffer.c`: integer big-endian dan angka berskala/float-auto  
  https://github.com/vedderb/bldc/blob/master/util/buffer.c
- `comm/commands.c`: isi request/reply dan skala tiap field  
  https://github.com/vedderb/bldc/blob/master/comm/commands.c
- `datatypes.h`: nomor `COMM_PACKET_ID` dan fault code  
  https://github.com/vedderb/bldc/blob/master/datatypes.h

Tidak ada kode `pyvesc`, `PyCRC`, atau `pyserial` yang diimpor. Implementasi ini
memakai modul standard library Python saja.
