import serial, time, struct

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def packet(cmd, payload=b""):
    data = bytes([cmd]) + payload
    crc = crc16(data)
    n = len(data)
    body = bytes([0x02, n]) + data if n < 256 else bytes([0x03,0x00]) + struct.pack(">H", n) + data
    return body + struct.pack(">H", crc) + bytes([0x03])

class Vesc:
    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.2)
        self.s.reset_input_buffer()
    def close(self): self.s.close()
    def _read_packet(self, timeout=2.0):
        # streaming parser: cari 0x02, len, crc, 0x03
        buf = b""
        start = time.time()
        while time.time() - start < timeout:
            if self.s.in_waiting:
                buf += self.s.read(self.s.in_waiting)
            i = buf.find(0x02)
            while i >= 0 and i+3 < len(buf):
                n = buf[i+1]
                if n < 254:
                    plen, idx = n, i+2
                else:
                    if i+5 >= len(buf): break
                    plen, idx = struct.unpack(">H", buf[i+2:i+4])[0], i+4
                if idx + plen + 3 > len(buf):
                    i = buf.find(0x02, i+1); continue
                payload = buf[idx:idx+plen]
                crcb = struct.unpack(">H", buf[idx+plen:idx+plen+2])[0]
                end = buf[idx+plen+2]
                if end == 0x03 and crc16(payload) == crcb:
                    self.s.reset_input_buffer()
                    return payload[0], payload[1:]
                i = buf.find(0x02, i+1)
            time.sleep(0.01)
        return None
    def send(self, cmd, payload=b"", expect=None, timeout=2.0):
        self.s.write(packet(cmd, payload))
        if expect is None: return None
        r = self._read_packet(timeout)
        if r is None: return None
        return r[1] if r[0]==expect else None
