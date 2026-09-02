# VESC 6.00 RT-data, sensor-config, UART burst, and startup melody revision

This revision is intentionally aligned with the VESC 6.00 command/configuration ABI.

## Changes

- Keeps the native FreeRTOS five-application-task architecture.
- Keeps FOC in `DMA1_Channel1_IRQHandler -> foc_adc_dma_isr()`.
- Keeps SmartESC-style USART3 RX DMA circular/CNDTR polling in the packet task.
- Increases USART3 RX DMA storage from 512 to 1024 bytes so a 482-byte VESC 6.00
  `COMM_SET_MCCONF` frame plus nearby requests cannot overwrite a nearly-full
  circular receive buffer while the packet task is synchronously transmitting.
- `COMM_GET_VALUES` / `COMM_GET_VALUES_SELECTIVE` expose the VESC 6.00 fields
  current_motor, current_in, Id, Iq, Vd and Vq. Id/Iq/Vd/Vq are sourced directly
  from the fast FOC fixed-point state for the averaging path.
- The complete 481-byte VESC 6.00 MCCONF image is accepted/preserved instead of
  rejecting bytes that have no local F103 runtime consumer. Runtime code validates
  only values/enums it actually dereferences and keeps the bridge safety limits.
- `foc_sensor_mode` and `m_sensor_port_mode` are kept as independent VESC wire
  fields. The fixed PCB mux is derived locally from `foc_sensor_mode` rather than
  rewriting the VESC wire image.
- Added `tools/debug.py rt-data-abi` and step-by-step output in
  `vesc-tool-dual-basic`.
- Added an approximately three-second non-blocking VESC-inspired buzzer arpeggio.
  The power bridges remain off; the melody is serviced from the status state
  machine and does not delay the communication/FOC scheduler.

## Recommended passive checks after flashing

```bash
python3 tools/debug.py handshake --port /dev/ttyUSB0 --baud 115200 --attempts 5 --timeout 0.7
python3 tools/debug.py motor2-forward --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py rt-data-abi --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py vesc-tool-dual-basic --port /dev/ttyUSB0 --baud 115200
python3 tools/debug.py vesc-tool-check --port /dev/ttyUSB0 --baud 115200 --motor 0
```

For a byte-exact SET/GET MCCONF/APPCONF write-back test while both motors are
stopped, use `vesc-tool-check --write-back --yes`.
