# V10 Validation

## Purpose
Restore the exact V8 VESC Tool communication behavior while retaining the V9 motor-control/fixed-point implementation.

## Passed host checks
- V8 UART/packet files byte-identical.
- V10 dispatcher/reply behavior byte-identical to V8 after normalizing only the renamed duty API.
- Packet parser remains in packet_process_thread, never in UART DMA ISR.
- Detect commands remain on blocking_thread.
- Experimental V9 MCCONF/APPCONF command path is not exposed.
- V8 packet/framing regression tests PASS.
- FOC math host tests PASS.
- Python diagnostic self-test PASS.

## Deliberate recovery behavior
The wire firmware identity remains `hoverboard-vesc6-rtos-v8` so the first handshake is exactly the envelope already proven on the user's board. V10 is identified by the project archive/changelog, not by changing the working handshake bytes.

Full VESC 6.00 MCCONF/APPCONF will be reintroduced only after porting the exact serializer/deserializer from the proven reference and moving flash persistence out of packet_process_thread.
