# VESC Tool Dual-Motor Basic Compatibility Hardening

Baseline: `vesc_stm32f103rct6_batch10_part2_adaptive_hall_inputmap_speedsource.zip`

## Goal

Make the basic VESC Tool connection model coherent before further algorithm porting:

1. connect over USART3 at 115200 using normal VESC framing/CRC;
2. root `COMM_FW_VERSION` identifies Motor-1 / ID 1;
3. normal VESC Tool `COMM_PING_CAN` discovery returns Motor-2 / ID 2;
4. selecting ID 2 in VESC Tool uses `COMM_FORWARD_CAN,2,<inner command>`;
5. forwarded replies are standard inner replies, as VESC Tool expects;
6. FW/version, telemetry, MCCONF and APPCONF are readable for both logical motors;
7. MCCONF writes are per-motor and APPCONF ownership remains shared but identity-safe.

## Important fix

Before this hardening, local Motor-2 forwarding existed but `COMM_PING_CAN` was not
implemented. VESC Tool uses `COMM_PING_CAN` for its CAN/device scan, so automatic
Motor-2 discovery could not be guaranteed. The root now returns exactly:

`[COMM_PING_CAN, 2]`

ID 1 is deliberately omitted because it is already the directly connected controller.
A forwarded ping to Motor-2 returns only `[COMM_PING_CAN]`, meaning no downstream nodes.

## Identity policy

- Motor-1: fixed controller ID 1.
- Motor-2: fixed controller ID 2.
- Motor-2 FW UUID differs from Motor-1 by one byte, matching upstream dual-motor logic.
- Motor-2 APPCONF reads with public controller ID 2.
- A Motor-2 APPCONF write is accepted only if the incoming image still requests ID 2;
  the image is then normalized internally to the shared application owner ID 1.
- Unsupported attempts to remap Motor-2 to another controller ID are rejected rather
  than acknowledged and silently discarded.

## Passive live checker

`tools/debug.py` now provides:

`vesc-tool-dual-basic`

It passively checks the same essential chain used by VESC Tool:

- M1 FW_VERSION / VESC 6.00 ABI;
- M1 GET_VALUES controller ID 1;
- PING_CAN returns exactly ID 2;
- M2 forwarded FW_VERSION / unique UUID;
- M2 GET_VALUES controller ID 2;
- M1/M2 MCCONF size/signature;
- M1/M2 APPCONF size/signature and controller IDs.

`can-scan` is also available to test only the discovery response.

## Scope limit

This proves source/host protocol coherence, not a physical UART or VESC Tool GUI test.
A real STM32F103 build and serial bench test are still required. No physical CAN,
CAN driver or CAN PHY was added.
