#!/usr/bin/env bash
set -euo pipefail

ELF=".pio/build/stm32f103rc/firmware.elf"
GDB_SCRIPT="openocd_debug/gdb_debug.gdb"

if [[ ! -f "$ELF" ]]; then
    echo "ERROR: $ELF not found. Build firmware first:" >&2
    echo "  pio run -e stm32f103rc" >&2
    exit 1
fi

if command -v arm-none-eabi-gdb >/dev/null 2>&1; then
    exec arm-none-eabi-gdb -x "$GDB_SCRIPT"
fi

for GDB in \
    "$HOME/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb" \
    "$HOME/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb-py"; do
    if [[ -x "$GDB" ]]; then
        exec "$GDB" -x "$GDB_SCRIPT"
    fi
done

echo "ERROR: arm-none-eabi-gdb not found." >&2
exit 1
