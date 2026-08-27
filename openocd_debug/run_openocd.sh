#!/usr/bin/env bash
set -euo pipefail

# Run this script from the firmware project root.
CFG="openocd_debug/openocd.cfg"

if command -v openocd >/dev/null 2>&1; then
    exec openocd -f "$CFG"
fi

PIO_OCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
PIO_SCRIPTS="$HOME/.platformio/packages/tool-openocd/openocd/scripts"
if [[ -x "$PIO_OCD" ]]; then
    exec "$PIO_OCD" -s "$PIO_SCRIPTS" -f "$CFG"
fi

echo "ERROR: openocd not found." >&2
echo "Install OpenOCD or let PlatformIO install tool-openocd." >&2
exit 1
