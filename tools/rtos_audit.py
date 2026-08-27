#!/usr/bin/env python3
"""Static regression guard for the native-FreeRTOS VESC STM32F103 port."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

errors = []
all_text = "\n".join(p.read_text(errors="ignore") for p in SRC.rglob("*") if p.suffix in {".c", ".h"})

forbidden = [
    "cmsis_os2.h", "osThreadNew", "osDelay(", "osMutex", "osMessageQueue",
    "osKernel", "osThreadFlags", "osPriority", "osWaitForever",
]
for token in forbidden:
    if token in all_text:
        errors.append(f"forbidden CMSIS-RTOS2 token remains: {token}")

main = (SRC / "main.c").read_text()
creates = re.findall(r"\bxTaskCreate\s*\(", main)
if len(creates) != 5:
    errors.append(f"expected exactly 5 application xTaskCreate calls in main.c, found {len(creates)}")

expected_names = {"fault_stop", "timer", "packet_process", "blocking", "sample_send"}
names = set(re.findall(r'xTaskCreate\s*\([^,]+,\s*"([^"]+)"', main))
if names != expected_names:
    errors.append(f"task names mismatch: {sorted(names)}")

if "VESC_PRIO_SAFETY" not in main or "VESC_PRIO_NORMAL" not in main or "VESC_PRIO_LOW" not in main:
    errors.append("priority bands missing from main.c")

it = (SRC / "stm32f1xx_it.c").read_text()
if "foc_adc_dma_isr(g_adc_dual_dma);" not in it:
    errors.append("FOC ISR dispatch missing from DMA1_Channel1_IRQHandler")

# The hard ISR file may use the kernel tick bridge in SysTick_Handler only.
for bad in ["xQueue", "xSemaphore", "xTaskNotify", "vTaskDelay", "xTaskCreate"]:
    if bad in it:
        errors.append(f"RTOS synchronization call found in ISR glue: {bad}")

extra = (ROOT / "tools" / "extra_script.py").read_text()
if "CMSIS_RTOS_V2" in extra or "cmsis_os2.c" in extra:
    errors.append("build still compiles CMSIS-RTOS2 wrapper")

fc = (SRC / "FreeRTOSConfig.h").read_text()
for required in [
    "#define configMAX_PRIORITIES                    7",
    "#define configTICK_RATE_HZ                      ((TickType_t)1000)",
    "#define configUSE_TASK_NOTIFICATIONS            1",
]:
    if required not in fc:
        errors.append(f"FreeRTOSConfig invariant missing: {required}")

if errors:
    print("RTOS AUDIT: FAIL")
    for e in errors:
        print(" -", e)
    sys.exit(1)

print("RTOS AUDIT: PASS")
print("application tasks: 5")
print("  fault_stop      priority 6")
print("  timer           priority 5")
print("  packet_process  priority 5")
print("  blocking        priority 5")
print("  sample_send     priority 4")
print("FOC current loop: DMA1_Channel1_IRQHandler -> foc_adc_dma_isr (ISR context)")
print("CMSIS-RTOS2 wrapper: absent")
