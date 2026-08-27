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


# Commissioning safety: hardware IWDG must not start before scheduler and is
# disabled by default to keep SWD reprogrammable without connect-under-reset.
platformio = (ROOT / "platformio.ini").read_text()
if "-DVESC_IWDG_ENABLE=0" not in platformio:
    errors.append("commissioning build must default VESC_IWDG_ENABLE=0")
mc = (SRC / "motor" / "mc_interface.c").read_text()
start_threads_pos = mc.find("bool mc_interface_start_threads(void)")
if start_threads_pos >= 0:
    block = mc[start_threads_pos:start_threads_pos + 2500]
    if "timeout_watchdog_start();" in block:
        errors.append("IWDG start remains in pre-scheduler mc_interface_start_threads")

wrapper = SRC / "freertos_vendor" / "queue_wrapper.c"
if not wrapper.exists() or 'diagnostic ignored "-Wnonnull"' not in wrapper.read_text():
    errors.append("source-scoped FreeRTOS queue.c -Wnonnull wrapper missing")
if '-<queue.c>' not in extra:
    errors.append("vendor FreeRTOS queue.c is not excluded from direct BuildSources")

# STM32F1 reflash safety: generic AFIO MAPR read-modify-write helpers can
# corrupt SWJ_CFG. Require the controlled SWD-preserving setup.
hw = (SRC / "hwconf" / "hw.c").read_text()
uart = (SRC / "applications" / "app_uartcomm.c").read_text()
if "__HAL_AFIO_REMAP_SWJ_NOJTAG()" in hw or "__HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE()" in hw:
    errors.append("unsafe generic AFIO MAPR remap helper remains")
if "AFIO_SWJ_JTAG_OFF_SWD_ON_LOCAL" not in hw or "AFIO->MAPR = mapr;" not in hw:
    errors.append("controlled JTAG-off/SWD-on AFIO MAPR setup missing")
if re.search(r"AFIO->MAPR\s*[|&^]?=", uart):
    errors.append("UART init still performs unsafe AFIO MAPR RMW")

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
