Import("env")
import os

framework = env.PioPlatform().get_package_dir("framework-stm32cubef1")
if not framework:
    raise RuntimeError("framework-stm32cubef1 package not found")

fr = os.path.join(framework, "Middlewares", "Third_Party", "FreeRTOS", "Source")
portable = os.path.join(fr, "portable", "GCC", "ARM_CM3")
memmang = os.path.join(fr, "portable", "MemMang")

# Add the STM32CubeF1 FreeRTOS include paths. queue.c itself is intentionally
# excluded below and is compiled through src/freertos_vendor/queue_wrapper.c.
# That wrapper suppresses GCC 7.2.1's vendor-only false-positive -Wnonnull
# without hiding the diagnostic from the rest of this firmware.
env.Append(CPPPATH=[
    fr,
    os.path.join(fr, "include"),
    portable,
])

# Native FreeRTOS kernel. Do not compile upstream queue.c here, otherwise it
# would be built twice (and the GCC 7.2.1 warning would reappear).
env.BuildSources(
    os.path.join("$BUILD_DIR", "freertos_kernel"),
    fr,
    src_filter=["+<*.c>", "-<queue.c>"]
)

env.BuildSources(
    os.path.join("$BUILD_DIR", "freertos_port"),
    portable,
    src_filter=["+<port.c>"]
)

env.BuildSources(
    os.path.join("$BUILD_DIR", "freertos_heap"),
    memmang,
    src_filter=["+<heap_4.c>"]
)
