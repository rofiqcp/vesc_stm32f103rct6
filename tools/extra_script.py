Import("env")
import os

framework = env.PioPlatform().get_package_dir("framework-stm32cubef1")
if not framework:
    raise RuntimeError("framework-stm32cubef1 package not found")

fr = os.path.join(framework, "Middlewares", "Third_Party", "FreeRTOS", "Source")
cmsis2 = os.path.join(fr, "CMSIS_RTOS_V2")
portable = os.path.join(fr, "portable", "GCC", "ARM_CM3")
memmang = os.path.join(fr, "portable", "MemMang")

env.Append(CPPPATH=[
    fr,
    os.path.join(fr, "include"),
    cmsis2,
    portable,
])

env.BuildSources(os.path.join("$BUILD_DIR", "freertos_kernel"), fr,
                 src_filter=["+<*.c>"])
env.BuildSources(os.path.join("$BUILD_DIR", "freertos_port"), portable,
                 src_filter=["+<port.c>"])
env.BuildSources(os.path.join("$BUILD_DIR", "freertos_heap"), memmang,
                 src_filter=["+<heap_4.c>"])
env.BuildSources(os.path.join("$BUILD_DIR", "cmsis_rtos2"), cmsis2,
                 src_filter=["+<cmsis_os2.c>"])
