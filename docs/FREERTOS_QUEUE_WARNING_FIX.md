# FreeRTOS queue.c GCC 7.2.1 warning fix

PlatformIO `ststm32` with the bundled GCC 7.2.1 can emit:

`warning: argument 2 null where non-null expected [-Wnonnull]`

inside STM32CubeF1's upstream FreeRTOS `queue.c` while building
`xQueueCreateMutex()`. For mutex queues FreeRTOS uses an item size of zero, so
that source path intentionally permits a null item pointer with a zero-byte
copy. The warning is limited to the old compiler's static analysis of vendor
code.

This project excludes upstream `queue.c` from the manual FreeRTOS kernel build
in `tools/extra_script.py`. `src/freertos_vendor/queue_wrapper.c` then includes
the same upstream `queue.c` while locally disabling only `-Wnonnull` for that
single translation unit.

No FreeRTOS algorithm is modified and project/application warnings are not
suppressed globally.
