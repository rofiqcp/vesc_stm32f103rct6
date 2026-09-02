/*
 * FreeRTOS queue.c build wrapper for the STM32CubeF1 FreeRTOS version.
 *
 * GCC 7.2.1 (the toolchain bundled with the selected PlatformIO platform)
 * emits -Wnonnull in xQueueCreateMutex() for the upstream zero-length
 * mutex queue copy path. FreeRTOS intentionally permits pvItemToQueue == NULL
 * when uxItemSize == 0, therefore this warning is a compiler false-positive
 * in vendor code, not a firmware logic error.
 *
 * Keep the diagnostic suppression local to this one vendor translation unit;
 * project/application sources still compile with their normal warning policy.
 */
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#endif

#include "queue.c"

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
