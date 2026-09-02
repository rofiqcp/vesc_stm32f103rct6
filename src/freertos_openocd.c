#include "FreeRTOS.h"

/* OpenOCD FreeRTOS thread-awareness compatibility symbol.
 *
 * STM32CubeF1 ships an older FreeRTOS kernel where uxTopUsedPriority may not
 * be exported. OpenOCD uses this value to size the ready-list scan. Keep this
 * symbol independent of the application task count; configMAX_PRIORITIES is
 * the kernel's canonical bound. The debug environment passes
 * --undefined=uxTopUsedPriority so --gc-sections cannot discard it. */
#if defined(__GNUC__)
__attribute__((used))
#endif
// Variabel uxTopUsedPriority: tingkat prioritas task atau interrupt.
const int uxTopUsedPriority = configMAX_PRIORITIES - 1;
