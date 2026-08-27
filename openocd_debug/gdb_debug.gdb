# =============================================================================
# GDB companion script for openocd.cfg
# Run from firmware project root:
#   arm-none-eabi-gdb -x openocd_debug/gdb_debug.gdb
# =============================================================================

set pagination off
set confirm off
set print pretty on
set print elements 0
set remotetimeout 10
set breakpoint pending on
set disassemble-next-line on

# Exact ELF produced by this project's PlatformIO environment.
file .pio/build/stm32f103rc/firmware.elf

target extended-remote localhost:3333
monitor reset halt

# Cortex-M hardware vector catch for faults. Does not consume regular breakpoints.
monitor cortex_m vector_catch hard_err mm_err bus_err irq_err state_err chk_err nocp_err

# -----------------------------------------------------------------------------
# Utility commands
# -----------------------------------------------------------------------------

define rtos
    printf "\n=== FreeRTOS THREAD LIST ===\n"
    info threads
    printf "\n=== ALL THREAD BACKTRACES ===\n"
    thread apply all bt
end
document rtos
Show every FreeRTOS task known by OpenOCD and a backtrace for every task.
end

define regs
    printf "\n=== CPU REGISTERS ===\n"
    info registers
    printf "\n=== EXCEPTION NUMBER (IPSR = xPSR[8:0]) ===\n"
    p/x ($xpsr & 0x1ff)
end

define irqstate
    monitor vesc_nvic_state
end

define faultstate
    monitor vesc_fault_state
    printf "\n=== CURRENT BACKTRACE ===\n"
    bt full
    printf "\n=== CPU REGISTERS ===\n"
    info registers
end

define motorregs
    monitor vesc_motor_periph_state
end

define restart
    monitor reset halt
    tbreak main
    continue
end

define flash_firmware
    monitor reset halt
    load
    monitor reset halt
end
document flash_firmware
Flash the current .pio/build/stm32f103rc/firmware.elf through GDB/OpenOCD.
end

# -----------------------------------------------------------------------------
# Critical RTOS failure breakpoints (2 hardware breakpoints)
# Enable only when needed; command 'delete breakpoints' frees them.
# -----------------------------------------------------------------------------
define bp_rtos_faults
    hbreak vApplicationMallocFailedHook
    hbreak vApplicationStackOverflowHook
end

# -----------------------------------------------------------------------------
# APPLICATION THREAD ENTRY BREAKPOINT GROUPS
# The STM32F103 Cortex-M3 has a small FPB breakpoint budget, therefore groups are
# deliberately smaller than the complete thread set. These are temporary HW BPs.
# Reset-halt first, arm ONE group, then continue.
# -----------------------------------------------------------------------------
define bp_threads_boot_once
    thbreak status_thread
    thbreak packet_process_thread
    thbreak blocking_thread
    thbreak motor_boot_thread
end

define bp_threads_motor_once
    thbreak timer_thread
    thbreak sample_send_thread
    thbreak fault_stop_thread
end

# Observe scheduler activity with a single breakpoint. This is intrusive because
# context switching is frequent. Use only for short inspection, then delete it.
define bp_thread_create
    hbreak osThreadNew
end

define bp_scheduler
    hbreak vTaskSwitchContext
end

# -----------------------------------------------------------------------------
# ISR BREAKPOINT GROUPS
# Use ONE group at a time. Fast control ISR DMA1_Channel1 runs at ~16 kHz, so
# halting on it changes real-time behavior. 'thbreak' makes each stop one-shot.
# -----------------------------------------------------------------------------

# Priority 0 safety/current path
#   PVD_IRQn             prio 0
#   TIM1_BRK_IRQn        prio 0
#   TIM8_BRK_IRQn        prio 0
#   DMA2_Channel4_5_IRQn prio 0
#   DMA1_Channel1_IRQn   prio 0 (FOC ADC DMA fast path)
define bp_isr_safety_once
    thbreak PVD_IRQHandler
    thbreak TIM1_BRK_IRQHandler
    thbreak TIM8_BRK_IRQHandler
    thbreak DMA2_Channel4_5_IRQHandler
end

define bp_isr_foc_once
    thbreak DMA1_Channel1_IRQHandler
end

# Sensor / encoder path
#   EXTI9_5_IRQn     prio 3 : left hall
#   EXTI15_10_IRQn   prio 3 : right hall
#   TIM4_IRQn        prio 2 : encoder overflow/update
define bp_isr_sensor_once
    thbreak EXTI9_5_IRQHandler
    thbreak EXTI15_10_IRQHandler
    thbreak TIM4_IRQHandler
end

# UART + status path
#   DMA1_Channel2_IRQn prio 2 : USART3 TX DMA
#   DMA1_Channel3_IRQn prio 2 : USART3 RX DMA
#   USART3_IRQn        prio 2
#   TIM3_IRQn          prio 8 : status LED/buzzer timer
#   TIM2_IRQHandler is defensive-clear only in this firmware.
define bp_isr_comm_once
    thbreak DMA1_Channel2_IRQHandler
    thbreak DMA1_Channel3_IRQHandler
    thbreak USART3_IRQHandler
    thbreak TIM3_IRQHandler
    thbreak TIM2_IRQHandler
end

# RTOS/core exceptions. SysTick and PendSV are very frequent; each BP is one-shot.
define bp_isr_kernel_once
    thbreak SysTick_Handler
    thbreak SVC_Handler
    thbreak PendSV_Handler
end

# Optional Cortex-M exception handlers supplied weakly by the STM32 framework.
# Depending on the framework ELF they may resolve to Default_Handler.
define bp_isr_core_optional_once
    thbreak NMI_Handler
    thbreak DebugMon_Handler
end

# Project-defined fault handlers. Hardware vector catch is already active, so
# normally the CPU stops before/at exception handling. This group is useful when
# you specifically want source-level entry to these functions.
define bp_isr_fault_handlers_once
    thbreak HardFault_Handler
    thbreak MemManage_Handler
    thbreak BusFault_Handler
    thbreak UsageFault_Handler
end

# -----------------------------------------------------------------------------
# Frequently useful firmware internals called from ISRs
# -----------------------------------------------------------------------------
define bp_isr_helpers_once
    thbreak foc_adc_dma_isr
    thbreak motor_hall_edge_isr
    thbreak motor_request_fault_from_isr
    thbreak app_uartcomm_irq_handler
    thbreak app_uartcomm_dma_rx_irq_handler
    thbreak app_uartcomm_dma_tx_irq_handler
end

define bp_isr_helpers_safety_once
    thbreak motor_hw_pvd_irq_handler
    thbreak motor_hw_break_irq_handler
    thbreak hw_status_tim3_irq_handler
end

# -----------------------------------------------------------------------------
# One-shot boot walkthrough
# -----------------------------------------------------------------------------
define boot_debug
    monitor reset halt
    thbreak main
    continue
end

printf "\n============================================================\n"
printf "VESC STM32F103RCT6 GDB debug commands loaded\n"
printf "============================================================\n"
printf "Application threads from this firmware:\n"
printf "  StatusIO            priority Normal       stack 512 B\n"
printf "  uartcomm proc       priority High         stack 1536 B\n"
printf "  comm_block          priority Normal       stack 3072 B\n"
printf "  motor_boot_thread   priority BelowNormal  stack 2048 B (exits after boot)\n"
printf "  mc timer            priority High         stack 896 B\n"
printf "  mc sample           priority BelowNormal  stack 640 B\n"
printf "  mc fault            priority High         stack 512 B\n"
printf "  + FreeRTOS Idle and Timer service tasks\n"
printf "\nIRQ priorities configured by firmware:\n"
printf "  prio 0: DMA1_CH1, DMA2_CH4_5, PVD, TIM1_BRK, TIM8_BRK\n"
printf "  prio 2: DMA1_CH2, DMA1_CH3, USART3, TIM4\n"
printf "  prio 3: EXTI9_5, EXTI15_10\n"
printf "  prio 8: TIM3\n"
printf "  kernel: PendSV/SysTick at FreeRTOS lowest interrupt priority\n"
printf "\n"
printf "Core commands:\n"
printf "  boot_debug               reset -> stop once at main\n"
printf "  rtos                     list all FreeRTOS threads + backtraces\n"
printf "  regs                     CPU registers + active exception number\n"
printf "  irqstate                 NVIC enabled/pending/active/priorities\n"
printf "  faultstate               SCB fault regs + full backtrace\n"
printf "  motorregs                DMA/ADC/TIM/UART/EXTI register dump\n"
printf "  flash_firmware           flash current ELF\n"
printf "\nThread breakpoints (arm ONE group after reset-halt):\n"
printf "  bp_threads_boot_once     StatusIO / uartcomm / comm_block / motor_boot\n"
printf "  bp_threads_motor_once    mc timer / mc sample / mc fault\n"
printf "  bp_thread_create         stop at CMSIS osThreadNew\n"
printf "  bp_scheduler             stop at FreeRTOS context switch (intrusive)\n"
printf "\nISR one-shot groups:\n"
printf "  bp_isr_safety_once       PVD / TIM1_BRK / TIM8_BRK / DMA2_CH4_5\n"
printf "  bp_isr_foc_once          DMA1_CH1 fast FOC ADC ISR\n"
printf "  bp_isr_sensor_once       EXTI left/right hall + TIM4 encoder\n"
printf "  bp_isr_comm_once         UART DMA RX/TX + USART3 + TIM3 + TIM2\n"
printf "  bp_isr_kernel_once       SysTick / SVC / PendSV\n"
printf "  bp_isr_core_optional_once NMI / DebugMon framework handlers\n"
printf "  bp_isr_fault_handlers_once Hard/Mem/Bus/Usage fault handlers\n"
printf "  bp_isr_helpers_once      FOC/hall/UART ISR helper functions\n"
printf "  bp_isr_helpers_safety_once PVD/break/status helper functions\n"
printf "\nTarget is currently RESET + HALTED. Use 'continue' when ready.\n"
printf "============================================================\n\n"
