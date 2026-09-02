# VESC STM32F103RCT6 GDB helper commands.
# Loaded automatically by run_debug.sh after the ELF symbols are loaded.
set pagination off
set print pretty on
set print frame-arguments all
set breakpoint pending on
set remotetimeout 10

set $USART3_SR   = 0x40004800
set $USART3_BRR  = 0x40004808
set $USART3_CR1  = 0x4000480c
set $USART3_CR3  = 0x40004814
set $DMA1_ISR    = 0x40020000
set $DMA1_CH2    = 0x4002001c
set $DMA1_CH3    = 0x40020030

# Boot stages from src/main.c:
#   1 HAL, 10 clock/status, 20 task create, 30 VESC UART/commands,
#   40 motor_hw, 50 motor runtime, 60 config, 70 services,
#   80 task resources, 90 ADC sampling, 100 scheduler.
define vesc_boot
  printf "\n=== VESC BOOT ===\n"
  printf "stage=%lu error=%lu sampling_flags=0x%08lx\n", g_vesc_boot_stage, g_vesc_boot_error, g_vesc_sampling_contract_flags
  printf "task fault_stop     = %p\n", g_task_fault_stop
  printf "task timer          = %p\n", g_task_timer
  printf "task packet_process = %p\n", g_task_packet_process
  printf "task blocking       = %p\n", g_task_blocking
  printf "task sample_send    = %p\n", g_task_sample_send
end
document vesc_boot
Show boot stage/error and all five native-FreeRTOS application task handles.
end

define vesc_uart
  printf "\n=== VESC USART3 / DMA ===\n"
  printf "HAL UART gState=%u RxState=%u ErrorCode=0x%08lx\n", huart3_vesc.gState, huart3_vesc.RxState, huart3_vesc.ErrorCode
  printf "DMA TX State=%u ErrorCode=0x%08lx CNDTR=%lu\n", hdma_usart3_tx.State, hdma_usart3_tx.ErrorCode, *(unsigned long*)0x40020020
  printf "DMA RX State=%u ErrorCode=0x%08lx CNDTR=%lu\n", hdma_usart3_rx.State, hdma_usart3_rx.ErrorCode, *(unsigned long*)0x40020034
  printf "USART3 SR=0x%08lx BRR=0x%08lx CR1=0x%08lx CR3=0x%08lx\n", *(unsigned long*)$USART3_SR, *(unsigned long*)$USART3_BRR, *(unsigned long*)$USART3_CR1, *(unsigned long*)$USART3_CR3
  printf "DMA1 ISR=0x%08lx\n", *(unsigned long*)$DMA1_ISR
  p g_vesc_uart_stats
end
document vesc_uart
Show HAL UART/DMA state, USART3 registers, DMA CNDTR and firmware UART counters.
For RX DMA size 1024, CNDTR should change when bytes arrive on PB11.
end

define vesc_motor_left
  printf "\n=== MOTOR LEFT ===\n"
  p g_motor_left.state
  p g_motor_left.control_mode
  p g_motor_left.fault
  p g_motor_left.pwm_enabled
  p g_motor_left.sensor_mode
  p g_motor_left.foc_sensor_mode
  p g_motor_left.vbus
  p g_motor_left.erpm
  p g_motor_left.ia
  p g_motor_left.ib
  p g_motor_left.ic
  p g_motor_left.id_meas
  p g_motor_left.iq_meas
  p g_motor_left.id_target
  p g_motor_left.iq_target
  p g_motor_left.isr_max_cycles
  p g_motor_left.isr_overruns
  p g_motor_left.sampling_window_clamp_count
end

define vesc_motor_right
  printf "\n=== MOTOR RIGHT ===\n"
  p g_motor_right.state
  p g_motor_right.control_mode
  p g_motor_right.fault
  p g_motor_right.pwm_enabled
  p g_motor_right.sensor_mode
  p g_motor_right.foc_sensor_mode
  p g_motor_right.vbus
  p g_motor_right.erpm
  p g_motor_right.ia
  p g_motor_right.ib
  p g_motor_right.ic
  p g_motor_right.id_meas
  p g_motor_right.iq_meas
  p g_motor_right.id_target
  p g_motor_right.iq_target
  p g_motor_right.isr_max_cycles
  p g_motor_right.isr_overruns
  p g_motor_right.sampling_window_clamp_count
end

define vesc_motors
  vesc_motor_left
  vesc_motor_right
end


define vesc_sampling
  printf "\n=== ADC / PWM SAMPLING CONTRACT ===\n"
  printf "flags=0x%08lx (0 means valid)\n", g_vesc_sampling_contract_flags
  printf "TIM1 CR1=0x%08lx CR2=0x%08lx ARR=%lu RCR=%lu\n", TIM1->CR1, TIM1->CR2, TIM1->ARR, TIM1->RCR
  printf "TIM8 CR1=0x%08lx CR2=0x%08lx SMCR=0x%08lx ARR=%lu RCR=%lu\n", TIM8->CR1, TIM8->CR2, TIM8->SMCR, TIM8->ARR, TIM8->RCR
  printf "ADC1 SQR1=0x%08lx SQR3=0x%08lx CR1=0x%08lx CR2=0x%08lx\n", ADC1->SQR1, ADC1->SQR3, ADC1->CR1, ADC1->CR2
  printf "ADC2 SQR1=0x%08lx SQR3=0x%08lx CR1=0x%08lx CR2=0x%08lx\n", ADC2->SQR1, ADC2->SQR3, ADC2->CR1, ADC2->CR2
  printf "DMA1_CH1 CCR=0x%08lx CNDTR=%lu DMA2_CH5 CCR=0x%08lx CNDTR=%lu\n", DMA1_Channel1->CCR, DMA1_Channel1->CNDTR, DMA2_Channel5->CCR, DMA2_Channel5->CNDTR
end
document vesc_sampling
Show the exact ADC/PWM/DMA sampling contract state. flags=0 is required for motor-ready.
end

define vesc_tasks
  printf "\n=== FreeRTOS THREADS ===\n"
  printf "FreeRTOS thread awareness is meaningful only after boot stage 100.\n"
  info threads
end

define vesc_faults
  printf "\n=== FAULT SUMMARY ===\n"
  printf "boot stage=%lu error=%lu\n", g_vesc_boot_stage, g_vesc_boot_error
  p g_motor_left.fault
  p g_motor_right.fault
  monitor vesc_fault_hw
end


define vesc_comm_trace
  printf "\n=== VESC COMMAND / VIRTUAL-CAN TRACE ===\n"
  p g_vesc_comm_trace
  printf "packet_process stack free (words): "
  p g_vesc_packet_stack_free_words
end

define vesc_current_fault
  printf "\n=== CURRENT FAULT SNAPSHOT ===\n"
  p s_fault_snapshot
  p g_motor_left.abs_current_fault_count
  p g_motor_right.abs_current_fault_count
  p g_motor_left.abs_current_peak_q15
  p g_motor_right.abs_current_peak_q15
end

define vesc_buzzer
  printf "\n=== BUZZER / STARTUP MELODY ===\n"
  p g_vesc_buzzer_running
  p g_vesc_buzzer_hz
  p g_vesc_buzzer_remaining
  p g_vesc_startup_melody_active
  p g_vesc_startup_melody_index
end

define vesc_snapshot
  vesc_boot
  vesc_uart
  vesc_motors
  vesc_sampling
  vesc_comm_trace
  vesc_current_fault
  vesc_buzzer
  vesc_tasks
  monitor vesc_clock_hw
  monitor vesc_nvic_hw
  monitor vesc_fault_hw
end

document vesc_snapshot
One-stop halted snapshot: boot, UART/DMA, both motors, FreeRTOS tasks, clocks/NVIC/fault registers.
Do not repeatedly halt a powered running motor for snapshots.
end


define vesc_run
  printf "Resetting and running target...\n"
  monitor reset run
end
document vesc_run
Reset the STM32 and leave it running. Use this before exiting GDB if desired.
end

define vesc_break_comm
  thbreak packet_process_thread
  thbreak process_payload
end

define vesc_break_uart_tx
  thbreak DMA1_Channel2_IRQHandler
end

define vesc_break_foc
  printf "WARNING: DMA1_Channel1_IRQHandler is a 16-kHz FOC path. Keep power stage safe.\n"
  thbreak DMA1_Channel1_IRQHandler
end

printf "Loaded VESC GDB helpers: vesc_boot, vesc_uart, vesc_motors, vesc_sampling, vesc_comm_trace, vesc_current_fault, vesc_buzzer, vesc_tasks, vesc_faults, vesc_snapshot, vesc_run, vesc_break_comm, vesc_break_uart_tx, vesc_break_foc\n"
