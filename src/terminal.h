#pragma once

/* Reduced VESC terminal for STM32F103 hoverboard diagnostics. The command
 * transport is still COMM_TERMINAL_CMD / COMM_TERMINAL_CMD_SYNC and output is
 * standard COMM_PRINT. */
// Parameter str: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi terminal_process_string: melayani terminal process string sebagai diagnostik terminal tanpa menambah
// beban pada loop kontrol real-time.
void terminal_process_string(char *str);
