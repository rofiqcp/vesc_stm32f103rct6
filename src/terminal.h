#pragma once

/* Reduced VESC terminal for STM32F103 hoverboard diagnostics. The command
 * transport is still COMM_TERMINAL_CMD / COMM_TERMINAL_CMD_SYNC and output is
 * standard COMM_PRINT. */
void terminal_process_string(char *str);
