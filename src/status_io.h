#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Board-liveness indicators, intentionally independent of motor PWM/ADC. */
void status_io_early_gpio_init(void);
void status_io_tone_timer_init(void);
void status_io_led(bool on);
void status_io_tone_start(uint16_t hz);
void status_io_tone_stop(void);
void status_io_note_vesc_packet(void);
bool status_io_vesc_link_recent(uint32_t age_ms);
void status_io_early_fatal_loop(void) __attribute__((noreturn));
void status_io_tim3_irq_handler(void);
