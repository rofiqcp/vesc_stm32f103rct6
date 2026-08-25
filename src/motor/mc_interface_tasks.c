#include "motor/mc_interface.h"

#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "comm/commands.h"
#include "hwconf/hw.h"
#include "motor/mc_interface_sample.h"
#include "motor/mcpwm_foc.h"
#include "telemetry.h"
#include "timeout.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

/*
 * Upstream VESC owns the service, sample sender and fault-stop threads in
 * mc_interface.c. This F103 port keeps their implementation in this private
 * translation unit to keep the already large compatibility layer reviewable.
 * The public API and ownership still remain in mc_interface.
 */
#define RTOS_READY_HEAP_RESERVE_BYTES 2048U

static osThreadId_t s_timer_thread;
static osThreadId_t s_sample_send_thread;
static osThreadId_t s_fault_stop_thread;
static bool s_threads_started;

static void timer_thread(void *argument);
static void sample_send_thread(void *argument);
static void fault_stop_thread(void *argument);

static uint32_t thread_stack_free_bytes(osThreadId_t thread) {
	if (thread == NULL) {
		return 0U;
	}
	return (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)thread) *
			(uint32_t)sizeof(StackType_t);
}

static void fault_signal(motor_id_t motor) {
	if (s_fault_stop_thread != NULL) {
		(void)osThreadFlagsSet(s_fault_stop_thread, 1UL << (uint32_t)motor);
	}
}

static void sample_signal(void) {
	if (s_sample_send_thread != NULL) {
		(void)osThreadFlagsSet(s_sample_send_thread, 1UL);
	}
}

uint32_t mc_interface_free_heap_bytes(void) {
	return (uint32_t)xPortGetFreeHeapSize();
}

uint32_t mc_interface_min_ever_free_heap_bytes(void) {
	return (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

void mc_interface_get_resource_stats(mc_interface_resource_stats_t *stats) {
	if (stats == NULL) {
		return;
	}

	stats->heap_free_bytes = mc_interface_free_heap_bytes();
	stats->heap_min_ever_bytes = mc_interface_min_ever_free_heap_bytes();
	stats->motor_service_stack_free_bytes =
			thread_stack_free_bytes(s_timer_thread);
	stats->sample_sender_stack_free_bytes =
			thread_stack_free_bytes(s_sample_send_thread);
	stats->fault_stack_free_bytes =
			thread_stack_free_bytes(s_fault_stop_thread);
	stats->status_stack_free_bytes = hw_status_stack_free_bytes();
}

static void timer_thread(void *argument) {
	(void)argument;
	bool current_offset_fault_reported = false;
	uint8_t calibration_divider = 0U;
	uint8_t ten_ms_divider = 0U;
	uint32_t next = osKernelGetTickCount();

	for (;;) {
		next += 1U;
		const uint32_t now = osKernelGetTickCount();
		timeout_heartbeat(TIMEOUT_HEARTBEAT_MOTOR_SERVICE);

		motor_slow_update_1khz(&g_motor_left, now);
		motor_slow_update_1khz(&g_motor_right, now);
		motor_rpm_update_1khz(&g_motor_left);
		motor_rpm_update_1khz(&g_motor_right);

		/* APP ADC and serial commands share this central command arbitration. */
		app_command_service_1khz(now);
		app_adc_service_1khz(now);

		motor_pid_update_1khz(&g_motor_left);
		motor_pid_update_1khz(&g_motor_right);

		ten_ms_divider++;
		if (ten_ms_divider >= 10U) {
			ten_ms_divider = 0U;
			timeout_update_10ms(now);
			timeout_watchdog_update_10ms(now);
			telemetry_stats_update_100hz();
			telemetry_snapshot_100hz();
			vesc_comm_periodic_100hz();
		}

		/* The ISR only accumulates fixed-point calibration statistics. */
		calibration_divider++;
		if (calibration_divider >= 5U) {
			calibration_divider = 0U;
			foc_calibration_service_task();
			if (foc_calibration_done()) {
				if (!foc_calibration_valid() && !current_offset_fault_reported) {
					current_offset_fault_reported = true;
					motor_raise_fault_from_task(&g_motor_left,
							MOTOR_FAULT_CURRENT_OFFSET);
					motor_raise_fault_from_task(&g_motor_right,
							MOTOR_FAULT_CURRENT_OFFSET);
					fault_signal(MOTOR_LEFT);
					fault_signal(MOTOR_RIGHT);
				} else if (foc_calibration_valid()) {
					current_offset_fault_reported = false;
					if (g_motor_left.fault == MOTOR_FAULT_CURRENT_OFFSET) {
						motor_clear_fault(&g_motor_left);
					}
					if (g_motor_right.fault == MOTOR_FAULT_CURRENT_OFFSET) {
						motor_clear_fault(&g_motor_right);
					}
				}
			}
		}

		const uint32_t pending = motor_take_pending_fault_mask();
		if ((pending & (1UL << MOTOR_LEFT)) != 0U) {
			fault_signal(MOTOR_LEFT);
		}
		if ((pending & (1UL << MOTOR_RIGHT)) != 0U) {
			fault_signal(MOTOR_RIGHT);
		}

		if (mc_interface_sample_ready()) {
			sample_signal();
		}
		osDelayUntil(next);
	}
}

static void sample_send_thread(void *argument) {
	(void)argument;
	for (;;) {
		const uint32_t flags = osThreadFlagsWait(1UL, osFlagsWaitAny,
				osWaitForever);
		if ((flags & osFlagsError) != 0U) {
			continue;
		}
		if (mc_interface_sample_ready()) {
			vesc_comm_send_sample_buffer(mc_interface_sample_data(),
					mc_interface_sample_count());
			mc_interface_sample_mark_sent();
		}
	}
}

static void fault_stop_thread(void *argument) {
	(void)argument;
	const uint32_t mask = (1UL << MOTOR_LEFT) | (1UL << MOTOR_RIGHT);

	for (;;) {
		const uint32_t flags = osThreadFlagsWait(mask, osFlagsWaitAny, 50U);
		timeout_heartbeat(TIMEOUT_HEARTBEAT_FAULT);
		if ((flags & osFlagsError) != 0U) {
			continue;
		}
		if ((flags & (1UL << MOTOR_LEFT)) != 0U) {
			motor_hw_set_pwm_enabled(&g_motor_left, false);
		}
		if ((flags & (1UL << MOTOR_RIGHT)) != 0U) {
			motor_hw_set_pwm_enabled(&g_motor_right, false);
		}
	}
}

bool mc_interface_start_threads(void) {
	if (s_threads_started) {
		return s_timer_thread != NULL && s_sample_send_thread != NULL &&
				s_fault_stop_thread != NULL;
	}
	s_threads_started = true;

	app_command_init();
	app_adc_init();
	if (!timeout_init()) {
		return false;
	}

	const osThreadAttr_t timer_attributes = {
		.name = "mc timer",
		.priority = osPriorityHigh,
		.stack_size = 896U
	};
	const osThreadAttr_t sample_attributes = {
		.name = "mc sample",
		.priority = osPriorityBelowNormal,
		.stack_size = 640U
	};
	const osThreadAttr_t fault_attributes = {
		.name = "mc fault",
		.priority = osPriorityHigh,
		.stack_size = 512U
	};

	s_timer_thread = osThreadNew(timer_thread, NULL, &timer_attributes);
	s_sample_send_thread = osThreadNew(sample_send_thread, NULL,
			&sample_attributes);
	s_fault_stop_thread = osThreadNew(fault_stop_thread, NULL,
			&fault_attributes);

	const bool threads_ok = s_timer_thread != NULL &&
			s_sample_send_thread != NULL && s_fault_stop_thread != NULL;
	const bool heap_ok = mc_interface_free_heap_bytes() >=
			RTOS_READY_HEAP_RESERVE_BYTES;
	const bool ready = threads_ok && heap_ok;
	if (ready) {
		timeout_watchdog_start();
	}
	return ready;
}
