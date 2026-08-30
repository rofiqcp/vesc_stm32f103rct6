#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TIMEOUT_HEARTBEAT_FOC = 0,
    TIMEOUT_HEARTBEAT_MOTOR_SERVICE = 1,
    TIMEOUT_HEARTBEAT_COMM = 2,
    TIMEOUT_HEARTBEAT_FAULT = 3,
    TIMEOUT_HEARTBEAT_COUNT = 4
} timeout_heartbeat_id_t;

/* Capture RCC reset flags early in main(), before they are cleared. */
void timeout_capture_reset_reason(void);
uint32_t timeout_get_reset_flags(void);
bool timeout_had_iwdg_reset(void);

bool timeout_init(void);
void timeout_update_10ms(uint32_t now_ms);
void timeout_reset(void);
void timeout_configure(uint32_t timeout_ms, float brake_current_a);
bool timeout_has_timeout(void);
uint32_t timeout_get_timeout_ms(void);
float timeout_get_brake_current(void);

/* Hardware watchdog / liveness gate. The watchdog is started only after the
 * service/communication threads exist. FOC becomes a required heartbeat only
 * after synchronized ADC sampling has actually started. */
void timeout_watchdog_start(void);
void timeout_watchdog_require_foc(bool required);
void timeout_watchdog_update_10ms(uint32_t now_ms);
void timeout_heartbeat(timeout_heartbeat_id_t id);
void timeout_heartbeat_from_isr(timeout_heartbeat_id_t id);
bool timeout_watchdog_started(void);
bool timeout_watchdog_healthy(void);
uint32_t timeout_watchdog_required_mask(void);
uint32_t timeout_watchdog_unhealthy_mask(void);
uint32_t timeout_watchdog_miss_count(timeout_heartbeat_id_t id);
uint32_t timeout_heartbeat_count(timeout_heartbeat_id_t id);
