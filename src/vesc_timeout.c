#include "vesc_timeout.h"
#include "motor_control.h"
#include "app_config.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>

static volatile uint32_t s_timeout_ms = MOTOR_COMMAND_TIMEOUT_MS;
static volatile uint32_t s_last_update_ms = 0U;
static volatile float s_brake_current_a = 0.0f;
static volatile bool s_has_timeout = false;
static osThreadId_t s_timeout_thread;

static void timeout_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 10U;
        uint32_t now = osKernelGetTickCount();
        uint32_t timeout = s_timeout_ms;
        bool expired = timeout != 0U && (uint32_t)(now - s_last_update_ms) > timeout;

        if (expired) {
            if (!s_has_timeout) {
                s_has_timeout = true;
                g_motor_left.timeout_active = true;
                g_motor_right.timeout_active = true;
            }

            float brake = s_brake_current_a;
            if (fabsf(brake) > 0.001f) {
                /* Same policy as upstream timeout: apply the configured brake
                 * to both motor contexts. */
                motor_set_brake_current(&g_motor_left, fabsf(brake));
                motor_set_brake_current(&g_motor_right, fabsf(brake));
                /* Do not let the control setter reset the global timer. */
                g_motor_left.timeout_active = true;
                g_motor_right.timeout_active = true;
            } else {
                motor_stop(&g_motor_left);
                motor_stop(&g_motor_right);
                g_motor_left.timeout_active = true;
                g_motor_right.timeout_active = true;
            }
        } else {
            s_has_timeout = false;
            g_motor_left.timeout_active = false;
            g_motor_right.timeout_active = false;
        }

        osDelayUntil(next);
    }
}

void vesc_timeout_init(void) {
    s_last_update_ms = osKernelGetTickCount();
    s_has_timeout = false;
    const osThreadAttr_t attr = {
        .name = "timeout_thread",
        .priority = osPriorityHigh,
        .stack_size = 448U
    };
    s_timeout_thread = osThreadNew(timeout_thread, NULL, &attr);
    (void)s_timeout_thread;
}

void vesc_timeout_reset(void) {
    s_last_update_ms = osKernelGetTickCount();
    s_has_timeout = false;
    g_motor_left.timeout_active = false;
    g_motor_right.timeout_active = false;
}

void vesc_timeout_configure(uint32_t timeout_ms, float brake_current_a) {
    s_timeout_ms = timeout_ms;
    s_brake_current_a = brake_current_a;
}

bool vesc_timeout_has_timeout(void) { return s_has_timeout; }
uint32_t vesc_timeout_get_timeout_ms(void) { return s_timeout_ms; }

float vesc_timeout_get_brake_current(void) { return s_brake_current_a; }
