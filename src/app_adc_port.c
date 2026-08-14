#include "app_adc_port.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stddef.h>

/* This board revision does not expose dedicated throttle ADC inputs. The
 * upstream app_adc thread is nevertheless kept as an application-level task,
 * disabled by default. Integrators can feed two voltages through the setter
 * without mixing it with the motor-current ADC/FOC path. */
static volatile bool s_enabled = false;
static volatile float s_v1 = 0.0f;
static volatile float s_v2 = 0.0f;
static volatile float s_level1 = 0.0f;
static volatile float s_level2 = 0.0f;
static osThreadId_t s_adc_thread;

static float decode_0_3v3(float v) {
    float x = (v - 0.8f) / (3.0f - 0.8f);
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
}

static void adc_thread(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    for (;;) {
        next += 2U; /* 500 Hz application input update; motor FOC stays in ISR. */
        if (s_enabled) {
            s_level1 += 0.2f * (decode_0_3v3(s_v1) - s_level1);
            s_level2 += 0.2f * (decode_0_3v3(s_v2) - s_level2);
        } else {
            s_level1 = 0.0f;
            s_level2 = 0.0f;
        }
        osDelayUntil(next);
    }
}

void app_adc_port_init(void) {
    const osThreadAttr_t attr = {
        .name = "adc_thread",
        .priority = osPriorityNormal,
        .stack_size = 384U
    };
    s_adc_thread = osThreadNew(adc_thread, NULL, &attr);
    (void)s_adc_thread;
}

void app_adc_port_set_enabled(bool enabled) { s_enabled = enabled; }
void app_adc_port_set_inputs(float voltage1, float voltage2) { s_v1 = voltage1; s_v2 = voltage2; }
float app_adc_get_decoded_level(void) { return s_level1; }
float app_adc_get_decoded_level2(void) { return s_level2; }
float app_adc_get_voltage(void) { return s_v1; }
float app_adc_get_voltage2(void) { return s_v2; }
