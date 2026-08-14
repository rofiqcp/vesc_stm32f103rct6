#include "vesc_comm.h"
#include "vesc_uart.h"
#include "status_io.h"
#include "vesc_packet.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "telemetry.h"
#include "sensor_detect.h"
#include "debug_sample.h"
#include "foc_control.h"
#include "app_config.h"
#include "app_adc_port.h"
#include "vesc_timeout.h"
#include "config_store.h"
#include "vesc_config.h"
#include "cmsis_os2.h"
#include <string.h>
#include <math.h>

#define BLOCK_QUEUE_DEPTH 1U
#define BLOCK_DATA_MAX    VESC_PACKET_MAX_PAYLOAD

/* Command numbers are kept aligned with current vedderb/bldc datatypes.h for
 * the subset this reduced STM32F103 FOC port intentionally implements. */
enum {
    COMM_FW_VERSION = 0,
    COMM_GET_VALUES = 4,
    COMM_SET_DUTY = 5,
    COMM_SET_CURRENT = 6,
    COMM_SET_CURRENT_BRAKE = 7,
    COMM_SET_RPM = 8,
    COMM_SET_POS = 9,
    COMM_SET_HANDBRAKE = 10,
    COMM_SET_DETECT = 11,
    COMM_SET_MCCONF = 13,
    COMM_GET_MCCONF = 14,
    COMM_GET_MCCONF_DEFAULT = 15,
    COMM_SET_APPCONF = 16,
    COMM_GET_APPCONF = 17,
    COMM_GET_APPCONF_DEFAULT = 18,
    COMM_SAMPLE_PRINT = 19,
    COMM_PRINT = 21,
    COMM_ROTOR_POSITION = 22,
    COMM_DETECT_MOTOR_PARAM = 24,
    COMM_DETECT_MOTOR_R_L = 25,
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26,
    COMM_DETECT_ENCODER = 27,
    COMM_DETECT_HALL_FOC = 28,
    COMM_REBOOT = 29,
    COMM_ALIVE = 30,
    COMM_GET_DECODED_ADC = 32,
    COMM_FORWARD_CAN = 34,
    COMM_CUSTOM_APP_DATA = 36,
    COMM_GET_VALUES_SETUP = 47,
    COMM_GET_VALUES_SELECTIVE = 50,
    COMM_GET_VALUES_SETUP_SELECTIVE = 51,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57,
    COMM_DETECT_APPLY_ALL_FOC = 58,
    COMM_PING_CAN = 62,
    COMM_SET_CURRENT_REL = 84
};

enum {
    CUSTOM_SELECT_MOTOR = 0xA0,
    CUSTOM_DUAL_SUMMARY = 0xA1,
    CUSTOM_CLEAR_FAULT = 0xA2,
    CUSTOM_STOP = 0xA3,
    CUSTOM_SENSOR_SELECT = 0xA4,
    CUSTOM_SENSOR_DETECT = 0xA5,
    CUSTOM_CURRENT_CAL = 0xA6,
    CUSTOM_SAMPLE_START = 0xA7,
    CUSTOM_EXT_TELEMETRY = 0xA8,
    CUSTOM_SENSOR_INFO = 0xA9,
    CUSTOM_COMM_DIAG = 0xAA,
    CUSTOM_CONFIG_SAVE = 0xAB,
    CUSTOM_CONFIG_STATUS = 0xAC
};

typedef struct {
    uint8_t cmd;
    uint8_t motor;
    uint16_t len;
    uint8_t data[BLOCK_DATA_MAX];
} blocking_job_t;

typedef struct {
    volatile uint32_t rx_frames_ok;
    volatile uint32_t tx_frames;
    volatile uint32_t blocking_busy_drops;
    volatile uint32_t virtual_can_forwards;
    volatile uint32_t virtual_can_unknown_ids;
} comm_diag_t;

static vesc_packet_parser_t s_parser;
static osMessageQueueId_t s_block_queue;
static osMutexId_t s_payload_mutex;
static osMutexId_t s_send_mutex;
static volatile uint8_t s_display_mode[2];
static uint8_t s_tx_payload[VESC_PACKET_MAX_PAYLOAD];
static uint8_t s_tx_frame[VESC_PACKET_BUFFER_SIZE];
static comm_diag_t s_diag;
static vesc_appdata_handler_t s_appdata_handler = NULL;
static volatile bool s_motor_ready = false;

static void process_payload(const uint8_t *data, uint16_t len);
static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id);
static void packet_process_thread(void *argument);
static void blocking_thread(void *argument);
static void vesc_comm_reply_diag(void);

static int32_t get_i32_be(const uint8_t *p) {
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}
static uint16_t get_u16_be(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static uint32_t get_u32_be(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static void put_i16(uint8_t *b, uint16_t *i, int16_t v) {
    uint16_t u = (uint16_t)v; b[(*i)++] = (uint8_t)(u >> 8); b[(*i)++] = (uint8_t)u;
}
static void put_u16(uint8_t *b, uint16_t *i, uint16_t v) {
    b[(*i)++] = (uint8_t)(v >> 8); b[(*i)++] = (uint8_t)v;
}
static void put_i32(uint8_t *b, uint16_t *i, int32_t v) {
    uint32_t u = (uint32_t)v;
    b[(*i)++] = (uint8_t)(u >> 24); b[(*i)++] = (uint8_t)(u >> 16);
    b[(*i)++] = (uint8_t)(u >> 8); b[(*i)++] = (uint8_t)u;
}
static void put_u32(uint8_t *b, uint16_t *i, uint32_t v) { put_i32(b, i, (int32_t)v); }

/* Fully-defined float32-auto representation used by VESC sample/plot packets. */
static void put_float32_auto(uint8_t *b, uint16_t *i, float number) {
    if (fabsf(number) < 1.5e-38f) number = 0.0f;
    int exp = 0;
    float sig = frexpf(number, &exp);
    float mag = fabsf(sig);
    uint32_t fraction = 0U;
    if (mag >= 0.5f) {
        fraction = (uint32_t)((mag - 0.5f) * 16777216.0f);
        exp += 126;
    }
    uint32_t bits = (((uint32_t)exp & 0xFFU) << 23) | (fraction & 0x7FFFFFU);
    if (sig < 0.0f) bits |= 0x80000000UL;
    put_u32(b, i, bits);
}

static int16_t scaled_i16(float v, float scale) {
    float x = v * scale;
    if (x > 32767.0f) x = 32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)lroundf(x);
}
static int32_t scaled_i32(float v, float scale) {
    double x = (double)v * (double)scale;
    if (x > 2147483647.0) x = 2147483647.0;
    if (x < -2147483648.0) x = -2147483648.0;
    return (int32_t)llround(x);
}

static uint8_t vesc_fault_code(uint8_t native_fault) {
    switch ((motor_fault_t)native_fault) {
        case MOTOR_FAULT_NONE:             return 0U;
        case MOTOR_FAULT_OVER_VOLTAGE:     return 1U;
        case MOTOR_FAULT_UNDER_VOLTAGE:    return 2U;
        case MOTOR_FAULT_ABS_OVER_CURRENT: return 4U;
        case MOTOR_FAULT_CURRENT_OFFSET:   return 15U;
        case MOTOR_FAULT_HALL_INVALID:
        case MOTOR_FAULT_SENSOR_DETECT:    return 27U;
        case MOTOR_FAULT_COMMAND_TIMEOUT:  return 0U;
        case MOTOR_FAULT_ADC_DMA:
        case MOTOR_FAULT_FOC_ISR_OVERRUN:
        default:                           return 3U;
    }
}

static uint8_t controller_id_for_motor(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? VESC_CONTROLLER_ID_RIGHT : VESC_CONTROLLER_ID_LEFT;
}

static uint8_t *payload_begin(void) {
    if (s_payload_mutex != NULL && osMutexAcquire(s_payload_mutex, osWaitForever) != osOK) return NULL;
    return s_tx_payload;
}
static void payload_end(uint16_t len) {
    vesc_comm_send_payload(s_tx_payload, len);
    if (s_payload_mutex != NULL) (void)osMutexRelease(s_payload_mutex);
}

static void send_print(const char *msg) {
    if (msg == NULL) return;
    uint8_t p[160];
    uint16_t i = 0U;
    p[i++] = COMM_PRINT;
    size_t n = strlen(msg);
    if (n > sizeof(p) - 2U) n = sizeof(p) - 2U;
    memcpy(&p[i], msg, n); i = (uint16_t)(i + n); p[i++] = 0U;
    vesc_comm_send_payload(p, i);
}

static void reply_fw_version(motor_id_t id) {
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U;

    /* Match the proven hoverboard_vesc VESC-6.00 handshake field order. The
     * transport and this minimal first reply are deliberately conservative;
     * richer RT data/config support remains implemented by this firmware. */
    p[i++] = COMM_FW_VERSION;
    p[i++] = 6U;
    p[i++] = 0U;

    const char hw[] = "HOVERBOARD_DUAL_FOC";
    memcpy(&p[i], hw, sizeof(hw));
    i = (uint16_t)(i + sizeof(hw));

    const uint32_t *uid = (const uint32_t *)0x1FFFF7E8UL;
    for (uint8_t w = 0U; w < 3U; w++) {
        uint32_t v = uid[w];
        p[i++] = (uint8_t)v;
        p[i++] = (uint8_t)(v >> 8);
        p[i++] = (uint8_t)(v >> 16);
        p[i++] = (uint8_t)(v >> 24);
    }
    if (id == MOTOR_RIGHT) p[i - 1U]++;

    p[i++] = 1U; /* pairing done */
    p[i++] = 0U; /* FW test version */
    p[i++] = 0U; /* HW_TYPE_VESC */
    p[i++] = 0U; /* custom configs */
    p[i++] = 0U; /* phase filters */
    p[i++] = 0U; /* qml hw */
    p[i++] = 0U; /* qml app */
    p[i++] = 0U; /* nrf flags */

    const char fw[] = "hoverboard-vesc6-rtos-v8";
    memcpy(&p[i], fw, sizeof(fw));
    i = (uint16_t)(i + sizeof(fw));

    payload_end(i);
}

static void append_get_values_fields(uint8_t *p, uint16_t *i,
                                     const motor_telemetry_t *t, uint32_t mask) {
    if (mask & (1UL << 0)) put_i16(p, i, 0); /* NTC excluded by requirement */
    if (mask & (1UL << 1)) put_i16(p, i, 0);
    if (mask & (1UL << 2)) put_i32(p, i, scaled_i32(t->current_motor, 100.0f));
    if (mask & (1UL << 3)) put_i32(p, i, scaled_i32(t->current_in, 100.0f));
    if (mask & (1UL << 4)) put_i32(p, i, scaled_i32(t->id_filter, 100.0f));
    if (mask & (1UL << 5)) put_i32(p, i, scaled_i32(t->iq_filter, 100.0f));
    if (mask & (1UL << 6)) put_i16(p, i, scaled_i16(t->duty, 1000.0f));
    if (mask & (1UL << 7)) put_i32(p, i, scaled_i32(t->erpm, 1.0f));
    if (mask & (1UL << 8)) put_i16(p, i, scaled_i16(t->vbus, 10.0f));
    if (mask & (1UL << 9)) put_i32(p, i, scaled_i32(t->amp_hours, 10000.0f));
    if (mask & (1UL << 10)) put_i32(p, i, scaled_i32(t->amp_hours_charged, 10000.0f));
    if (mask & (1UL << 11)) put_i32(p, i, scaled_i32(t->watt_hours, 10000.0f));
    if (mask & (1UL << 12)) put_i32(p, i, scaled_i32(t->watt_hours_charged, 10000.0f));
    if (mask & (1UL << 13)) put_i32(p, i, t->tachometer);
    if (mask & (1UL << 14)) put_i32(p, i, t->tachometer_abs);
    if (mask & (1UL << 15)) p[(*i)++] = vesc_fault_code(t->fault);
    if (mask & (1UL << 16)) put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if (mask & (1UL << 17)) p[(*i)++] = t->controller_id;
    if (mask & (1UL << 18)) { put_i16(p, i, 0); put_i16(p, i, 0); put_i16(p, i, 0); }
    if (mask & (1UL << 19)) put_i32(p, i, scaled_i32(t->vd, 1000.0f));
    if (mask & (1UL << 20)) put_i32(p, i, scaled_i32(t->vq, 1000.0f));
    if (mask & (1UL << 21)) p[(*i)++] = vesc_timeout_has_timeout() ? 1U : 0U;
}

static void reply_get_values(uint8_t command, uint32_t mask, motor_id_t id) {
    motor_telemetry_t t; telemetry_get(id, &t);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = command;
    if (command == COMM_GET_VALUES_SELECTIVE) put_u32(p, &i, mask);
    append_get_values_fields(p, &i, &t, mask);
    payload_end(i);
}

static void append_setup_fields(uint8_t *p, uint16_t *i,
                                const motor_telemetry_t *t, uint32_t mask) {
    if (mask & (1UL << 0)) put_i16(p, i, 0);
    if (mask & (1UL << 1)) put_i16(p, i, 0);
    if (mask & (1UL << 2)) put_i32(p, i, scaled_i32(t->current_motor, 100.0f));
    if (mask & (1UL << 3)) put_i32(p, i, scaled_i32(t->current_in, 100.0f));
    if (mask & (1UL << 4)) put_i16(p, i, scaled_i16(t->duty, 1000.0f));
    if (mask & (1UL << 5)) put_i32(p, i, scaled_i32(t->erpm, 1.0f));
    if (mask & (1UL << 6)) put_i32(p, i, 0); /* speed m/s unknown */
    if (mask & (1UL << 7)) put_i16(p, i, scaled_i16(t->vbus, 10.0f));
    if (mask & (1UL << 8)) put_i16(p, i, 0); /* battery level unknown */
    if (mask & (1UL << 9)) put_i32(p, i, scaled_i32(t->amp_hours, 10000.0f));
    if (mask & (1UL << 10)) put_i32(p, i, scaled_i32(t->amp_hours_charged, 10000.0f));
    if (mask & (1UL << 11)) put_i32(p, i, scaled_i32(t->watt_hours, 10000.0f));
    if (mask & (1UL << 12)) put_i32(p, i, scaled_i32(t->watt_hours_charged, 10000.0f));
    if (mask & (1UL << 13)) put_i32(p, i, 0); /* distance */
    if (mask & (1UL << 14)) put_i32(p, i, 0); /* distance abs */
    if (mask & (1UL << 15)) put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if (mask & (1UL << 16)) p[(*i)++] = vesc_fault_code(t->fault);
    if (mask & (1UL << 17)) p[(*i)++] = t->controller_id;
    if (mask & (1UL << 18)) p[(*i)++] = 2U; /* local + virtual-CAN right */
    if (mask & (1UL << 19)) put_i32(p, i, 0); /* Wh battery left unknown */
    if (mask & (1UL << 20)) put_u32(p, i, 0U); /* odometer */
    if (mask & (1UL << 21)) put_u32(p, i, osKernelGetTickCount());
}

static void reply_setup_values(uint8_t command, uint32_t mask, motor_id_t id) {
    motor_telemetry_t t; telemetry_get(id, &t);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = command;
    if (command == COMM_GET_VALUES_SETUP_SELECTIVE) put_u32(p, &i, mask);
    append_setup_fields(p, &i, &t, mask);
    payload_end(i);
}

static void reply_decoded_adc(void) {
    uint8_t p[17]; uint16_t i = 0U;
    p[i++] = COMM_GET_DECODED_ADC;
    put_i32(p, &i, scaled_i32(app_adc_get_decoded_level(), 1000000.0f));
    put_i32(p, &i, scaled_i32(app_adc_get_voltage(), 1000000.0f));
    put_i32(p, &i, scaled_i32(app_adc_get_decoded_level2(), 1000000.0f));
    put_i32(p, &i, scaled_i32(app_adc_get_voltage2(), 1000000.0f));
    vesc_comm_send_payload(p, i);
}

static void send_rotor_position(motor_id_t id) {
    uint8_t mode = s_display_mode[(id == MOTOR_RIGHT) ? 1U : 0U];
    if (mode == DISP_POS_MODE_NONE) return;
    motor_telemetry_t t; telemetry_get(id, &t);
    float pos = t.rotor_elec_deg;
    if (mode == DISP_POS_MODE_ENCODER || mode == DISP_POS_MODE_PID_POS) pos = t.position_deg;
    if (mode == DISP_POS_MODE_PID_POS_ERROR) {
        MotorRuntime *m = motor_get(id);
        pos = m->position_target_deg - t.position_deg;
        while (pos > 180.0f) pos -= 360.0f;
        while (pos < -180.0f) pos += 360.0f;
    }
    uint8_t p[5]; uint16_t i = 0U;
    p[i++] = COMM_ROTOR_POSITION; put_i32(p, &i, scaled_i32(pos, 100000.0f));
    vesc_comm_send_payload(p, i);
}

static void reply_custom_summary(void) {
    motor_telemetry_t l, r; telemetry_get(MOTOR_LEFT, &l); telemetry_get(MOTOR_RIGHT, &r);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_DUAL_SUMMARY; p[i++] = 4U;
    const motor_telemetry_t *arr[2] = {&l, &r};
    for (uint8_t k = 0U; k < 2U; k++) {
        const motor_telemetry_t *t = arr[k];
        p[i++] = k; p[i++] = t->controller_id; p[i++] = t->sensor_mode;
        p[i++] = t->fault; p[i++] = t->sensor_detect_state;
        put_i32(p, &i, scaled_i32(t->erpm, 1.0f));
        put_i32(p, &i, scaled_i32(t->iq, 100.0f));
        put_i32(p, &i, scaled_i32(t->id, 100.0f));
        put_i16(p, &i, scaled_i16(t->vbus, 10.0f));
    }
    put_u32(p, &i, l.isr_max_cycles); put_u32(p, &i, l.isr_overruns);
    p[i++] = l.calibration_done; p[i++] = l.calibration_valid;
    p[i++] = vesc_timeout_has_timeout() ? 1U : 0U;
    payload_end(i);
}

static void reply_extended(motor_id_t id) {
    motor_telemetry_t t; telemetry_get(id, &t); MotorRuntime *m = motor_get(id);
    uint32_t cal_count = 0U, cal_target = 0U; foc_get_calibration_progress(&cal_count, &cal_target);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_EXT_TELEMETRY;
    p[i++] = (uint8_t)id; p[i++] = 4U; p[i++] = t.controller_id; p[i++] = t.sensor_mode;
    p[i++] = t.fault; p[i++] = t.sensor_detect_state; p[i++] = t.calibration_done; p[i++] = t.calibration_valid;
    p[i++] = m->hall.raw_state; p[i++] = m->pole_pairs; p[i++] = m->encoder.inverted ? 1U : 0U;
    put_i32(p, &i, scaled_i32(t.id, 1000.0f)); put_i32(p, &i, scaled_i32(t.iq, 1000.0f));
    put_i32(p, &i, scaled_i32(t.id_filter, 1000.0f)); put_i32(p, &i, scaled_i32(t.iq_filter, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vd, 1000.0f)); put_i32(p, &i, scaled_i32(t.vq, 1000.0f));
    put_i32(p, &i, scaled_i32(t.current_motor, 1000.0f)); put_i32(p, &i, scaled_i32(t.current_in, 1000.0f));
    put_i32(p, &i, scaled_i32(t.erpm, 1.0f)); put_i32(p, &i, scaled_i32(t.mech_rpm, 10.0f));
    put_i32(p, &i, scaled_i32(t.position_deg, 1000.0f)); put_i32(p, &i, scaled_i32(t.rotor_elec_deg, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vbus, 1000.0f)); put_i32(p, &i, scaled_i32(t.duty, 100000.0f));
    put_i32(p, &i, (int32_t)t.current_offset_u); put_i32(p, &i, (int32_t)t.current_offset_v);
    put_i32(p, &i, (int32_t)t.dc_current_offset); put_u32(p, &i, cal_count); put_u32(p, &i, cal_target);
    put_u32(p, &i, t.isr_max_cycles); put_u32(p, &i, t.isr_overruns);
    put_i32(p, &i, m->encoder.extended_count); put_u16(p, &i, m->encoder.elec_offset_u16);
    payload_end(i);
}

static void reply_sensor_info(motor_id_t id) {
    MotorRuntime *m = motor_get(id);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_SENSOR_INFO; p[i++] = (uint8_t)id;
    p[i++] = controller_id_for_motor(id); p[i++] = m->sensor_mode; p[i++] = m->sensor_request_mode;
    p[i++] = (uint8_t)m->detect.state; p[i++] = m->detect.success ? 1U : 0U; p[i++] = m->pole_pairs;
    p[i++] = m->encoder.inverted ? 1U : 0U; put_u16(p, &i, m->encoder.elec_offset_u16);
    for (uint8_t k = 0U; k < 8U; k++) p[i++] = m->foc_hall_table[k];
    for (uint8_t k = 0U; k < 8U; k++) put_u16(p, &i, m->hall_angle_u16[k]);
    payload_end(i);
}

static void reply_current_cal(void) {
    uint32_t count = 0U, target = 0U; foc_get_calibration_progress(&count, &target);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_CURRENT_CAL;
    p[i++] = foc_calibration_done() ? 1U : 0U; p[i++] = foc_calibration_valid() ? 1U : 0U;
    put_u32(p, &i, count); put_u32(p, &i, target);
    put_i32(p, &i, g_motor_left.current_offset_u_counts); put_i32(p, &i, g_motor_left.current_offset_v_counts);
    put_i32(p, &i, g_motor_left.dc_current_offset_counts); put_i32(p, &i, g_motor_right.current_offset_u_counts);
    put_i32(p, &i, g_motor_right.current_offset_v_counts); put_i32(p, &i, g_motor_right.dc_current_offset_counts);
    /* Optional V11 hardware-timing diagnostics. They are after the legacy
       current-cal fields so older host tools can ignore them safely. */
    put_u32(p, &i, foc_adc_isr_count());
    put_u16(p, &i, (uint16_t)DMA1_Channel1->CNDTR);
    put_u16(p, &i, (uint16_t)TIM2->CNT);
    p[i++] = (TIM1->CR1 & TIM_CR1_DIR) ? 1U : 0U;
    p[i++] = (TIM1->CR1 & TIM_CR1_CEN) ? 1U : 0U;
    p[i++] = (TIM8->CR1 & TIM_CR1_CEN) ? 1U : 0U;
    p[i++] = (TIM2->CR1 & TIM_CR1_CEN) ? 1U : 0U;
    p[i++] = (ADC1->CR2 & ADC_CR2_ADON) ? 1U : 0U;
    p[i++] = (ADC2->CR2 & ADC_CR2_ADON) ? 1U : 0U;
    p[i++] = ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U) ? 1U : 0U;

    /* V12 detailed calibration diagnostics. Keep legacy fields first. */
    foc_cal_diag_t cd; foc_get_calibration_diag(&cd);
    p[i++] = 12U; /* calibration diagnostic revision */
    put_u16(p, &i, cd.warn_mask);
    put_u16(p, &i, cd.fail_range_mask);
    put_u16(p, &i, cd.fail_noise_mask);
    for (uint8_t k=0U;k<6U;k++) {
        put_i32(p, &i, cd.ch[k].mean);
        put_u16(p, &i, cd.ch[k].min);
        put_u16(p, &i, cd.ch[k].max);
        put_u32(p, &i, cd.ch[k].variance_x100);
    }
    /* Register snapshots make TIM2/ADC/DMA activity auditable from one txt. */
    put_u32(p, &i, RCC->CFGR);
    put_u32(p, &i, ADC1->CR1); put_u32(p, &i, ADC1->CR2); put_u32(p, &i, ADC1->SQR1); put_u32(p, &i, ADC1->SQR3);
    put_u32(p, &i, ADC2->CR1); put_u32(p, &i, ADC2->CR2); put_u32(p, &i, ADC2->SQR1); put_u32(p, &i, ADC2->SQR3);
    put_u32(p, &i, DMA1_Channel1->CCR); put_u32(p, &i, DMA1_Channel1->CNDTR); put_u32(p, &i, DMA1->ISR);
    put_u32(p, &i, TIM1->CR1); put_u32(p, &i, TIM1->ARR); put_u32(p, &i, TIM1->CNT); put_u32(p, &i, TIM1->BDTR);
    put_u32(p, &i, TIM8->CR1); put_u32(p, &i, TIM8->ARR); put_u32(p, &i, TIM8->CNT); put_u32(p, &i, TIM8->BDTR);
    put_u32(p, &i, TIM2->CR1); put_u32(p, &i, TIM2->SMCR); put_u32(p, &i, TIM2->CCR2); put_u32(p, &i, TIM2->CNT);
    for (uint8_t k=0U;k<6U;k++) put_u32(p, &i, g_adc_dual_dma[k]);
    payload_end(i);
}

static void reply_config_status(bool last_save_ok) {
    uint8_t p[16]; uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_CONFIG_STATUS;
    p[i++] = config_store_valid() ? 1U : 0U; p[i++] = last_save_ok ? 1U : 0U;
    put_u32(p, &i, config_store_save_count()); put_u32(p, &i, vesc_timeout_get_timeout_ms());
    vesc_comm_send_payload(p, i);
}

static void process_custom(const uint8_t *data, uint16_t len, motor_id_t context) {
    /* Upstream COMMANDS_APP_DATA callback semantics: `data` starts at the
     * application payload, i.e. COMM_CUSTOM_APP_DATA itself is stripped. */
    if (data == NULL || len < 1U) return;
    uint8_t sub = data[0];
    motor_id_t explicit_id = context;
    if (len >= 2U && data[1] <= 1U) explicit_id = data[1] ? MOTOR_RIGHT : MOTOR_LEFT;

    if (sub == CUSTOM_SELECT_MOTOR) {
        /* Legacy diagnostic command retained only for host scripts. It does
         * not change the standard UART-local motor; COMM_FORWARD_CAN does. */
        reply_sensor_info(explicit_id);
    } else if (sub == CUSTOM_DUAL_SUMMARY) {
        reply_custom_summary();
    } else if (sub == CUSTOM_CLEAR_FAULT && len >= 2U) {
        motor_clear_fault(motor_get(explicit_id));
    } else if (sub == CUSTOM_STOP && len >= 2U) {
        motor_stop(motor_get(explicit_id));
    } else if ((sub == CUSTOM_SENSOR_SELECT || sub == CUSTOM_SENSOR_DETECT) && len >= 3U) {
        MotorRuntime *m = motor_get(explicit_id); uint8_t mode = data[2];
        if (sub == CUSTOM_SENSOR_DETECT || mode == SENSOR_MODE_AUTO) (void)sensor_detect_request(m, mode);
        else (void)motor_select_sensor_mode(m, mode);
        reply_sensor_info(m->id);
    } else if (sub == CUSTOM_CURRENT_CAL) {
        if (len >= 2U && data[1] == 1U) {
            motor_stop(&g_motor_left); motor_stop(&g_motor_right); foc_request_recalibration();
        }
        reply_current_cal();
    } else if (sub == CUSTOM_SAMPLE_START && len >= 6U) {
        uint16_t count = get_u16_be(&data[2]); uint16_t decimation = get_u16_be(&data[4]);
        debug_sample_start(explicit_id, count, decimation);
    } else if (sub == CUSTOM_EXT_TELEMETRY && len >= 2U) {
        reply_extended(explicit_id);
    } else if (sub == CUSTOM_SENSOR_INFO && len >= 2U) {
        reply_sensor_info(explicit_id);
    } else if (sub == CUSTOM_COMM_DIAG) {
        vesc_comm_reply_diag();
    } else if (sub == CUSTOM_CONFIG_SAVE) {
        bool ok = config_store_save_all(); reply_config_status(ok);
    } else if (sub == CUSTOM_CONFIG_STATUS) {
        reply_config_status(false);
    }
}

static bool is_blocking_command(uint8_t cmd) {
    switch (cmd) {
        case COMM_DETECT_MOTOR_PARAM:
        case COMM_DETECT_MOTOR_R_L:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_ENCODER:
        case COMM_DETECT_HALL_FOC:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
        case COMM_DETECT_APPLY_ALL_FOC:
        case COMM_PING_CAN:
        case COMM_SET_MCCONF:
        case COMM_SET_APPCONF:
            return true;
        default:
            return false;
    }
}

static bool queue_blocking_job(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len == 0U || s_block_queue == NULL || len > BLOCK_DATA_MAX) return false;
    blocking_job_t job;
    memset(&job, 0, sizeof(job));
    job.cmd = data[0]; job.motor = (uint8_t)id; job.len = len; memcpy(job.data, data, len);
    if (osMessageQueuePut(s_block_queue, &job, 0U, 0U) != osOK) {
        s_diag.blocking_busy_drops++;
        return false;
    }
    return true;
}

static void reply_config_wire(uint8_t cmd, motor_id_t id, bool defaults) {
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    p[0] = cmd;
    if (cmd == COMM_GET_MCCONF || cmd == COMM_GET_MCCONF_DEFAULT) {
        const uint8_t *w = vesc_config_mc_wire(id, defaults);
        memcpy(&p[1], w, VESC6_MCCONF_WIRE_SIZE);
        payload_end((uint16_t)(1U + VESC6_MCCONF_WIRE_SIZE));
    } else {
        const uint8_t *w = vesc_config_app_wire(defaults);
        memcpy(&p[1], w, VESC6_APPCONF_WIRE_SIZE);
        /* VESC dual-motor semantics expose the second motor with its own
           controller ID even though this port has one physical app instance.
           APPCONF signature occupies bytes 0..3 and controller_id is byte 4. */
        if (id == MOTOR_RIGHT) p[1U + 4U] = VESC_VIRTUAL_CAN_RIGHT_ID;
        payload_end((uint16_t)(1U + VESC6_APPCONF_WIRE_SIZE));
    }
}

static void reply_ack(uint8_t cmd) {
    uint8_t p[1] = {cmd};
    vesc_comm_send_payload(p, 1U);
}

static void reply_standard_detect(MotorRuntime *m, uint8_t command) {
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = command;
    if (command == COMM_DETECT_HALL_FOC) {
        bool ok = m->detect.success && m->sensor_mode == SENSOR_MODE_HALL;
        for (uint8_t k = 0U; k < 8U; k++) p[i++] = ok ? m->foc_hall_table[k] : 255U;
        p[i++] = ok ? 0U : 1U;
    } else if (command == COMM_DETECT_ENCODER) {
        bool ok = m->detect.success && m->sensor_mode == SENSOR_MODE_ENCODER;
        if (ok) {
            float offset_deg = ((float)m->encoder.elec_offset_u16 * 360.0f) / 65536.0f;
            put_i32(p, &i, scaled_i32(offset_deg, 1000000.0f));
            put_i32(p, &i, scaled_i32((float)m->pole_pairs, 1000000.0f));
            p[i++] = m->encoder.inverted ? 1U : 0U;
        } else {
            put_i32(p, &i, 1001000000L); put_i32(p, &i, 0); p[i++] = 0U;
        }
    }
    payload_end(i);
}

static void reply_unsupported_detect(uint8_t cmd) {
    uint8_t p[32]; uint16_t i = 0U; p[i++] = cmd;
    switch (cmd) {
        case COMM_DETECT_MOTOR_PARAM:
            put_i32(p, &i, 0); put_i32(p, &i, 0); for (uint8_t k = 0U; k < 8U; k++) p[i++] = 255U; p[i++] = 1U; break;
        case COMM_DETECT_MOTOR_R_L:
            put_i32(p, &i, 0); put_i32(p, &i, 0); put_i32(p, &i, 0); break;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
            put_i32(p, &i, 0); break;
        case COMM_DETECT_APPLY_ALL_FOC:
            put_i16(p, &i, -1); break;
        default: break;
    }
    vesc_comm_send_payload(p, i);
    send_print("F103 V8: R/L/flux auto-detect is intentionally unsupported; no fabricated result.");
}

static bool ensure_current_calibration_valid(uint32_t timeout_ms) {
    /* VESC detect-all performs DC offset calibration before reading/applying
       motor parameters. Do the same for every motor-moving detect job. */
    if (foc_calibration_valid()) return true;
    if (foc_calibration_done()) foc_request_recalibration();
    uint32_t start = osKernelGetTickCount();
    while ((uint32_t)(osKernelGetTickCount() - start) < timeout_ms) {
        if (foc_calibration_done()) return foc_calibration_valid();
        osDelay(5U);
    }
    return false;
}

static bool wait_sensor_detect(MotorRuntime *m, uint32_t timeout_ms) {
    uint32_t start = osKernelGetTickCount();
    while ((uint32_t)(osKernelGetTickCount() - start) < timeout_ms) {
        if (m->detect.state == SENSOR_DETECT_DONE) return true;
        if (m->detect.state == SENSOR_DETECT_FAILED) return false;
        osDelay(10U);
    }
    return false;
}

static void blocking_thread(void *argument) {
    (void)argument;
    blocking_job_t job;
    for (;;) {
        if (osMessageQueueGet(s_block_queue, &job, NULL, osWaitForever) != osOK) continue;
        MotorRuntime *m = motor_get(job.motor == MOTOR_RIGHT ? MOTOR_RIGHT : MOTOR_LEFT);
        if (job.cmd == COMM_PING_CAN) {
            uint8_t p[2] = {COMM_PING_CAN, VESC_VIRTUAL_CAN_RIGHT_ID};
            vesc_comm_send_payload(p, sizeof(p));
        } else if (job.cmd == COMM_SET_MCCONF) {
            bool ok = job.len == (1U + VESC6_MCCONF_WIRE_SIZE) &&
                      vesc_config_set_mc_wire(m->id, &job.data[1],
                                              (uint16_t)(job.len - 1U), true);
            if (ok) reply_ack(COMM_SET_MCCONF);
            else send_print("F103 V11: MCCONF rejected; motor must be OFF and VESC6 signature/layout valid.");
        } else if (job.cmd == COMM_SET_APPCONF) {
            bool ok = false;
            if (job.len == (1U + VESC6_APPCONF_WIRE_SIZE)) {
                uint8_t app_wire[VESC6_APPCONF_WIRE_SIZE];
                memcpy(app_wire, &job.data[1], sizeof(app_wire));
                /* Upstream ignores controller_id when APPCONF is written in
                   second-motor context. Preserve physical local ID=1. */
                if (m->id == MOTOR_RIGHT) app_wire[4] = VESC_CONTROLLER_ID_LEFT;
                ok = vesc_config_set_app_wire(app_wire, sizeof(app_wire), true);
            }
            if (ok) reply_ack(COMM_SET_APPCONF);
            else send_print("F103 V11: APPCONF rejected; VESC6 signature/layout invalid.");
        } else if (job.cmd == COMM_DETECT_ENCODER) {
            float current = (job.len >= 5U) ? (float)get_i32_be(&job.data[1]) / 1000.0f : SENSOR_DETECT_CURRENT_A;
            bool cal_ok = ensure_current_calibration_valid(10000U);
            bool started = cal_ok && (m->id == MOTOR_LEFT) && sensor_detect_request_current(m, SENSOR_MODE_ENCODER, fabsf(current));
            if (started) (void)wait_sensor_detect(m, 30000U);
            reply_standard_detect(m, COMM_DETECT_ENCODER);
        } else if (job.cmd == COMM_DETECT_HALL_FOC) {
            float current = (job.len >= 5U) ? (float)get_i32_be(&job.data[1]) / 1000.0f : SENSOR_DETECT_CURRENT_A;
            bool cal_ok = ensure_current_calibration_valid(10000U);
            bool started = cal_ok && sensor_detect_request_current(m, SENSOR_MODE_HALL, fabsf(current));
            if (started) (void)wait_sensor_detect(m, 30000U);
            reply_standard_detect(m, COMM_DETECT_HALL_FOC);
        } else if (job.cmd == COMM_DETECT_APPLY_ALL_FOC) {
            /* Standards-correct VESC detect-all semantics:
             * conf_general_detect_apply_all_foc() first measures real motor R/L
             * and flux linkage, then applies current gains/limits and finally
             * runs sensor detection. This reduced F103 port does not yet have
             * validated physical R/L/flux measurement routines, so it MUST NOT
             * report a Hall/encoder-only transaction as detect-all success.
             * Upstream uses -10 for flux-linkage detection failure. Individual
             * COMM_DETECT_HALL_FOC / COMM_DETECT_ENCODER below are fully usable
             * and use forced-angle sensor-independent SVPWM. */
            int16_t result = -10;
            if (!ensure_current_calibration_valid(10000U)) {
                result = -1;
            } else {
                send_print("F103 V11: full VESC Detect All requires validated R/L/flux measurement; use Hall/Encoder detect meanwhile.");
            }
            uint8_t p[3]; uint16_t i=0U; p[i++]=COMM_DETECT_APPLY_ALL_FOC; put_i16(p,&i,result);
            vesc_comm_send_payload(p,i);
        } else {
            reply_unsupported_detect(job.cmd);
        }
    }
}


static bool command_requires_motor_ready(uint8_t cmd) {
    switch (cmd) {
        case COMM_SET_DUTY:
        case COMM_SET_CURRENT:
        case COMM_SET_CURRENT_BRAKE:
        case COMM_SET_RPM:
        case COMM_SET_POS:
        case COMM_SET_HANDBRAKE:
        case COMM_SET_CURRENT_REL:
        case COMM_SET_MCCONF:
        case COMM_SET_APPCONF:
        case COMM_SAMPLE_PRINT:
        case COMM_DETECT_MOTOR_PARAM:
        case COMM_DETECT_MOTOR_R_L:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_ENCODER:
        case COMM_DETECT_HALL_FOC:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
        case COMM_DETECT_APPLY_ALL_FOC:
            return true;
        default:
            return false;
    }
}

static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len == 0U) return;
    uint8_t cmd = data[0];
    MotorRuntime *m = motor_get(id);

    /* COMM_FW_VERSION and the virtual-CAN FW_VERSION path must work even when
     * ADC/PWM/FOC initialization has failed. Motor-driving and detection
     * commands stay inhibited until motor_boot_thread completes. */
    if (!s_motor_ready && command_requires_motor_ready(cmd)) {
        return;
    }

    if (is_blocking_command(cmd)) {
        (void)queue_blocking_job(data, len, id);
        return;
    }

    switch (cmd) {
        case COMM_FW_VERSION:
            reply_fw_version(id);
            break;
        case COMM_GET_VALUES:
            reply_get_values(COMM_GET_VALUES, 0x003FFFFFUL, id);
            break;
        case COMM_GET_VALUES_SELECTIVE:
            if (len >= 5U) reply_get_values(COMM_GET_VALUES_SELECTIVE, get_u32_be(&data[1]), id);
            break;
        case COMM_GET_VALUES_SETUP:
            reply_setup_values(COMM_GET_VALUES_SETUP, 0x003FFFFFUL, id);
            break;
        case COMM_GET_VALUES_SETUP_SELECTIVE:
            if (len >= 5U) reply_setup_values(COMM_GET_VALUES_SETUP_SELECTIVE, get_u32_be(&data[1]), id);
            break;
        case COMM_SET_DUTY:
            if (len >= 5U) { motor_set_duty(m, (float)get_i32_be(&data[1]) / 100000.0f); vesc_timeout_reset(); }
            break;
        case COMM_SET_CURRENT:
            if (len >= 5U) { motor_set_current(m, (float)get_i32_be(&data[1]) / 1000.0f); vesc_timeout_reset(); }
            break;
        case COMM_SET_CURRENT_BRAKE:
            if (len >= 5U) { motor_set_brake_current(m, (float)get_i32_be(&data[1]) / 1000.0f); vesc_timeout_reset(); }
            break;
        case COMM_SET_RPM:
            if (len >= 5U) { motor_set_speed(m, (float)get_i32_be(&data[1])); vesc_timeout_reset(); }
            break;
        case COMM_SET_POS:
            if (len >= 5U) { motor_set_position(m, (float)get_i32_be(&data[1]) / 1000000.0f); vesc_timeout_reset(); }
            break;
        case COMM_SET_HANDBRAKE:
            if (len >= 5U) { motor_set_handbrake(m, (float)get_i32_be(&data[1]) / 1000.0f); vesc_timeout_reset(); }
            break;
        case COMM_SET_CURRENT_REL:
            if (len >= 5U) { motor_set_current_rel(m, (float)get_i32_be(&data[1]) / 100000.0f); vesc_timeout_reset(); }
            break;
        case COMM_SET_DETECT:
            if (len >= 2U) s_display_mode[(id == MOTOR_RIGHT) ? 1U : 0U] = data[1];
            break;
        case COMM_SAMPLE_PRINT:
            if (len >= 5U) {
                uint8_t mode = data[1];
                uint16_t sample_len = get_u16_be(&data[2]);
                uint16_t decimation = data[4];
                bool raw = (len >= 6U) ? (data[5] != 0U) : false;
                /* NOW/START are captured immediately. Trigger modes fall back
                 * to immediate capture because this reduced F103 ISR has no
                 * separate fault-trigger prebuffer yet. */
                (void)mode;
                debug_sample_start_ex(id, sample_len, decimation, raw);
            }
            break;
        case COMM_GET_DECODED_ADC:
            reply_decoded_adc();
            break;
        case COMM_FORWARD_CAN:
            if (len >= 3U) {
                uint8_t target = data[1];
                if (target == VESC_VIRTUAL_CAN_RIGHT_ID) {
                    s_diag.virtual_can_forwards++;
                    process_payload_for_motor(&data[2], (uint16_t)(len - 2U), MOTOR_RIGHT);
                } else {
                    /* Physical CAN is intentionally excluded. No fake route. */
                    s_diag.virtual_can_unknown_ids++;
                }
            }
            break;
        case COMM_REBOOT:
            motor_hw_emergency_all_off(); osDelay(20U); NVIC_SystemReset();
            break;
        case COMM_ALIVE:
            vesc_timeout_reset();
            break;
        case COMM_CUSTOM_APP_DATA:
            if (len >= 2U) {
                if (s_appdata_handler != NULL) s_appdata_handler(&data[1], (uint16_t)(len - 1U), id);
                else process_custom(&data[1], (uint16_t)(len - 1U), id);
            }
            break;
        case COMM_GET_MCCONF:
            reply_config_wire(COMM_GET_MCCONF, id, false);
            break;
        case COMM_GET_MCCONF_DEFAULT:
            reply_config_wire(COMM_GET_MCCONF_DEFAULT, id, true);
            break;
        case COMM_GET_APPCONF:
            reply_config_wire(COMM_GET_APPCONF, id, false);
            break;
        case COMM_GET_APPCONF_DEFAULT:
            reply_config_wire(COMM_GET_APPCONF_DEFAULT, id, true);
            break;
        /* SET_MCCONF/SET_APPCONF are queued above and handled by the blocking
           worker because flash programming must never stall uartcomm parsing. */
        default:
            break;
    }
}

static void process_payload(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U) return;
    s_diag.rx_frames_ok++;
    status_io_note_vesc_packet();
    /* UART is always local motor-left. Right motor is exposed only through
     * COMM_FORWARD_CAN using the current upstream dual-motor semantic. */
    process_payload_for_motor(data, len, MOTOR_LEFT);
}

static void packet_process_thread(void *argument) {
    (void)argument;
    for (;;) {
        /* Proven hoverboard transport services circular RX DMA by reading
         * CNDTR in normal context. Here that normal context is the dedicated
         * CMSIS-RTOS2 packet thread. */
        vesc_uart_service();

        bool received = false;
        uint8_t byte;
        while (vesc_uart_rx_get(&byte)) {
            received = true;
            vesc_packet_process_byte(&s_parser, byte, process_payload);
        }

        vesc_uart_service();
        if (received) {
            (void)osThreadYield();
        } else {
            osDelay(1U);
        }
    }
}

void vesc_comm_send_payload(const uint8_t *payload, uint16_t len) {
    if (payload == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD) return;
    if (s_send_mutex != NULL && osMutexAcquire(s_send_mutex, osWaitForever) != osOK) return;
    uint16_t frame_len = vesc_packet_encode(payload, len, s_tx_frame, sizeof(s_tx_frame));
    if (frame_len != 0U && vesc_uart_write_raw(s_tx_frame, frame_len)) s_diag.tx_frames++;
    if (s_send_mutex != NULL) (void)osMutexRelease(s_send_mutex);
}

static void vesc_comm_reply_diag(void) {
    const vesc_uart_stats_t *u = vesc_uart_get_stats();
    uint8_t p[72]; uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_COMM_DIAG; p[i++] = 6U;
    put_u32(p, &i, u->rx_bytes); put_u32(p, &i, u->rx_overruns); put_u32(p, &i, s_diag.rx_frames_ok);
    put_u32(p, &i, u->tx_bytes); put_u32(p, &i, u->uart_errors); put_u32(p, &i, s_diag.tx_frames);
    put_u32(p, &i, u->tx_overruns); put_u32(p, &i, u->tx_complete_count); put_u32(p, &i, s_diag.blocking_busy_drops);
    put_u32(p, &i, s_diag.virtual_can_forwards); put_u32(p, &i, s_diag.virtual_can_unknown_ids);
    put_u32(p, &i, VESC_UART_BAUD); p[i++] = (uint8_t)osMessageQueueGetCount(s_block_queue);
    p[i++] = vesc_timeout_has_timeout() ? 1U : 0U; p[i++] = config_store_valid() ? 1U : 0U;
    vesc_comm_send_payload(p, i);
}

void vesc_comm_periodic_100hz(void) {
    if (!s_motor_ready) return;
    /* In dual-motor mode a forwarded SET_DETECT can enable rotor/position
     * streaming for the virtual CAN motor as well. Replies stay in the inner
     * VESC command format, just like forwarded request/reply semantics. */
    send_rotor_position(MOTOR_LEFT);
    send_rotor_position(MOTOR_RIGHT);
}

void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count) {
    if (samples == NULL || count == 0U) return;
    bool raw = debug_sample_raw();
    for (uint16_t n = 0U; n < count; n++) {
        const debug_sample_t *d = &samples[n];
        uint8_t p[56]; uint16_t i = 0U;
        p[i++] = COMM_SAMPLE_PRINT;
        put_i16(p, &i, (int16_t)n);

        float ia, ib, ic, ph1, ph2, ph3, vzero, current_fir;
        if (raw) {
            ia = (float)d->current_raw_u;
            ib = (float)d->current_raw_v;
            ic = 0.0f; /* third phase current ADC is not present on this PCB */
            ph1 = (float)d->duty_u_q15 * (4095.0f / 32768.0f);
            ph2 = (float)d->duty_v_q15 * (4095.0f / 32768.0f);
            ph3 = (float)d->duty_w_q15 * (4095.0f / 32768.0f);
            vzero = 2048.0f;
            current_fir = (float)d->iq_cA;
        } else {
            ia = (float)d->ia_cA / 100.0f;
            ib = (float)d->ib_cA / 100.0f;
            ic = -(ia + ib);
            float vbus = (float)d->vbus_dV / 10.0f;
            ph1 = ((float)d->duty_u_q15 / 32768.0f) * vbus;
            ph2 = ((float)d->duty_v_q15 / 32768.0f) * vbus;
            ph3 = ((float)d->duty_w_q15 / 32768.0f) * vbus;
            vzero = vbus * 0.5f;
            current_fir = (float)d->iq_cA / 100.0f;
        }

        put_float32_auto(p, &i, ia); put_float32_auto(p, &i, ib); put_float32_auto(p, &i, ic);
        put_float32_auto(p, &i, ph1); put_float32_auto(p, &i, ph2); put_float32_auto(p, &i, ph3);
        put_float32_auto(p, &i, vzero); put_float32_auto(p, &i, current_fir);
        put_float32_auto(p, &i, (float)PWM_FREQUENCY_HZ);
        p[i++] = 0U; /* reduced sampler status */
        p[i++] = (uint8_t)(((uint32_t)d->phase_u16 * 250U) >> 16);
        put_i32(p, &i, (int32_t)n);
        vesc_comm_send_payload(p, i);
        /* At 115200 the software TX ring is back-pressure aware; yielding also
         * keeps the sample sender from starving control threads. */
        osDelay(1U);
    }
}

void vesc_comm_register_appdata_handler(vesc_appdata_handler_t handler) {
    s_appdata_handler = handler;
}

bool vesc_comm_task_init(void) {
    memset((void *)&s_diag, 0, sizeof(s_diag));
    memset((void *)s_display_mode, 0, sizeof(s_display_mode));
    s_motor_ready = false;
    vesc_packet_parser_init(&s_parser);

    const osMutexAttr_t payload_attr = {.name = "VescPayload"};
    const osMutexAttr_t send_attr = {.name = "VescSend"};
    s_payload_mutex = osMutexNew(&payload_attr);
    s_send_mutex = osMutexNew(&send_attr);

    const osMessageQueueAttr_t blockq_attr = {.name = "VescBlockQ"};
    s_block_queue = osMessageQueueNew(BLOCK_QUEUE_DEPTH, sizeof(blocking_job_t), &blockq_attr);

    if (s_payload_mutex == NULL || s_send_mutex == NULL || s_block_queue == NULL) {
        return false;
    }

    const osThreadAttr_t packet_attr = {
        .name="packet_process_thread", .priority=osPriorityAboveNormal, .stack_size=1536U
    };
    const osThreadAttr_t block_attr = {
        .name="blocking_thread", .priority=osPriorityNormal, .stack_size=3072U
    };

    osThreadId_t packet_tp = osThreadNew(packet_process_thread, NULL, &packet_attr);
    osThreadId_t blocking_tp = osThreadNew(blocking_thread, NULL, &block_attr);
    if (packet_tp == NULL || blocking_tp == NULL) {
        return false;
    }

    /* Match app_uartcomm_start ordering conceptually: packet state/thread
     * resources exist before the serial peripheral starts receiving bytes. */
    if (!vesc_uart_init()) {
        return false;
    }

    return true;
}

void vesc_comm_set_motor_ready(bool ready) {
    s_motor_ready = ready;
}

bool vesc_comm_motor_ready(void) {
    return s_motor_ready;
}

