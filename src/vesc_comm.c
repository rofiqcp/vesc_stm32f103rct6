#include "vesc_comm.h"
#include "vesc_uart.h"
#include "vesc_packet.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "telemetry.h"
#include "sensor_detect.h"
#include "debug_sample.h"
#include "foc_control.h"
#include "app_config.h"
#include "cmsis_os2.h"
#include <string.h>
#include <math.h>

#define TX_FRAME_SIZE     224U
#define BLOCK_QUEUE_DEPTH 1U
#define BLOCK_DATA_MAX    96U

/* VESC command IDs used by this firmware. */
enum {
    COMM_FW_VERSION = 0,
    COMM_GET_VALUES = 4,
    COMM_SET_DUTY = 5,
    COMM_SET_CURRENT = 6,
    COMM_SET_CURRENT_BRAKE = 7,
    COMM_SET_RPM = 8,
    COMM_SET_POS = 9,
    COMM_SET_DETECT = 11,
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
    COMM_CUSTOM_APP_DATA = 36,
    COMM_GET_VALUES_SETUP = 47,
    COMM_GET_VALUES_SELECTIVE = 50,
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57,
    COMM_DETECT_APPLY_ALL_FOC = 58
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
    CUSTOM_SAMPLE_DATA = 0xB0
};

typedef struct {
    uint8_t cmd;
    uint8_t motor;
    uint16_t len;
    uint8_t data[BLOCK_DATA_MAX];
} detect_job_t;

typedef struct {
    volatile uint32_t rx_frames_ok;
    volatile uint32_t tx_frames;
    volatile uint32_t blocking_busy_drops;
} comm_diag_t;

static vesc_packet_parser_t s_parser;

static osThreadId_t s_blocking_tid;
static osMessageQueueId_t s_block_queue;
static osMutexId_t s_payload_mutex;

static volatile motor_id_t s_selected_motor = MOTOR_LEFT;
static volatile uint8_t s_display_mode[2];
static uint8_t s_tx_payload[TX_FRAME_SIZE - 8U];
static comm_diag_t s_diag;

static void process_payload(const uint8_t *data, uint16_t len);
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
    uint16_t u = (uint16_t)v;
    b[(*i)++] = (uint8_t)(u >> 8);
    b[(*i)++] = (uint8_t)u;
}

static void put_u16(uint8_t *b, uint16_t *i, uint16_t v) {
    b[(*i)++] = (uint8_t)(v >> 8);
    b[(*i)++] = (uint8_t)v;
}

static void put_i32(uint8_t *b, uint16_t *i, int32_t v) {
    uint32_t u = (uint32_t)v;
    b[(*i)++] = (uint8_t)(u >> 24);
    b[(*i)++] = (uint8_t)(u >> 16);
    b[(*i)++] = (uint8_t)(u >> 8);
    b[(*i)++] = (uint8_t)u;
}

static void put_u32(uint8_t *b, uint16_t *i, uint32_t v) {
    put_i32(b, i, (int32_t)v);
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
    /* Map local protection faults onto the current VESC mc_fault_code values
       so VESC Tool does not mis-label them. Native details remain available
       in COMM_CUSTOM_APP_DATA telemetry. */
    switch ((motor_fault_t)native_fault) {
        case MOTOR_FAULT_NONE:             return 0U;  /* NONE */
        case MOTOR_FAULT_OVER_VOLTAGE:     return 1U;  /* OVER_VOLTAGE */
        case MOTOR_FAULT_UNDER_VOLTAGE:    return 2U;  /* UNDER_VOLTAGE */
        case MOTOR_FAULT_ADC_DMA:          return 3U;  /* DRV / HW fault class */
        case MOTOR_FAULT_ABS_OVER_CURRENT: return 4U;  /* ABS_OVER_CURRENT */
        case MOTOR_FAULT_CURRENT_OFFSET:   return 15U; /* HIGH_OFFSET_CURRENT_SENSOR_1 */
        case MOTOR_FAULT_HALL_INVALID:     return 27U; /* ENCODER_FAULT / sensor class */
        case MOTOR_FAULT_SENSOR_DETECT:    return 27U;
        case MOTOR_FAULT_FOC_ISR_OVERRUN:  return 3U;
        case MOTOR_FAULT_COMMAND_TIMEOUT:  return 0U;  /* reported in status byte */
        default:                           return 3U;
    }
}

static uint8_t *payload_begin(void) {
    if (s_payload_mutex != NULL) {
        if (osMutexAcquire(s_payload_mutex, 100U) != osOK) {
            return NULL;
        }
    }
    return s_tx_payload;
}

static void payload_end(uint16_t len) {
    vesc_comm_send_payload(s_tx_payload, len);
    if (s_payload_mutex != NULL) {
        (void)osMutexRelease(s_payload_mutex);
    }
}

static void reply_fw_version(void) {
    uint16_t i = 0U;
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    p[i++] = COMM_FW_VERSION;
    /* Protocol version fields mirror the current upstream VESC 7.01 packet
       layout so current VESC Tool parses all trailing FW_RX_PARAMS fields. */
    p[i++] = 7U;
    p[i++] = 1U;

    const char hw[] = "STM32F103RC_DUAL_FOC";
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
    if (s_selected_motor == MOTOR_RIGHT) {
        p[i - 1U]++;
    }

    p[i++] = 1U; /* pairing done */
    p[i++] = 1U; /* VESC 7.01 master test-version field */
    p[i++] = 0U; /* HW_TYPE_VESC */
    p[i++] = 0U; /* custom-config count */
    p[i++] = 0U; /* phase filters */
    p[i++] = 0U; /* HW QML */
    p[i++] = 0U; /* APP QML */
    p[i++] = 0U; /* NRF flags */

    const char fw[] = "F103_DUAL_FOC_RTOS2_V5";
    memcpy(&p[i], fw, sizeof(fw));
    i = (uint16_t)(i + sizeof(fw));
    put_u32(p, &i, 0U);
    payload_end(i);
}

static void append_get_values_fields(uint8_t *p, uint16_t *i,
                                     const motor_telemetry_t *t, uint32_t mask) {
    if ((mask & (1UL << 0)) != 0U) put_i16(p, i, 0); /* FET temp unavailable */
    if ((mask & (1UL << 1)) != 0U) put_i16(p, i, 0); /* motor temp unavailable */
    if ((mask & (1UL << 2)) != 0U) put_i32(p, i, scaled_i32(t->current_motor, 100.0f));
    if ((mask & (1UL << 3)) != 0U) put_i32(p, i, scaled_i32(t->current_in, 100.0f));
    if ((mask & (1UL << 4)) != 0U) put_i32(p, i, scaled_i32(t->id_filter, 100.0f));
    if ((mask & (1UL << 5)) != 0U) put_i32(p, i, scaled_i32(t->iq_filter, 100.0f));
    if ((mask & (1UL << 6)) != 0U) put_i16(p, i, scaled_i16(t->duty, 1000.0f));
    if ((mask & (1UL << 7)) != 0U) put_i32(p, i, scaled_i32(t->erpm, 1.0f));
    if ((mask & (1UL << 8)) != 0U) put_i16(p, i, scaled_i16(t->vbus, 10.0f));
    if ((mask & (1UL << 9)) != 0U) put_i32(p, i, scaled_i32(t->amp_hours, 10000.0f));
    if ((mask & (1UL << 10)) != 0U) put_i32(p, i, scaled_i32(t->amp_hours_charged, 10000.0f));
    if ((mask & (1UL << 11)) != 0U) put_i32(p, i, scaled_i32(t->watt_hours, 10000.0f));
    if ((mask & (1UL << 12)) != 0U) put_i32(p, i, scaled_i32(t->watt_hours_charged, 10000.0f));
    if ((mask & (1UL << 13)) != 0U) put_i32(p, i, t->tachometer);
    if ((mask & (1UL << 14)) != 0U) put_i32(p, i, t->tachometer_abs);
    if ((mask & (1UL << 15)) != 0U) p[(*i)++] = vesc_fault_code(t->fault);
    if ((mask & (1UL << 16)) != 0U) put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if ((mask & (1UL << 17)) != 0U) p[(*i)++] = t->controller_id;
    if ((mask & (1UL << 18)) != 0U) {
        put_i16(p, i, 0);
        put_i16(p, i, 0);
        put_i16(p, i, 0);
    }
    if ((mask & (1UL << 19)) != 0U) put_i32(p, i, scaled_i32(t->vd, 1000.0f));
    if ((mask & (1UL << 20)) != 0U) put_i32(p, i, scaled_i32(t->vq, 1000.0f));
    if ((mask & (1UL << 21)) != 0U) p[(*i)++] = t->timeout_active ? 1U : 0U;
}

static void reply_get_values(uint8_t command, uint32_t mask) {
    motor_telemetry_t t;
    telemetry_get(s_selected_motor, &t);
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = command;
    if (command == COMM_GET_VALUES_SELECTIVE) {
        put_u32(p, &i, mask);
    }
    append_get_values_fields(p, &i, &t, mask);
    payload_end(i);
}

static void reply_setup_values(void) {
    motor_telemetry_t t;
    telemetry_get(s_selected_motor, &t);
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = COMM_GET_VALUES_SETUP;
    put_i16(p, &i, 0);
    put_i16(p, &i, 0);
    put_i32(p, &i, scaled_i32(t.current_motor, 100.0f));
    put_i32(p, &i, scaled_i32(t.current_in, 100.0f));
    put_i16(p, &i, scaled_i16(t.duty, 1000.0f));
    put_i32(p, &i, scaled_i32(t.erpm, 1.0f));
    put_i32(p, &i, 0); /* speed m/s unknown */
    put_i16(p, &i, scaled_i16(t.vbus, 10.0f));
    put_i16(p, &i, 0); /* battery level unknown */
    put_i32(p, &i, scaled_i32(t.amp_hours, 10000.0f));
    put_i32(p, &i, scaled_i32(t.amp_hours_charged, 10000.0f));
    put_i32(p, &i, scaled_i32(t.watt_hours, 10000.0f));
    put_i32(p, &i, scaled_i32(t.watt_hours_charged, 10000.0f));
    put_i32(p, &i, 0); /* distance */
    put_i32(p, &i, 0); /* distance abs */
    put_i32(p, &i, scaled_i32(t.position_deg, 1000000.0f));
    p[i++] = vesc_fault_code(t.fault);
    p[i++] = t.controller_id;
    p[i++] = 2U; /* two local motor controllers */
    put_i32(p, &i, 0); /* Wh battery left unknown */
    put_u32(p, &i, 0U); /* odometer */
    put_u32(p, &i, osKernelGetTickCount());
    payload_end(i);
}

static void send_rotor_position(motor_id_t id) {
    motor_telemetry_t t;
    telemetry_get(id, &t);
    uint8_t mode = s_display_mode[(id == MOTOR_RIGHT) ? 1U : 0U];
    if (mode == DISP_POS_MODE_NONE) return;

    float pos = 0.0f;
    switch (mode) {
        case DISP_POS_MODE_ENCODER:
            pos = t.position_deg;
            break;
        case DISP_POS_MODE_PID_POS:
            pos = t.position_deg;
            break;
        case DISP_POS_MODE_PID_POS_ERROR: {
            MotorRuntime *m = motor_get(id);
            pos = m->position_target_deg - t.position_deg;
            while (pos > 180.0f) pos -= 360.0f;
            while (pos < -180.0f) pos += 360.0f;
        } break;
        case DISP_POS_MODE_OBSERVER:
        case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
        case DISP_POS_MODE_HALL_OBSERVER_ERROR:
        case DISP_POS_MODE_INDUCTANCE:
        default:
            pos = t.rotor_elec_deg;
            break;
    }

    uint8_t p[5];
    uint16_t i = 0U;
    p[i++] = COMM_ROTOR_POSITION;
    put_i32(p, &i, scaled_i32(pos, 100000.0f));
    vesc_comm_send_payload(p, i);
}

static void reply_custom_summary(void) {
    motor_telemetry_t l, r;
    telemetry_get(MOTOR_LEFT, &l);
    telemetry_get(MOTOR_RIGHT, &r);
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_DUAL_SUMMARY;
    p[i++] = 3U; /* protocol revision */

    const motor_telemetry_t *arr[2] = {&l, &r};
    for (uint8_t k = 0U; k < 2U; k++) {
        const motor_telemetry_t *t = arr[k];
        p[i++] = k;
        p[i++] = t->sensor_mode;
        p[i++] = t->fault;
        p[i++] = t->sensor_detect_state;
        put_i32(p, &i, scaled_i32(t->erpm, 1.0f));
        put_i32(p, &i, scaled_i32(t->iq, 100.0f));
        put_i32(p, &i, scaled_i32(t->id, 100.0f));
        put_i16(p, &i, scaled_i16(t->vbus, 10.0f));
    }
    put_u32(p, &i, l.isr_max_cycles);
    put_u32(p, &i, l.isr_overruns);
    p[i++] = l.calibration_done;
    p[i++] = l.calibration_valid;
    payload_end(i);
}

static void reply_extended(motor_id_t id) {
    motor_telemetry_t t;
    telemetry_get(id, &t);
    MotorRuntime *m = motor_get(id);
    uint32_t cal_count = 0U, cal_target = 0U;
    foc_get_calibration_progress(&cal_count, &cal_target);

    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_EXT_TELEMETRY;
    p[i++] = (uint8_t)id;
    p[i++] = 3U;
    p[i++] = t.sensor_mode;
    p[i++] = t.fault;
    p[i++] = t.sensor_detect_state;
    p[i++] = t.calibration_done;
    p[i++] = t.calibration_valid;
    p[i++] = m->hall.raw_state;
    p[i++] = (uint8_t)m->pole_pairs;
    p[i++] = m->encoder.inverted ? 1U : 0U;

    put_i32(p, &i, scaled_i32(t.id, 1000.0f));
    put_i32(p, &i, scaled_i32(t.iq, 1000.0f));
    put_i32(p, &i, scaled_i32(t.id_filter, 1000.0f));
    put_i32(p, &i, scaled_i32(t.iq_filter, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vd, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vq, 1000.0f));
    put_i32(p, &i, scaled_i32(t.current_motor, 1000.0f));
    put_i32(p, &i, scaled_i32(t.current_in, 1000.0f));
    put_i32(p, &i, scaled_i32(t.erpm, 1.0f));
    put_i32(p, &i, scaled_i32(t.mech_rpm, 10.0f));
    put_i32(p, &i, scaled_i32(t.position_deg, 1000.0f));
    put_i32(p, &i, scaled_i32(t.rotor_elec_deg, 1000.0f));
    put_i32(p, &i, scaled_i32(t.vbus, 1000.0f));
    put_i32(p, &i, scaled_i32(t.duty, 100000.0f));
    put_i32(p, &i, (int32_t)t.current_offset_u);
    put_i32(p, &i, (int32_t)t.current_offset_v);
    put_i32(p, &i, (int32_t)t.dc_current_offset);
    put_u32(p, &i, cal_count);
    put_u32(p, &i, cal_target);
    put_u32(p, &i, t.isr_max_cycles);
    put_u32(p, &i, t.isr_overruns);
    put_i32(p, &i, m->encoder.extended_count);
    put_u16(p, &i, m->encoder.elec_offset_u16);
    payload_end(i);
}

static void reply_sensor_info(motor_id_t id) {
    MotorRuntime *m = motor_get(id);
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_SENSOR_INFO;
    p[i++] = (uint8_t)id;
    p[i++] = m->sensor_mode;
    p[i++] = m->sensor_request_mode;
    p[i++] = (uint8_t)m->detect.state;
    p[i++] = m->detect.success ? 1U : 0U;
    p[i++] = m->pole_pairs;
    p[i++] = m->encoder.inverted ? 1U : 0U;
    put_u16(p, &i, m->encoder.elec_offset_u16);
    for (uint8_t k = 0U; k < 8U; k++) p[i++] = m->foc_hall_table[k];
    for (uint8_t k = 0U; k < 8U; k++) put_u16(p, &i, m->hall_angle_u16[k]);
    payload_end(i);
}

static void reply_current_cal(void) {
    uint32_t count = 0U, target = 0U;
    foc_get_calibration_progress(&count, &target);
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_CURRENT_CAL;
    p[i++] = foc_calibration_done() ? 1U : 0U;
    p[i++] = foc_calibration_valid() ? 1U : 0U;
    put_u32(p, &i, count);
    put_u32(p, &i, target);
    put_i32(p, &i, g_motor_left.current_offset_u_counts);
    put_i32(p, &i, g_motor_left.current_offset_v_counts);
    put_i32(p, &i, g_motor_left.dc_current_offset_counts);
    put_i32(p, &i, g_motor_right.current_offset_u_counts);
    put_i32(p, &i, g_motor_right.current_offset_v_counts);
    put_i32(p, &i, g_motor_right.dc_current_offset_counts);
    payload_end(i);
}

static void reply_standard_detect(MotorRuntime *m, uint8_t command) {
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = command;

    if (command == COMM_DETECT_HALL_FOC) {
        for (uint8_t k = 0U; k < 8U; k++) {
            p[i++] = (m->detect.success && m->sensor_mode == SENSOR_MODE_HALL) ? m->foc_hall_table[k] : 255U;
        }
        p[i++] = (uint8_t)((m->detect.success && m->sensor_mode == SENSOR_MODE_HALL) ? 0 : 1);
    } else if (command == COMM_DETECT_ENCODER) {
        if (m->detect.success && m->sensor_mode == SENSOR_MODE_ENCODER) {
            float offset_deg = ((float)m->encoder.elec_offset_u16 * 360.0f) / 65536.0f;
            put_i32(p, &i, scaled_i32(offset_deg, 1000000.0f));
            put_i32(p, &i, scaled_i32((float)m->pole_pairs, 1000000.0f));
            p[i++] = m->encoder.inverted ? 1U : 0U;
        } else {
            put_i32(p, &i, 1001000000L);
            put_i32(p, &i, 0);
            p[i++] = 0U;
        }
    }
    payload_end(i);
}

static void process_custom(const uint8_t *data, uint16_t len) {
    if (len < 2U) return;
    uint8_t sub = data[1];

    if (sub == CUSTOM_SELECT_MOTOR && len >= 3U) {
        s_selected_motor = (data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT;
    } else if (sub == CUSTOM_DUAL_SUMMARY) {
        reply_custom_summary();
    } else if (sub == CUSTOM_CLEAR_FAULT && len >= 3U) {
        motor_clear_fault(motor_get((data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT));
    } else if (sub == CUSTOM_STOP && len >= 3U) {
        motor_stop(motor_get((data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT));
    } else if ((sub == CUSTOM_SENSOR_SELECT || sub == CUSTOM_SENSOR_DETECT) && len >= 4U) {
        MotorRuntime *m = motor_get((data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT);
        uint8_t mode = data[3];
        if (sub == CUSTOM_SENSOR_DETECT || mode == SENSOR_MODE_AUTO) {
            (void)sensor_detect_request(m, mode);
        } else {
            (void)motor_select_sensor_mode(m, mode);
        }
        reply_sensor_info(m->id);
    } else if (sub == CUSTOM_CURRENT_CAL) {
        if (len >= 3U && data[2] == 1U) {
            motor_stop(&g_motor_left);
            motor_stop(&g_motor_right);
            foc_request_recalibration();
        }
        reply_current_cal();
    } else if (sub == CUSTOM_SAMPLE_START && len >= 7U) {
        motor_id_t id = (data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT;
        uint16_t count = get_u16_be(&data[3]);
        uint16_t decimation = get_u16_be(&data[5]);
        debug_sample_start(id, count, decimation);
    } else if (sub == CUSTOM_EXT_TELEMETRY && len >= 3U) {
        reply_extended((data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT);
    } else if (sub == CUSTOM_SENSOR_INFO && len >= 3U) {
        reply_sensor_info((data[2] == 1U) ? MOTOR_RIGHT : MOTOR_LEFT);
    } else if (sub == CUSTOM_COMM_DIAG) {
        vesc_comm_reply_diag();
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
            return true;
        default:
            return false;
    }
}

static bool queue_blocking_job(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U || s_block_queue == NULL) return false;
    detect_job_t job;
    memset(&job, 0, sizeof(job));
    job.cmd = data[0];
    job.motor = (uint8_t)s_selected_motor;
    job.len = len > BLOCK_DATA_MAX ? BLOCK_DATA_MAX : len;
    memcpy(job.data, data, job.len);
    if (osMessageQueuePut(s_block_queue, &job, 0U, 0U) != osOK) {
        s_diag.blocking_busy_drops++;
        return false;
    }
    return true;
}

static void send_print(const char *msg) {
    if (msg == NULL) return;
    uint8_t p[96];
    uint16_t i = 0U;
    p[i++] = COMM_PRINT;
    size_t n = strlen(msg);
    if (n > sizeof(p) - 2U) n = sizeof(p) - 2U;
    memcpy(&p[i], msg, n);
    i = (uint16_t)(i + n);
    p[i++] = 0U;
    vesc_comm_send_payload(p, i);
}

static void reply_unsupported_blocking(uint8_t cmd) {
    uint8_t p[24];
    uint16_t i = 0U;
    p[i++] = cmd;
    switch (cmd) {
        case COMM_DETECT_MOTOR_PARAM:
            /* cycle_int_limit, bemf coupling, hall table, hall result */
            put_i32(p, &i, 0);
            put_i32(p, &i, 0);
            for (uint8_t k = 0U; k < 8U; k++) p[i++] = 255U;
            p[i++] = 1U;
            break;
        case COMM_DETECT_MOTOR_R_L:
            /* R, L, Ld-Lq. Zero is a deliberate failure sentinel here. */
            put_i32(p, &i, 0);
            put_i32(p, &i, 0);
            put_i32(p, &i, 0);
            break;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
            put_i32(p, &i, 0);
            break;
        case COMM_DETECT_APPLY_ALL_FOC:
            /* Negative result: this build does not fake R/L/flux measurements. */
            put_i16(p, &i, -1);
            break;
        default:
            break;
    }
    vesc_comm_send_payload(p, i);
    send_print("F103 V5: requested full motor-parameter detect is not implemented; sensor detect is available.");
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
    detect_job_t job;
    for (;;) {
        if (osMessageQueueGet(s_block_queue, &job, NULL, osWaitForever) != osOK) continue;
        MotorRuntime *m = motor_get(job.motor == (uint8_t)MOTOR_RIGHT ? MOTOR_RIGHT : MOTOR_LEFT);

        if (job.cmd == COMM_DETECT_ENCODER) {
            bool started = (m->id == MOTOR_LEFT) && sensor_detect_request(m, SENSOR_MODE_ENCODER);
            if (started) (void)wait_sensor_detect(m, 30000U);
            reply_standard_detect(m, COMM_DETECT_ENCODER);
        } else if (job.cmd == COMM_DETECT_HALL_FOC) {
            bool started = sensor_detect_request(m, SENSOR_MODE_HALL);
            if (started) (void)wait_sensor_detect(m, 30000U);
            reply_standard_detect(m, COMM_DETECT_HALL_FOC);
        } else {
            reply_unsupported_blocking(job.cmd);
        }
    }
}

static void process_payload(const uint8_t *data, uint16_t len) {
    if (len == 0U) return;
    s_diag.rx_frames_ok++;
    uint8_t cmd = data[0];
    MotorRuntime *m = motor_get(s_selected_motor);

    if (is_blocking_command(cmd)) {
        (void)queue_blocking_job(data, len);
        return;
    }

    switch (cmd) {
        case COMM_FW_VERSION:
            /* Handshake is intentionally immediate: no periodic thread, no ADC
               thread, and no motor state is required to answer this command. */
            reply_fw_version();
            break;
        case COMM_GET_VALUES:
            reply_get_values(COMM_GET_VALUES, 0x003FFFFFUL);
            break;
        case COMM_GET_VALUES_SELECTIVE:
            if (len >= 5U) reply_get_values(COMM_GET_VALUES_SELECTIVE, get_u32_be(&data[1]));
            break;
        case COMM_GET_VALUES_SETUP:
            reply_setup_values();
            break;
        case COMM_SET_DUTY:
            if (len >= 5U) motor_set_duty_approx(m, (float)get_i32_be(&data[1]) / 100000.0f);
            break;
        case COMM_SET_CURRENT:
            if (len >= 5U) motor_set_current(m, (float)get_i32_be(&data[1]) / 1000.0f);
            break;
        case COMM_SET_CURRENT_BRAKE:
            if (len >= 5U) motor_set_brake_current(m, (float)get_i32_be(&data[1]) / 1000.0f);
            break;
        case COMM_SET_RPM:
            if (len >= 5U) motor_set_speed(m, (float)get_i32_be(&data[1]));
            break;
        case COMM_SET_POS:
            if (len >= 5U) motor_set_position(m, (float)get_i32_be(&data[1]) / 1000000.0f);
            break;
        case COMM_SET_DETECT:
            if (len >= 2U) s_display_mode[(s_selected_motor == MOTOR_RIGHT) ? 1U : 0U] = data[1];
            break;
        case COMM_SAMPLE_PRINT:
            if (len >= 5U) {
                uint16_t sample_len = get_u16_be(&data[2]);
                uint16_t decimation = data[4];
                debug_sample_start(s_selected_motor, sample_len, decimation);
            }
            break;
        case COMM_REBOOT:
            motor_hw_emergency_all_off();
            osDelay(20U);
            NVIC_SystemReset();
            break;
        case COMM_ALIVE:
            motor_keepalive(m);
            break;
        case COMM_CUSTOM_APP_DATA:
            process_custom(data, len);
            break;
        default:
            break;
    }
}

static void packet_process_thread(void *argument) {
    (void)argument;
    vesc_packet_parser_init(&s_parser);
    vesc_comm_thread_id = osThreadGetId();

    for (;;) {
        /* Wake on the empty->non-empty transition, but retain a short timeout
           so bytes received before this thread ID became visible cannot get
           stranded in the ring. */
        uint32_t flags = osThreadFlagsWait(VESC_RX_AVAILABLE | VESC_TX_COMPLETE,
                                           osFlagsWaitAny, 10U);
        if ((flags & osFlagsError) != 0U) flags = 0U;
        (void)flags;

        uint8_t byte;
        while (vesc_uart_rx_get(&byte)) {
            vesc_packet_process_byte(&s_parser, byte, process_payload);
        }
    }
}

void vesc_comm_send_payload(const uint8_t *payload, uint16_t len) {
    if (payload == NULL || len == 0U) return;

    uint8_t frame[TX_FRAME_SIZE];
    uint16_t frame_len = vesc_packet_encode(payload, len, frame, sizeof(frame));
    if (frame_len == 0U) {
        return;
    }

    /* packet.c-style framing is complete before bytes enter the TX software
       ring. The USART3 TXE ISR is the only code that writes them to DR. */
    if (vesc_uart_write_raw(frame, frame_len)) {
        s_diag.tx_frames++;
    }
}

static void vesc_comm_reply_diag(void) {
    const vesc_uart_stats_t *u = vesc_uart_get_stats();
    uint8_t p[64];
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA;
    p[i++] = CUSTOM_COMM_DIAG;
    p[i++] = 5U;
    put_u32(p, &i, u->rx_bytes);
    put_u32(p, &i, u->rx_overruns);
    put_u32(p, &i, s_diag.rx_frames_ok);
    put_u32(p, &i, u->tx_bytes);
    put_u32(p, &i, u->uart_errors);
    put_u32(p, &i, s_diag.tx_frames);
    put_u32(p, &i, u->tx_overruns);
    put_u32(p, &i, u->tx_complete_count);
    put_u32(p, &i, s_diag.blocking_busy_drops);
    put_u32(p, &i, VESC_UART_BAUD);
    p[i++] = 0U; /* no RTOS TX message queue in V5 */
    p[i++] = (uint8_t)osMessageQueueGetCount(s_block_queue);
    vesc_comm_send_payload(p, i);
}

void vesc_comm_periodic_100hz(void) {
    send_rotor_position(s_selected_motor);
}

void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count) {
    if (samples == NULL || count == 0U) return;
    const uint8_t samples_per_packet = 8U;
    uint16_t offset = 0U;
    while (offset < count) {
        uint16_t i = 0U;
        uint16_t remain = (uint16_t)(count - offset);
        uint8_t n = (remain > samples_per_packet) ? samples_per_packet : (uint8_t)remain;
        uint8_t *p = payload_begin();
        if (p == NULL) return;
        p[i++] = COMM_CUSTOM_APP_DATA;
        p[i++] = CUSTOM_SAMPLE_DATA;
        put_u16(p, &i, offset);
        put_u16(p, &i, count);
        p[i++] = n;
        for (uint8_t k = 0U; k < n; k++) {
            const debug_sample_t *d = &samples[offset + k];
            put_i16(p, &i, d->ia_cA); put_i16(p, &i, d->ib_cA);
            put_i16(p, &i, d->id_cA); put_i16(p, &i, d->iq_cA);
            put_i16(p, &i, d->vd_cV); put_i16(p, &i, d->vq_cV);
            put_i16(p, &i, d->erpm); put_u16(p, &i, d->phase_u16);
            put_u16(p, &i, d->duty_u_q15); put_u16(p, &i, d->duty_v_q15);
            put_u16(p, &i, d->duty_w_q15); p[i++] = d->motor; p[i++] = d->hall_raw;
        }
        payload_end(i);
        offset = (uint16_t)(offset + n);
        osDelay(1U);
    }
}

void vesc_comm_task_init(void) {
    memset((void *)&s_diag, 0, sizeof(s_diag));

    /* Must be called after osKernelInitialize(): vesc_uart_init creates the TX
       mutex used to serialize packet writers. UART DMA requests stay disabled. */
    vesc_uart_init();

    const osMutexAttr_t payload_mutex_attr = {.name = "VescPayload"};
    s_payload_mutex = osMutexNew(&payload_mutex_attr);

    const osMessageQueueAttr_t blockq_attr = {.name = "VescBlockQ"};
    s_block_queue = osMessageQueueNew(BLOCK_QUEUE_DEPTH, sizeof(detect_job_t), &blockq_attr);

    const osThreadAttr_t packet_attr = {.name="packet_process_thread", .priority=osPriorityAboveNormal, .stack_size=896U};
    const osThreadAttr_t block_attr = {.name="blocking_thread", .priority=osPriorityAboveNormal, .stack_size=896U};
    (void)osThreadNew(packet_process_thread, NULL, &packet_attr);
    s_blocking_tid = osThreadNew(blocking_thread, NULL, &block_attr);
    (void)s_blocking_tid;
}
