#include "comm/commands.h"
#include "applications/app_uartcomm.h"
#include "applications/app.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "util/buffer.h"
#include "status_io.h"
#include "comm/packet.h"
#include "motor/mc_interface.h"
#include "hwconf/hw.h"
#include "telemetry.h"
#include "debug_sample.h"
#include "motor/mcpwm_foc.h"
#include "motor/foc_math.h"
#include "applications/appconf_default.h"
#include "timeout.h"
#include "motor_tasks.h"
#include "conf_general.h"
#include "confgenerator.h"
#include "cmsis_os2.h"
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <limits.h>

#define BLOCK_QUEUE_DEPTH 1U
#define BLOCK_DATA_MAX    VESC_PACKET_MAX_PAYLOAD
#define VESC_TEMP_UNAVAILABLE_DECIC (-3000)

/* Command numbers are locked to the VESC firmware 6.00 ABI for the subset
 * this reduced STM32F103 FOC port intentionally implements. */
/* COMM_PACKET_ID is canonical and shared from datatypes.h. */

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
    CUSTOM_CONFIG_STATUS = 0xAC,
    INTERNAL_CUSTOM_SENSOR_DETECT = 0xF0
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
    volatile uint32_t motor2_forwards;
    volatile uint32_t unsupported_forward_ids;
} comm_diag_t;

static vesc_packet_parser_t s_parser;
static osMessageQueueId_t s_block_queue;
static osThreadId_t s_packet_tp;
static osThreadId_t s_blocking_tp;
static osMutexId_t s_payload_mutex;
static osMutexId_t s_send_mutex;
static volatile uint8_t s_display_mode[2];
static volatile int8_t s_display_owner = -1;
static uint8_t s_tx_payload[VESC_PACKET_MAX_PAYLOAD];
static uint8_t s_tx_frame[VESC_PACKET_BUFFER_SIZE];
/* Blocking-worker-owned config scratch. Static BSS avoids placing ~1.5 KiB
   of MCCONF rollback/work images on the 3-KiB RTOS worker stack. */
static uint8_t s_mc_backup[2][VESC6_MCCONF_WIRE_SIZE];
static uint8_t s_mc_work[VESC6_MCCONF_WIRE_SIZE];
static comm_diag_t s_diag;
static vesc_appdata_handler_t s_appdata_handler = NULL;
static volatile bool s_motor_ready = false;
static volatile bool s_config_ready = false;
static volatile bool s_comm_initialized = false;
static volatile bool s_shutdown_latched = false;

static void process_payload(const uint8_t *data, uint16_t len);
static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id);
static void packet_process_thread(void *argument);
static void blocking_thread(void *argument);
static void vesc_comm_reply_diag(void);
static void vesc_comm_send_payload_low_priority(const uint8_t *payload, uint16_t len);
static void process_terminal_text(const uint8_t *data, uint16_t len, motor_id_t id);
static void reply_stats(const uint8_t *data, uint16_t len, motor_id_t id);
static void reply_mcconf_temp(motor_id_t id);
static void set_mcconf_temp(const uint8_t *data, uint16_t len, motor_id_t id, bool setup);

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

void commands_send_packet(unsigned char *data, unsigned int len) {
    if (data == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD) return;
    vesc_comm_send_payload((const uint8_t *)data, (uint16_t)len);
}

int commands_printf(const char *fmt, ...) {
    if (fmt == NULL) return 0;
    char msg[156];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    msg[sizeof(msg) - 1U] = '\0';
    commands_send_print(msg);
    return n;
}

void commands_send_print(const char *msg) {
    if (msg == NULL) return;
    uint8_t p[160];
    uint16_t i = 0U;
    p[i++] = COMM_PRINT;
    size_t n = strlen(msg);
    if (n > sizeof(p) - 2U) n = sizeof(p) - 2U;
    memcpy(&p[i], msg, n); i = (uint16_t)(i + n); p[i++] = 0U;
    vesc_comm_send_payload(p, i);
}

void commands_send_rotor_pos(float rotor_pos) {
    uint8_t p[5]; int32_t i=0; p[i++]=COMM_ROTOR_POSITION;
    buffer_append_int32(p,(int32_t)(rotor_pos*100000.0f),&i); commands_send_packet(p,(unsigned)i);
}
void commands_send_experiment_samples(float *samples, int len) {
    if (samples == NULL || len <= 0 || len > 63) return;
    uint8_t p[253];
    int32_t i = 0;
    p[i++] = COMM_EXPERIMENT_SAMPLE;
    for (int n = 0; n < len; n++) {
        buffer_append_int32(p, (int32_t)(samples[n] * 10000.0f), &i);
    }
    commands_send_packet(p, (unsigned)i);
}
void commands_init_plot(const char *namex,const char *namey) {
    if (namex == NULL || namey == NULL) return;
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0;
    p[i++] = COMM_PLOT_INIT;
    size_t nx = strlen(namex), ny = strlen(namey);
    if (nx > 100U) nx = 100U;
    if (ny > 100U) ny = 100U;
    memcpy(&p[i], namex, nx); i = (uint16_t)(i + nx); p[i++] = 0U;
    memcpy(&p[i], namey, ny); i = (uint16_t)(i + ny); p[i++] = 0U;
    payload_end(i);
}
void commands_plot_add_graph(const char *name) {
    if (name == NULL) return;
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0;
    p[i++] = COMM_PLOT_ADD_GRAPH;
    size_t n = strlen(name);
    if (n > 120U) n = 120U;
    memcpy(&p[i], name, n);
    i = (uint16_t)(i + n);
    p[i++] = 0U;
    payload_end(i);
}
void commands_plot_set_graph(int graph){uint8_t p[2]={COMM_PLOT_SET_GRAPH,(uint8_t)graph};commands_send_packet(p,2U);}
void commands_send_plot_points(float x,float y){uint8_t p[9];int32_t i=0;p[i++]=COMM_PLOT_DATA;buffer_append_float32_auto(p,x,&i);buffer_append_float32_auto(p,y,&i);commands_send_packet(p,(unsigned)i);}

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

    const char fw[] = "vesc-f103-hoverboard-v32-full-reaudit";
    memcpy(&p[i], fw, sizeof(fw));
    i = (uint16_t)(i + sizeof(fw));

    payload_end(i);
}

static void reply_fw_info(void) {
    uint8_t p[16]; uint16_t i = 0U;
    p[i++] = COMM_FW_INFO;
    p[i++] = 6U; p[i++] = 0U; p[i++] = 0U; /* major, minor, test */
    /* This build is produced outside the upstream git tree, so do not invent
       commit hashes. Empty NUL-terminated strings are the truthful VESC format. */
    p[i++] = 0U;
    p[i++] = 0U;
    vesc_comm_send_payload(p, i);
}

static void append_get_values_fields(uint8_t *p, uint16_t *i,
                                     const motor_telemetry_t *t, uint32_t mask) {
    /* Board ini tidak memiliki kanal NTC yang didefinisikan. Slot wajib
     * protocol tetap dikirim dengan sentinel -300 C, bukan 0 C palsu. */
    if (mask & (1UL << 0)) put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    if (mask & (1UL << 1)) put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
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
    if (mask & (1UL << 15)) p[(*i)++] = (uint8_t)motor_fault_to_vesc((motor_fault_t)t->fault);
    if (mask & (1UL << 16)) put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if (mask & (1UL << 17)) p[(*i)++] = t->controller_id;
    if (mask & (1UL << 18)) {
        put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
        put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
        put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    }
    if (mask & (1UL << 19)) put_i32(p, i, scaled_i32(t->vd, 1000.0f));
    if (mask & (1UL << 20)) put_i32(p, i, scaled_i32(t->vq, 1000.0f));
    if (mask & (1UL << 21)) p[(*i)++] = timeout_has_timeout() ? 1U : 0U;
}

static void reply_get_values(uint8_t command, uint32_t mask, motor_id_t id) {
    motor_telemetry_t t; telemetry_get(id, &t);
    motor_telemetry_avg_t avg; telemetry_read_reset_avg(id, mask, &avg);
    if (mask & (1UL << 2)) t.current_motor = avg.current_motor;
    if (mask & (1UL << 3)) t.current_in = avg.current_in;
    if (mask & (1UL << 4)) t.id_filter = avg.id;
    if (mask & (1UL << 5)) t.iq_filter = avg.iq;
    if (mask & (1UL << 19)) t.vd = avg.vd;
    if (mask & (1UL << 20)) t.vq = avg.vq;
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = command;
    if (command == COMM_GET_VALUES_SELECTIVE) put_u32(p, &i, mask);
    append_get_values_fields(p, &i, &t, mask);
    payload_end(i);
}

static void append_setup_fields(uint8_t *p, uint16_t *i,
                                const motor_telemetry_t *t, const setup_values *sv, uint32_t mask,
                                float speed, float battery_level, float distance,
                                float distance_abs, float wh_left, uint32_t odometer) {
    if (mask & (1UL << 0)) put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    if (mask & (1UL << 1)) put_i16(p, i, VESC_TEMP_UNAVAILABLE_DECIC);
    if (mask & (1UL << 2)) put_i32(p, i, scaled_i32(sv->current_tot, 100.0f));
    if (mask & (1UL << 3)) put_i32(p, i, scaled_i32(sv->current_in_tot, 100.0f));
    if (mask & (1UL << 4)) put_i16(p, i, scaled_i16(t->duty, 1000.0f));
    if (mask & (1UL << 5)) put_i32(p, i, scaled_i32(t->erpm, 1.0f));
    if (mask & (1UL << 6)) put_i32(p, i, scaled_i32(speed, 1000.0f));
    if (mask & (1UL << 7)) put_i16(p, i, scaled_i16(t->vbus, 10.0f));
    if (mask & (1UL << 8)) put_i16(p, i, scaled_i16(battery_level, 1000.0f));
    if (mask & (1UL << 9)) put_i32(p, i, scaled_i32(sv->ah_tot, 10000.0f));
    if (mask & (1UL << 10)) put_i32(p, i, scaled_i32(sv->ah_charge_tot, 10000.0f));
    if (mask & (1UL << 11)) put_i32(p, i, scaled_i32(sv->wh_tot, 10000.0f));
    if (mask & (1UL << 12)) put_i32(p, i, scaled_i32(sv->wh_charge_tot, 10000.0f));
    if (mask & (1UL << 13)) put_i32(p, i, scaled_i32(distance, 1000.0f));
    if (mask & (1UL << 14)) put_i32(p, i, scaled_i32(distance_abs, 1000.0f));
    if (mask & (1UL << 15)) put_i32(p, i, scaled_i32(t->position_deg, 1000000.0f));
    if (mask & (1UL << 16)) p[(*i)++] = (uint8_t)motor_fault_to_vesc((motor_fault_t)t->fault);
    if (mask & (1UL << 17)) p[(*i)++] = t->controller_id;
    if (mask & (1UL << 18)) p[(*i)++] = sv->num_vescs;
    if (mask & (1UL << 19)) put_i32(p, i, scaled_i32(wh_left, 1000.0f));
    if (mask & (1UL << 20)) put_u32(p, i, odometer);
    if (mask & (1UL << 21)) put_u32(p, i, osKernelGetTickCount());
}

static void reply_setup_values(uint8_t command, uint32_t mask, motor_id_t id) {
    motor_telemetry_t t; telemetry_get(id, &t);
    int old_motor = mc_interface_get_motor_thread();
    mc_interface_select_motor_thread(id == MOTOR_RIGHT ? 2 : 1);
    float wh_left = 0.0f;
    float battery_level = mc_interface_get_battery_level(&wh_left);
    float speed = mc_interface_get_speed();
    float distance = mc_interface_get_distance();
    float distance_abs = mc_interface_get_distance_abs();
    uint64_t odo64 = mc_interface_get_odometer();
    setup_values sv = mc_interface_get_setup_values();
    mc_interface_select_motor_thread(old_motor);
    uint32_t odometer = odo64 > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : (uint32_t)odo64;

    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = command;
    if (command == COMM_GET_VALUES_SETUP_SELECTIVE) put_u32(p, &i, mask);
    append_setup_fields(p, &i, &t, &sv, mask, speed, battery_level, distance, distance_abs, wh_left, odometer);
    payload_end(i);
}


static float wrap_error_deg(float a, float b) {
    float e = a - b;
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

static void send_rotor_position(motor_id_t id) {
    uint8_t index = (id == MOTOR_RIGHT) ? 1U : 0U;
    if (s_display_owner != (int8_t)index) return;
    uint8_t mode = s_display_mode[index];
    if (mode == DISP_POS_MODE_NONE) return;

    motor_telemetry_t t; telemetry_get(id, &t);
    MotorRuntime *m = motor_get(id);
    float observer = mcpwm_foc_get_phase_observer_rt(m);
    float encoder = mcpwm_foc_get_phase_encoder_rt(m);
    float hall = mcpwm_foc_get_phase_hall_rt(m);
    float pos;

    switch ((disp_pos_mode_t)mode) {
        case DISP_POS_MODE_ENCODER: pos = encoder; break;
        case DISP_POS_MODE_PID_POS: pos = t.position_deg; break;
        case DISP_POS_MODE_PID_POS_ERROR:
            pos = wrap_error_deg(m->position_target_deg, t.position_deg); break;
        case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
            pos = wrap_error_deg(encoder, observer); break;
        case DISP_POS_MODE_HALL_OBSERVER_ERROR:
            pos = wrap_error_deg(hall, observer); break;
        case DISP_POS_MODE_OBSERVER:
        case DISP_POS_MODE_INDUCTANCE:
        default: pos = observer; break;
    }

    uint8_t p[5]; uint16_t i = 0U;
    p[i++] = COMM_ROTOR_POSITION;
    put_i32(p, &i, scaled_i32(pos, 100000.0f));
    vesc_comm_send_payload_low_priority(p, i);
}

static void reply_custom_summary(void) {
    motor_telemetry_t l, r; telemetry_get(MOTOR_LEFT, &l); telemetry_get(MOTOR_RIGHT, &r);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_DUAL_SUMMARY; p[i++] = 5U;
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
    p[i++] = timeout_has_timeout() ? 1U : 0U;
    p[i++] = l.foc_sensor_mode; p[i++] = r.foc_sensor_mode;
    payload_end(i);
}

static void reply_extended(motor_id_t id) {
    motor_telemetry_t t; telemetry_get(id, &t); MotorRuntime *m = motor_get(id);
    uint32_t cal_count = 0U, cal_target = 0U; foc_get_calibration_progress(&cal_count, &cal_target);
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_EXT_TELEMETRY;
    p[i++] = (uint8_t)id; p[i++] = 7U; p[i++] = t.controller_id; p[i++] = t.sensor_mode;
    p[i++] = t.fault; p[i++] = t.sensor_detect_state; p[i++] = t.calibration_done; p[i++] = t.calibration_valid;
    p[i++] = m->hall.raw_state; p[i++] = m->pole_pairs; p[i++] = m->encoder.inverted ? 1U : 0U;
    put_i32(p, &i, scaled_i32(t.phase_current_a, 1000.0f));
    put_i32(p, &i, scaled_i32(t.phase_current_b, 1000.0f));
    put_i32(p, &i, scaled_i32(t.phase_current_c, 1000.0f));
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
    /* V22: phase A/B/C ditambahkan sebelum dq untuk audit arus fasa.
     * Observer/model fields tetap mengikuti blok yang sama setelah core FOC. */
    p[i++]=t.observer_valid; p[i++]=t.using_encoder; p[i++]=t.encoder_synced;
    put_i32(p,&i,scaled_i32(t.observer_phase_deg,1000.0f));
    put_i32(p,&i,scaled_i32(t.observer_erpm,1.0f));
    put_i32(p,&i,scaled_i32(t.observer_quality,100000.0f));
    put_i32(p,&i,scaled_i32(t.foc_motor_r,1000000.0f));
    put_i32(p,&i,scaled_i32(t.foc_motor_l,1000000000.0f));
    put_i32(p,&i,scaled_i32(t.foc_motor_ld_lq_diff,1000000000.0f));
    put_i32(p,&i,scaled_i32(t.foc_motor_flux_linkage,10000000.0f));
    put_i32(p,&i,scaled_i32(t.foc_sl_erpm_start,1.0f));
    put_i32(p,&i,scaled_i32(t.foc_sl_erpm,1.0f));
    put_i32(p,&i,scaled_i32(t.foc_openloop_rpm,1.0f));
    put_i32(p,&i,scaled_i32(t.foc_openloop_rpm_low,1.0f));
    put_u32(p,&i,t.current_loop_hz); put_u32(p,&i,t.telemetry_snapshot_hz);
    p[i++] = t.foc_sensor_mode;
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

    /* Extended detect/current diagnostics. Legacy fields stay first. */
    p[i++] = 16U;
    put_i32(p, &i, scaled_i32(m->detect.drive_current_a, 1000.0f));
    put_i32(p, &i, scaled_i32(m->id_target, 1000.0f));
    put_i32(p, &i, scaled_i32(m->iq_target, 1000.0f));
    put_u32(p, &i, m->detect.sweep_index);
    for (uint8_t k = 0U; k < 8U; k++) put_u32(p, &i, m->detect.hall_samples[k]);
    for (uint8_t k = 0U; k < 8U; k++) p[i++] = m->detect.result_hall_table[k];
    put_u16(p, &i, m->current_raw_u); put_u16(p, &i, m->current_raw_v); put_u16(p, &i, m->dc_current_raw);
    put_i32(p, &i, scaled_i32(m->ia, 1000.0f));
    put_i32(p, &i, scaled_i32(m->ib, 1000.0f));
    put_i32(p, &i, scaled_i32(m->ic, 1000.0f));
    p[i++] = m->pwm_enabled ? 1U : 0U;
    p[i++] = (m->pwm_tim->BDTR & TIM_BDTR_MOE) ? 1U : 0U;
    put_i32(p, &i, scaled_i32(m->current_scale, 1000000.0f));
    /* Synchronized-enable and stock-board sampling state. */
    p[i++] = m->pwm_enable_pending_events;
    put_u16(p, &i, m->pwm_enable_blank_cycles);
    put_u16(p, &i, (uint16_t)m->pwm_tim->CNT);
    put_u16(p, &i, (uint16_t)DMA1_Channel1->CNDTR);
    put_u16(p, &i, (uint16_t)TIM8->RCR);
    put_u16(p, &i, (uint16_t)ADC_MOTOR_PHASE_OFFSET_TICKS);
    p[i++] = (uint8_t)m->foc_sensor_mode;
    p[i++] = m->sensorless_start_failures;
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
    /* Optional hardware-timing diagnostics. They are after the legacy
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

    /* Calibration diagnostics keep the established field prefix and only
       append new timing/synchronized-enable fields at the end. */
    foc_cal_diag_t cd; foc_get_calibration_diag(&cd);
    p[i++] = 16U; /* calibration diagnostic revision */
    put_u16(p, &i, cd.warn_mask);
    put_u16(p, &i, cd.fail_range_mask);
    put_u16(p, &i, cd.fail_noise_mask);
    for (uint8_t k=0U;k<6U;k++) {
        put_i32(p, &i, cd.ch[k].mean);
        put_u16(p, &i, cd.ch[k].min);
        put_u16(p, &i, cd.ch[k].max);
        put_u32(p, &i, cd.ch[k].variance_x100);
    }
    /* Register snapshots retain legacy TIM2 plus active TIM8/ADC/DMA timing for one-file audits. */
    put_u32(p, &i, RCC->CFGR);
    put_u32(p, &i, ADC1->CR1); put_u32(p, &i, ADC1->CR2); put_u32(p, &i, ADC1->SQR1); put_u32(p, &i, ADC1->SQR3);
    put_u32(p, &i, ADC2->CR1); put_u32(p, &i, ADC2->CR2); put_u32(p, &i, ADC2->SQR1); put_u32(p, &i, ADC2->SQR3);
    put_u32(p, &i, DMA1_Channel1->CCR); put_u32(p, &i, DMA1_Channel1->CNDTR); put_u32(p, &i, DMA1->ISR);
    put_u32(p, &i, TIM1->CR1); put_u32(p, &i, TIM1->ARR); put_u32(p, &i, TIM1->CNT); put_u32(p, &i, TIM1->BDTR);
    put_u32(p, &i, TIM8->CR1); put_u32(p, &i, TIM8->ARR); put_u32(p, &i, TIM8->CNT); put_u32(p, &i, TIM8->BDTR);
    put_u32(p, &i, TIM2->CR1); put_u32(p, &i, TIM2->SMCR); put_u32(p, &i, TIM2->CCR2); put_u32(p, &i, TIM2->CNT);
    for (uint8_t k=0U;k<6U;k++) put_u32(p, &i, g_adc_dual_dma[k]);

    /* VESC-style driven/undriven current-offset diagnostics plus the
       first active-drive over-current snapshot. Appended after fields so
       older debug clients still parse the legacy prefix. */
    p[i++] = (uint8_t)cd.stage;
    put_u16(p, &i, cd.shift_warn_mask);
    for (uint8_t k=0U;k<6U;k++) put_i32(p, &i, cd.undriven_mean[k]);
    for (uint8_t k=0U;k<6U;k++) put_i32(p, &i, cd.driven_mean[k]);

    foc_fault_snapshot_t fs; foc_get_fault_snapshot(&fs);
    p[i++] = fs.valid; p[i++] = fs.motor; p[i++] = fs.fault; p[i++] = fs.cal_stage;
    put_u16(p, &i, fs.raw_u); put_u16(p, &i, fs.raw_v); put_u16(p, &i, fs.raw_dc);
    put_i32(p, &i, fs.offset_u); put_i32(p, &i, fs.offset_v); put_i32(p, &i, fs.offset_dc);
    put_i32(p, &i, fs.ia_q15); put_i32(p, &i, fs.ib_q15); put_i32(p, &i, fs.ic_q15);
    put_i32(p, &i, fs.trip_q15); put_i32(p, &i, fs.id_target_q15); put_i32(p, &i, fs.iq_target_q15);
    put_u16(p, &i, fs.ccr1); put_u16(p, &i, fs.ccr2); put_u16(p, &i, fs.ccr3);
    put_u16(p, &i, fs.tim_cnt); put_u16(p, &i, fs.dma_cndtr); put_u32(p, &i, fs.adc_isr_count);

    /* Fault edge state and immutable ADC/PWM schedule metadata. */
    put_u16(p, &i, fs.blank_cycles);
    p[i++] = fs.pwm_enabled; p[i++] = fs.moe; p[i++] = fs.pending_events; p[i++] = fs.reserved;
    put_u16(p, &i, (uint16_t)ADC_MOTOR_PHASE_OFFSET_TICKS);
    put_u32(p, &i, FOC_ISR_EVENT_HZ);
    put_u32(p, &i, FOC_ISR_SLOT_CYCLES);
    put_u16(p, &i, (uint16_t)TIM8->RCR);
    put_u16(p, &i, (uint16_t)TIM1->CNT);
    put_u16(p, &i, (uint16_t)TIM8->CNT);

    /* Revision 16: independent ADC3/DMA2 Vbus path. Appended strictly after
       the rev-15 prefix so older host tools can stop parsing safely. */
    p[i++] = (ADC3->CR2 & ADC_CR2_ADON) ? 1U : 0U;
    p[i++] = ((DMA2_Channel5->CCR & DMA_CCR_EN) != 0U) ? 1U : 0U;
    put_u16(p, &i, (uint16_t)DMA2_Channel5->CNDTR);
    put_u16(p, &i, g_adc3_vbus_dma[0]);
    put_u16(p, &i, g_adc3_vbus_dma[1]);
    p[i++] = foc_vbus_dma_stale_count();
    p[i++] = 0U;
    put_u32(p, &i, foc_vbus_dma_stale_events());
    put_u32(p, &i, ADC3->CR1); put_u32(p, &i, ADC3->CR2);
    put_u32(p, &i, ADC3->SQR1); put_u32(p, &i, ADC3->SQR3);
    put_u32(p, &i, DMA2_Channel5->CCR); put_u32(p, &i, DMA2_Channel5->CNDTR);
    put_u32(p, &i, DMA2->ISR);
    payload_end(i);
}

static void reply_config_status(bool last_save_ok) {
    /* Legacy payload already exceeded 16 bytes (21 bytes before Stage2), so
       keep explicit headroom and append boot-state without stack overwrite. */
    uint8_t p[32]; uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_CONFIG_STATUS;
    p[i++] = conf_general_is_valid() ? 1U : 0U; p[i++] = last_save_ok ? 1U : 0U;
    put_u32(p, &i, conf_general_get_save_count()); put_u32(p, &i, timeout_get_timeout_ms());
    p[i++] = conf_general_integrity_ok() ? 1U : 0U;
    put_u32(p, &i, conf_general_get_integrity_checks());
    put_u32(p, &i, conf_general_get_integrity_failures());
    p[i++] = (uint8_t)conf_general_boot_status();
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
        /* Pemilihan eksplisit untuk tool diagnostik custom. UART standar tetap
         * motor-1; motor-2 dapat dicapai melalui local dual-motor forwarding. */
        reply_sensor_info(explicit_id);
    } else if (sub == CUSTOM_DUAL_SUMMARY) {
        reply_custom_summary();
    } else if (sub == CUSTOM_CLEAR_FAULT && len >= 2U) {
        motor_clear_fault(motor_get(explicit_id));
    } else if (sub == CUSTOM_STOP && len >= 2U) {
        app_command_release(explicit_id, true);
    } else if ((sub == CUSTOM_SENSOR_SELECT || sub == CUSTOM_SENSOR_DETECT) && len >= 3U) {
        MotorRuntime *m = motor_get(explicit_id); uint8_t mode = data[2];
        if (sub == CUSTOM_SENSOR_DETECT || mode == SENSOR_MODE_AUTO) {
            /* Detection is blocking in upstream VESC and must never run in the
             * UART packet thread. Queue it on the same comm_block worker used
             * by COMM_DETECT_ENCODER / COMM_DETECT_HALL_FOC. */
            float current = SENSOR_DETECT_CURRENT_A;
            if (len >= 7U) current = (float)get_i32_be(&data[3]) / 1000.0f;
            blocking_job_t job = {0};
            job.cmd = INTERNAL_CUSTOM_SENSOR_DETECT;
            job.motor = (uint8_t)m->id;
            job.len = 5U;
            job.data[0] = mode;
            int32_t ca = scaled_i32(current, 1000.0f);
            job.data[1] = (uint8_t)((uint32_t)ca >> 24);
            job.data[2] = (uint8_t)((uint32_t)ca >> 16);
            job.data[3] = (uint8_t)((uint32_t)ca >> 8);
            job.data[4] = (uint8_t)ca;
            if (osMessageQueuePut(s_block_queue, &job, 0U, 0U) != osOK) {
                s_diag.blocking_busy_drops++;
                commands_send_print("VESC F103: detection worker busy.");
            }
        } else {
            if (motor_select_sensor_mode(m, mode)) {
                /* Explicit sensor selection is a configuration change, not a
                   temporary detect operation. Mirror it into the VESC wire
                   image and EEPROM emulation immediately so Encoder<->Hall
                   survives a cold boot exactly like COMM_SET_MCCONF. */
                if (!vesc_config_commit_motor_runtime(m->id)) {
                    commands_send_print("VESC F103: sensor config persistence failed; previous MCCONF restored.");
                }
            }
        }
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
        bool ok = conf_general_store_all(); reply_config_status(ok);
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
        case COMM_SET_MCCONF:
        case COMM_SET_APPCONF:
        case COMM_SET_APPCONF_NO_STORE:
        case COMM_TERMINAL_CMD:
        case COMM_SET_MCCONF_TEMP:
        case COMM_SET_MCCONF_TEMP_SETUP:
        case COMM_SET_BATTERY_CUT:
            return true;
        default:
            return false;
    }
}

static bool blocking_command_length_valid(uint8_t cmd, uint16_t len) {
    switch (cmd) {
        case COMM_SET_MCCONF:
            return len == (uint16_t)(1U + VESC6_MCCONF_WIRE_SIZE);
        case COMM_SET_APPCONF:
        case COMM_SET_APPCONF_NO_STORE:
            return len == (uint16_t)(1U + VESC6_APPCONF_WIRE_SIZE);
        case COMM_TERMINAL_CMD:
            return len >= 1U && len <= 241U;
        case COMM_SET_BATTERY_CUT:
            /* command + start/end int32(1e3) + store + forward-all flag */
            return len == 11U;
        case COMM_SET_MCCONF_TEMP:
        case COMM_SET_MCCONF_TEMP_SETUP:
            /* command byte + four flag bytes + eight float32_auto values.
             * Two optional input-current values may follow. */
            return len >= 37U && len <= 45U;
        case COMM_DETECT_MOTOR_R_L:
            return len == 1U;
        case COMM_DETECT_ENCODER:
        case COMM_DETECT_HALL_FOC:
            return len == 5U;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE:
            return len == 17U;
        case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
            return len == 21U;
        case COMM_DETECT_APPLY_ALL_FOC:
            return len == 22U;
        case COMM_DETECT_MOTOR_PARAM:
            /* Legacy BLDC detect is unsupported here; accept its command byte
               so the worker can return the explicit unsupported response. */
            return len >= 1U;
        default:
            return true;
    }
}

static bool queue_blocking_job(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len == 0U || s_block_queue == NULL || len > BLOCK_DATA_MAX) return false;
    if (!blocking_command_length_valid(data[0], len)) return false;
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
        if (id == MOTOR_RIGHT) p[1U + 4U] = VESC_LOCAL_MOTOR2_FORWARD_ID;
        payload_end((uint16_t)(1U + VESC6_APPCONF_WIRE_SIZE));
    }
}

static void reply_ack(uint8_t cmd) {
    uint8_t p[1] = {cmd};
    vesc_comm_send_payload(p, 1U);
}

static void reply_detect_hall_standard(const uint8_t hall_table[8], bool ok) {
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_DETECT_HALL_FOC;
    for (uint8_t k = 0U; k < 8U; k++) p[i++] = (ok && hall_table != NULL) ? hall_table[k] : 255U;
    /* VESC COMM_DETECT_HALL_FOC appends the detector boolean result:
       true/success is 1, false/failure is 0. */
    p[i++] = ok ? 1U : 0U;
    payload_end(i);
}

static void reply_detect_encoder_standard(float offset_deg, float ratio,
                                          bool inverted, bool ok) {
    uint8_t *p = payload_begin(); if (p == NULL) return;
    uint16_t i = 0U; p[i++] = COMM_DETECT_ENCODER;
    if (ok && isfinite(offset_deg) && isfinite(ratio) && ratio > 0.0f) {
        put_i32(p, &i, scaled_i32(offset_deg, 1000000.0f));
        put_i32(p, &i, scaled_i32(ratio, 1000000.0f));
        p[i++] = inverted ? 1U : 0U;
    } else {
        /* VESC encoder-detect failure sentinel: offset > 1000 degrees. */
        put_i32(p, &i, 1001000000L);
        put_i32(p, &i, 0);
        p[i++] = 0U;
    }
    payload_end(i);
}


static void reply_detect_rl(float r_ohm, float l_h, float ld_lq_h, bool ok) {
    uint8_t p[13]; uint16_t i=0U; p[i++]=COMM_DETECT_MOTOR_R_L;
    /* Current VESC Tool decodes R at 1e6 and L/Ld-Lq at 1e3, with L values
       expressed in microhenry on this command. */
    put_i32(p,&i,ok?scaled_i32(r_ohm,1.0e6f):0);
    put_i32(p,&i,ok?scaled_i32(l_h*1.0e6f,1.0e3f):0);
    put_i32(p,&i,ok?scaled_i32(ld_lq_h*1.0e6f,1.0e3f):0);
    vesc_comm_send_payload(p,i);
}

static void reply_detect_flux(uint8_t cmd, float flux_wb, bool ok) {
    uint8_t p[5]; uint16_t i=0U; p[i++]=cmd;
    put_i32(p,&i,ok?scaled_i32(flux_wb,1.0e7f):0);
    vesc_comm_send_payload(p,i);
}

static void reply_unsupported_detect(uint8_t cmd);

static void reply_detect_motor_param_bldc(MotorRuntime *m, bool run_ok) {
    (void)m;
    (void)run_ok;
    reply_unsupported_detect(COMM_DETECT_MOTOR_PARAM);
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
    commands_send_print("VESC F103: legacy BLDC motor-param detect is unsupported; use FOC R/L/flux detect.");
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

static bool force_current_calibration_valid(uint32_t timeout_ms) {
    /* Full Detect-All deliberately takes a fresh six-channel offset snapshot.
       The shared ADC calibration covers LEFT/RIGHT phase and DC-current paths,
       including the driven zero-vector stages. Wait for the new transaction,
       not the previous s_cal_done result. */
    motor_stop(&g_motor_left);
    motor_stop(&g_motor_right);
    foc_request_recalibration();
    uint32_t start = osKernelGetTickCount();
    bool saw_active = false;
    while ((uint32_t)(osKernelGetTickCount() - start) < timeout_ms) {
        if (!foc_calibration_done()) saw_active = true;
        if (saw_active && foc_calibration_done()) return foc_calibration_valid();
        osDelay(2U);
    }
    return false;
}


static void apply_hall_detect_result(MotorRuntime *m, const uint8_t table[8]) {
    typedef struct { uint8_t raw; uint16_t ang; } hall_item_t;
    hall_item_t items[6];
    uint8_t n = 0U;
    for (uint8_t raw = 0U; raw < 8U; raw++) {
        uint8_t v = table[raw];
        m->foc_hall_table[raw] = v;
        if (v == 255U) {
            m->hall_angle_u16[raw] = 0U;
            m->hall_table[raw] = -1;
        } else {
            uint16_t ang = (uint16_t)(((uint32_t)v * 65536U) / 200U);
            m->hall_angle_u16[raw] = ang;
            if (n < 6U) { items[n].raw = raw; items[n].ang = ang; n++; }
        }
    }
    for (uint8_t i = 0U; i < n; i++) {
        for (uint8_t j = (uint8_t)(i + 1U); j < n; j++) {
            if (items[j].ang < items[i].ang) {
                hall_item_t t = items[i]; items[i] = items[j]; items[j] = t;
            }
        }
    }
    for (uint8_t i = 0U; i < n; i++) m->hall_table[items[i].raw] = (int8_t)i;
    motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
    m->sensor_mode = SENSOR_MODE_HALL;
    m->sensor_request_mode = SENSOR_MODE_HALL;
    m->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
    m->stats.tachometer_source_valid = false;
    m->hall.valid = false;
    m->hall.sector = -1;
    m->hall.phase_per_cycle_q16 = 0;
    motor_hall_edge_isr(m);
}

static void apply_encoder_detect_result(MotorRuntime *m, float offset_deg,
                                        float ratio, bool inverted) {
    if (!m || m->id != MOTOR_LEFT || !isfinite(ratio) || ratio < 1.0f) return;
    motor_hw_configure_sensor(m, SENSOR_MODE_ENCODER);
    m->encoder.electrical_ratio = ratio;
    m->encoder.electrical_ratio_q16 = (uint32_t)lroundf(ratio * 65536.0f);
    m->encoder.elec_offset_u16 = foc_deg_to_u16(offset_deg);
    m->encoder.inverted = inverted;
    if (m->encoder.cpr > 0U) {
        m->encoder.phase_per_count_q16 =
            (uint32_t)(((uint64_t)m->encoder.electrical_ratio_q16 << 16) /
                       (uint64_t)m->encoder.cpr);
    }
    /* A/B is incremental: applying persistent ratio/offset does not make the
       cold-boot runtime counter absolute. Normal ENCODER_AB startup must sync
       it from the observer before encoder low-speed control is used. */
    m->encoder.synced = false;
    m->encoder.motion_proved = false;
    m->encoder.sync_active = false;
    m->using_encoder = false;
    m->sensor_mode = SENSOR_MODE_ENCODER;
    m->sensor_request_mode = SENSOR_MODE_ENCODER;
    m->foc_sensor_mode = FOC_SENSOR_MODE_ENCODER_AB;
    m->stats.tachometer_source_valid = false;
}

static void apply_sensorless_result(MotorRuntime *m) {
    if (!m) return;
    /* There is no phase-voltage sensor peripheral to mux. Keep Hall GPIOs as
       harmless inputs (and release LEFT TIM4 when needed), while FOC phase and
       speed ownership are explicitly observer-only. */
    if (m->id == MOTOR_LEFT) encoder_deinit(m);
    motor_hw_configure_sensor(m, SENSOR_MODE_HALL);
    m->sensor_request_mode = SENSOR_MODE_HALL;
    m->foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
    m->using_encoder = false;
    m->stats.tachometer_source_valid = false;
    foc_sensorless_startup_abort(m);
    foc_observer_reset(m, m->observer_phase_u16);
}

static bool detect_failure_is_sensor_absent(MotorRuntime *m, int fault) {
    return m != NULL && m->fault == MOTOR_FAULT_NONE &&
           fault == MOTOR_FAULT_SENSOR_DETECT;
}

static bool validate_sensorless_runtime(MotorRuntime *m, float detect_current_a) {
    if (m == NULL || m->fault != MOTOR_FAULT_NONE ||
        m->foc_sensor_mode != FOC_SENSOR_MODE_SENSORLESS) return false;

    /* Sensorless fallback is a real detected result only after the observer
       proves that it can acquire the motor. The motor-service task owns the
       VESC-style forced-startup/blend state machine; this blocking worker only
       applies a small positive-current request and watches the resulting
       observer state. Ordinary APP ADC/UART inputs are already masked by the
       Detect-All input-ignore window. */
    float test_current = fabsf(detect_current_a);
    const float current_cap = fmaxf(fminf(fabsf(m->current_max_a) * 0.25f, 5.0f), 0.5f);
    if (test_current > current_cap) test_current = current_cap;
    if (test_current < 0.5f) test_current = 0.5f;

    m->sensorless_start_failures = 0U;
    foc_sensorless_startup_abort(m);
    foc_observer_reset(m, m->observer_phase_u16);
    motor_set_current(m, test_current);

    const uint32_t start = osKernelGetTickCount();
    uint32_t stable_ms = 0U;
    bool ok = false;
    while ((uint32_t)(osKernelGetTickCount() - start) < 8000U) {
        if (m->fault != MOTOR_FAULT_NONE || m->sensorless_start_failures >= 3U) break;
        int32_t sp = m->speed_est_fast_erpm_q16;
        if (sp < 0) sp = (sp == INT32_MIN) ? INT32_MAX : -sp;
        const bool acquired = m->observer_valid &&
                              sp >= m->foc_sl_erpm_q16 &&
                              !m->phase_observer_override &&
                              !m->openloop_started;
        if (acquired) {
            stable_ms += 10U;
            if (stable_ms >= 150U) { ok = true; break; }
        } else {
            stable_ms = 0U;
        }
        osDelay(10U);
    }

    motor_stop(m);
    foc_sensorless_startup_abort(m);
    if (!ok && m->fault == MOTOR_FAULT_NONE) {
        commands_send_print("Detect-All: sensorless observer failed acquisition validation; not saving fallback.");
    }
    return ok;
}




static int16_t detect_apply_all_one_runtime(MotorRuntime *m, float max_power_loss,
                                            float min_input_current, float max_input_current,
                                            float openloop, float sl) {
    if (m == NULL) return -1;

    m->foc_openloop_rpm = fabsf(openloop);
    m->foc_sl_erpm = fabsf(sl);

    float detect_current = fabsf(m->current_max_a) / 3.0f;
    if (detect_current < 1.0f) detect_current = FOC_DETECT_CURRENT_A;
    if (detect_current > FOC_DETECT_MAX_CURRENT_A) detect_current = FOC_DETECT_MAX_CURRENT_A;

    int16_t result = mcpwm_foc_detect_apply_all_motor(m, detect_current);
    if (result != 0) return result;

    /* Match VESC Detect-All intent: derive symmetric motor-current capability
       from measured copper resistance and requested maximum copper loss, while
       never exceeding the board/current-protection envelope. */
    float r = m->foc_motor_r;
    if (!isfinite(r) || r <= 0.00001f) return -2;
    float current_limit = sqrtf(max_power_loss / r);
    float hard_limit = fminf(FOC_MAX_CURRENT_A, m->abs_current_max_a);
    if (!isfinite(current_limit) || current_limit < 0.1f) return -3;
    current_limit = fminf(current_limit, hard_limit);
    m->current_max_a = current_limit;
    m->current_min_a = -current_limit;
    m->input_current_min_a = fmaxf(min_input_current, -FOC_MAX_CURRENT_A);
    m->input_current_max_a = fminf(max_input_current, FOC_MAX_CURRENT_A);

    /* VESC-style sensor discovery is independent of the sensor that happened
       to be selected before Detect-All. Physical sensor absence is not a motor
       fault: LEFT tries ABI -> Hall -> sensorless; RIGHT tries Hall ->
       sensorless. A real latched motor/power-stage fault still aborts. */
    if (m->id == MOTOR_LEFT) {
        float off = 0.0f, rat = 0.0f;
        bool inv = false;
        int sf = mcpwm_foc_encoder_detect_motor(m, SENSOR_DETECT_CURRENT_A, false,
                                                &off, &rat, &inv);
        if (sf == MOTOR_FAULT_NONE) {
            apply_encoder_detect_result(m, off, rat, inv);
            return 0;
        }
        if (!detect_failure_is_sensor_absent(m, sf)) return -11;
    }

    uint8_t hall[8];
    bool hall_ok = false;
    memset(hall, 255, sizeof(hall));
    int hf = mcpwm_foc_hall_detect_motor(m, SENSOR_DETECT_CURRENT_A, hall, &hall_ok);
    if (hf == MOTOR_FAULT_NONE && hall_ok) {
        apply_hall_detect_result(m, hall);
        return 0;
    }
    if (!detect_failure_is_sensor_absent(m, hf)) return -11;

    apply_sensorless_result(m);
    if (!validate_sensorless_runtime(m, detect_current)) return -13;
    return 0;
}

static void rollback_detect_all_runtime_both(void) {
    (void)vesc_config_reapply_active_mc(MOTOR_LEFT);
    (void)vesc_config_reapply_active_mc(MOTOR_RIGHT);
}

static bool battery_cut_build_wire(motor_id_t id, float start, float end, uint8_t out[VESC6_MCCONF_WIRE_SIZE]) {
    mc_configuration c;
    if (!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), &c)) return false;
    c.l_battery_cut_start = start;
    c.l_battery_cut_end = end;
    return confgenerator_serialize_mcconf_motor(out, &c, id) == (int32_t)VESC6_MCCONF_WIRE_SIZE;
}

static bool battery_cut_apply_one(motor_id_t id, float start, float end, bool store) {
    if (!battery_cut_build_wire(id, start, end, s_mc_work)) return false;
    return vesc_config_set_mc_wire(id, s_mc_work, VESC6_MCCONF_WIRE_SIZE, store);
}

static bool battery_cut_apply_both(float start, float end, bool store) {
    memcpy(s_mc_backup[MOTOR_LEFT], vesc_config_mc_wire(MOTOR_LEFT, false), VESC6_MCCONF_WIRE_SIZE);
    memcpy(s_mc_backup[MOTOR_RIGHT], vesc_config_mc_wire(MOTOR_RIGHT, false), VESC6_MCCONF_WIRE_SIZE);
    if (!battery_cut_build_wire(MOTOR_LEFT, start, end, s_mc_work) ||
        !vesc_config_set_mc_wire(MOTOR_LEFT, s_mc_work, VESC6_MCCONF_WIRE_SIZE, false)) return false;
    if (!battery_cut_build_wire(MOTOR_RIGHT, start, end, s_mc_work) ||
        !vesc_config_set_mc_wire(MOTOR_RIGHT, s_mc_work, VESC6_MCCONF_WIRE_SIZE, false)) {
        (void)vesc_config_set_mc_wire(MOTOR_LEFT, s_mc_backup[MOTOR_LEFT], VESC6_MCCONF_WIRE_SIZE, false);
        return false;
    }
    if (store && !conf_general_store_all()) {
        (void)vesc_config_set_mc_wire(MOTOR_LEFT, s_mc_backup[MOTOR_LEFT], VESC6_MCCONF_WIRE_SIZE, false);
        (void)vesc_config_set_mc_wire(MOTOR_RIGHT, s_mc_backup[MOTOR_RIGHT], VESC6_MCCONF_WIRE_SIZE, false);
        return false;
    }
    return true;
}

static void blocking_thread(void *argument) {
    (void)argument;
    blocking_job_t job;
    for (;;) {
        memset(&job, 0, sizeof(job));
        if (osMessageQueueGet(s_block_queue, &job, NULL, osWaitForever) != osOK) continue;
        MotorRuntime *m = motor_get(job.motor == MOTOR_RIGHT ? MOTOR_RIGHT : MOTOR_LEFT);
        int old_motor = mc_interface_get_motor_thread();
        mc_interface_select_motor_thread(m->id == MOTOR_RIGHT ? 2 : 1);
        if (job.cmd == COMM_SET_MCCONF) {
            bool ok = job.len == (1U + VESC6_MCCONF_WIRE_SIZE) &&
                      vesc_config_set_mc_wire(m->id, &job.data[1],
                                              (uint16_t)(job.len - 1U), true);
            if (ok) reply_ack(COMM_SET_MCCONF);
            else commands_send_print("VESC F103: MCCONF rejected; motor must be OFF and VESC6 signature/layout valid.");
        } else if (job.cmd == COMM_SET_APPCONF || job.cmd == COMM_SET_APPCONF_NO_STORE) {
            const bool store = job.cmd == COMM_SET_APPCONF;
            bool ok = false;
            if (job.len == (1U + VESC6_APPCONF_WIRE_SIZE)) {
                /* The queue owns a private 512-byte copy already. Motor-2 is a
                   fixed local VESC identity, not a mutable CAN node: only an
                   APPCONF image that still identifies it as ID 2 is accepted.
                   After validating that public identity, normalize the shared
                   internal app-config ID to motor-1 before persistence. */
                if (m->id == MOTOR_RIGHT) {
                    if (job.data[1U + VESC6_APP_OFF_CONTROLLER_ID] == VESC_CONTROLLER_ID_RIGHT) {
                        job.data[1U + VESC6_APP_OFF_CONTROLLER_ID] = VESC_CONTROLLER_ID_LEFT;
                        ok = vesc_config_set_app_wire(&job.data[1], VESC6_APPCONF_WIRE_SIZE, store);
                    }
                } else {
                    ok = vesc_config_set_app_wire(&job.data[1], VESC6_APPCONF_WIRE_SIZE, store);
                }
            }
            if (ok) reply_ack(job.cmd);
            else commands_send_print("VESC F103: APPCONF rejected; VESC6 signature/layout invalid.");
        } else if (job.cmd == COMM_TERMINAL_CMD) {
            process_terminal_text(&job.data[1], (uint16_t)(job.len > 0U ? job.len - 1U : 0U), m->id);
        } else if (job.cmd == COMM_SET_MCCONF_TEMP || job.cmd == COMM_SET_MCCONF_TEMP_SETUP) {
            set_mcconf_temp(&job.data[1], (uint16_t)(job.len - 1U), m->id,
                            job.cmd == COMM_SET_MCCONF_TEMP_SETUP);
        } else if (job.cmd == COMM_SET_BATTERY_CUT) {
            float start = (float)get_i32_be(&job.data[1]) / 1000.0f;
            float end = (float)get_i32_be(&job.data[5]) / 1000.0f;
            bool store = job.data[9] != 0U;
            bool forward_all = job.data[10] != 0U;
            bool ok = (m->id == MOTOR_LEFT && forward_all) ?
                      battery_cut_apply_both(start, end, store) :
                      battery_cut_apply_one(m->id, start, end, store);
            if (ok) reply_ack(COMM_SET_BATTERY_CUT);
            else commands_send_print("VESC F103: battery-cut update rejected or persistence failed.");
        } else if (job.cmd == COMM_DETECT_MOTOR_PARAM) {
            /* Legacy six-step BLDC detector is intentionally unavailable in
             * this FOC-only build. Return the canonical unsupported payload;
             * all FOC detection commands below remain fully active. */
            reply_detect_motor_param_bldc(m, false);
        } else if (job.cmd == COMM_DETECT_ENCODER) {
            float current = (float)get_i32_be(&job.data[1]) / 1000.0f;
            float off = 0.0f, rat = 0.0f;
            bool inv = false;
            bool ok = ensure_current_calibration_valid(10000U) &&
                      mcpwm_foc_encoder_detect_motor(m, fabsf(current), false,
                                               &off, &rat, &inv) == MOTOR_FAULT_NONE;
            /* Never source the reply from m->detect: if current calibration
               fails before detect_begin(), that structure may contain a
               successful result from an older transaction. */
            reply_detect_encoder_standard(off, rat, inv, ok);
        } else if (job.cmd == COMM_DETECT_HALL_FOC) {
            float current = (float)get_i32_be(&job.data[1]) / 1000.0f;
            uint8_t hall[8]; memset(hall, 255, sizeof(hall));
            bool hall_ok = false;
            bool ok = ensure_current_calibration_valid(10000U) &&
                      mcpwm_foc_hall_detect_motor(m, fabsf(current), hall, &hall_ok) == MOTOR_FAULT_NONE &&
                      hall_ok;
            reply_detect_hall_standard(hall, ok);
        } else if (job.cmd == COMM_DETECT_MOTOR_R_L) {
            float r = 0.0f, l = 0.0f, ldq = 0.0f;
            bool ok = ensure_current_calibration_valid(10000U) &&
                      mcpwm_foc_measure_res_ind_motor(m, &r, &l, &ldq) == MOTOR_FAULT_NONE;
            reply_detect_rl(r, l, ldq, ok);
        } else if (job.cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE ||
                   job.cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
            /* VESC 6.00 standard payloads:
               normal   = current, min_erpm, duty, resistance
               openloop = current, erpm_per_sec, duty, resistance, inductance */
            float current = (float)get_i32_be(&job.data[1]) / 1000.0f;
            float second = (float)get_i32_be(&job.data[5]) / 1000.0f;
            float duty = (float)get_i32_be(&job.data[9]) / 1000.0f;
            float resistance = (float)get_i32_be(&job.data[13]) / 1000000.0f;
            float inductance = m->detect_rl_valid ? m->detect_inductance_h : m->foc_motor_l;
            if (job.cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
                inductance = (float)get_i32_be(&job.data[17]) / 100000000.0f;
            }

            float flux = 0.0f;
            bool params_ok = isfinite(current) && isfinite(second) && isfinite(duty) &&
                             isfinite(resistance) && isfinite(inductance) &&
                             fabsf(current) >= 0.05f && fabsf(duty) >= 0.01f &&
                             resistance > 0.00001f && inductance >= 0.0f;
            bool ok = false;
            if (params_ok && ensure_current_calibration_valid(10000U)) {
                float target;
                float accel;
                if (job.cmd == COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP) {
                    target = fminf(fmaxf(fabsf(m->max_erpm), FOC_DETECT_FLUX_ERPM), 5000.0f);
                    accel = fmaxf(fabsf(second), 100.0f);
                } else {
                    target = fmaxf(fabsf(second), 400.0f);
                    accel = 3000.0f;
                }
                ok = mcpwm_foc_measure_flux_linkage_motor_bounded(m, current, target, accel,
                            fabsf(duty), resistance, inductance, &flux) == MOTOR_FAULT_NONE;
            }
            reply_detect_flux(job.cmd, flux, ok);
        } else if (job.cmd == COMM_DETECT_APPLY_ALL_FOC) {
            /* VESC6/VESC Tool payload:
               detect_can, max_power_loss, min_input_current, max_input_current,
               openloop_erpm, sl_erpm. On this board detect_can=true means
               "include the local forwarded Motor-2", not physical CAN. */
            int16_t result = -1;
            const bool detect_can = job.data[1] != 0U;
            const float max_power_loss = (float)get_i32_be(&job.data[2]) / 1000.0f;
            const float min_input_current = (float)get_i32_be(&job.data[6]) / 1000.0f;
            const float max_input_current = (float)get_i32_be(&job.data[10]) / 1000.0f;
            const float openloop = (float)get_i32_be(&job.data[14]) / 1000.0f;
            const float sl = (float)get_i32_be(&job.data[18]) / 1000.0f;
            const bool params_ok = isfinite(max_power_loss) && max_power_loss > 0.0f &&
                                   isfinite(min_input_current) && isfinite(max_input_current) &&
                                   min_input_current <= 0.0f && max_input_current >= 0.0f &&
                                   min_input_current <= max_input_current &&
                                   isfinite(openloop) && fabsf(openloop) >= 100.0f &&
                                   isfinite(sl) && fabsf(sl) >= 100.0f;

            if (params_ok) {
                /* Block ordinary duty/current/speed/position commands on both
                   local motors for the complete calibration/detection window. */
                mc_interface_ignore_input_both(180000);
                mc_interface_release_motor_override_both();
                motor_stop(&g_motor_left);
                motor_stop(&g_motor_right);
            }

            if (!params_ok) {
                result = -14;
            } else if (!force_current_calibration_valid(15000U)) {
                result = -15;
            } else if (detect_can && m->id == MOTOR_LEFT) {
                /* VESC Tool Board Setup calls the directly-connected controller
                   with detect_can=true, then reads each discovered MCCONF.
                   Treat local ID2 exactly as the second controller for that
                   transaction. No physical CAN driver is involved. */
                commands_send_print("Detect-All: calibrating local Motor-1/LEFT...");
                result = detect_apply_all_one_runtime(&g_motor_left, max_power_loss,
                                                       min_input_current, max_input_current,
                                                       openloop, sl);
                if (result == 0) {
                    commands_send_print("Detect-All: calibrating local Motor-2/RIGHT...");
                    result = detect_apply_all_one_runtime(&g_motor_right, max_power_loss,
                                                          min_input_current, max_input_current,
                                                          openloop, sl);
                }

                if (result == 0) {
                    if (!vesc_config_commit_detect_all_runtime_dual()) {
                        result = -12;
                    } else {
                        commands_send_print("Detect-All: both local motors validated and saved atomically.");
                    }
                }

                if (result != 0) {
                    /* Active MCCONF wire images are still the previous known-good
                       values until the atomic dual commit. Reapply both so a
                       failed M2 detection cannot leave M1 running detected-but-
                       unsaved parameters (or vice versa). */
                    rollback_detect_all_runtime_both();
                    commands_send_print("Detect-All: failed; both motor runtimes restored to last committed MCCONF.");
                }
            } else {
                /* Individual Detect-All remains valid for the directly selected
                   motor (or forwarded ID2). A forwarded node has no downstream
                   CAN devices, so detect_can=true there still means that motor
                   only. */
                result = detect_apply_all_one_runtime(m, max_power_loss,
                                                      min_input_current, max_input_current,
                                                      openloop, sl);
                if (result == 0) {
                    if (!vesc_config_commit_detect_all_runtime(m->id)) result = -12;
                }
                if (result != 0) (void)vesc_config_reapply_active_mc(m->id);
            }

            if (params_ok) {
                mc_interface_ignore_input_both(0);
                timeout_reset();
            }

            uint8_t p[3];
            uint16_t i = 0U;
            p[i++] = COMM_DETECT_APPLY_ALL_FOC;
            put_i16(p, &i, result);
            vesc_comm_send_payload(p, i);
        } else if (job.cmd == INTERNAL_CUSTOM_SENSOR_DETECT) {
            uint8_t mode = job.len >= 1U ? job.data[0] : SENSOR_MODE_AUTO;
            float current = job.len >= 5U ?
                    (float)get_i32_be(&job.data[1]) / 1000.0f : SENSOR_DETECT_CURRENT_A;
            bool applied = false;
            /* Sensor discovery/calibration has exclusive motor ownership just
               like Detect-All. This prevents APP_ADC/APP_UART from replacing
               the excitation command while sensorless fallback is validated. */
            mc_interface_ignore_input_both(30000U);
            motor_stop(&g_motor_left);
            motor_stop(&g_motor_right);
            if (ensure_current_calibration_valid(10000U)) {
                if (mode == SENSOR_MODE_ENCODER ||
                    (mode == SENSOR_MODE_AUTO && m->id == MOTOR_LEFT)) {
                    float off = 0.0f, rat = 0.0f; bool inv = false;
                    int sf = mcpwm_foc_encoder_detect_motor(m, current, false,
                                                             &off, &rat, &inv);
                    if (sf == MOTOR_FAULT_NONE) {
                        apply_encoder_detect_result(m, off, rat, inv);
                        applied = true;
                    } else if (mode == SENSOR_MODE_ENCODER ||
                               !detect_failure_is_sensor_absent(m, sf)) {
                        /* Explicit encoder request stays explicit; real motor
                           faults also stop AUTO rather than being hidden. */
                        mode = SENSOR_MODE_ENCODER;
                    }
                }
                if (!applied && mode != SENSOR_MODE_ENCODER) {
                    uint8_t hall[8]; bool hall_ok = false;
                    int hf = mcpwm_foc_hall_detect_motor(m, current, hall, &hall_ok);
                    if (hf == MOTOR_FAULT_NONE && hall_ok) {
                        apply_hall_detect_result(m, hall);
                        applied = true;
                    } else if (mode == SENSOR_MODE_AUTO &&
                               detect_failure_is_sensor_absent(m, hf)) {
                        apply_sensorless_result(m);
                        applied = validate_sensorless_runtime(m, current);
                    }
                }
            }
            mc_interface_ignore_input_both(0U);
            timeout_reset();
            if (applied && !vesc_config_commit_motor_runtime(m->id)) {
                commands_send_print("VESC F103: detected sensor config persistence failed; previous MCCONF restored.");
            }
            reply_sensor_info(m->id);
        } else {
            reply_unsupported_detect(job.cmd);
        }
        mc_interface_select_motor_thread(old_motor);
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


static void process_terminal_text(const uint8_t *data,uint16_t len,motor_id_t id) {
    char line[242];if(!data)return;if(len>=sizeof(line))len=(uint16_t)(sizeof(line)-1U);memcpy(line,data,len);line[len]='\0';
    int old=mc_interface_get_motor_thread();mc_interface_select_motor_thread(id==MOTOR_RIGHT?2:1);terminal_process_string(line);mc_interface_select_motor_thread(old);
}

static void reply_stats(const uint8_t *data,uint16_t len,motor_id_t id) {
    if (data == NULL || len < 2U) return;
    int32_t bi = 0;
    uint32_t mask = (uint32_t)buffer_get_uint16(data, &bi);
    uint8_t p[60];int32_t i=0;p[i++]=COMM_GET_STATS;buffer_append_uint32(p,mask,&i);
    int old=mc_interface_get_motor_thread();mc_interface_select_motor_thread(id==MOTOR_RIGHT?2:1);
    if(mask&(1UL<<0))buffer_append_float32_auto(p,mc_interface_stat_speed_avg(),&i);
    if(mask&(1UL<<1))buffer_append_float32_auto(p,mc_interface_stat_speed_max(),&i);
    if(mask&(1UL<<2))buffer_append_float32_auto(p,mc_interface_stat_power_avg(),&i);
    if(mask&(1UL<<3))buffer_append_float32_auto(p,mc_interface_stat_power_max(),&i);
    if(mask&(1UL<<4))buffer_append_float32_auto(p,mc_interface_stat_current_avg(),&i);
    if(mask&(1UL<<5))buffer_append_float32_auto(p,mc_interface_stat_current_max(),&i);
    if(mask&(1UL<<6))buffer_append_float32_auto(p,mc_interface_stat_temp_mosfet_avg(),&i);
    if(mask&(1UL<<7))buffer_append_float32_auto(p,mc_interface_stat_temp_mosfet_max(),&i);
    if(mask&(1UL<<8))buffer_append_float32_auto(p,mc_interface_stat_temp_motor_avg(),&i);
    if(mask&(1UL<<9))buffer_append_float32_auto(p,mc_interface_stat_temp_motor_max(),&i);
    if(mask&(1UL<<10))buffer_append_float32_auto(p,mc_interface_stat_count_time(),&i);
    mc_interface_select_motor_thread(old);vesc_comm_send_payload(p,(uint16_t)i);
}

static bool temp_conf_apply_one(motor_id_t id,mc_configuration *c,bool store) {
    static uint8_t wire[VESC6_MCCONF_WIRE_SIZE];
    if(confgenerator_serialize_mcconf_motor(wire,c,id)!=(int32_t)VESC6_MCCONF_WIRE_SIZE)return false;
    return vesc_config_set_mc_wire(id,wire,VESC6_MCCONF_WIRE_SIZE,store);
}

static void set_mcconf_temp(const uint8_t *d,uint16_t len,motor_id_t id,bool setup) {
    if (d == NULL || len < 36U) return;
    int32_t i = 0;
    bool store = d[i++] != 0U;
    bool forward = d[i++] != 0U;
    bool ack = d[i++] != 0U;
    bool divide = d[i++] != 0U;
    (void)forward;
    mc_configuration c;if(!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id,false),&c))return;
    float controllers=divide?2.0f:1.0f;
    c.l_current_min_scale=foc_clampf(buffer_get_float32_auto(d,&i),0.0f,1.0f);
    c.l_current_max_scale=foc_clampf(buffer_get_float32_auto(d,&i),0.0f,1.0f);
    float mn=buffer_get_float32_auto(d,&i),mx=buffer_get_float32_auto(d,&i);
    if(setup){float fact=((float)c.si_motor_poles*0.5f*60.0f*c.si_gear_ratio)/(c.si_wheel_diameter*3.14159265358979323846f);if(!isfinite(fact)||fact<=0.0f)return;mn*=fact;mx*=fact;}
    c.l_min_erpm=mn;c.l_max_erpm=mx;c.l_min_duty=buffer_get_float32_auto(d,&i);c.l_max_duty=buffer_get_float32_auto(d,&i);
    c.l_watt_min=buffer_get_float32_auto(d,&i)/controllers;c.l_watt_max=buffer_get_float32_auto(d,&i)/controllers;
    if((uint16_t)(i+8)<=len){c.l_in_current_min=buffer_get_float32_auto(d,&i);c.l_in_current_max=buffer_get_float32_auto(d,&i);}
    bool ok=temp_conf_apply_one(id,&c,store);if(ack&&ok)reply_ack(setup?COMM_SET_MCCONF_TEMP_SETUP:COMM_SET_MCCONF_TEMP);
    if(!ok)commands_send_print("VESC F103: temporary MCCONF rejected by hardware ownership/limit validation.");
}

static void reply_mcconf_temp(motor_id_t id) {
    mc_configuration c;if(!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id,false),&c))return;
    uint8_t p[60];int32_t i=0;p[i++]=COMM_GET_MCCONF_TEMP;
    buffer_append_float32_auto(p,c.l_current_min_scale,&i);buffer_append_float32_auto(p,c.l_current_max_scale,&i);
    buffer_append_float32_auto(p,c.l_min_erpm,&i);buffer_append_float32_auto(p,c.l_max_erpm,&i);
    buffer_append_float32_auto(p,c.l_min_duty,&i);buffer_append_float32_auto(p,c.l_max_duty,&i);
    buffer_append_float32_auto(p,c.l_watt_min,&i);buffer_append_float32_auto(p,c.l_watt_max,&i);
    buffer_append_float32_auto(p,c.l_in_current_min,&i);buffer_append_float32_auto(p,c.l_in_current_max,&i);
    p[i++]=c.si_motor_poles;buffer_append_float32_auto(p,c.si_gear_ratio,&i);buffer_append_float32_auto(p,c.si_wheel_diameter,&i);
    vesc_comm_send_payload(p,(uint16_t)i);
}

static void process_payload_for_motor(const uint8_t *data, uint16_t len, motor_id_t id) {
    if (data == NULL || len == 0U) return;
    uint8_t cmd = data[0];
    MotorRuntime *m = motor_get(id);

    /* COMM_FW_VERSION and the local motor-2 forwarding FW_VERSION path must work even when
     * ADC/PWM/FOC initialization has failed. Motor-driving and detection
     * commands stay inhibited until motor_boot_thread completes. */
    if (s_shutdown_latched && command_requires_motor_ready(cmd)) return;
    if (!s_motor_ready && command_requires_motor_ready(cmd)) {
        return;
    }
    /* Config recovery must remain available even if telemetry/control task
       startup fails. The flag is raised only after motor_control_init() and
       config initialization, so SET_MCCONF can safely stop/apply the runtime. */
    if ((cmd == COMM_SET_MCCONF || cmd == COMM_SET_APPCONF ||
         cmd == COMM_SET_APPCONF_NO_STORE || cmd == COMM_SET_BATTERY_CUT) && !s_config_ready) {
        return;
    }

    if (is_blocking_command(cmd)) {
        if (!queue_blocking_job(data, len, id)) {
            if (!blocking_command_length_valid(cmd, len)) {
                commands_send_print("VESC F103: rejected malformed blocking command length.");
            }
        }
        return;
    }

    switch (cmd) {
        case COMM_FW_VERSION:
            reply_fw_version(id);
            break;
        case COMM_FW_INFO:
            reply_fw_info();
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
        case COMM_PING_CAN:
            if (id == MOTOR_LEFT) {
                /* VESC Tool discovers additional controllers by issuing
                 * COMM_PING_CAN on the directly-connected controller. This
                 * board has no physical CAN PHY, but it is a true dual-motor
                 * target. Advertise only the local second-motor ID; the
                 * directly-connected motor-1 is already represented by the
                 * serial connection and must not be duplicated in the scan. */
                uint8_t p[2] = {COMM_PING_CAN, VESC_LOCAL_MOTOR2_FORWARD_ID};
                vesc_comm_send_payload(p, 2U);
            } else {
                /* A forwarded ping is not a scan of another CAN bus. Return an
                 * empty list, matching a node with no downstream devices. */
                uint8_t p[1] = {COMM_PING_CAN};
                vesc_comm_send_payload(p, 1U);
            }
            break;
        case COMM_SET_DUTY:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_duty(m, (float)get_i32_be(&data[1]) / 100000.0f);
                timeout_reset();
            }
            break;
        case COMM_SET_CURRENT:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_current(m, (float)get_i32_be(&data[1]) / 1000.0f);
                timeout_reset();
            }
            break;
        case COMM_SET_CURRENT_BRAKE:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_brake_current(m, (float)get_i32_be(&data[1]) / 1000.0f);
                timeout_reset();
            }
            break;
        case COMM_SET_RPM:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_speed(m, (float)get_i32_be(&data[1]));
                timeout_reset();
            }
            break;
        case COMM_SET_POS:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_position(m, (float)get_i32_be(&data[1]) / 1000000.0f);
                timeout_reset();
            }
            break;
        case COMM_SET_HANDBRAKE:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_handbrake(m, (float)get_i32_be(&data[1]) / 1000.0f);
                timeout_reset();
            }
            break;
        case COMM_SET_CURRENT_REL:
            if (len >= 5U && mc_interface_try_input_motor(id) && app_command_uart_claim(id)) {
                motor_set_current_rel(m, (float)get_i32_be(&data[1]) / 100000.0f);
                timeout_reset();
            }
            break;
        case COMM_GET_DECODED_ADC: {
            /* Canonical VESC payload: decoded ADC1, voltage ADC1, decoded ADC2,
             * voltage ADC2, all scaled by 1e6. Values come from real PA2/PA3
             * samples. Before the first complete rank-6 sample there is no
             * truthful value to report, so do not manufacture a zero reply. */
            if (!app_adc_data_ready()) break;
            uint8_t p[17];
            uint16_t bi = 0U;
            p[bi++] = COMM_GET_DECODED_ADC;
            put_i32(p, &bi, scaled_i32(app_adc_get_decoded_level(), 1000000.0f));
            put_i32(p, &bi, scaled_i32(app_adc_get_voltage(), 1000000.0f));
            put_i32(p, &bi, scaled_i32(app_adc_get_decoded_level2(), 1000000.0f));
            put_i32(p, &bi, scaled_i32(app_adc_get_voltage2(), 1000000.0f));
            vesc_comm_send_payload(p, bi);
        } break;
        case COMM_GET_BATTERY_CUT: {
            mc_configuration c;
            if (confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), &c)) {
                uint8_t p[9]; uint16_t bi = 0U; p[bi++] = COMM_GET_BATTERY_CUT;
                put_i32(p, &bi, scaled_i32(c.l_battery_cut_start, 1000.0f));
                put_i32(p, &bi, scaled_i32(c.l_battery_cut_end, 1000.0f));
                vesc_comm_send_payload(p, bi);
            }
        } break;
        case COMM_SET_DETECT:
            if (len >= 2U && data[1] <= (uint8_t)DISP_POS_MODE_HALL_OBSERVER_ERROR) {
                /* One USART transport can carry only one untagged
                 * COMM_ROTOR_POSITION stream at a time. A forwarded motor-2
                 * request therefore becomes the stream owner instead of
                 * interleaving two indistinguishable command-22 replies. */
                uint8_t index = (id == MOTOR_RIGHT) ? 1U : 0U;
                if (data[1] == (uint8_t)DISP_POS_MODE_NONE) {
                    s_display_mode[index] = (uint8_t)DISP_POS_MODE_NONE;
                    if (s_display_owner == (int8_t)index) s_display_owner = -1;
                } else {
                    uint8_t other = (uint8_t)(index ^ 1U);
                    s_display_mode[other] = (uint8_t)DISP_POS_MODE_NONE;
                    s_display_mode[index] = data[1];
                    s_display_owner = (int8_t)index;
                }
            }
            break;
        case COMM_SAMPLE_PRINT:
            if (len >= 5U) {
                uint8_t mode = data[1];
                uint16_t sample_len = get_u16_be(&data[2]);
                uint16_t decimation = data[4];
                bool raw = (len >= 6U) ? (data[5] != 0U) : false;
                if (mode <= (uint8_t)DEBUG_SAMPLING_SEND_SINGLE_SAMPLE) {
                    if (!debug_sample_control((debug_sampling_mode)mode, id,
                                              sample_len, decimation, raw)) {
                        commands_send_print("VESC F103: sample request busy or no previous capture.");
                    }
                }
            }
            break;
        case COMM_FORWARD_CAN:
            if (len >= 3U) {
                uint8_t target = data[1];
                if (target == VESC_LOCAL_MOTOR2_FORWARD_ID) {
                    s_diag.motor2_forwards++;
                    process_payload_for_motor(&data[2], (uint16_t)(len - 2U), MOTOR_RIGHT);
                } else {
                    /* Tidak ada CAN PHY/driver pada build ini. ID selain motor-2
                     * lokal sengaja tidak diteruskan dan tidak diberi reply palsu. */
                    s_diag.unsupported_forward_ids++;
                }
            }
            break;
        case COMM_TERMINAL_CMD_SYNC:
            process_terminal_text(&data[1], (uint16_t)(len > 0U ? len - 1U : 0U), id);
            break;
        case COMM_APP_DISABLE_OUTPUT:
            if (len >= 6U) app_disable_output(get_i32_be(&data[2]));
            break;
        /* COMM_SET_MCCONF_TEMP / _SETUP are queued above. On STM32F1 a
         * store request may stall flash fetch, so temporary config writes share
         * the same blocking worker as persistent MCCONF/APPCONF. */
        case COMM_GET_MCCONF_TEMP:
            reply_mcconf_temp(id);
            break;
        case COMM_GET_STATS:
            reply_stats(&data[1], (uint16_t)(len - 1U), id);
            break;
        case COMM_RESET_STATS: {
            int old=mc_interface_get_motor_thread();mc_interface_select_motor_thread(id==MOTOR_RIGHT?2:1);
            mc_interface_stat_reset();mc_interface_select_motor_thread(old);
            if(len>=2U&&data[1])reply_ack(COMM_RESET_STATS);
        } break;
        case COMM_SET_ODOMETER:
            if(len>=5U){mc_interface_set_odometer_motor(id,(uint64_t)get_u32_be(&data[1]));conf_general_request_aux_store();timeout_reset();}
            break;
        case COMM_MOTOR_ESTOP:
            if(len>=3U){int old=mc_interface_get_motor_thread();mc_interface_select_motor_thread(1);mc_interface_ignore_input_both((int)get_u16_be(&data[1]));mc_interface_release_motor_override_both();mc_interface_select_motor_thread(old);}
            break;
        case COMM_SHUTDOWN:
            if (len >= 3U) {
                bool force = data[1] == 1U;
                bool restart = data[2] == 1U;
                if (force || (fabsf(g_motor_left.erpm) <= 100.0f && fabsf(g_motor_right.erpm) <= 100.0f)) {
                    s_shutdown_latched = true;
                    app_disable_output(-1);
                    app_command_release(MOTOR_LEFT, true);
                    app_command_release(MOTOR_RIGHT, true);
                    motor_hw_emergency_all_off(); status_io_tone_stop(); status_io_led(false);
                    osDelay(20U);
                    if (restart) NVIC_SystemReset();
                    else status_io_power_hold(false);
                }
            }
            break;
        case COMM_REBOOT:
            s_shutdown_latched = true; motor_hw_emergency_all_off(); osDelay(20U); NVIC_SystemReset();
            break;
        case COMM_ALIVE:
            timeout_reset();
            app_command_uart_keepalive(id);
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
    /* UART default memilih motor-1. Seperti cabang HW_HAS_DUAL_MOTORS upstream,
     * COMM_FORWARD_CAN untuk ID motor-2 dipakai sebagai local motor selector;
     * build ini tidak memuat driver CAN fisik. */
    process_payload_for_motor(data, len, MOTOR_LEFT);
}

static void packet_process_thread(void *argument) {
    (void)argument;
    for (;;) {
        timeout_heartbeat(TIMEOUT_HEARTBEAT_COMM);
        /* USART3 RX DMA hanya memindahkan byte ke buffer/ring. Framing/CRC dan
         * commands diproses di task context, sesuai pemisahan ISR-vs-parser. */
        bool received = false;
        uint8_t byte;
        while (app_uartcomm_rx_get(&byte)) {
            received = true;
            vesc_packet_process_byte(&s_parser, byte, process_payload);
        }

        if (received) {
            (void)osThreadYield();
        } else {
            osDelay(1U);
        }
    }
}

static bool vesc_comm_send_payload_class(const uint8_t *payload, uint16_t len, bool low_priority) {
    if (payload == NULL || len == 0U || len > VESC_PACKET_MAX_PAYLOAD) return false;
    if (s_send_mutex != NULL && osMutexAcquire(s_send_mutex, osWaitForever) != osOK) return false;
    uint16_t frame_len = vesc_packet_encode(payload, len, s_tx_frame, sizeof(s_tx_frame));
    bool queued = frame_len != 0U && (low_priority
        ? app_uartcomm_write_raw_low_priority(s_tx_frame, frame_len)
        : app_uartcomm_write_raw(s_tx_frame, frame_len));
    if (queued) s_diag.tx_frames++;
    if (s_send_mutex != NULL) (void)osMutexRelease(s_send_mutex);
    return queued;
}

void vesc_comm_send_payload(const uint8_t *payload, uint16_t len) {
    (void)vesc_comm_send_payload_class(payload, len, false);
}

static void vesc_comm_send_payload_low_priority(const uint8_t *payload, uint16_t len) {
    (void)vesc_comm_send_payload_class(payload, len, true);
}

static void vesc_comm_reply_diag(void) {
    const app_uartcomm_stats_t *u = app_uartcomm_get_stats();
    app_adc_status_t a;
    app_adc_get_status(&a);
    /* Use the shared bounded payload scratch instead of a hand-sized stack
     * array. This prevents future diagnostic-field additions from recreating
     * the rev10 stack-overflow class. */
    uint8_t *p = payload_begin();
    if (p == NULL) return;
    uint16_t i = 0U;
    p[i++] = COMM_CUSTOM_APP_DATA; p[i++] = CUSTOM_COMM_DIAG; p[i++] = 14U;
    put_u32(p, &i, u->rx_bytes); put_u32(p, &i, u->rx_overruns); put_u32(p, &i, s_diag.rx_frames_ok);
    put_u32(p, &i, u->tx_bytes); put_u32(p, &i, u->uart_errors); put_u32(p, &i, s_diag.tx_frames);
    put_u32(p, &i, u->tx_overruns); put_u32(p, &i, u->tx_complete_count); put_u32(p, &i, s_diag.blocking_busy_drops);
    put_u32(p, &i, s_diag.motor2_forwards); put_u32(p, &i, s_diag.unsupported_forward_ids);
    put_u32(p, &i, VESC_UART_BAUD); p[i++] = (uint8_t)osMessageQueueGetCount(s_block_queue);
    p[i++] = timeout_has_timeout() ? 1U : 0U; p[i++] = conf_general_is_valid() ? 1U : 0U;
    put_u32(p, &i, u->rx_dma_irq_count); put_u32(p, &i, u->tx_dma_irq_count);
    put_u32(p, &i, u->idle_irq_count); put_u32(p, &i, u->dma_errors);
    put_u32(p, &i, timeout_get_reset_flags());
    p[i++] = timeout_had_iwdg_reset() ? 1U : 0U;
    p[i++] = timeout_watchdog_started() ? 1U : 0U;
    p[i++] = timeout_watchdog_healthy() ? 1U : 0U;
    p[i++] = conf_general_integrity_ok() ? 1U : 0U;
    put_u32(p, &i, conf_general_get_integrity_checks());
    put_u32(p, &i, conf_general_get_integrity_failures());
    p[i++] = status_io_power_is_held() ? 1U : 0U;
    p[i++] = s_shutdown_latched ? 1U : 0U;
    put_u32(p, &i, timeout_watchdog_required_mask());
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_FOC));
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_MOTOR_SERVICE));
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_COMM));
    put_u32(p, &i, timeout_heartbeat_count(TIMEOUT_HEARTBEAT_FAULT));
    put_u32(p, &i, timeout_watchdog_unhealthy_mask());
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_FOC));
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_MOTOR_SERVICE));
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_COMM));
    put_u32(p, &i, timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_FAULT));
    p[i++] = (uint8_t)conf_general_boot_status();
    put_u32(p, &i, g_motor_left.sampling_window_clamp_count);
    put_u32(p, &i, g_motor_right.sampling_window_clamp_count);
    put_u16(p, &i, g_motor_left.sampling_margin_min_q15);
    put_u16(p, &i, g_motor_right.sampling_margin_min_q15);
    /* Revision 12: actual PA2/PA3 plus fault-manager/config-health state. */
    put_u16(p, &i, a.raw1); put_u16(p, &i, a.raw2);
    put_u16(p, &i, (uint16_t)scaled_i32(a.voltage1, 1000.0f));
    put_u16(p, &i, (uint16_t)scaled_i32(a.voltage2, 1000.0f));
    put_i16(p, &i, (int16_t)scaled_i32(a.decoded1, 1000.0f));
    put_i16(p, &i, (int16_t)scaled_i32(a.decoded2, 1000.0f));
    put_i16(p, &i, (int16_t)scaled_i32(a.command, 1000.0f));
    p[i++] = a.fault_flags;
    p[i++] = a.range_ok ? 1U : 0U;
    p[i++] = a.armed_left ? 1U : 0U;
    p[i++] = a.armed_right ? 1U : 0U;
    p[i++] = (uint8_t)app_command_get_source(MOTOR_LEFT);
    p[i++] = (uint8_t)app_command_get_source(MOTOR_RIGHT);

    /* Revision 13: hardware sampling contract, complete dual-motor ISR timing
     * and RTOS/UART resource headroom. Diagnostics are task-side only. */
    motor_runtime_resource_stats_t r;
    vesc_comm_resource_stats_t cr;
    motor_threads_get_resource_stats(&r);
    vesc_comm_get_resource_stats(&cr);
    put_u32(p, &i, motor_hw_sampling_contract_flags());
    put_u32(p, &i, foc_isr_total_max_cycles());
    put_u32(p, &i, foc_isr_near_deadline_count());
    put_u32(p, &i, foc_isr_period_min_cycles());
    put_u32(p, &i, foc_isr_period_max_cycles());
    put_u32(p, &i, r.heap_free_bytes);
    put_u32(p, &i, r.heap_min_ever_bytes);
    put_u16(p, &i, (uint16_t)(r.motor_service_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.motor_service_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(r.sample_sender_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.sample_sender_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(r.fault_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.fault_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(r.status_stack_free_bytes > UINT16_MAX ? UINT16_MAX : r.status_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(cr.packet_stack_free_bytes > UINT16_MAX ? UINT16_MAX : cr.packet_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(cr.blocking_stack_free_bytes > UINT16_MAX ? UINT16_MAX : cr.blocking_stack_free_bytes));
    put_u16(p, &i, (uint16_t)(u->tx_queue_high_water > UINT16_MAX ? UINT16_MAX : u->tx_queue_high_water));
    put_u32(p, &i, u->tx_queue_busy_drops);
    put_u32(p, &i, u->tx_low_priority_drops);
    payload_end(i);
}

void vesc_comm_periodic_100hz(void) {
    conf_general_service_100hz();
    if (!s_motor_ready) return;
    /* Kedua bridge lokal memiliki display-position state sendiri. Motor-2
     * mengikuti semantics local dual-motor forwarding, tanpa driver CAN. */
    if (s_display_owner == 0) send_rotor_position(MOTOR_LEFT);
    else if (s_display_owner == 1) send_rotor_position(MOTOR_RIGHT);
}

void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count) {
    if (samples == NULL || count == 0U) return;
    bool raw = debug_sample_raw();
    for (uint16_t n = 0U; n < count; n++) {
        const debug_sample_t *d = debug_sample_at(n);
        if (d == NULL) d = &samples[n];
        uint8_t p[56]; uint16_t i = 0U;
        p[i++] = COMM_SAMPLE_PRINT;
        put_i16(p, &i, (int16_t)n);

        float ia, ib, ic, ph1, ph2, ph3, vzero, current_fir;
        if (raw) {
            MotorRuntime *sample_motor = motor_get((motor_id_t)d->motor);
            float phase_a_raw = 0.0f;
            float phase_b_raw = 0.0f;
            float phase_c_raw = 0.0f;
            if (sample_motor != NULL) {
                float synthetic_offset = 0.5f * ((float)sample_motor->current_offset_u_counts +
                                                 (float)sample_motor->current_offset_v_counts);
                float current_scale = fmaxf(sample_motor->current_scale, 1.0e-6f);
                float ia_amp = (float)d->ia_cA / 100.0f;
                float ib_amp = (float)d->ib_cA / 100.0f;
                float ic_amp = -(ia_amp + ib_amp);
                if (sample_motor->id == MOTOR_LEFT) {
                    phase_a_raw = (float)d->current_raw_u;
                    phase_b_raw = (float)d->current_raw_v;
                    phase_c_raw = synthetic_offset - (ic_amp / current_scale);
                } else {
                    phase_a_raw = synthetic_offset - (ia_amp / current_scale);
                    phase_b_raw = (float)d->current_raw_u;
                    phase_c_raw = (float)d->current_raw_v;
                }
            }
            ia = phase_a_raw;
            ib = phase_b_raw;
            ic = phase_c_raw;
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
        vesc_comm_send_payload_low_priority(p, i);
        /* At 115200 the software TX ring is back-pressure aware; yielding also
         * keeps the sample sender from starving control threads. */
        osDelay(1U);
    }
}

void vesc_comm_register_appdata_handler(vesc_appdata_handler_t handler) {
    s_appdata_handler = handler;
}

void vesc_comm_get_resource_stats(vesc_comm_resource_stats_t *out) {
    if (out == NULL) return;
    out->packet_stack_free_bytes = s_packet_tp == NULL ? 0U :
        osThreadGetStackSpace(s_packet_tp);
    out->blocking_stack_free_bytes = s_blocking_tp == NULL ? 0U :
        osThreadGetStackSpace(s_blocking_tp);
}

bool vesc_comm_task_init(void) {
    if (s_comm_initialized) return true;
    memset((void *)&s_diag, 0, sizeof(s_diag));
    memset((void *)s_display_mode, 0, sizeof(s_display_mode));
    s_display_owner = -1;
    s_motor_ready = false;
    s_config_ready = false;
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
        .name="uartcomm proc", .priority=osPriorityAboveNormal, .stack_size=1536U
    };
    const osThreadAttr_t block_attr = {
        .name="comm_block", .priority=osPriorityNormal, .stack_size=3072U
    };

    s_packet_tp = osThreadNew(packet_process_thread, NULL, &packet_attr);
    s_blocking_tp = osThreadNew(blocking_thread, NULL, &block_attr);
    if (s_packet_tp == NULL || s_blocking_tp == NULL) {
        return false;
    }

    /* Match app_uartcomm_start ordering conceptually: packet state/thread
     * resources exist before the serial peripheral starts receiving bytes. */
    if (!app_uartcomm_init()) {
        return false;
    }

    s_comm_initialized = true;
    return true;
}

void commands_init(void) {
    (void)vesc_comm_task_init();
}

bool commands_is_initialized(void) {
    return s_comm_initialized;
}

void vesc_comm_set_config_ready(bool ready) {
    s_config_ready = ready;
}

void vesc_comm_set_motor_ready(bool ready) {
    s_motor_ready = ready;
}

bool vesc_comm_motor_ready(void) {
    return s_motor_ready;
}

