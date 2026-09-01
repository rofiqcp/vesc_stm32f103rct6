#include "applications/app_adc.h"
#include "applications/app.h"
#include "applications/app_command.h"
#include "hwconf/hw.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "timeout.h"
#include <math.h>
#include <string.h>

#define APP_ADC_VREF_V              3.3f
#define APP_ADC_COUNTS_FS           4095.0f
#define APP_ADC_SAFE_NEUTRAL_MS     500U
#define APP_ADC_FILTER_DIV          5.0f
#define APP_ADC_SHORT_GND_V         0.05f
#define APP_ADC_SHORT_VCC_V         3.25f
#define APP_ADC_IMPLAUSIBLE_STEP_V  2.50f

typedef struct {
    // Variabel pub: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    app_adc_status_t pub;
    // Variabel filt_v1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float filt_v1;
    // Variabel filt_v2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float filt_v2;
    // Variabel ramp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ramp;
    // Variabel prev_v1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float prev_v1;
    // Variabel prev_v2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float prev_v2;
    // Variabel neutral_since_ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t neutral_since_ms[2];
    // Variabel rate_accum: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t rate_accum;
    // Variabel neutral_tracking: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool neutral_tracking[2];
    // Variabel filter_seeded: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool filter_seeded;
    // Variabel previous_seeded: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool previous_seeded;
} app_adc_runtime_t;

// Variabel s_adc: nilai atau state ADC pada jalur pengukuran.
static app_adc_runtime_t s_adc;
// Variabel s_pub_seq: nomor urut untuk konsistensi snapshot atau record.
static volatile uint32_t s_pub_seq = 0U;

// Fungsi app_adc_publish_begin: menjalankan operasi app adc publish begin sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static inline void app_adc_publish_begin(void) {
    s_pub_seq++;
    __DMB();
}

// Fungsi app_adc_publish_end: menjalankan operasi app adc publish end sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static inline void app_adc_publish_end(void) {
    __DMB();
    s_pub_seq++;
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi clampf_local: menjalankan operasi clampf local sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
static float clampf_local(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi map01: menjalankan operasi map01 sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static float map01(float v, float start, float end) {
    // Variabel span: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float span = end - start;
    if (fabsf(span) < 1.0e-6f)
        return 0.0f;
    return clampf_local((v - start) / span, 0.0f, 1.0f);
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hyst: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi deadband: menjalankan operasi deadband sesuai tanggung jawab modul dengan input tervalidasi dan state
// yang konsisten.
static float deadband(float x, float hyst) {
    hyst = clampf_local(hyst, 0.0f, 0.95f);
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float a = fabsf(x);
    if (a <= hyst)
        return 0.0f;
    // Variabel y: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float y = (a - hyst) / (1.0f - hyst);
    return x < 0.0f ? -y : y;
}

/* Same three curve families used by VESC. This runs in the 1-kHz service,
 * never in the 16-kHz FOC ISR. The common linear configuration (curve=0)
 * takes the fast path and does not call powf/expf. */
// Parameter val: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter curve_acc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curve_brake: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi throttle_curve: menjalankan operasi throttle curve sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float throttle_curve(float val, float curve_acc, float curve_brake, thr_exp_mode mode) {
    val = clampf_local(val, -1.0f, 1.0f);
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float a = fabsf(val);
    // Variabel curve: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float curve = val >= 0.0f ? curve_acc : curve_brake;
    // Variabel ret: hasil eksekusi untuk menentukan keberhasilan operasi.
    float ret;
    if (fabsf(curve) < 1.0e-7f) {
        ret = a;
    }
    else if (mode == THR_EXP_EXPO) {
        ret = curve >= 0.0f ? 1.0f - powf(1.0f - a, 1.0f + curve)
                             : powf(a, 1.0f - curve);
    }
    else if (mode == THR_EXP_NATURAL) {
        ret = curve >= 0.0f ? 1.0f - ((expf(curve * (1.0f - a)) - 1.0f) / (expf(curve) - 1.0f))
                             : (expf(-curve * a) - 1.0f) / (expf(-curve) - 1.0f);
    }
    else if (mode == THR_EXP_POLY) {
        ret = curve >= 0.0f ? 1.0f - ((1.0f - a) / (1.0f + curve * a))
                             : a / (1.0f - curve * (1.0f - a));
    }
    else {
        ret = a;
    }
    ret = clampf_local(ret, 0.0f, 1.0f);
    return val < 0.0f ? -ret : ret;
}

// Parameter t: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi control_uses_brake_adc: menjalankan operasi control uses brake adc sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static bool control_uses_brake_adc(adc_control_type t) {
    return t == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC;
}

// Parameter app: state atau konfigurasi aplikasi VESC.
// Fungsi app_adc_active: menjalankan operasi app adc active sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool app_adc_active(app_use app) {
    return app == APP_ADC || app == APP_ADC_UART;
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter configured_min: batas atau nilai minimum untuk validasi dan proteksi.
// Parameter configured_max: batas atau nilai maksimum untuk validasi dan proteksi.
// Parameter configured_start: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter configured_end: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi electrical_range_ok: menjalankan operasi electrical range ok sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool electrical_range_ok(float v, float configured_min, float configured_max,
                                float configured_start, float configured_end) {
    if (v < configured_min || v > configured_max)
        return false;
    /* Detect hard shorts only when the configured useful span does not itself
     * intentionally include the corresponding rail. */
    if (configured_start > 0.10f && v < APP_ADC_SHORT_GND_V)
        return false;
    if (configured_end < 3.20f && v > APP_ADC_SHORT_VCC_V)
        return false;
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ctrl: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter cmd: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi apply_to_motor: menjalankan operasi apply to motor sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void apply_to_motor(MotorRuntime *m, adc_control_type ctrl, float cmd) {
    if (m == NULL)
        return;
    if (ctrl == ADC_CTRL_TYPE_CURRENT) {
        motor_set_current_rel(m, clampf_local(cmd, 0.0f, 1.0f));
    }
    else if (ctrl == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC) {
        if (cmd >= 0.0f) {
            motor_set_current_rel(m, clampf_local(cmd, 0.0f, 1.0f));
        }
        else {
            // Variabel brake_max: batas atau nilai maksimum untuk validasi dan proteksi.
            const float brake_max = fabsf(m->current_min_a);
            motor_set_brake_current(m, clampf_local(-cmd, 0.0f, 1.0f) * brake_max);
        }
    }
    else if (ctrl == ADC_CTRL_TYPE_DUTY) {
        motor_set_duty(m, clampf_local(cmd, 0.0f, 1.0f) * fabsf(m->max_duty));
    }
    else if (ctrl == ADC_CTRL_TYPE_PID) {
        motor_set_speed(m, clampf_local(cmd, 0.0f, 1.0f) * fabsf(m->max_erpm));
    }
}

// Fungsi app_adc_init: menginisialisasi app adc init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
void app_adc_init(void) {
    memset(&s_adc, 0, sizeof(s_adc));
    s_adc.pub.fault_flags = APP_ADC_FAULT_NOT_READY;
}

// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi app_adc_service_1khz: menjalankan operasi app adc service 1khz sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void app_adc_service_1khz(uint32_t now_ms) {
    // Variabel app: state atau konfigurasi aplikasi VESC.
    const app_configuration *app = app_get_configuration();
    if (app == NULL)
        return;
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const adc_config *c = &app->app_adc_conf;
    app_adc_publish_begin();


    // Variabel raw1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel raw2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t raw1 = 0U, raw2 = 0U;
    if (!motor_hw_get_app_adc_raw(&raw1, &raw2)) {
        s_adc.pub.fault_flags = APP_ADC_FAULT_NOT_READY;
        app_command_adc_block(MOTOR_LEFT);
        app_command_adc_block(MOTOR_RIGHT);
        app_adc_publish_end();
        return;
    }

    s_adc.pub.raw1 = raw1;
    s_adc.pub.raw2 = raw2;
    // Variabel v1_raw: nilai mentah sebelum konversi ke satuan fisik.
    const float v1_raw = (float)raw1 * (APP_ADC_VREF_V / APP_ADC_COUNTS_FS);
    // Variabel v2_raw: nilai mentah sebelum konversi ke satuan fisik.
    const float v2_raw = (float)raw2 * (APP_ADC_VREF_V / APP_ADC_COUNTS_FS);

    if (!s_adc.filter_seeded) {
        s_adc.filt_v1 = v1_raw;
        s_adc.filt_v2 = v2_raw;
        s_adc.filter_seeded = true;
    }
    else {
        s_adc.filt_v1 += (v1_raw - s_adc.filt_v1) / APP_ADC_FILTER_DIV;
        s_adc.filt_v2 += (v2_raw - s_adc.filt_v2) / APP_ADC_FILTER_DIV;
    }
    // Variabel v1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v1 = c->use_filter ? s_adc.filt_v1 : v1_raw;
    // Variabel v2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v2 = c->use_filter ? s_adc.filt_v2 : v2_raw;
    s_adc.pub.voltage1 = v1;
    s_adc.pub.voltage2 = v2;

    // Variabel faults: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t faults = APP_ADC_FAULT_NONE;
    // Variabel pwr_range: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool pwr_range = electrical_range_ok(v1, c->voltage_min, c->voltage_max,
                                                c->voltage_start, c->voltage_end);
    // Variabel brake_range: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool brake_range = true;
    if (control_uses_brake_adc(c->ctrl_type)) {
        brake_range = v2 >= 0.0f && v2 <= APP_ADC_VREF_V;
        if (c->voltage2_start > 0.10f && v2 < APP_ADC_SHORT_GND_V)
            brake_range = false;
        if (c->voltage2_end < 3.20f && v2 > APP_ADC_SHORT_VCC_V)
            brake_range = false;
    }
    if (!pwr_range)
        faults |= APP_ADC_FAULT_THROTTLE_RANGE;
    if (!brake_range)
        faults |= APP_ADC_FAULT_BRAKE_RANGE;

    if (s_adc.previous_seeded &&
        (fabsf(v1 - s_adc.prev_v1) > APP_ADC_IMPLAUSIBLE_STEP_V ||
         (control_uses_brake_adc(c->ctrl_type) && fabsf(v2 - s_adc.prev_v2) > APP_ADC_IMPLAUSIBLE_STEP_V))) {
        faults |= APP_ADC_FAULT_IMPLAUSIBLE;
    }
    s_adc.prev_v1 = v1;
    s_adc.prev_v2 = v2;
    s_adc.previous_seeded = true;

    // Variabel pwr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float pwr = map01(v1, c->voltage_start, c->voltage_end);
    // Variabel brake: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float brake = map01(v2, c->voltage2_start, c->voltage2_end);
    if (c->voltage_inverted)
        pwr = 1.0f - pwr;
    if (c->voltage2_inverted)
        brake = 1.0f - brake;
    s_adc.pub.decoded1 = pwr;
    s_adc.pub.decoded2 = brake;

    if (control_uses_brake_adc(c->ctrl_type)) {
        /* Port has no physical ADC buttons; brake channel is always treated
         * as an intentional brake input. A brake reading above 5% cancels
         * positive throttle then generates regen. */
        if (brake > 0.05f)
            pwr = 0.0f;
        pwr -= brake;
    }
    pwr = deadband(pwr, c->hyst);
    pwr = throttle_curve(pwr, c->throttle_exp, c->throttle_exp_brake, c->throttle_exp_mode);

    // Variabel rate: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint16_t rate = c->update_rate_hz == 0U ? 1U : c->update_rate_hz;
    s_adc.rate_accum = (uint16_t)(s_adc.rate_accum + (rate > 1000U ? 1000U : rate));
    if (s_adc.rate_accum < 1000U) {
        s_adc.pub.fault_flags = faults;
        s_adc.pub.range_ok = faults == APP_ADC_FAULT_NONE;
        app_adc_publish_end();
        return;
    }
    s_adc.rate_accum = (uint16_t)(s_adc.rate_accum - 1000U);

    // Variabel step_dt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float step_dt = 1.0f / (float)(rate > 1000U ? 1000U : rate);
    // Variabel ramp_time: nilai waktu untuk penjadwalan atau pengawasan.
    const float ramp_time = fabsf(pwr) > fabsf(s_adc.ramp) ? c->ramp_time_pos : c->ramp_time_neg;
    if (ramp_time > 0.01f) {
        // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float step = step_dt / ramp_time;
        if (s_adc.ramp < pwr)
            s_adc.ramp = fminf(pwr, s_adc.ramp + step);
        else if (s_adc.ramp > pwr) {
            s_adc.ramp = fmaxf(pwr, s_adc.ramp - step);
        }
    }
    else {
        s_adc.ramp = pwr;
    }
    s_adc.pub.command = s_adc.ramp;

    // Variabel neutral: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool neutral = fabsf(s_adc.ramp) <= 0.01f && pwr <= 0.01f &&
                         (!control_uses_brake_adc(c->ctrl_type) || brake <= 0.01f);
    // Variabel config_mode: data konfigurasi yang mengatur perilaku firmware.
    const bool config_mode = c->ctrl_type == ADC_CTRL_TYPE_NONE ||
                             c->ctrl_type == ADC_CTRL_TYPE_CURRENT ||
                             c->ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC ||
                             c->ctrl_type == ADC_CTRL_TYPE_DUTY ||
                             c->ctrl_type == ADC_CTRL_TYPE_PID;
    if (!config_mode)
        faults |= APP_ADC_FAULT_CONFIG;

    if (!app_adc_active(app->app_to_use) || c->ctrl_type == ADC_CTRL_TYPE_NONE) {
        app_command_adc_release(MOTOR_LEFT, true);
        app_command_adc_release(MOTOR_RIGHT, true);
        s_adc.pub.armed_left = false;
        s_adc.pub.armed_right = false;
        s_adc.pub.fault_flags = faults;
        s_adc.pub.range_ok = faults == APP_ADC_FAULT_NONE;
        app_adc_publish_end();
        return;
    }

    // Variabel calibration_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool calibration_ok = foc_calibration_done() && foc_calibration_valid();
    if (!calibration_ok || app_is_output_disabled()) {
        app_command_adc_block(MOTOR_LEFT);
        app_command_adc_block(MOTOR_RIGHT);
        s_adc.neutral_since_ms[0] = s_adc.neutral_since_ms[1] = 0U;
        s_adc.neutral_tracking[0] = s_adc.neutral_tracking[1] = false;
        s_adc.pub.armed_left = false;
        s_adc.pub.armed_right = false;
        s_adc.pub.fault_flags = faults | APP_ADC_FAULT_NOT_READY;
        s_adc.pub.range_ok = false;
        app_adc_publish_end();
        return;
    }

    // Variabel range_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool range_ok = (faults & (APP_ADC_FAULT_CONFIG | APP_ADC_FAULT_THROTTLE_RANGE |
                                     APP_ADC_FAULT_BRAKE_RANGE | APP_ADC_FAULT_IMPLAUSIBLE)) == 0U;
    if (!range_ok) {
        app_command_adc_release(MOTOR_LEFT, true);
        app_command_adc_release(MOTOR_RIGHT, true);
        s_adc.neutral_since_ms[0] = s_adc.neutral_since_ms[1] = 0U;
        s_adc.neutral_tracking[0] = s_adc.neutral_tracking[1] = false;
        s_adc.pub.armed_left = false;
        s_adc.pub.armed_right = false;
        s_adc.pub.fault_flags = faults;
        s_adc.pub.range_ok = false;
        app_adc_publish_end();
        return;
    }

    // Variabel target_right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool target_right = c->multi_esc;
    // Variabel ids: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const motor_id_t ids[2] = {
        MOTOR_LEFT, MOTOR_RIGHT
    }
    ;
    // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool target[2] = {
        true, target_right
    }
    ;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0U; i < 2U; i++) {
        // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
        const motor_id_t id = ids[i];
        if (!target[i]) {
            app_command_adc_release(id, true);
            s_adc.neutral_since_ms[i] = 0U;
            s_adc.neutral_tracking[i] = false;
            continue;
        }
        // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        MotorRuntime *m = motor_get(id);
        if (m->detect.busy) {
            app_command_adc_block(id);
            s_adc.neutral_since_ms[i] = 0U;
            s_adc.neutral_tracking[i] = false;
            continue;
        }

        // Variabel src: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const app_command_source_t src = app_command_get_source(id);
        if (src == APP_CMD_SRC_UART || src == APP_CMD_SRC_DETECTION || src == APP_CMD_SRC_CALIBRATION) {
            s_adc.neutral_since_ms[i] = 0U;
            s_adc.neutral_tracking[i] = false;
            continue;
        }

        // Variabel neutral_stable: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        bool neutral_stable = c->safe_start == SAFE_START_DISABLED && !app_command_adc_rearm_required(id);
        if (c->safe_start != SAFE_START_DISABLED || app_command_adc_rearm_required(id)) {
            if (neutral) {
                if (!s_adc.neutral_tracking[i]) {
                    s_adc.neutral_tracking[i] = true;
                    s_adc.neutral_since_ms[i] = now_ms;
                }
                neutral_stable = (uint32_t)(now_ms - s_adc.neutral_since_ms[i]) >= APP_ADC_SAFE_NEUTRAL_MS;
            }
            else {
                s_adc.neutral_tracking[i] = false;
                s_adc.neutral_since_ms[i] = 0U;
                neutral_stable = false;
            }
        }

        if (!app_command_adc_claim(id, neutral_stable)) {
            if (!neutral_stable && c->safe_start == SAFE_START_REGULAR)
                faults |= APP_ADC_FAULT_START_ACTIVE;
            continue;
        }
        apply_to_motor(m, c->ctrl_type, s_adc.ramp);
        timeout_reset();
    }

    s_adc.pub.armed_left = app_command_get_source(MOTOR_LEFT) == APP_CMD_SRC_ADC;
    s_adc.pub.armed_right = app_command_get_source(MOTOR_RIGHT) == APP_CMD_SRC_ADC;
    s_adc.pub.fault_flags = faults;
    s_adc.pub.range_ok = range_ok;
    app_adc_publish_end();
}

// Fungsi app_adc_get_decoded_level: membaca app adc get decoded level tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float app_adc_get_decoded_level(void) {
    return s_adc.pub.decoded1;
}
// Fungsi app_adc_get_voltage: membaca app adc get voltage tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float app_adc_get_voltage(void) {
    return s_adc.pub.voltage1;
}
// Fungsi app_adc_get_decoded_level2: membaca app adc get decoded level2 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float app_adc_get_decoded_level2(void) {
    return s_adc.pub.decoded2;
}
// Fungsi app_adc_get_voltage2: membaca app adc get voltage2 tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float app_adc_get_voltage2(void) {
    return s_adc.pub.voltage2;
}
// Fungsi app_adc_range_ok: menjalankan operasi app adc range ok sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool app_adc_range_ok(void) {
    return s_adc.pub.range_ok;
}
// Fungsi app_adc_data_ready: menjalankan operasi app adc data ready sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool app_adc_data_ready(void) {
    return (s_adc.pub.fault_flags & APP_ADC_FAULT_NOT_READY) == 0U;
}
// Fungsi app_adc_fault_flags: menangani app adc fault flags dengan memprioritaskan pemadaman keluaran daya,
// pencatatan penyebab, dan pemulihan yang aman.
uint8_t app_adc_fault_flags(void) {
    return s_adc.pub.fault_flags;
}
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi app_adc_get_status: membaca app adc get status tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
void app_adc_get_status(app_adc_status_t *out) {
    if (out == NULL)
        return;
    for (;; ) {
        // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint32_t a = s_pub_seq;
        if ((a & 1U) != 0U)
            continue;
        __DMB();
        *out = s_adc.pub;
        __DMB();
        // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint32_t b = s_pub_seq;
        if (a == b && (b & 1U) == 0U)
            break;
    }
}
