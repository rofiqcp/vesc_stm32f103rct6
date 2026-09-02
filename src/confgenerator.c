#include "confgenerator.h"
#include "util/buffer.h"
#include "util/maths.h"
#include "motor/mc_interface.h"
#include "motor/mcconf_default.h"
#include "encoder/encoder.h"
#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "motor/mcpwm_foc.h"
#include "motor/foc_math.h"
#include "timeout.h"
#include "conf_general.h"
#include "applications/appconf_default.h"
#include "applications/app.h"
#include <string.h>
#include <math.h>

#define VESC_PWM_SYNCHRONOUS 1U
#define VESC_COMM_INTEGRATE  0U
#define VESC_MOTOR_BLDC      0U
#define VESC_MOTOR_DC        1U
#define VESC_MOTOR_FOC       2U
#define VESC_SENSOR_SENSORLESS 0U
#define VESC_SENSOR_SENSORED   1U
/* VESC firmware 6.00 wire enum. ENCODER_AB=9 was introduced later and must
 * never appear on a 6.00 MCCONF wire image. Internally this port can still use
 * FOC_SENSOR_MODE_ENCODER_AB for the incremental A/B implementation. */
#define VESC_FOC_SENSOR_ENCODER    1U
#define VESC_FOC_SENSOR_HALL       2U
#define VESC_SENSOR_PORT_HALL   0U
#define VESC_SENSOR_PORT_ABI    1U
#define VESC_APP_NONE           0U
#define VESC_APP_ADC            2U
#define VESC_APP_UART           3U
#define VESC_APP_ADC_UART       5U

// Variabel s_mc_factory: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_mc_factory[2][VESC6_MCCONF_WIRE_SIZE];
// Variabel s_mc_active: penanda bahwa state atau fitur sedang aktif.
static uint8_t s_mc_active[2][VESC6_MCCONF_WIRE_SIZE];
// Variabel s_app_factory: state atau konfigurasi aplikasi VESC.
static uint8_t s_app_factory[VESC6_APPCONF_WIRE_SIZE];
// Variabel s_app_active: state atau konfigurasi aplikasi VESC.
static uint8_t s_app_active[VESC6_APPCONF_WIRE_SIZE];
/* Import runs from the 2-KiB boot thread. Keep the ~1.5-KiB transactional
 * rollback image out of that thread stack. vesc_config_import_wire() is only
 * used by the serialized boot-time flash loader. */
/* Shared transactional rollback scratch. SET_MCCONF/APPCONF runs in the
 * serialized comm_block worker, while import runs during serialized boot, so
 * these paths cannot overlap. Reusing the same exact-size images removes the
 * risky RTOS-stack copies without spending another ~1 KiB of F103 SRAM. */
// Variabel s_rollback_mc: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_rollback_mc[2][VESC6_MCCONF_WIRE_SIZE];
// Variabel s_rollback_app: state atau konfigurasi aplikasi VESC.
static uint8_t s_rollback_app[VESC6_APPCONF_WIRE_SIZE];
// Variabel s_initialized: state internal modul yang dipertahankan antar pemanggilan fungsi.
static bool s_initialized = false;
// Variabel s_layout_ok: state internal modul yang dipertahankan antar pemanggilan fungsi.
static bool s_layout_ok = false;

// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter value: nilai kerja yang digunakan oleh algoritma pada konteks tersebut.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi append_float32_auto_field: menyusun append float32 auto field ke buffer/wire format dengan urutan
// field, skala, dan batas data yang konsisten.
static void append_float32_auto_field(uint8_t *buffer, float value, int32_t *index) {
    vesc_buf_append_float32_auto(buffer, value, index);
}
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter value: nilai kerja yang digunakan oleh algoritma pada konteks tersebut.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi append_float16_field: menyusun append float16 field ke buffer/wire format dengan urutan field, skala,
// dan batas data yang konsisten.
static void append_float16_field(uint8_t *buffer, float value, float scale, int32_t *index) {
    vesc_buf_append_float16(buffer, value, scale, index);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_auto_at: membaca get auto at tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static float get_auto_at(const uint8_t *b, int off) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    return vesc_buf_get_float32_auto(b, &i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_f16_at: membaca get f16 at tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static float get_f16_at(const uint8_t *b, int off, float scale) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    return vesc_buf_get_float16(b, scale, &i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_u16_at: membaca get u16 at tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static uint16_t get_u16_at(const uint8_t *b, int off) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    return vesc_buf_get_u16(b, &i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_u32_at: membaca get u32 at tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static uint32_t get_u32_at(const uint8_t *b, int off) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    return vesc_buf_get_u32(b, &i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_auto_at: menyusun put auto at ke buffer/wire format dengan urutan field, skala, dan batas data
// yang konsisten.
static void put_auto_at(uint8_t *b, int off, float v) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    vesc_buf_append_float32_auto(b, v, &i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter scale: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_f16_at: menyusun put f16 at ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_f16_at(uint8_t *b, int off, float v, float scale) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    vesc_buf_append_float16(b, v, scale, &i);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_u16_at: menyusun put u16 at ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_u16_at(uint8_t *b, int off, uint16_t v) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    vesc_buf_append_u16(b, v, &i);
}

/* VESC firmware 6.00 memakai HFI Start Samples default 5. Run24--Run30
   dapat mempersist nilai legacy 0/2 di image MCCONF lengkap walaupun backend
   HFI memang tidak dijalankan pada port STM32F103 ini. Hanya nilai legacy yang
   tidak valid (<5) yang dimigrasikan ke default resmi 5; nilai user yang sudah
   valid dipertahankan byte-for-byte. Migrasi boot hanya di RAM; save EEPROM
   eksplisit berikutnya yang mempersist nilai hasil migrasi. */
static void normalize_vesc6_hfi_ui_fields(uint8_t *w) {
    if (!w)
        return;
    if (get_u16_at(w, VESC6_MC_OFF_FOC_HFI_START_SAMPLES) < 5U)
        put_u16_at(w, VESC6_MC_OFF_FOC_HFI_START_SAMPLES, 5U);
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter off: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_u32_at: menyusun put u32 at ke buffer/wire format dengan urutan field, skala, dan batas data yang
// konsisten.
static void put_u32_at(uint8_t *b, int off, uint32_t v) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = off;
    vesc_buf_append_u32(b, v, &i);
}
/* Forward declaration so vesc_config_set_mc_wire() can reuse the wire decoder
   for the range-validation clamp. Defined further below. */
// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcconf_decode_wire: mengurai mcconf decode wire dari buffer komunikasi menjadi data internal setelah
// format dan batas input diperiksa.
static void mcconf_decode_wire(const uint8_t *w, mc_configuration *c);
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi clampf: menjalankan operasi clampf sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi gain_q16: menjalankan operasi gain q16 sesuai tanggung jawab modul dengan input tervalidasi dan state
// yang konsisten.
static int32_t gain_q16(float v) {
    if (!isfinite(v))
        return 0;
    v = clampf(v, -32768.0f, 32767.999f);
    return (int32_t)lrintf(v*65536.0f);
}
// Parameter kp: penguatan proporsional regulator.
// Fungsi current_gain_to_fast_q16: menjalankan operasi current gain to fast q16 sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static int32_t current_gain_to_fast_q16(float kp) {
    return gain_q16(kp * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V);
}
// Parameter ki: penguatan integral regulator.
// Fungsi current_ki_to_fast_q16: menjalankan operasi current ki to fast q16 sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static int32_t current_ki_to_fast_q16(float ki) {
    return gain_q16(ki * FOC_DT_S * FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V);
}
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi amp_to_q15: menjalankan operasi amp to q15 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int32_t amp_to_q15(float a) {
    // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float q = (a/FOC_CURRENT_Q_BASE_A)*32768.0f;
    q = clampf(q, -32768.0f, 32767.0f);
    return (int32_t)lrintf(q);
}
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi volt_to_q15: menjalankan operasi volt to q15 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int32_t volt_to_q15(float v) {
    // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float q = (v/FOC_VOLTAGE_Q_BASE_V)*32768.0f;
    q = clampf(q, 0.0f, 32767.0f);
    return (int32_t)lrintf(q);
}
// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sig: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi sig_ok: menjalankan operasi sig ok sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static bool sig_ok(const uint8_t *w, uint32_t sig) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    return vesc_buf_get_u32(w, &i) == sig;
}

// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi runtime_mc_auto_fields_finite: menjalankan operasi runtime mc auto fields finite sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static bool runtime_mc_auto_fields_finite(const uint8_t *w) {
    /* Every float32-auto field consumed by this hardware backend must decode to
       a finite value before MotorRuntime is touched. Unsupported wire-only
       fields remain byte-preserved and are intentionally not interpreted. */
    // Variabel off: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static const uint16_t off[] = {
        VESC6_MC_OFF_L_CURRENT_MAX, VESC6_MC_OFF_L_CURRENT_MIN,
        VESC6_MC_OFF_L_IN_CURRENT_MAX, VESC6_MC_OFF_L_IN_CURRENT_MIN,
        VESC6_MC_OFF_L_ABS_CURRENT_MAX, VESC6_MC_OFF_L_MIN_ERPM,
        VESC6_MC_OFF_L_MAX_ERPM, VESC6_MC_OFF_L_MIN_VIN, VESC6_MC_OFF_L_MAX_VIN,
        VESC6_MC_OFF_L_BAT_CUT_START, VESC6_MC_OFF_L_BAT_CUT_END,
        VESC6_MC_OFF_L_WATT_MAX, VESC6_MC_OFF_L_WATT_MIN,
        VESC6_MC_OFF_FOC_CURRENT_KP, VESC6_MC_OFF_FOC_CURRENT_KI,
        153U, 157U, 161U, 165U, 169U, 173U, 177U, 181U,
        VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI,
        VESC6_MC_OFF_FOC_START_CURR_DEC_RPM, 201U, VESC6_MC_OFF_FOC_FW_CURRENT_MAX,
        VESC6_MC_OFF_FOC_HALL_INTERP_ERPM, VESC6_MC_OFF_FOC_SL_ERPM,
        VESC6_MC_OFF_S_PID_KP, VESC6_MC_OFF_S_PID_KI, VESC6_MC_OFF_S_PID_KD,
        VESC6_MC_OFF_P_PID_KP, VESC6_MC_OFF_P_PID_KI, VESC6_MC_OFF_P_PID_KD,
        VESC6_MC_OFF_FOC_ENCODER_OFFSET, VESC6_MC_OFF_FOC_ENCODER_RATIO,
        VESC6_MC_OFF_SI_GEAR_RATIO, VESC6_MC_OFF_SI_WHEEL_DIAMETER,
        VESC6_MC_OFF_SI_BATTERY_AH, VESC6_MC_OFF_SI_MOTOR_NL_CURRENT
    };
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0U; k < sizeof(off) / sizeof(off[0]); k++) {
        if (!isfinite(get_auto_at(w, (int)off[k])))
            return false;
    }
    return true;
}

/* APPCONF VESC disimpan sebagai wire image lengkap. Field yang belum punya
 * backend runtime tetap dipertahankan byte-for-byte agar SET/GET/default dan
 * persistence tidak memunculkan false "Parameters truncated". Validasi runtime
 * tetap dilakukan oleh apply_app() hanya untuk subset yang dipakai hardware. */

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi runtime_mc_ready: menjalankan operasi runtime mc ready sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool runtime_mc_ready(const MotorRuntime *m) {
    if (m == NULL)
        return false;
    /* A zeroed BSS has motor_type==BLDC numerically. Require the hardware
       timer binding as the initialization sentinel so early GET_MCCONF does
       not accidentally publish a non-FOC backend before motor_defaults(). */
    if (m->pwm_tim == NULL)
        return false;
    if (m->pole_pairs < 1U || m->pole_pairs > 60U)
        return false;
    if (!isfinite(m->current_max_a) || m->current_max_a < 0.1f)
        return false;
    if (!isfinite(m->max_duty) || fabsf(m->max_duty) < 0.01f)
        return false;
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi build_foc_hall: menjalankan bagian build foc hall pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
static void build_foc_hall(const MotorRuntime *m, uint8_t out[8]) {
    /* GET_MCCONF may be requested before the motor runtime is fully ready.
       Never serialize zeroed BSS as a Hall table. */
    // Variabel safe: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static const uint8_t safe[8] = {
        255, 17, 83, 50, 150, 183, 117, 255
    }
    ;
    if (!runtime_mc_ready(m)) {
        memcpy(out, safe, 8);
        return;
    }
    // Variabel sane: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool sane = (m->foc_hall_table[0] == 255U && m->foc_hall_table[7] == 255U);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 1; k < 7U; k++)
        sane = sane && (m->foc_hall_table[k] <= 200U);
    if (!sane) {
        memcpy(out, safe, 8);
        return;
    }
    memcpy(out, m->foc_hall_table, 8);
}

/* VESC 6.00 wire image. Unsupported subsystems start from conservative
   defaults, but all 481 bytes remain part of the canonical generated ABI and
   are preserved through SET/GET/flash exactly like upstream VESC. apply_mc()
   only consumes the subset that has a real STM32F103 runtime backend. */
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi build_mc_default: menjalankan operasi build mc default sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool build_mc_default(uint8_t *b, motor_id_t id) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool right = id == MOTOR_RIGHT;
    // Variabel mr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool mr = runtime_mc_ready(m);
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    // Variabel default_foc_sensor: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const mc_foc_sensor_mode default_foc_sensor = right ? MCCONF_FOC_SENSOR_RIGHT_DEFAULT : MCCONF_FOC_SENSOR_LEFT_DEFAULT;
    // Variabel foc_sensor_wire: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t foc_sensor_wire = VESC_FOC_SENSOR_HALL;
    if (mr) {
        if (m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS)
            foc_sensor_wire = FOC_SENSOR_MODE_SENSORLESS;
        else if (!right && (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ||
                            m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB)) foc_sensor_wire = VESC_FOC_SENSOR_ENCODER;
    }
    else if (default_foc_sensor == FOC_SENSOR_MODE_SENSORLESS) {
        foc_sensor_wire = FOC_SENSOR_MODE_SENSORLESS;
    } else if (!right && (default_foc_sensor == FOC_SENSOR_MODE_ENCODER ||
                          default_foc_sensor == FOC_SENSOR_MODE_ENCODER_AB)) {
        foc_sensor_wire = VESC_FOC_SENSOR_ENCODER;
    }
    // Variabel legacy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t legacy[8] = {
        255, 1, 3, 2, 5, 6, 4, 255
    }
    ;
    // Variabel hall: data sensor Hall untuk menentukan sektor atau posisi rotor.
    uint8_t hall[8];
    build_foc_hall(m, hall);
    memset(b, 0, VESC6_MCCONF_WIRE_SIZE);
    vesc_buf_append_u32(b, VESC6_MCCONF_SIGNATURE, &i);
    b[i++] = mr ? (uint8_t)m->pwm_mode : VESC_PWM_SYNCHRONOUS;
    b[i++] = mr ? (uint8_t)m->comm_mode : VESC_COMM_INTEGRATE;
    b[i++] = VESC_MOTOR_FOC;
    b[i++] = (foc_sensor_wire == FOC_SENSOR_MODE_SENSORLESS) ? VESC_SENSOR_SENSORLESS : VESC_SENSOR_SENSORED;
    append_float32_auto_field(b, mr ? m->current_max_a : FOC_MAX_CURRENT_A, &i);
    append_float32_auto_field(b, mr ? m->current_min_a : -FOC_MAX_CURRENT_A, &i);
    append_float32_auto_field(b, mr ? m->input_current_max_a : FOC_MAX_CURRENT_A, &i);
    append_float32_auto_field(b, mr ? m->input_current_min_a : -FOC_MAX_CURRENT_A, &i);
    append_float32_auto_field(b, mr ? m->abs_current_max_a : FOC_ABS_CURRENT_TRIP_A, &i);
    append_float32_auto_field(b, mr ? m->min_erpm : MOTOR_DEFAULT_MIN_ERPM, &i);
    append_float32_auto_field(b, mr ? m->max_erpm : MOTOR_DEFAULT_MAX_ERPM, &i);
    append_float16_field(b, mr ? m->erpm_start : MCCONF_L_ERPM_START_DEFAULT, 10000, &i);
    append_float32_auto_field(b, mr ? m->max_erpm : MOTOR_DEFAULT_MAX_ERPM, &i);
    append_float32_auto_field(b, mr ? m->max_erpm : MOTOR_DEFAULT_MAX_ERPM, &i);
    append_float32_auto_field(b, mr ? m->min_vin : VBUS_MIN_RUN_V, &i);
    append_float32_auto_field(b, mr ? m->max_vin : VBUS_MAX_RUN_V, &i);
    append_float32_auto_field(b, mr ? m->battery_cut_start : 36.0f, &i);
    append_float32_auto_field(b, mr ? m->battery_cut_end : 32.0f, &i);
    b[i++] = (mr && m->slow_abs_current) ? 1U : 0U;
    append_float16_field(b, mr ? m->temp_fet_start : MCCONF_L_TEMP_FET_START_DEFAULT, 10, &i);
    append_float16_field(b, mr ? m->temp_fet_end : MCCONF_L_TEMP_FET_END_DEFAULT, 10, &i);
    append_float16_field(b, mr ? m->temp_motor_start : MCCONF_L_TEMP_MOTOR_START_DEFAULT, 10, &i);
    append_float16_field(b, mr ? m->temp_motor_end : MCCONF_L_TEMP_MOTOR_END_DEFAULT, 10, &i);
    append_float16_field(b, mr ? m->temp_accel_dec : MCCONF_L_TEMP_ACCEL_DEC_DEFAULT, 10000, &i);
    /* VESC l_min_duty is a positive low-duty threshold, not a negative
       reverse limit. Reverse is represented by the sign of COMM_SET_DUTY. */
    append_float16_field(b, mr ? m->min_duty : MCCONF_L_MIN_DUTY_DEFAULT, 10000, &i);
    append_float16_field(b, mr ? m->max_duty : MCCONF_L_MAX_DUTY_DEFAULT, 10000, &i);
    append_float32_auto_field(b, mr ? m->watt_max : MCCONF_L_WATT_MAX_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->watt_min : MCCONF_L_WATT_MIN_DEFAULT, &i);
    append_float16_field(b, mr ? m->current_max_scale : MCCONF_L_CURRENT_MAX_SCALE_DEFAULT, 10000, &i);
    append_float16_field(b, mr ? m->current_min_scale : MCCONF_L_CURRENT_MIN_SCALE_DEFAULT, 10000, &i);
    append_float16_field(b, mr ? m->duty_start : MCCONF_L_DUTY_START_DEFAULT, 10000, &i);
    append_float32_auto_field(b, 250, &i);
    append_float32_auto_field(b, 250, &i);
    append_float32_auto_field(b, 10, &i);
    append_float16_field(b, 0, 10, &i);
    append_float16_field(b, 0, 10000, &i);
    append_float32_auto_field(b, 1000, &i);
    append_float32_auto_field(b, 0, &i);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++) {
        b[i++] = legacy[k];
    }
    append_float32_auto_field(b, 2000, &i);
    append_float32_auto_field(b, mr ? m->current_kp : (right ? RIGHT_FOC_KP : LEFT_FOC_KP), &i);
    append_float32_auto_field(b, mr ? m->current_ki : (right ? RIGHT_FOC_KI : LEFT_FOC_KI), &i);
    append_float32_auto_field(b, (float)VESC_FOC_F_ZV_HZ, &i);
    append_float32_auto_field(b, mr ? m->foc_dt_us : FOC_DEADTIME_COMP_US, &i);
    b[i++] = (!right && mr && m->encoder.inverted) ? 1 : 0;
    append_float32_auto_field(b, (!right && mr) ? ((float)m->encoder.elec_offset_u16*360.0f/65536.0f) : 0.0f, &i);
    append_float32_auto_field(b, (!right && mr) ? m->encoder.electrical_ratio : (float)(mr ? m->pole_pairs : (right ? RIGHT_POLE_PAIRS : LEFT_POLE_PAIRS)), &i);
    b[i++] = foc_sensor_wire;
    append_float32_auto_field(b, mr ? m->foc_pll_kp : MCCONF_FOC_PLL_KP_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_pll_ki : MCCONF_FOC_PLL_KI_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_motor_l : MCCONF_FOC_MOTOR_L_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_motor_ld_lq_diff : 0.0f, &i);
    append_float32_auto_field(b, mr ? m->foc_motor_r : MCCONF_FOC_MOTOR_R_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_motor_flux_linkage : MCCONF_FOC_MOTOR_FLUX_LINKAGE_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_observer_gain : MCCONF_FOC_OBSERVER_GAIN_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_observer_gain_slow : MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT, &i);
    append_float16_field(b, mr ? m->foc_observer_offset : MCCONF_FOC_OBSERVER_OFFSET_DEFAULT, 1000, &i);
    append_float32_auto_field(b, mr ? m->foc_duty_dowmramp_kp : MCCONF_FOC_DUTY_DOWNRAMP_KP_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_duty_dowmramp_ki : MCCONF_FOC_DUTY_DOWNRAMP_KI_DEFAULT, &i);
    append_float16_field(b, mr ? m->foc_start_curr_dec : MCCONF_FOC_START_CURR_DEC_DEFAULT, 10000, &i);
    append_float32_auto_field(b, mr ? m->foc_start_curr_dec_rpm : MCCONF_FOC_START_CURR_DEC_RPM_DEFAULT, &i);
    append_float32_auto_field(b, mr ? m->foc_openloop_rpm : MCCONF_FOC_OPENLOOP_RPM_DEFAULT, &i);
    /* Exact VESC 6.00 MCCONF order. Two d-gain scaling fields exist before
       sensorless open-loop hysteresis. VESC 6.00 has one foc_sl_erpm field;
       foc_sl_erpm_start is a private runtime parameter in this F103 port. */
    append_float16_field(b, mr ? m->foc_openloop_rpm_low : MCCONF_FOC_OPENLOOP_RPM_LOW_DEFAULT, 1000, &i);
    append_float16_field(b,0.0f,1000,&i); /* foc_d_gain_scale_start: wire-only */
    append_float16_field(b,0.0f,1000,&i); /* foc_d_gain_scale_max_mod: wire-only */
    append_float16_field(b, mr ? m->foc_sl_openloop_hyst : MCCONF_FOC_SL_OPENLOOP_HYST_DEFAULT, 100, &i);
    append_float16_field(b, mr ? m->foc_sl_openloop_time_lock : MCCONF_FOC_SL_OPENLOOP_T_LOCK_DEFAULT, 100, &i);
    append_float16_field(b, mr ? m->foc_sl_openloop_time_ramp : MCCONF_FOC_SL_OPENLOOP_T_RAMP_DEFAULT, 100, &i);
    append_float16_field(b, mr ? m->foc_sl_openloop_time : MCCONF_FOC_SL_OPENLOOP_TIME_DEFAULT, 100, &i);
    append_float16_field(b, mr ? m->foc_sl_openloop_boost_q : MCCONF_FOC_SL_OPENLOOP_BOOST_Q_DEFAULT, 100, &i);
    append_float16_field(b, mr ? m->foc_sl_openloop_max_q : MCCONF_FOC_SL_OPENLOOP_MAX_Q_DEFAULT, 100, &i);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++) {
        b[i++] = hall[k];
    }
    append_float32_auto_field(b, mr ? m->foc_hall_interp_erpm : 500.0f, &i);
    append_float32_auto_field(b, mr ? m->foc_sl_erpm : MCCONF_FOC_SL_ERPM_DEFAULT, &i);
    /* Dua-shunt stock hoverboard hanya menjalankan satu current-control sample
       per periode PWM. Jangan iklankan foc_sample_v0_v7=true ke VESC Tool. */
    b[i++]=0; /* foc_sample_v0_v7 */
    b[i++]=0; /* foc_sample_high_current */
    b[i++] = (uint8_t)(mr ? m->foc_sat_comp_mode : MCCONF_FOC_SAT_COMP_MODE_DEFAULT);
    append_float16_field(b, mr ? m->foc_sat_comp : MCCONF_FOC_SAT_COMP_DEFAULT, 1000, &i);
    b[i++]=0; /* foc_temp_comp: no NTC runtime */
    append_float16_field(b, 25, 100, &i);
    append_float16_field(b, mr ? m->foc_current_filter_const : MCCONF_FOC_CURRENT_FILTER_CONST_DEFAULT, 10000, &i);
    b[i++] = (uint8_t)(mr ? m->foc_cc_decoupling : MCCONF_FOC_CC_DECOUPLING_DEFAULT);
    b[i++]=(uint8_t)(mr?m->foc_observer_type:MCCONF_FOC_OBSERVER_TYPE_DEFAULT); /* Ortega remains compiled default */
    /* Exact VESC firmware 6.00 HFI wire layout. HFI execution itself remains
       unsupported on this reduced F103 port, tetapi GET_MCCONF wajib memberi
       nilai default 6.00 yang valid agar VESC Tool tidak melakukan truncation. */
    append_float16_field(b, 20.0f, 10, &i);       /* foc_hfi_voltage_start */
    append_float16_field(b, 4.0f, 10, &i);        /* foc_hfi_voltage_run */
    append_float16_field(b, 6.0f, 10, &i);        /* foc_hfi_voltage_max: VESC 6.00 default */
    append_float16_field(b, 0.30f, 1000, &i);     /* foc_hfi_gain */
    append_float16_field(b, 0.0f, 100, &i);       /* foc_hfi_hyst */
    append_float32_auto_field(b, 3000.0f, &i);    /* foc_sl_erpm_hfi: VESC 6.00 default */
    vesc_buf_append_u16(b, 5U, &i);               /* foc_hfi_start_samples: VESC 6.00 default */
    append_float32_auto_field(b, 0.001f, &i);      /* foc_hfi_obs_ovr_sec */
    b[i++] = 1U;                                  /* HFI_SAMPLES_16 */
    b[i++]=1; /* offsets calibrated on boot */ append_float32_auto_field(b,0,&i);append_float32_auto_field(b,0,&i);append_float32_auto_field(b,0,&i);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 6; k++) {
        append_float16_field(b, 0, 10000, &i);
    }
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    /* Offset 303 in the exact VESC-6.00 wire image is foc_mtpa_mode. The
       original placeholder byte already occupied this slot; do not append a
       second byte here or every following field shifts by one. */
    b[i++] = (uint8_t)(mr ? m->foc_mtpa_mode : MCCONF_FOC_MTPA_MODE_DEFAULT);
    append_float32_auto_field(b, mr ? m->foc_fw_current_max : MCCONF_FOC_FW_CURRENT_MAX_DEFAULT, &i);
    append_float16_field(b, mr ? m->foc_fw_duty_start : MCCONF_FOC_FW_DUTY_START_DEFAULT, 10000, &i);
    append_float16_field(b, mr ? m->foc_fw_ramp_time : MCCONF_FOC_FW_RAMP_TIME_DEFAULT, 1000, &i);
    append_float16_field(b, mr ? m->foc_fw_q_current_factor : MCCONF_FOC_FW_Q_CURRENT_FACTOR_DEFAULT, 10000, &i);
    b[i++] = (uint8_t)(mr ? m->foc_speed_source : MCCONF_FOC_SPEED_SOURCE_DEFAULT);
    vesc_buf_append_i16(b, 0, &i);
    vesc_buf_append_i16(b, 0, &i);
    append_float16_field(b, 0, 10000, &i);
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    b[i++] = 5; /* 1 kHz */
    append_float32_auto_field(b, mr ? m->speed_pid.kp : SPEED_PID_KP, &i);
    append_float32_auto_field(b, mr ? m->speed_pid.ki : SPEED_PID_KI, &i);
    append_float32_auto_field(b, mr ? m->speed_pid.kd : SPEED_PID_KD, &i);
    append_float16_field(b, mr ? m->speed_kd_filter : SPEED_PID_D_FILTER, 10000, &i);
    append_float32_auto_field(b, mr ? m->speed_pid_min_erpm : SPEED_PID_MIN_ERPM, &i);
    b[i++] = (mr ? m->speed_pid_allow_braking : SPEED_PID_ALLOW_BRAKING) ? 1U : 0U;
    append_float32_auto_field(b, mr ? m->speed_pid_ramp_erpms_s : SPEED_PID_RAMP_ERPMS_S, &i);
    append_float32_auto_field(b, mr ? m->position_pid.kp : POSITION_PID_KP_CURRENT_PER_DEG, &i);
    append_float32_auto_field(b, mr ? m->position_pid.ki : POSITION_PID_KI_CURRENT_PER_DEG_S, &i);
    append_float32_auto_field(b, mr ? m->position_pid.kd : POSITION_PID_KD_CURRENT_PER_DEGPS, &i);
    append_float32_auto_field(b, mr ? m->position_kd_proc : POSITION_PID_KD_PROC, &i);
    append_float16_field(b, mr ? m->position_kd_filter : POSITION_PID_D_FILTER, 10000, &i);
    append_float32_auto_field(b, mr ? m->position_ang_div : POSITION_PID_ANG_DIV, &i);
    append_float16_field(b, mr ? m->position_gain_dec_angle : POSITION_PID_GAIN_DEC_ANGLE, 10, &i);
    append_float32_auto_field(b, mr ? m->position_offset_deg : POSITION_PID_OFFSET_DEG, &i);
    append_float16_field(b,0,10000,&i); /* BLDC-only upstream field; preserved for VESC6 ABI */
    append_float32_auto_field(b, mr ? m->cc_min_current : CURRENT_CTRL_MIN_CURRENT_A, &i);
    append_float32_auto_field(b, 1, &i);
    append_float16_field(b, 0.01f, 10000, &i);
    vesc_buf_append_i32(b, 500, &i);
    append_float16_field(b, 0.02f, 10000, &i);
    append_float32_auto_field(b, 1, &i);
    vesc_buf_append_u32(b, (!right && mr) ? m->encoder.cpr : (!right ? LEFT_ENCODER_CPR : 0U), &i);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 6; k++)
        append_float16_field(b, 0, 1000, &i);
    b[i++] = (foc_sensor_wire == VESC_FOC_SENSOR_ENCODER) ? VESC_SENSOR_PORT_ABI : VESC_SENSOR_PORT_HALL;
    b[i++] = (mr && m->invert_direction) ? 1 : 0;
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, PWM_FREQUENCY_HZ, &i);
    append_float32_auto_field(b, PWM_FREQUENCY_HZ, &i);
    append_float32_auto_field(b, PWM_FREQUENCY_HZ, &i);
    append_float32_auto_field(b, 3435, &i);
    b[i++] = 0;
    b[i++] = 8;
    append_float32_auto_field(b, 1, &i);
    append_float16_field(b, 10000, 0.1f, &i);
    append_float16_field(b, 25, 10, &i);
    b[i++] = 0;
    b[i++] = 8;
    b[i++] = (uint8_t)((mr ? m->pole_pairs : (right ? RIGHT_POLE_PAIRS : LEFT_POLE_PAIRS))*2U);
    append_float32_auto_field(b, mr ? m->si_gear_ratio : 1.0f, &i);
    append_float32_auto_field(b, mr ? m->si_wheel_diameter : 0.1f, &i);
    b[i++] = mr ? m->si_battery_type : 0U;
    b[i++] = mr ? m->si_battery_cells : 10U;
    append_float32_auto_field(b, mr ? m->si_battery_ah : 10.0f, &i);
    append_float32_auto_field(b, mr ? m->si_motor_nl_current : 1.0f, &i);
    b[i++] = 0;
    b[i++] = 0;
    append_float16_field(b, 60, 100, &i);
    append_float16_field(b, 80, 100, &i);
    append_float16_field(b, 0.8f, 1000, &i);
    append_float16_field(b, 0.9f, 1000, &i);
    b[i++] = 0;
    return i == (int32_t)VESC6_MCCONF_WIRE_SIZE;
}

// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi build_app_default: menjalankan operasi build app default sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool build_app_default(uint8_t *b) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    int32_t i = 0;
    memset(b, 0, VESC6_APPCONF_WIRE_SIZE);
    vesc_buf_append_u32(b, VESC6_APPCONF_SIGNATURE, &i);
    b[i++] = VESC_CONTROLLER_ID_LEFT;
    vesc_buf_append_u32(b, MOTOR_COMMAND_TIMEOUT_MS, &i);
    append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 1;
    b[i++] = 1;
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, 100000, &i);
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = VESC_APP_UART;
    b[i++] = 0;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 5; n++)
        append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    b[i++] = 1;
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    append_float16_field(b, 0, 1, &i);
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    append_float32_auto_field(b, 0.05f, &i);
    append_float16_field(b, 0.9f, 1000, &i);
    append_float16_field(b, 3.0f, 1000, &i);
    append_float16_field(b, 0.0f, 1000, &i);
    append_float16_field(b, 3.3f, 1000, &i);
    append_float16_field(b, 1.65f, 1000, &i);
    append_float16_field(b, 0.9f, 1000, &i);
    append_float16_field(b, 3.0f, 1000, &i);
    b[i++] = 1;
    b[i++] = 1;
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    append_float32_auto_field(b, 0.4f, &i);
    append_float32_auto_field(b, 0.2f, &i);
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 500, &i);
    vesc_buf_append_u32(b, VESC_UART_BAUD, &i);
    b[i++] = 0;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 6; n++)
        append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 10; n++)
        b[i++] = 0;
    b[i++] = 0;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 6; n++)
        append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 5; n++)
        append_float32_auto_field(b, 0, &i);
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 6; n++)
        vesc_buf_append_u16(b, 0, &i);
    b[i++] = 0;
    append_float16_field(b, 0, 100, &i);
    append_float16_field(b, 0, 100, &i);
    append_float16_field(b, 0, 1000, &i);
    append_float16_field(b, 0, 100, &i);
    append_float16_field(b, 0, 100, &i);
    append_float32_auto_field(b, 0, &i);
    append_float16_field(b, 0, 100, &i);
    append_float16_field(b, 0, 100, &i);
    append_float32_auto_field(b, 0, &i);
    append_float16_field(b, 0, 100, &i);
    append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    append_float16_field(b, 0, 100, &i);
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 4; n++)
        append_float32_auto_field(b, 0, &i);
    b[i++] = 0;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 6; n++)
        append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 12; n++)
        append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    append_float32_auto_field(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    vesc_buf_append_u16(b, 0, &i);
    b[i++] = 0;
    b[i++] = 0;
    append_float16_field(b, 0, 1000, &i);
    append_float16_field(b, 0, 10, &i);
    append_float16_field(b, 0, 10, &i);
    b[i++] = 0;
    vesc_buf_append_u16(b, 0, &i);
    b[i++] = 0;
    append_float16_field(b, 0, 100, &i);
    append_float16_field(b, 0, 100, &i);
    vesc_buf_append_u16(b, 0, &i);
    b[i++] = 0;
    b[i++] = 0;
    b[i++] = 0;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 4; n++)
        append_float16_field(b, 0, 1, &i);
    vesc_buf_append_u16(b, 0, &i);
    b[i++] = 0;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned n = 0; n < 13; n++)
        append_float32_auto_field(b, 0, &i);
    return i == (int32_t)VESC6_APPCONF_WIRE_SIZE;
}

// Fungsi vesc_config_init_defaults: menginisialisasi vesc config init defaults sehingga resource, konfigurasi
// awal, dan state modul siap digunakan dengan aman.
void vesc_config_init_defaults(void) {
    if (s_initialized)
        return;
    // Variabel ml: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ml = build_mc_default(s_mc_factory[MOTOR_LEFT], MOTOR_LEFT);
    // Variabel mr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool mr = build_mc_default(s_mc_factory[MOTOR_RIGHT], MOTOR_RIGHT);
    // Variabel ap: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ap = build_app_default(s_app_factory);
    s_layout_ok = ml && mr && ap;
    memcpy(s_mc_active, s_mc_factory, sizeof(s_mc_active));
    memcpy(s_app_active, s_app_factory, sizeof(s_app_active));
    s_initialized = true;
}
// Fungsi vesc_config_layout_ok: menjalankan operasi vesc config layout ok sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool vesc_config_layout_ok(void) {
    vesc_config_init_defaults();
    return s_layout_ok;
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter defaults: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_config_mc_wire: menjalankan operasi vesc config mc wire sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
const uint8_t *vesc_config_mc_wire(motor_id_t id, bool defaults) {
    vesc_config_init_defaults();
    return defaults ? s_mc_factory[id] : s_mc_active[id];
}
// Parameter defaults: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_config_app_wire: menjalankan operasi vesc config app wire sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
const uint8_t *vesc_config_app_wire(bool defaults) {
    vesc_config_init_defaults();
    return defaults ? s_app_factory : s_app_active;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi apply_mc: menjalankan operasi apply mc sesuai tanggung jawab modul dengan input tervalidasi dan state
// yang konsisten.
static bool apply_mc(motor_id_t id, const uint8_t *w) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (!m || !sig_ok(w, VESC6_MCCONF_SIGNATURE))
        return false;
    /* Preserve the live incremental coordinate across unrelated MCCONF writes.
       VESC Tool commonly writes the complete 481-byte image when only one
       limit/PID field changed; that must not silently rebase TIM4. */
    // Variabel encoder_was_active: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const bool encoder_was_active = (id == MOTOR_LEFT && m->sensor_mode == SENSOR_MODE_ENCODER);
    // Variabel old_pole_pairs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t old_pole_pairs = m->pole_pairs;
    // Variabel old_invert_direction: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool old_invert_direction = m->invert_direction;
    // Variabel old_foc_sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
    const mc_foc_sensor_mode old_foc_sensor_mode = m->foc_sensor_mode;
    // Variabel encoder_old_cpr: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const uint32_t encoder_old_cpr = m->encoder.cpr;
    // Variabel encoder_old_inverted: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const bool encoder_old_inverted = m->encoder.inverted;
    // Variabel encoder_old_offset_u16: offset kalibrasi untuk mengoreksi bias pengukuran.
    const uint16_t encoder_old_offset_u16 = m->encoder.elec_offset_u16;
    // Variabel encoder_old_ratio: data encoder untuk pengukuran posisi atau kecepatan rotor.
    const float encoder_old_ratio = m->encoder.electrical_ratio;
    /* Port F103 ini FOC-only. The complete VESC MCCONF wire schema is preserved,
       but BLDC/DC and HFI execution are deliberately rejected instead of
       silently mapping them to a different algorithm. */
    if (w[VESC6_MC_OFF_PWM_MODE] > PWM_MODE_BIPOLAR || w[VESC6_MC_OFF_COMM_MODE] > COMM_MODE_DELAY)
        return false;
    if (w[VESC6_MC_OFF_MOTOR_TYPE] != VESC_MOTOR_FOC)
        return false;
    if (w[VESC6_MC_OFF_FOC_SENSOR_MODE] >= FOC_SENSOR_MODE_HFI && w[VESC6_MC_OFF_FOC_SENSOR_MODE] <= FOC_SENSOR_MODE_HFI_V5)
        return false;
    /* Part-1 sensor policy follows the VESC FOC model while respecting this
       board's physical inputs: LEFT may use sensorless, Hall or incremental
       A/B; RIGHT may use sensorless or Hall only. HFI stays rejected above. */
    if (id == MOTOR_LEFT) {
        if (w[VESC6_MC_OFF_FOC_SENSOR_MODE] != FOC_SENSOR_MODE_SENSORLESS &&
            w[VESC6_MC_OFF_FOC_SENSOR_MODE] != VESC_FOC_SENSOR_ENCODER &&
            w[VESC6_MC_OFF_FOC_SENSOR_MODE] != VESC_FOC_SENSOR_HALL)
            return false;
    }
    else {
        if (w[VESC6_MC_OFF_FOC_SENSOR_MODE] != FOC_SENSOR_MODE_SENSORLESS &&
            w[VESC6_MC_OFF_FOC_SENSOR_MODE] != VESC_FOC_SENSOR_HALL)
            return false;
    }
    if (!runtime_mc_auto_fields_finite(w))
        return false;
    /* Match upstream VESC generated-MCCONF semantics: do not reject a complete
       6.00 configuration merely because this reduced F103 backend does not
       execute every UI field. Only enums that are dereferenced by the local
       FOC backend are range-checked here; numeric fields used by hardware are
       bounded below before assignment. The complete wire image is preserved. */
    if (w[VESC6_MC_OFF_FOC_CC_DECOUPLING] > FOC_CC_DECOUPLING_CROSS_BEMF ||
        w[VESC6_MC_OFF_FOC_SAT_COMP_MODE] > SAT_COMP_LAMBDA_AND_FACTOR ||
        w[VESC6_MC_OFF_FOC_OBSERVER_TYPE] > FOC_OBSERVER_MXV_LAMBDA_COMP_LIN ||
        w[VESC6_MC_OFF_FOC_MTPA_MODE] > MTPA_MODE_IQ_MEASURED ||
        w[VESC6_MC_OFF_FOC_SPEED_SOURCE] > FOC_SPEED_SRC_OBSERVER)
        return false;
    m->pwm_mode = (mc_pwm_mode)w[VESC6_MC_OFF_PWM_MODE];
    m->comm_mode = (mc_comm_mode)w[VESC6_MC_OFF_COMM_MODE];
    m->motor_type = MOTOR_TYPE_FOC;
    (void)w[VESC6_MC_OFF_SENSOR_MODE]; /* BLDC sensor byte is wire-only in this FOC-only build. */
    // Variabel current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_max = get_auto_at(w, VESC6_MC_OFF_L_CURRENT_MAX);
    // Variabel current_min: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_min = get_auto_at(w, VESC6_MC_OFF_L_CURRENT_MIN);
    // Variabel input_current_max: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float input_current_max = get_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MAX);
    // Variabel input_current_min: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float input_current_min = get_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MIN);
    // Variabel abs_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float abs_current = get_auto_at(w, VESC6_MC_OFF_L_ABS_CURRENT_MAX);
    // Variabel min_erpm: kecepatan listrik motor dalam electrical RPM.
    const float min_erpm = get_auto_at(w, VESC6_MC_OFF_L_MIN_ERPM);
    // Variabel max_erpm: kecepatan listrik motor dalam electrical RPM.
    const float max_erpm = get_auto_at(w, VESC6_MC_OFF_L_MAX_ERPM);
    // Variabel erpm_start: kecepatan listrik motor dalam electrical RPM.
    const float erpm_start = get_f16_at(w, VESC6_MC_OFF_L_ERPM_START, 10000.0f);
    // Variabel min_vin: batas atau nilai minimum untuk validasi dan proteksi.
    const float min_vin = get_auto_at(w, VESC6_MC_OFF_L_MIN_VIN);
    // Variabel max_vin: batas atau nilai maksimum untuk validasi dan proteksi.
    const float max_vin = get_auto_at(w, VESC6_MC_OFF_L_MAX_VIN);
    // Variabel battery_cut_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float battery_cut_start = get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_START);
    // Variabel battery_cut_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float battery_cut_end = get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_END);
    // Variabel temp_fet_start: nilai sementara atau temperatur sesuai konteks modul.
    const float temp_fet_start = get_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_START, 10.0f);
    // Variabel temp_fet_end: nilai sementara atau temperatur sesuai konteks modul.
    const float temp_fet_end = get_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_END, 10.0f);
    // Variabel temp_motor_start: nilai sementara atau temperatur sesuai konteks modul.
    const float temp_motor_start = get_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_START, 10.0f);
    // Variabel temp_motor_end: nilai sementara atau temperatur sesuai konteks modul.
    const float temp_motor_end = get_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_END, 10.0f);
    // Variabel temp_accel_dec: nilai sementara atau temperatur sesuai konteks modul.
    const float temp_accel_dec = get_f16_at(w, VESC6_MC_OFF_L_TEMP_ACCEL_DEC, 10000.0f);
    // Variabel start_curr_dec: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float start_curr_dec = get_f16_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC, 10000.0f);
    // Variabel start_curr_dec_rpm: kecepatan putar yang digunakan oleh logika kendali.
    const float start_curr_dec_rpm = get_auto_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC_RPM);
    // Variabel duty_i: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    int32_t duty_i = (int32_t)VESC6_MC_OFF_L_MIN_DUTY;
    // Variabel vesc_min_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float vesc_min_duty = vesc_buf_get_float16(w, 10000.0f, &duty_i);
    duty_i = (int32_t)VESC6_MC_OFF_L_MAX_DUTY;
    // Variabel max_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float max_duty = vesc_buf_get_float16(w, 10000.0f, &duty_i);
    // Variabel watt_max: batas atau nilai maksimum untuk validasi dan proteksi.
    const float watt_max = get_auto_at(w, VESC6_MC_OFF_L_WATT_MAX);
    // Variabel watt_min: batas atau nilai minimum untuk validasi dan proteksi.
    const float watt_min = get_auto_at(w, VESC6_MC_OFF_L_WATT_MIN);
    // Variabel current_max_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_max_scale = get_f16_at(w, VESC6_MC_OFF_L_CURRENT_MAX_SCALE, 10000.0f);
    // Variabel current_min_scale: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_min_scale = get_f16_at(w, VESC6_MC_OFF_L_CURRENT_MIN_SCALE, 10000.0f);
    // Variabel duty_start: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const float duty_start = get_f16_at(w, VESC6_MC_OFF_L_DUTY_START, 10000.0f);
    // Variabel current_kp: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_kp = get_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KP);
    // Variabel current_ki: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float current_ki = get_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KI);
    // Variabel si_gear_ratio: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float si_gear_ratio = get_auto_at(w, VESC6_MC_OFF_SI_GEAR_RATIO);
    // Variabel si_wheel_diameter: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float si_wheel_diameter = get_auto_at(w, VESC6_MC_OFF_SI_WHEEL_DIAMETER);
    // Variabel si_battery_ah: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float si_battery_ah = get_auto_at(w, VESC6_MC_OFF_SI_BATTERY_AH);
    // Variabel si_motor_nl_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    float si_motor_nl_current = get_auto_at(w, VESC6_MC_OFF_SI_MOTOR_NL_CURRENT);
    if (!isfinite(current_max) || !isfinite(current_min) ||
       !isfinite(input_current_max) || !isfinite(input_current_min) ||
       !isfinite(abs_current) || !isfinite(min_erpm) || !isfinite(max_erpm) || !isfinite(erpm_start) ||
       !isfinite(min_vin) || !isfinite(max_vin) ||
       !isfinite(battery_cut_start) || !isfinite(battery_cut_end) ||
       !isfinite(temp_fet_start) || !isfinite(temp_fet_end) || !isfinite(temp_motor_start) || !isfinite(temp_motor_end) || !isfinite(temp_accel_dec) ||
       !isfinite(start_curr_dec) || !isfinite(start_curr_dec_rpm) ||
       !isfinite(max_duty) || !isfinite(vesc_min_duty) ||
       !isfinite(watt_max) || !isfinite(watt_min) || !isfinite(current_max_scale) ||
       !isfinite(current_min_scale) || !isfinite(duty_start) ||
       !isfinite(current_kp) || !isfinite(current_ki) ||
       !isfinite(si_gear_ratio) || !isfinite(si_wheel_diameter) ||
       !isfinite(si_battery_ah) || !isfinite(si_motor_nl_current))
        return false;

    /* Do not reinterpret malformed VESC limits. The canonical wire image and
       the runtime must agree on direction/sign; only magnitude clipping to the
       physical F103 power-stage envelope is allowed afterwards. */
    if (current_max < 0.1f || current_min > 0.0f ||
       input_current_max < 0.0f || input_current_min > 0.0f ||
       abs_current < fmaxf(current_max, fabsf(current_min)) ||
       min_erpm >= max_erpm || max_erpm <= 0.0f || min_erpm >= 0.0f ||
       max_duty < 0.0f || max_duty > 1.0f ||
       vesc_min_duty < 0.0f || vesc_min_duty > max_duty)
       return false;
    if (max_vin <= min_vin || si_gear_ratio <= 0.0f || si_wheel_diameter <= 0.0f || si_battery_ah < 0.0f)
        return false;
    // Variabel poles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t poles = w[VESC6_MC_OFF_SI_MOTOR_POLES];
    if (poles < 2U || (poles&1U) || poles > 120U)
        return false;
    // Variabel pp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t pp = poles/2U;
    /* The preflight above has already guaranteed that accepted VESC values fit
       the physical F103 envelope. The clamp calls below are defensive bounds,
       not a hidden reinterpretation of an ACKed configuration. */
    m->current_max_a = clampf(current_max, 0.1f, FOC_MAX_CURRENT_A);
    m->current_min_a = clampf(current_min, -FOC_MAX_CURRENT_A, 0.0f);
    m->input_current_max_a = clampf(input_current_max, 0.0f, FOC_MAX_CURRENT_A);
    m->input_current_min_a = clampf(input_current_min, -FOC_MAX_CURRENT_A, 0.0f);
    m->abs_current_max_a = clampf(abs_current, m->current_max_a, FOC_ABS_CURRENT_TRIP_A);
    m->abs_current_trip_q15 = amp_to_q15(m->abs_current_max_a);
    m->min_vin = clampf(min_vin, VBUS_MIN_RUN_V, VBUS_MAX_RUN_V-0.5f);
    m->max_vin = clampf(max_vin, m->min_vin+0.5f, VBUS_MAX_RUN_V);
    m->battery_cut_start = battery_cut_start;
    m->battery_cut_end = battery_cut_end;
    m->slow_abs_current = w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT] != 0U;
    m->temp_fet_start = temp_fet_start;
    m->temp_fet_end = temp_fet_end;
    m->temp_motor_start = temp_motor_start;
    m->temp_motor_end = temp_motor_end;
    m->temp_accel_dec = temp_accel_dec;
    m->additional_faults = MCCONF_L_ADDITIONAL_FAULTS_DEFAULT;
    m->foc_start_curr_dec = start_curr_dec;
    m->foc_start_curr_dec_rpm = start_curr_dec_rpm;
    m->battery_regen_cut_start = m->max_vin-MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    m->battery_regen_cut_end = m->max_vin-MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    m->min_vin_q15 = volt_to_q15(m->min_vin);
    m->max_vin_q15 = volt_to_q15(m->max_vin);
    m->hard_max_vin_q15 = volt_to_q15(fminf(m->max_vin+FOC_VBUS_HARD_OV_MARGIN_V, FOC_VBUS_HARD_MAX_V));
    m->hard_min_vin_q15 = volt_to_q15(fmaxf(m->min_vin-FOC_VBUS_HARD_UV_MARGIN_V, FOC_VBUS_HARD_MIN_V));
    m->over_voltage_fault_count = 0U;
    m->under_voltage_fault_count = 0U;
    m->min_erpm = clampf(min_erpm, -MOTOR_DEFAULT_MAX_ERPM, -1.0f);
    m->max_erpm = clampf(max_erpm, 1.0f, MOTOR_DEFAULT_MAX_ERPM);
    m->erpm_start = clampf(erpm_start, 0.0f, 1.0f);
    m->max_duty = clampf(max_duty, 0.01f, 0.98f);
    m->min_duty = clampf(vesc_min_duty, 0.0f, m->max_duty);
    m->watt_max = watt_max;
    m->watt_min = watt_min;
    m->current_max_scale = current_max_scale;
    m->current_min_scale = current_min_scale;
    m->duty_start = duty_start;
    m->current_kp = clampf(current_kp, 0.00001f, 10.0f);
    m->current_ki = clampf(current_ki, 0.0f, 200000.0f);
    m->current_kp_q16 = current_gain_to_fast_q16(m->current_kp);
    m->current_ki_dt_q16 = current_ki_to_fast_q16(m->current_ki);
    m->foc_pll_kp = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_PLL_KP), 0.0f, 100000.0f);
    m->foc_pll_ki = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_PLL_KI), 0.0f, 1000000.0f);
    m->foc_motor_l = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_L), 1.0e-7f, 0.1f);
    m->foc_motor_ld_lq_diff = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_LD_LQ_DIFF), -0.1f, 0.1f);
    m->foc_motor_r = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_R), 1.0e-5f, 100.0f);
    m->foc_motor_flux_linkage = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_FLUX_LINKAGE), 1.0e-6f, FOC_FLUX_Q_BASE_WB*1.90f);
    m->foc_observer_gain = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_OBSERVER_GAIN), 0.0f, 1000000.0f);
    m->foc_observer_gain_slow = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_OBSERVER_GAIN_SLOW), 0.0f, 1.0f);
    m->foc_observer_offset = clampf(get_f16_at(w, VESC6_MC_OFF_FOC_OBSERVER_OFFSET, 1000.0f), -10.0f, 10.0f);
    m->foc_duty_dowmramp_kp = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP), 0.0f, 100000.0f);
    m->foc_duty_dowmramp_ki = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI), 0.0f, 1000000.0f);
    m->foc_current_filter_const = clampf(get_f16_at(w, VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST, 10000.0f), 0.0f, 1.0f);
    m->foc_cc_decoupling = (mc_foc_cc_decoupling_mode)w[VESC6_MC_OFF_FOC_CC_DECOUPLING];
    m->foc_sat_comp_mode = (SAT_COMP_MODE)w[VESC6_MC_OFF_FOC_SAT_COMP_MODE];
    m->foc_sat_comp = clampf(get_f16_at(w, VESC6_MC_OFF_FOC_SAT_COMP, 1000.0f), 0.0f, 1.0f);
    m->foc_observer_type = (mc_foc_observer_type)w[VESC6_MC_OFF_FOC_OBSERVER_TYPE];
    m->foc_mtpa_mode = (MTPA_MODE)w[VESC6_MC_OFF_FOC_MTPA_MODE];
    m->foc_fw_current_max = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_FW_CURRENT_MAX), 0.0f, FOC_MAX_CURRENT_A);
    m->foc_fw_duty_start = clampf(get_f16_at(w, VESC6_MC_OFF_FOC_FW_DUTY_START, 10000.0f), 0.0f, 1.0f);
    m->foc_fw_ramp_time = clampf(get_f16_at(w, VESC6_MC_OFF_FOC_FW_RAMP_TIME, 1000.0f), 0.01f, 30.0f);
    m->foc_fw_q_current_factor = clampf(get_f16_at(w, VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR, 10000.0f), 0.0f, 1.0f);
    m->foc_speed_source = (FOC_SPEED_SRC)w[VESC6_MC_OFF_FOC_SPEED_SOURCE];
    m->foc_fw_backoff = MCCONF_FOC_FW_BACKOFF_DEFAULT;
    m->foc_mag_vd_max = MCCONF_FOC_MAG_VD_MAX_DEFAULT;
    m->foc_overmod_factor = MCCONF_FOC_OVERMOD_FACTOR_DEFAULT;
    m->foc_temp_comp = MCCONF_FOC_TEMP_COMP_DEFAULT;
    m->foc_temp_comp_base_temp = MCCONF_FOC_TEMP_COMP_BASE_TEMP_DEFAULT;
    m->foc_offsets_cal_mode = MCCONF_FOC_OFFSETS_CAL_MODE_DEFAULT;
    m->foc_openloop_rpm = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM), 10.0f, MOTOR_DEFAULT_MAX_ERPM);
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_OPENLOOP_RPM_LOW;
        m->foc_openloop_rpm_low = clampf(vesc_buf_get_float16(w, 1000.0f, &q), 0.0f, 1.0f);
    }
    /* foc_d_gain_scale_start/max_mod are part of the VESC-6.00 wire image.
       This backend preserves them byte-exact for VESC Tool compatibility even
       though they do not have a local controller consumer. */
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_SL_OPENLOOP_HYST;
        m->foc_sl_openloop_hyst = clampf(vesc_buf_get_float16(w, 100.0f, &q), 0.0f, 100.0f);
    }
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_SL_OPENLOOP_TIME_LOCK;
        m->foc_sl_openloop_time_lock = clampf(vesc_buf_get_float16(w, 100.0f, &q), 0.0f, 20.0f);
    }
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_SL_OPENLOOP_TIME_RAMP;
        m->foc_sl_openloop_time_ramp = clampf(vesc_buf_get_float16(w, 100.0f, &q), 0.01f, 20.0f);
    }
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_SL_OPENLOOP_TIME;
        m->foc_sl_openloop_time = clampf(vesc_buf_get_float16(w, 100.0f, &q), 0.01f, 20.0f);
    }
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_SL_OPENLOOP_BOOST_Q;
        m->foc_sl_openloop_boost_q = clampf(vesc_buf_get_float16(w, 100.0f, &q), 0.0f, FOC_MAX_CURRENT_A);
    }
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = VESC6_MC_OFF_FOC_SL_OPENLOOP_MAX_Q;
        m->foc_sl_openloop_max_q = clampf(vesc_buf_get_float16(w, 100.0f, &q), 0.1f, FOC_MAX_CURRENT_A);
    }
    m->foc_hall_interp_erpm = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_HALL_INTERP_ERPM), 0.0f, MOTOR_DEFAULT_MAX_ERPM);
    m->foc_hall_interp_erpm_u32 = (uint32_t)lrintf(m->foc_hall_interp_erpm);
    m->foc_sl_erpm = clampf(get_auto_at(w, VESC6_MC_OFF_FOC_SL_ERPM), 10.0f, MOTOR_DEFAULT_MAX_ERPM);
    /* Private startup-validity threshold. It must not consume a VESC 6.00
       wire field, so derive it conservatively from the configured handover. */
    m->foc_sl_erpm_start = clampf(MCCONF_FOC_SL_ERPM_START_DEFAULT, 10.0f, m->foc_sl_erpm);
    m->speed_pid.kp = clampf(get_auto_at(w, VESC6_MC_OFF_S_PID_KP), 0.0f, 1000.0f);
    m->speed_pid.ki = clampf(get_auto_at(w, VESC6_MC_OFF_S_PID_KI), 0.0f, 1000.0f);
    m->speed_pid.kd = clampf(get_auto_at(w, VESC6_MC_OFF_S_PID_KD), 0.0f, 1000.0f);
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = 342;
        m->speed_kd_filter = clampf(vesc_buf_get_float16(w, 10000.0f, &q), 0.0f, 1.0f);
    }
    m->speed_pid_min_erpm = clampf(get_auto_at(w, VESC6_MC_OFF_S_PID_MIN_ERPM), 0.0f, MOTOR_DEFAULT_MAX_ERPM);
    m->speed_pid_allow_braking = w[VESC6_MC_OFF_S_PID_ALLOW_BRAKING] != 0U;
    m->speed_pid_ramp_erpms_s = clampf(get_auto_at(w, VESC6_MC_OFF_S_PID_RAMP_ERPMS_S), 0.0f, 1000000.0f);
    m->speed_pid_source=S_PID_SPEED_SRC_PLL; /* VESC6 wire has no speed-source field. */
    m->position_pid.kp = clampf(get_auto_at(w, VESC6_MC_OFF_P_PID_KP), 0.0f, 1000.0f);
    m->position_pid.ki = clampf(get_auto_at(w, VESC6_MC_OFF_P_PID_KI), 0.0f, 1000.0f);
    m->position_pid.kd = clampf(get_auto_at(w, VESC6_MC_OFF_P_PID_KD), 0.0f, 1000.0f);
    m->position_kd_proc = clampf(get_auto_at(w, VESC6_MC_OFF_P_PID_KD_PROC), 0.0f, 1000.0f);
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = 369;
        m->position_kd_filter = clampf(vesc_buf_get_float16(w, 10000.0f, &q), 0.0f, 1.0f);
    }
    m->position_ang_div = clampf(get_auto_at(w, VESC6_MC_OFF_P_PID_ANG_DIV), 0.01f, 1000.0f);
    {
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t q = 375;
        m->position_gain_dec_angle = clampf(vesc_buf_get_float16(w, 10.0f, &q), 0.0f, 3600.0f);
    }
    m->position_offset_deg = clampf(get_auto_at(w, VESC6_MC_OFF_P_PID_OFFSET), -36000.0f, 36000.0f);
    m->cc_min_current = clampf(get_auto_at(w, VESC6_MC_OFF_CC_MIN_CURRENT), 0.0f, FOC_MAX_CURRENT_A);
    m->pole_pairs = pp;
    m->invert_direction = w[VESC6_MC_OFF_M_INVERT_DIRECTION] != 0U;
    m->si_gear_ratio = clampf(si_gear_ratio, 0.01f, 1000.0f);
    m->si_wheel_diameter = clampf(si_wheel_diameter, 0.001f, 10.0f);
    m->si_battery_type = w[VESC6_MC_OFF_SI_BATTERY_TYPE];
    m->si_battery_cells = w[VESC6_MC_OFF_SI_BATTERY_CELLS];
    m->si_battery_ah = clampf(si_battery_ah, 0.0f, 10000.0f);
    m->si_motor_nl_current = clampf(si_motor_nl_current, 0.0f, FOC_MAX_CURRENT_A);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++) {
        m->foc_hall_table[k] = w[223+k];
        if (w[223+k] == 255U) {
            m->hall_table[k] = -1;
            m->hall_angle_u16[k] = 0;
        }
        else {
            m->hall_angle_u16[k] = (uint16_t)(((uint32_t)w[223+k]*65536U)/200U);
            m->hall_table[k] = (int8_t)(((uint32_t)w[223+k]*6U)/200U);
        }
    }
    if (id == MOTOR_LEFT) {
        // Variabel enc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const bool enc = (w[VESC6_MC_OFF_FOC_SENSOR_MODE] == VESC_FOC_SENSOR_ENCODER);
        // Variabel cpr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint32_t cpr = get_u32_at(w, VESC6_MC_OFF_M_ENCODER_COUNTS);
        if (enc && (cpr < 4U || cpr > 65535U))
            return false;
        if (cpr >= 4U && cpr <= 65535U)
            m->encoder.cpr = cpr;

        // Variabel encoder_new_inverted: data encoder untuk pengukuran posisi atau kecepatan rotor.
        const bool encoder_new_inverted = w[VESC6_MC_OFF_FOC_ENCODER_INVERTED] != 0U;
        // Variabel off: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float off = get_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_OFFSET);
        while (off < 0.0f)
            off += 360.0f;
        while (off >= 360.0f)
            off -= 360.0f;
        // Variabel encoder_new_offset_u16: offset kalibrasi untuk mengoreksi bias pengukuran.
        const uint16_t encoder_new_offset_u16 = (uint16_t)lrintf(off*(65536.0f/360.0f));
        // Variabel ratio: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float ratio = get_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_RATIO);
        if (!isfinite(ratio) || ratio <= 0.0f || ratio > 1000.0f)
            return false;
        // Variabel rq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint64_t rq = (uint64_t)llrintf(ratio*65536.0f);
        // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint64_t step = (rq<<16)/m->encoder.cpr;
        if (rq == 0U || rq > 0xFFFFFFFFULL || step > 0xFFFFFFFFULL)
            return false;

        // Variabel encoder_hw_changed: data encoder untuk pengukuran posisi atau kecepatan rotor.
        const bool encoder_hw_changed = enc && (!encoder_was_active || encoder_old_cpr != m->encoder.cpr);
        // Variabel encoder_phase_changed: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
        const bool encoder_phase_changed = enc &&
            (encoder_old_inverted != encoder_new_inverted ||
             encoder_old_offset_u16 != encoder_new_offset_u16 ||
             fabsf(encoder_old_ratio - ratio) > 1.0e-6f);

        m->encoder.inverted = encoder_new_inverted;
        m->encoder.elec_offset_u16 = encoder_new_offset_u16;
        m->encoder.electrical_ratio = ratio;
        m->encoder.electrical_ratio_q16 = (uint32_t)rq;
        m->encoder.phase_per_count_q16 = (uint32_t)step;
        /* VESC 6.00 wire encoder enum is 1. Map it to the internal A/B-only
           runtime strategy used by this board without changing the wire ABI. */
        /* VESC menyimpan foc_sensor_mode dan m_sensor_port_mode sebagai field
           MCCONF terpisah. Pada PCB hoverboard ini mux fisik sudah pasti:
           encoder memakai ABI LEFT, Hall memakai EXTI Hall, sedangkan sensorless
           melepas keduanya. Byte m_sensor_port_mode tetap dipertahankan untuk
           round-trip VESC Tool, tetapi mux fisik diturunkan dari foc_sensor_mode. */
        m->foc_sensor_mode = enc ? FOC_SENSOR_MODE_ENCODER_AB : (mc_foc_sensor_mode)w[VESC6_MC_OFF_FOC_SENSOR_MODE];
        // Variabel sensorless: menandai bahwa FOC memakai observer tanpa input Hall/encoder fisik.
        const bool sensorless = m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
        m->sensor_request_mode = enc ? SENSOR_MODE_ENCODER :
                                 (sensorless ? SENSOR_MODE_NO_SENSOR : SENSOR_MODE_HALL);
        if (enc) {
            if (encoder_hw_changed) {
                /* Switching to ABI or changing CPR genuinely changes the TIM4
                   hardware coordinate and therefore requires re-init. */
                (void)encoder_init(m);
            }
            else {
                /* Keep TIM4 turns/extended/session-zero untouched for ordinary
                   full-image VESC Tool writes. */
                m->sensor_mode = SENSOR_MODE_ENCODER;
            }
            if (encoder_hw_changed || encoder_phase_changed) {
                m->encoder.synced = false;
                m->encoder.motion_proved = false;
                m->encoder.sync_active = false;
                m->encoder.speed_sample_valid = false;
                m->using_encoder = false;
            }
        }
        else {
            encoder_deinit(m);
            motor_hw_configure_sensor(m, sensorless ? SENSOR_MODE_NO_SENSOR : SENSOR_MODE_HALL);
            if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL)
                motor_hall_edge_isr(m);
        }
    }
    else {
        if (w[VESC6_MC_OFF_FOC_SENSOR_MODE] == VESC_FOC_SENSOR_ENCODER)
            return false;
        m->foc_sensor_mode = (mc_foc_sensor_mode)w[VESC6_MC_OFF_FOC_SENSOR_MODE];
        // Variabel sensorless: menandai bahwa motor RIGHT juga benar-benar melepas EXTI Hall saat observer dipilih.
        const bool sensorless = m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;
        m->sensor_request_mode = sensorless ? SENSOR_MODE_NO_SENSOR : SENSOR_MODE_HALL;
        motor_hw_configure_sensor(m, m->sensor_request_mode);
        if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL)
            motor_hall_edge_isr(m);
    }
    if (old_pole_pairs != m->pole_pairs || old_invert_direction != m->invert_direction ||
        old_foc_sensor_mode != m->foc_sensor_mode ||
        (id == MOTOR_LEFT && encoder_old_cpr != m->encoder.cpr)) {
        m->stats.tachometer_source_valid = false;
    }

    /* R/L/flux, decoupling, filter and observer-offset changes alter fixed
       coefficients immediately. SET_MCCONF already enforces motor-stopped. */
#ifndef VESC_CONFIG_UNIT_TEST
    foc_precalc_values(m);
    foc_observer_reset(m, m->observer_phase_u16);
#endif
    return true;
}

// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi app_adc_wire_valid: menjalankan operasi app adc wire valid sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool app_adc_wire_valid(const uint8_t *w) {
    // Variabel ctrl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t ctrl = w[VESC6_APP_OFF_ADC_CTRL_TYPE];
    /* Mode analog yang tidak membutuhkan tombol eksternal didukung penuh.
     * Variant *_BUTTON tetap ditolak karena board ini tidak mempunyai input
     * reverse/cruise digital yang dapat dipetakan secara aman. */
    if (ctrl != ADC_CTRL_TYPE_NONE &&
        ctrl != ADC_CTRL_TYPE_CURRENT &&
        ctrl != ADC_CTRL_TYPE_CURRENT_REV_CENTER &&
        ctrl != ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER &&
        ctrl != ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC &&
        ctrl != ADC_CTRL_TYPE_DUTY &&
        ctrl != ADC_CTRL_TYPE_DUTY_REV_CENTER &&
        ctrl != ADC_CTRL_TYPE_PID &&
        ctrl != ADC_CTRL_TYPE_PID_REV_CENTER)
        return false;

    // Variabel hyst: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float hyst = get_auto_at(w, VESC6_APP_OFF_ADC_HYST);
    // Variabel vs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float vs = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_START, 1000.0f);
    // Variabel ve: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ve = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_END, 1000.0f);
    // Variabel vmin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float vmin = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_MIN, 1000.0f);
    // Variabel vmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float vmax = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_MAX, 1000.0f);
    // Variabel vc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float vc = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE_CENTER, 1000.0f);
    // Variabel v2s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v2s = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE2_START, 1000.0f);
    // Variabel v2e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float v2e = get_f16_at(w, VESC6_APP_OFF_ADC_VOLTAGE2_END, 1000.0f);
    // Variabel expa: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float expa = get_auto_at(w, VESC6_APP_OFF_ADC_THROTTLE_EXP);
    // Variabel expb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float expb = get_auto_at(w, VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE);
    // Variabel rpos: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float rpos = get_auto_at(w, VESC6_APP_OFF_ADC_RAMP_TIME_POS);
    // Variabel rneg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float rneg = get_auto_at(w, VESC6_APP_OFF_ADC_RAMP_TIME_NEG);
    // Variabel tcd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float tcd = get_auto_at(w, VESC6_APP_OFF_ADC_TC_MAX_DIFF);
    // Variabel rate: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint16_t rate = get_u16_at(w, VESC6_APP_OFF_ADC_UPDATE_RATE_HZ);

    if (!isfinite(hyst) || !isfinite(expa) || !isfinite(expb) ||
        !isfinite(rpos) || !isfinite(rneg) || !isfinite(tcd))
        return false;
    if (hyst < 0.0f || hyst > 0.30f)
        return false;
    if (vmin < 0.0f || vmax > 3.30f || vmin > vmax)
        return false;
    if (vs < vmin || ve > vmax || ve - vs < 0.05f)
        return false;
    if (vc < 0.0f || vc > 3.30f || v2s < 0.0f || v2e > 3.30f || v2e - v2s < 0.05f)
        return false;
    const bool centered = ctrl == ADC_CTRL_TYPE_CURRENT_REV_CENTER ||
                          ctrl == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER ||
                          ctrl == ADC_CTRL_TYPE_DUTY_REV_CENTER ||
                          ctrl == ADC_CTRL_TYPE_PID_REV_CENTER;
    if (centered && (vc - vs < 0.025f || ve - vc < 0.025f))
        return false;
    if (w[VESC6_APP_OFF_ADC_USE_FILTER] > 1U ||
        w[VESC6_APP_OFF_ADC_SAFE_START] > SAFE_START_NO_FAULT ||
        w[VESC6_APP_OFF_ADC_VOLTAGE_INVERTED] > 1U ||
        w[VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED] > 1U ||
        w[VESC6_APP_OFF_ADC_MULTI_ESC] > 1U || w[VESC6_APP_OFF_ADC_TC] > 1U)
        return false;
    /* No external reverse/cruise buttons and no traction-control backend on
       this reduced board. Reject rather than fake support. */
    if (w[VESC6_APP_OFF_ADC_BUTTONS] != 0U || w[VESC6_APP_OFF_ADC_TC] != 0U || fabsf(tcd) > 1.0e-6f)
        return false;
    if (w[VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE] > THR_EXP_POLY)
        return false;
    if (expa < -1.0f || expa > 1.0f || expb < -1.0f || expb > 1.0f)
        return false;
    if (rpos < 0.0f || rpos > 20.0f || rneg < 0.0f || rneg > 20.0f)
        return false;
    if (rate < 10U || rate > 1000U)
        return false;
    return true;
}

// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi apply_app: menjalankan operasi apply app sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static bool apply_app(const uint8_t *w) {
    if (!sig_ok(w, VESC6_APPCONF_SIGNATURE))
        return false;
    // Variabel timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    uint32_t timeout = get_u32_at(w, VESC6_APP_OFF_TIMEOUT_MSEC);
    // Variabel brake: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float brake = get_auto_at(w, VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT);
    if (timeout > 600000U || !isfinite(brake))
        return false;

    if (w[VESC6_APP_OFF_CONTROLLER_ID] != VESC_CONTROLLER_ID_LEFT)
        return false;
    // Variabel app: state atau konfigurasi aplikasi VESC.
    const uint8_t app = w[VESC6_APP_OFF_APP_TO_USE];
    if (app != VESC_APP_NONE && app != VESC_APP_ADC && app != VESC_APP_UART && app != VESC_APP_ADC_UART)
        return false;
    if (!app_adc_wire_valid(w))
        return false;

    /* USART3 PB10/PB11 is permanent management transport even when APP_ADC is
       selected, so its VESC-6 baud field must describe the real hardware. */
    if (get_u32_at(w, VESC6_APP_OFF_UART_BAUD) != VESC_UART_BAUD)
        return false;

    timeout_configure(timeout, brake);
    app_notify_configuration_changed();
    return true;
}

/* Board-side counterpart of VESC commands_apply_mcconf_hw_limits(). Keep
   generated MCCONF semantics intact while truncating only values constrained
   by the STM32F103 hoverboard power stage. The call site writes every such
   truncation back to the canonical wire image before persistence/readback. */
// Parameter mcconf: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi vesc_config_apply_mcconf_hw_limits: menjalankan operasi vesc config apply mcconf hw limits sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void vesc_config_apply_mcconf_hw_limits(mc_configuration *mcconf) {
    if (mcconf == NULL)
        return;
    /* Same model as upstream commands_apply_mcconf_hw_limits(): values that
       are genuinely constrained by this power stage are truncated before the
       configuration is stored, so GET_MCCONF reports the value that is really
       executed. UI-only generated fields are not modified. */
    utils_truncate_number(&mcconf->l_current_max, 0.1f, FOC_MAX_CURRENT_A);
    utils_truncate_number(&mcconf->l_current_min, -FOC_MAX_CURRENT_A, 0.0f);
    utils_truncate_number(&mcconf->l_in_current_max, 0.0f, FOC_MAX_CURRENT_A);
    utils_truncate_number(&mcconf->l_in_current_min, -FOC_MAX_CURRENT_A, 0.0f);
    // Variabel abs_floor: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float abs_floor = fmaxf(mcconf->l_current_max, fabsf(mcconf->l_current_min));
    utils_truncate_number(&mcconf->l_abs_current_max, abs_floor, FOC_ABS_CURRENT_TRIP_A);
    utils_truncate_number(&mcconf->l_min_erpm, -MOTOR_DEFAULT_MAX_ERPM, -1.0f);
    utils_truncate_number(&mcconf->l_max_erpm, 1.0f, MOTOR_DEFAULT_MAX_ERPM);
    utils_truncate_number(&mcconf->l_min_vin, VBUS_MIN_RUN_V, VBUS_MAX_RUN_V - 0.5f);
    utils_truncate_number(&mcconf->l_max_vin, mcconf->l_min_vin + 0.5f, VBUS_MAX_RUN_V);
    utils_truncate_number(&mcconf->l_max_duty, 0.01f, 0.98f);
    utils_truncate_number(&mcconf->l_min_duty, 0.0f, mcconf->l_max_duty);
    utils_truncate_number(&mcconf->l_current_max_scale, 0.0f, 1.0f);
    utils_truncate_number(&mcconf->l_current_min_scale, 0.0f, 1.0f);
    utils_truncate_number(&mcconf->l_erpm_start, 0.0f, 1.0f);
    utils_truncate_number(&mcconf->foc_overmod_factor, 1.0f, 1.5f);
    utils_truncate_number_abs(&mcconf->foc_sl_erpm_start, mcconf->foc_sl_erpm * 0.9f);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter wire: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_config_set_mc_wire: mengatur vesc config set mc wire setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
bool vesc_config_set_mc_wire(motor_id_t id, const uint8_t *wire, uint16_t len, bool store) {
    vesc_config_init_defaults();
    if (!wire || len != VESC6_MCCONF_WIRE_SIZE || !sig_ok(wire, VESC6_MCCONF_SIGNATURE))
        return false;
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return false;
    /* VESC 6.00 stores the complete generated MCCONF image. Do the same:
       preserve every byte that VESC Tool sends, even when this reduced F103
       backend does not consume that particular field at run time. Runtime
       safety is enforced by apply_mc() for the subset that affects hardware.
       Rejecting unrelated bytes made VESC Tool re-read the old image and show
       "Parameters truncated" for otherwise valid sensor-mode changes. */
    /* Same-value writes are idempotent. In particular, do not revoke the
       LEFT no-index encoder alignment just because VESC Tool writes back the
       configuration it has just read. */
    const bool hfi_legacy_needs_migration =
        get_u16_at(wire, VESC6_MC_OFF_FOC_HFI_START_SAMPLES) < 5U;
    if (memcmp(wire, s_mc_active[id], VESC6_MCCONF_WIRE_SIZE) == 0 &&
        !hfi_legacy_needs_migration) {
        if (!store) {
            return true;
        }
        const bool saved = conf_general_store_mc_wire_persistent(id, s_mc_active[id]);
        return saved;
    }
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (m->pwm_enabled || m->detect.busy)
        return false;
    motor_stop(m);
    memcpy(s_rollback_mc[id], s_mc_active[id], sizeof(s_rollback_mc[id]));
    memcpy(s_mc_active[id], wire, len);
    normalize_vesc6_hfi_ui_fields(s_mc_active[id]);
    /* Apply the board hardware limits before storing, like upstream VESC.
       Every wire-backed value that is truncated here is written back to the
       canonical image, so a VESC Tool "Parameters truncated" dialog only
       occurs for a real hardware-limit clamp, not because unrelated MCCONF
       fields were silently rejected. */
    {
        // Variabel mc_trunc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        mc_configuration mc_trunc;
        mcconf_decode_wire(s_mc_active[id], &mc_trunc);
        vesc_config_apply_mcconf_hw_limits(&mc_trunc);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_CURRENT_MAX, mc_trunc.l_current_max);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_CURRENT_MIN, mc_trunc.l_current_min);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_IN_CURRENT_MAX, mc_trunc.l_in_current_max);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_IN_CURRENT_MIN, mc_trunc.l_in_current_min);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_ABS_CURRENT_MAX, mc_trunc.l_abs_current_max);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_MIN_ERPM, mc_trunc.l_min_erpm);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_MAX_ERPM, mc_trunc.l_max_erpm);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_MIN_VIN, mc_trunc.l_min_vin);
        put_auto_at(s_mc_active[id], VESC6_MC_OFF_L_MAX_VIN, mc_trunc.l_max_vin);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_MIN_DUTY, mc_trunc.l_min_duty, 10000.0f);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_MAX_DUTY, mc_trunc.l_max_duty, 10000.0f);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_CURRENT_MAX_SCALE, mc_trunc.l_current_max_scale, 10000.0f);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_CURRENT_MIN_SCALE, mc_trunc.l_current_min_scale, 10000.0f);
        put_f16_at(s_mc_active[id], VESC6_MC_OFF_L_ERPM_START, mc_trunc.l_erpm_start, 10000.0f);
    }
    if (!apply_mc(id, s_mc_active[id])) {
        memcpy(s_mc_active[id], s_rollback_mc[id], sizeof(s_rollback_mc[id]));
        (void)apply_mc(id, s_rollback_mc[id]);
        return false;
    }
    /* Keep the accepted/generated image as the source of truth. Hardware-limit
       truncations above are already reflected in it; unrelated VESC fields are
       preserved exactly, so GET_MCCONF cannot invent a second shadow config. */
    if (store) {
        if (!conf_general_store_mc_wire_persistent(id, s_mc_active[id])) {
            memcpy(s_mc_active[id], s_rollback_mc[id], sizeof(s_rollback_mc[id]));
            (void)apply_mc(id, s_rollback_mc[id]);
            return false;
        }
    }
    return true;
}
// Parameter wire: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter len: panjang data dalam byte yang boleh diproses.
// Parameter store: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_config_set_app_wire: mengatur vesc config set app wire setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
bool vesc_config_set_app_wire(const uint8_t *wire, uint16_t len, bool store) {
    vesc_config_init_defaults();
    if (!wire || len != VESC6_APPCONF_WIRE_SIZE || !sig_ok(wire, VESC6_APPCONF_SIGNATURE))
        return false;
    if (memcmp(wire, s_app_active, VESC6_APPCONF_WIRE_SIZE) == 0) {
        if (!store) {
            return true;
        }
        const bool saved = conf_general_store_app_wire_persistent(s_app_active);
        return saved;
    }
    /* Changing throttle calibration/control type while either bridge is live can
       create an instantaneous command discontinuity. Require both local motors
       OFF, exactly as MCCONF writes already do for motor-critical parameters. */
    // Variabel ml: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel mr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *ml = motor_get(MOTOR_LEFT), *mr = motor_get(MOTOR_RIGHT);
    if ((ml && (ml->pwm_enabled || ml->detect.busy)) ||
       (mr && (mr->pwm_enabled || mr->detect.busy)))
       return false;
    memcpy(s_rollback_app, s_app_active, sizeof(s_rollback_app));
    memcpy(s_app_active, wire, len);
    if (!apply_app(s_app_active)) {
        memcpy(s_app_active, s_rollback_app, sizeof(s_rollback_app));
        (void)apply_app(s_rollback_app);
        return false;
    }
    if (store) {
        if (!conf_general_store_app_wire_persistent(s_app_active)) {
            memcpy(s_app_active, s_rollback_app, sizeof(s_rollback_app));
            (void)apply_app(s_rollback_app);
            return false;
        }
    }
    return true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi vesc_config_sync_motor_runtime: menjalankan operasi vesc config sync motor runtime sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void vesc_config_sync_motor_runtime(motor_id_t id) {
    vesc_config_init_defaults();
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *w = s_mc_active[id];

    /* IMPORTANT OWNERSHIP RULE
     * ------------------------
     * s_mc_active[] is the exact VESC-6.00 wire image and therefore the
     * source of truth for GET_MCCONF and flash persistence. Accepted writable
     * values are preflighted against the F103 envelope; defensive runtime
     * clamps must therefore never become a second hidden configuration. This
     * function is only called after a detection or an
     * explicit sensor-selection operation, so update only fields that those
     * operations genuinely own/change.
     *
     * In particular preserve current/input-current limits, VIN/battery
     * limits, duty/ERPM limits, SI wheel/battery data and every unsupported
     * VESC UI field byte-for-byte. */

    /* Detect-apply owns FOC motor parameters/current gains. */
    put_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KP, m->current_kp);
    put_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KI, m->current_ki);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_L, m->foc_motor_l);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_LD_LQ_DIFF, m->foc_motor_ld_lq_diff);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_R, m->foc_motor_r);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_FLUX_LINKAGE, m->foc_motor_flux_linkage);

    /* COMM_DETECT_APPLY_ALL_FOC can also update these sensorless thresholds. */
    put_auto_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM, m->foc_openloop_rpm);
    put_f16_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM_LOW, m->foc_openloop_rpm_low, 1000.0f);
    put_auto_at(w, VESC6_MC_OFF_FOC_SL_ERPM, m->foc_sl_erpm);

    /* Hall/encoder detect or explicit sensor selection owns sensor fields. */
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++)
        w[VESC6_MC_OFF_FOC_HALL_TABLE+k] = m->foc_hall_table[k];
    put_auto_at(w, VESC6_MC_OFF_FOC_HALL_INTERP_ERPM, m->foc_hall_interp_erpm);
    if (id == MOTOR_LEFT) {
        w[VESC6_MC_OFF_FOC_ENCODER_INVERTED] = m->encoder.inverted ? 1U : 0U;
        put_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_OFFSET,
                    (float)m->encoder.elec_offset_u16*360.0f/65536.0f);
        put_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_RATIO, m->encoder.electrical_ratio);
        put_u32_at(w, VESC6_MC_OFF_M_ENCODER_COUNTS, m->encoder.cpr);
        // Variabel enc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        bool enc = (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
                  m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ||
                  m->sensor_request_mode == SENSOR_MODE_ENCODER);
        w[VESC6_MC_OFF_FOC_SENSOR_MODE] = enc ? VESC_FOC_SENSOR_ENCODER : (uint8_t)m->foc_sensor_mode;
        w[VESC6_MC_OFF_M_SENSOR_PORT_MODE] = enc ? VESC_SENSOR_PORT_ABI : VESC_SENSOR_PORT_HALL;
    }
    else {
        w[VESC6_MC_OFF_FOC_SENSOR_MODE] = (uint8_t)m->foc_sensor_mode;
        w[VESC6_MC_OFF_M_SENSOR_PORT_MODE] = VESC_SENSOR_PORT_HALL;
    }
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi vesc_config_sync_detect_all_runtime: menjalankan deteksi vesc config sync detect all runtime dengan
// proteksi motor dan memvalidasi hasil sebelum parameter diterapkan.
void vesc_config_sync_detect_all_runtime(motor_id_t id) {
    vesc_config_init_defaults();
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (!m)
        return;
    /* Detect-All owns the current limits derived from max_power_loss and the
       requested input-current limits. Keep this separate from the generic
       sensor/RL sync so a Hall-only detect cannot rewrite unrelated limits. */
    // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t *w = s_mc_active[id];
    put_auto_at(w, VESC6_MC_OFF_L_CURRENT_MAX, m->current_max_a);
    put_auto_at(w, VESC6_MC_OFF_L_CURRENT_MIN, m->current_min_a);
    put_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MAX, m->input_current_max_a);
    put_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MIN, m->input_current_min_a);
    vesc_config_sync_motor_runtime(id);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter detect_all: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi commit_runtime_owned_fields: menyimpan commit runtime owned fields secara transaksional dengan
// pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
static bool commit_runtime_owned_fields(motor_id_t id, bool detect_all) {
    vesc_config_init_defaults();
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return false;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (!m || m->pwm_enabled || m->detect.busy)
        return false;
    memcpy(s_rollback_mc[id], s_mc_active[id], sizeof(s_rollback_mc[id]));
    if (detect_all)
        vesc_config_sync_detect_all_runtime(id);
    else vesc_config_sync_motor_runtime(id);
    /* Validate the exact candidate that will be exposed through GET_MCCONF. */
    if (!apply_mc(id, s_mc_active[id]) || !conf_general_store_mc_wire_persistent(id, s_mc_active[id])) {
        memcpy(s_mc_active[id], s_rollback_mc[id], sizeof(s_rollback_mc[id]));
        (void)apply_mc(id, s_rollback_mc[id]);
        return false;
    }
    return true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi vesc_config_commit_motor_runtime: menyimpan vesc config commit motor runtime secara transaksional
// dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
bool vesc_config_commit_motor_runtime(motor_id_t id) {
    return commit_runtime_owned_fields(id, false);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi vesc_config_commit_detect_all_runtime: menjalankan deteksi vesc config commit detect all runtime
// dengan proteksi motor dan memvalidasi hasil sebelum parameter diterapkan.
bool vesc_config_commit_detect_all_runtime(motor_id_t id) {
    return commit_runtime_owned_fields(id, true);
}

// Fungsi vesc_config_commit_detect_all_runtime_dual: menjalankan deteksi vesc config commit detect all runtime
// dual dengan proteksi motor dan memvalidasi hasil sebelum parameter diterapkan.
bool vesc_config_commit_detect_all_runtime_dual(void) {
    vesc_config_init_defaults();
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *l = motor_get(MOTOR_LEFT);
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *r = motor_get(MOTOR_RIGHT);
    if (!l || !r || l->pwm_enabled || r->pwm_enabled || l->detect.busy || r->detect.busy)
        return false;

    /* s_mc_active[] is still the last committed wire image while the blocking
       detection worker mutates MotorRuntime. Snapshot it once, synthesize both
       detected candidates, validate/apply both, then write one transactional
       flash record. This gives VESC Tool Detect-All true all-or-nothing
       semantics even though motor-2 is local rather than a physical CAN node. */
    memcpy(s_rollback_mc[MOTOR_LEFT], s_mc_active[MOTOR_LEFT], sizeof(s_rollback_mc[MOTOR_LEFT]));
    memcpy(s_rollback_mc[MOTOR_RIGHT], s_mc_active[MOTOR_RIGHT], sizeof(s_rollback_mc[MOTOR_RIGHT]));
    vesc_config_sync_detect_all_runtime(MOTOR_LEFT);
    vesc_config_sync_detect_all_runtime(MOTOR_RIGHT);

    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = apply_mc(MOTOR_LEFT, s_mc_active[MOTOR_LEFT]) &&
            apply_mc(MOTOR_RIGHT, s_mc_active[MOTOR_RIGHT]) &&
            conf_general_store_all();
    if (!ok) {
        memcpy(s_mc_active[MOTOR_LEFT], s_rollback_mc[MOTOR_LEFT], sizeof(s_rollback_mc[MOTOR_LEFT]));
        memcpy(s_mc_active[MOTOR_RIGHT], s_rollback_mc[MOTOR_RIGHT], sizeof(s_rollback_mc[MOTOR_RIGHT]));
        (void)apply_mc(MOTOR_LEFT, s_rollback_mc[MOTOR_LEFT]);
        (void)apply_mc(MOTOR_RIGHT, s_rollback_mc[MOTOR_RIGHT]);
    }
    return ok;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi vesc_config_reapply_active_mc: menjalankan operasi vesc config reapply active mc sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
bool vesc_config_reapply_active_mc(motor_id_t id) {
    vesc_config_init_defaults();
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return false;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (!m)
        return false;
    motor_stop(m);
    return apply_mc(id, s_mc_active[id]);
}

// Parameter l: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_config_export_wire: menjalankan operasi vesc config export wire sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void vesc_config_export_wire(uint8_t l[VESC6_MCCONF_WIRE_SIZE], uint8_t r[VESC6_MCCONF_WIRE_SIZE], uint8_t a[VESC6_APPCONF_WIRE_SIZE]) {
    vesc_config_init_defaults();
    memcpy(l, s_mc_active[0], VESC6_MCCONF_WIRE_SIZE);
    memcpy(r, s_mc_active[1], VESC6_MCCONF_WIRE_SIZE);
    memcpy(a, s_app_active, VESC6_APPCONF_WIRE_SIZE);
}
// Parameter l: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi vesc_config_import_wire: menjalankan operasi vesc config import wire sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool vesc_config_import_wire(const uint8_t l[VESC6_MCCONF_WIRE_SIZE], const uint8_t r[VESC6_MCCONF_WIRE_SIZE], const uint8_t a[VESC6_APPCONF_WIRE_SIZE]) {
    vesc_config_init_defaults();
    if (!l || !r || !a || !sig_ok(l, VESC6_MCCONF_SIGNATURE) || !sig_ok(r, VESC6_MCCONF_SIGNATURE) || !sig_ok(a, VESC6_APPCONF_SIGNATURE))
        return false;
    /* VESC 6.00 MCCONF is a complete generated wire image. UI-only fields are
       allowed to survive flash just as they survive SET_MCCONF; apply_mc()
       validates/bounds the subset that this F103 backend actually executes.
       APPCONF juga dipertahankan sebagai image lengkap; apply_app() hanya
       mengeksekusi subset yang memang didukung board ini. */
    memcpy(s_rollback_mc[MOTOR_LEFT], s_mc_active[MOTOR_LEFT], sizeof(s_rollback_mc[MOTOR_LEFT]));
    memcpy(s_rollback_mc[MOTOR_RIGHT], s_mc_active[MOTOR_RIGHT], sizeof(s_rollback_mc[MOTOR_RIGHT]));
    memcpy(s_rollback_app, s_app_active, sizeof(s_rollback_app));
    memcpy(s_mc_active[MOTOR_LEFT], l, VESC6_MCCONF_WIRE_SIZE);
    memcpy(s_mc_active[MOTOR_RIGHT], r, VESC6_MCCONF_WIRE_SIZE);
    memcpy(s_app_active, a, VESC6_APPCONF_WIRE_SIZE);
    normalize_vesc6_hfi_ui_fields(s_mc_active[MOTOR_LEFT]);
    normalize_vesc6_hfi_ui_fields(s_mc_active[MOTOR_RIGHT]);
    /* Batch-2 used 1000/1000 as placeholder observer defaults before the
       VESC gain_slow semantics were implemented. Preserve every other MCCONF
       field on upgrade, but migrate exactly that legacy pair to the Batch-3
       VESC-style defaults instead of rejecting the whole flash record. */
    // Variabel mi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (unsigned mi = 0U; mi < 2U; mi++) {
        // Variabel legacy_gain: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float legacy_gain = get_auto_at(s_mc_active[mi], 177U);
        // Variabel legacy_slow: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float legacy_slow = get_auto_at(s_mc_active[mi], 181U);
        if (fabsf(legacy_gain - 1000.0f) < 0.01f &&
            fabsf(legacy_slow - 1000.0f) < 0.01f) {
            put_auto_at(s_mc_active[mi], 177U, MCCONF_FOC_OBSERVER_GAIN_DEFAULT);
            put_auto_at(s_mc_active[mi], 181U, MCCONF_FOC_OBSERVER_GAIN_SLOW_DEFAULT);
        }
    }
    if (!apply_mc(MOTOR_LEFT, s_mc_active[MOTOR_LEFT]) ||
       !apply_mc(MOTOR_RIGHT, s_mc_active[MOTOR_RIGHT]) ||
       !apply_app(s_app_active)) {
        memcpy(s_mc_active[MOTOR_LEFT], s_rollback_mc[MOTOR_LEFT], sizeof(s_rollback_mc[MOTOR_LEFT]));
        memcpy(s_mc_active[MOTOR_RIGHT], s_rollback_mc[MOTOR_RIGHT], sizeof(s_rollback_mc[MOTOR_RIGHT]));
        memcpy(s_app_active, s_rollback_app, sizeof(s_rollback_app));
        (void)apply_mc(MOTOR_LEFT, s_rollback_mc[MOTOR_LEFT]);
        (void)apply_mc(MOTOR_RIGHT, s_rollback_mc[MOTOR_RIGHT]);
        (void)apply_app(s_rollback_app);
        return false;
    }
    /* Persistent wire image remains the source of truth. Accepted writable
       values already passed the same runtime envelope validation used by SET. */
    return true;
}

// Fungsi vesc_config_apply_defaults: menjalankan operasi vesc config apply defaults sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool vesc_config_apply_defaults(void) {
    vesc_config_init_defaults();
    if (!s_layout_ok) {
        return false;
    }
    return vesc_config_import_wire(s_mc_factory[MOTOR_LEFT],
            s_mc_factory[MOTOR_RIGHT], s_app_factory);
}

/* ==================== Canonical confgenerator front-end ====================
 * The 481/493-byte VESC-6 image remains the source of truth. These functions
 * provide upstream-style typed access without duplicating persistent state. */
// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcconf_decode_wire: mengurai mcconf decode wire dari buffer komunikasi menjadi data internal setelah
// format dan batas input diperiksa.
static void mcconf_decode_wire(const uint8_t *w, mc_configuration *c) {
    memset(c, 0, sizeof(*c));
    c->pwm_mode = (mc_pwm_mode)w[VESC6_MC_OFF_PWM_MODE];
    c->comm_mode = (mc_comm_mode)w[VESC6_MC_OFF_COMM_MODE];
    c->motor_type = (mc_motor_type)w[VESC6_MC_OFF_MOTOR_TYPE];
    c->sensor_mode = (mc_sensor_mode)w[VESC6_MC_OFF_SENSOR_MODE];
    c->l_current_max = get_auto_at(w, VESC6_MC_OFF_L_CURRENT_MAX);
    c->l_current_min = get_auto_at(w, VESC6_MC_OFF_L_CURRENT_MIN);
    c->l_in_current_max = get_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MAX);
    c->l_in_current_min = get_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MIN);
    c->l_abs_current_max = get_auto_at(w, VESC6_MC_OFF_L_ABS_CURRENT_MAX);
    c->l_min_erpm = get_auto_at(w, VESC6_MC_OFF_L_MIN_ERPM);
    c->l_max_erpm = get_auto_at(w, VESC6_MC_OFF_L_MAX_ERPM);
    c->l_erpm_start = get_f16_at(w, VESC6_MC_OFF_L_ERPM_START, 10000.0f);
    c->l_min_vin = get_auto_at(w, VESC6_MC_OFF_L_MIN_VIN);
    c->l_max_vin = get_auto_at(w, VESC6_MC_OFF_L_MAX_VIN);
    c->l_battery_cut_start = get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_START);
    c->l_battery_cut_end = get_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_END);
    c->l_slow_abs_current = w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT] != 0U;
    c->l_temp_fet_start = get_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_START, 10.0f);
    c->l_temp_fet_end = get_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_END, 10.0f);
    c->l_temp_motor_start = get_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_START, 10.0f);
    c->l_temp_motor_end = get_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_END, 10.0f);
    c->l_temp_accel_dec = get_f16_at(w, VESC6_MC_OFF_L_TEMP_ACCEL_DEC, 10000.0f);
    c->l_additional_faults = MCCONF_L_ADDITIONAL_FAULTS_DEFAULT;
    c->foc_short_ls_on_zero_duty = MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT;
    c->l_min_duty = get_f16_at(w, VESC6_MC_OFF_L_MIN_DUTY, 10000.0f);
    c->l_max_duty = get_f16_at(w, VESC6_MC_OFF_L_MAX_DUTY, 10000.0f);
    c->l_watt_max = get_auto_at(w, VESC6_MC_OFF_L_WATT_MAX);
    c->l_watt_min = get_auto_at(w, VESC6_MC_OFF_L_WATT_MIN);
    c->l_current_max_scale = get_f16_at(w, VESC6_MC_OFF_L_CURRENT_MAX_SCALE, 10000.0f);
    c->l_current_min_scale = get_f16_at(w, VESC6_MC_OFF_L_CURRENT_MIN_SCALE, 10000.0f);
    c->l_duty_start = get_f16_at(w, VESC6_MC_OFF_L_DUTY_START, 10000.0f);
    /* Later-VESC/runtime-only fields are explicit defaults, never fabricated
       from unrelated VESC6 bytes. */
    c->l_in_current_map_start = MCCONF_L_IN_CURRENT_MAP_START_DEFAULT;
    c->l_in_current_map_filter = MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT;
    c->l_battery_regen_cut_start = c->l_max_vin-MCCONF_L_BATTERY_REGEN_CUT_START_MARGIN_V;
    c->l_battery_regen_cut_end = c->l_max_vin-MCCONF_L_BATTERY_REGEN_CUT_END_MARGIN_V;
    c->lo_current_max = c->l_current_max;
    c->lo_current_min = c->l_current_min;
    c->lo_in_current_max = c->l_in_current_max;
    c->lo_in_current_min = c->l_in_current_min;

    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++)
        c->hall_table[k] = (int8_t)w[115U+k];
    c->hall_sl_erpm = get_auto_at(w, VESC6_MC_OFF_HALL_SL_ERPM);
    c->foc_current_kp = get_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KP);
    c->foc_current_ki = get_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KI);
    c->foc_f_zv = get_auto_at(w, VESC6_MC_OFF_FOC_F_ZV);
    c->foc_dt_us = get_auto_at(w, VESC6_MC_OFF_FOC_DT_US);
    c->foc_encoder_inverted = w[VESC6_MC_OFF_FOC_ENCODER_INVERTED] != 0U;
    c->foc_encoder_offset = get_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_OFFSET);
    c->foc_encoder_ratio = get_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_RATIO);
    c->foc_sensor_mode = (mc_foc_sensor_mode)w[VESC6_MC_OFF_FOC_SENSOR_MODE];
    if (c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER && w[VESC6_MC_OFF_M_SENSOR_PORT_MODE] == VESC_SENSOR_PORT_ABI)
        c->foc_sensor_mode = FOC_SENSOR_MODE_ENCODER_AB;
    c->foc_pll_kp = get_auto_at(w, VESC6_MC_OFF_FOC_PLL_KP);
    c->foc_pll_ki = get_auto_at(w, VESC6_MC_OFF_FOC_PLL_KI);
    c->foc_motor_l = get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_L);
    c->foc_motor_ld_lq_diff = get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_LD_LQ_DIFF);
    c->foc_motor_r = get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_R);
    c->foc_motor_flux_linkage = get_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_FLUX_LINKAGE);
    c->foc_observer_gain = get_auto_at(w, VESC6_MC_OFF_FOC_OBSERVER_GAIN);
    c->foc_observer_gain_slow = get_auto_at(w, VESC6_MC_OFF_FOC_OBSERVER_GAIN_SLOW);
    c->foc_observer_offset = get_f16_at(w, VESC6_MC_OFF_FOC_OBSERVER_OFFSET, 1000.0f);
    c->foc_duty_dowmramp_kp = get_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP);
    c->foc_duty_dowmramp_ki = get_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI);
    c->foc_start_curr_dec = get_f16_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC, 10000.0f);
    c->foc_start_curr_dec_rpm = get_auto_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC_RPM);
    c->foc_openloop_rpm = get_auto_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM);
    c->foc_openloop_rpm_low = get_f16_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM_LOW, 1000.0f);
    c->foc_sl_openloop_hyst = get_f16_at(w, 211U, 100.0f);
    c->foc_sl_openloop_time_lock = get_f16_at(w, 213U, 100.0f);
    c->foc_sl_openloop_time_ramp = get_f16_at(w, 215U, 100.0f);
    c->foc_sl_openloop_time = get_f16_at(w, 217U, 100.0f);
    c->foc_sl_openloop_boost_q = get_f16_at(w, 219U, 100.0f);
    c->foc_sl_openloop_max_q = get_f16_at(w, 221U, 100.0f);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++)
        c->foc_hall_table[k] = w[VESC6_MC_OFF_FOC_HALL_TABLE+k];
    c->foc_hall_interp_erpm = get_auto_at(w, VESC6_MC_OFF_FOC_HALL_INTERP_ERPM);
    c->foc_sl_erpm = get_auto_at(w, VESC6_MC_OFF_FOC_SL_ERPM);
    c->foc_sl_erpm_start = MCCONF_FOC_SL_ERPM_START_DEFAULT;
    c->foc_sample_v0_v7 = w[VESC6_MC_OFF_FOC_SAMPLE_V0_V7] != 0U;
    c->foc_sample_high_current = w[VESC6_MC_OFF_FOC_SAMPLE_HIGH_CURRENT] != 0U;
    c->foc_speed_source = (FOC_SPEED_SRC)w[VESC6_MC_OFF_FOC_SPEED_SOURCE];
    c->foc_sat_comp_mode = (SAT_COMP_MODE)w[VESC6_MC_OFF_FOC_SAT_COMP_MODE];
    c->foc_sat_comp = get_f16_at(w, VESC6_MC_OFF_FOC_SAT_COMP, 1000.0f);
    c->foc_current_filter_const = get_f16_at(w, VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST, 10000.0f);
    c->foc_cc_decoupling = (mc_foc_cc_decoupling_mode)w[VESC6_MC_OFF_FOC_CC_DECOUPLING];
    c->foc_observer_type = (mc_foc_observer_type)w[VESC6_MC_OFF_FOC_OBSERVER_TYPE];
    c->foc_mtpa_mode = (MTPA_MODE)w[VESC6_MC_OFF_FOC_MTPA_MODE];
    c->foc_fw_current_max = get_auto_at(w, VESC6_MC_OFF_FOC_FW_CURRENT_MAX);
    c->foc_fw_duty_start = get_f16_at(w, VESC6_MC_OFF_FOC_FW_DUTY_START, 10000.0f);
    c->foc_fw_ramp_time = get_f16_at(w, VESC6_MC_OFF_FOC_FW_RAMP_TIME, 1000.0f);
    c->foc_fw_q_current_factor = get_f16_at(w, VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR, 10000.0f);
    c->foc_fw_backoff = MCCONF_FOC_FW_BACKOFF_DEFAULT;
    c->foc_mag_vd_max = MCCONF_FOC_MAG_VD_MAX_DEFAULT;
    c->foc_overmod_factor = MCCONF_FOC_OVERMOD_FACTOR_DEFAULT;
    c->foc_temp_comp = MCCONF_FOC_TEMP_COMP_DEFAULT;
    c->foc_temp_comp_base_temp = MCCONF_FOC_TEMP_COMP_BASE_TEMP_DEFAULT;
    c->foc_offsets_cal_mode = MCCONF_FOC_OFFSETS_CAL_MODE_DEFAULT;

    c->s_pid_kp = get_auto_at(w, VESC6_MC_OFF_S_PID_KP);
    c->s_pid_ki = get_auto_at(w, VESC6_MC_OFF_S_PID_KI);
    c->s_pid_kd = get_auto_at(w, VESC6_MC_OFF_S_PID_KD);
    c->s_pid_kd_filter = get_f16_at(w, VESC6_MC_OFF_S_PID_KD_FILTER, 10000.0f);
    c->s_pid_min_erpm = get_auto_at(w, VESC6_MC_OFF_S_PID_MIN_ERPM);
    c->s_pid_allow_braking = w[VESC6_MC_OFF_S_PID_ALLOW_BRAKING] != 0U;
    c->s_pid_ramp_erpms_s = get_auto_at(w, VESC6_MC_OFF_S_PID_RAMP_ERPMS_S);
    c->s_pid_speed_source=S_PID_SPEED_SRC_PLL; /* not present in VESC6 wire */
    c->p_pid_kp = get_auto_at(w, VESC6_MC_OFF_P_PID_KP);
    c->p_pid_ki = get_auto_at(w, VESC6_MC_OFF_P_PID_KI);
    c->p_pid_kd = get_auto_at(w, VESC6_MC_OFF_P_PID_KD);
    c->p_pid_kd_proc = get_auto_at(w, VESC6_MC_OFF_P_PID_KD_PROC);
    c->p_pid_kd_filter = get_f16_at(w, VESC6_MC_OFF_P_PID_KD_FILTER, 10000.0f);
    c->p_pid_ang_div = get_auto_at(w, VESC6_MC_OFF_P_PID_ANG_DIV);
    c->p_pid_gain_dec_angle = get_f16_at(w, VESC6_MC_OFF_P_PID_GAIN_DEC_ANGLE, 10.0f);
    c->p_pid_offset = get_auto_at(w, VESC6_MC_OFF_P_PID_OFFSET);
    c->cc_startup_boost_duty = get_f16_at(w, VESC6_MC_OFF_CC_STARTUP_BOOST_DUTY, 10000.0f);
    c->cc_min_current = get_auto_at(w, VESC6_MC_OFF_CC_MIN_CURRENT);
    c->cc_gain = get_auto_at(w, VESC6_MC_OFF_CC_GAIN);
    c->cc_ramp_step_max = get_f16_at(w, VESC6_MC_OFF_CC_RAMP_STEP_MAX, 10000.0f);
    c->m_encoder_counts = get_u32_at(w, VESC6_MC_OFF_M_ENCODER_COUNTS);
    c->m_sensor_port_mode = (sensor_port_mode)w[VESC6_MC_OFF_M_SENSOR_PORT_MODE];
    c->m_invert_direction = w[VESC6_MC_OFF_M_INVERT_DIRECTION] != 0U;
    c->si_motor_poles = w[VESC6_MC_OFF_SI_MOTOR_POLES];
    c->si_gear_ratio = get_auto_at(w, VESC6_MC_OFF_SI_GEAR_RATIO);
    c->si_wheel_diameter = get_auto_at(w, VESC6_MC_OFF_SI_WHEEL_DIAMETER);
    c->si_battery_type = w[VESC6_MC_OFF_SI_BATTERY_TYPE];
    c->si_battery_cells = w[VESC6_MC_OFF_SI_BATTERY_CELLS];
    c->si_battery_ah = get_auto_at(w, VESC6_MC_OFF_SI_BATTERY_AH);
    c->si_motor_nl_current = get_auto_at(w, VESC6_MC_OFF_SI_MOTOR_NL_CURRENT);
}

// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcconf_patch_wire: menjalankan operasi mcconf patch wire sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static void mcconf_patch_wire(uint8_t *w, const mc_configuration *c) {
    w[VESC6_MC_OFF_PWM_MODE] = (uint8_t)c->pwm_mode;
    w[VESC6_MC_OFF_COMM_MODE] = (uint8_t)c->comm_mode;
    w[VESC6_MC_OFF_MOTOR_TYPE] = (uint8_t)c->motor_type;
    w[VESC6_MC_OFF_SENSOR_MODE] = (uint8_t)c->sensor_mode;
    put_auto_at(w, VESC6_MC_OFF_L_CURRENT_MAX, c->l_current_max);
    put_auto_at(w, VESC6_MC_OFF_L_CURRENT_MIN, c->l_current_min);
    put_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MAX, c->l_in_current_max);
    put_auto_at(w, VESC6_MC_OFF_L_IN_CURRENT_MIN, c->l_in_current_min);
    put_auto_at(w, VESC6_MC_OFF_L_ABS_CURRENT_MAX, c->l_abs_current_max);
    put_auto_at(w, VESC6_MC_OFF_L_MIN_ERPM, c->l_min_erpm);
    put_auto_at(w, VESC6_MC_OFF_L_MAX_ERPM, c->l_max_erpm);
    put_f16_at(w, VESC6_MC_OFF_L_ERPM_START, c->l_erpm_start, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_L_MIN_VIN, c->l_min_vin);
    put_auto_at(w, VESC6_MC_OFF_L_MAX_VIN, c->l_max_vin);
    put_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_START, c->l_battery_cut_start);
    put_auto_at(w, VESC6_MC_OFF_L_BAT_CUT_END, c->l_battery_cut_end);
    w[VESC6_MC_OFF_L_SLOW_ABS_CURRENT] = c->l_slow_abs_current ? 1U : 0U;
    put_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_START, c->l_temp_fet_start, 10.0f);
    put_f16_at(w, VESC6_MC_OFF_L_TEMP_FET_END, c->l_temp_fet_end, 10.0f);
    put_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_START, c->l_temp_motor_start, 10.0f);
    put_f16_at(w, VESC6_MC_OFF_L_TEMP_MOTOR_END, c->l_temp_motor_end, 10.0f);
    put_f16_at(w, VESC6_MC_OFF_L_TEMP_ACCEL_DEC, c->l_temp_accel_dec, 10000.0f);
    put_f16_at(w, VESC6_MC_OFF_L_MIN_DUTY, c->l_min_duty, 10000.0f);
    put_f16_at(w, VESC6_MC_OFF_L_MAX_DUTY, c->l_max_duty, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_L_WATT_MAX, c->l_watt_max);
    put_auto_at(w, VESC6_MC_OFF_L_WATT_MIN, c->l_watt_min);
    put_f16_at(w, VESC6_MC_OFF_L_CURRENT_MAX_SCALE, c->l_current_max_scale, 10000.0f);
    put_f16_at(w, VESC6_MC_OFF_L_CURRENT_MIN_SCALE, c->l_current_min_scale, 10000.0f);
    put_f16_at(w, VESC6_MC_OFF_L_DUTY_START, c->l_duty_start, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KP, c->foc_current_kp);
    put_auto_at(w, VESC6_MC_OFF_FOC_CURRENT_KI, c->foc_current_ki);
    put_auto_at(w, VESC6_MC_OFF_FOC_F_ZV, c->foc_f_zv);
    put_auto_at(w, VESC6_MC_OFF_FOC_DT_US, c->foc_dt_us);
    w[VESC6_MC_OFF_FOC_ENCODER_INVERTED] = c->foc_encoder_inverted ? 1U : 0U;
    put_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_OFFSET, c->foc_encoder_offset);
    put_auto_at(w, VESC6_MC_OFF_FOC_ENCODER_RATIO, c->foc_encoder_ratio);
    w[VESC6_MC_OFF_FOC_SENSOR_MODE] = (uint8_t)(c->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ? FOC_SENSOR_MODE_ENCODER : c->foc_sensor_mode);
    put_auto_at(w, VESC6_MC_OFF_FOC_PLL_KP, c->foc_pll_kp);
    put_auto_at(w, VESC6_MC_OFF_FOC_PLL_KI, c->foc_pll_ki);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_L, c->foc_motor_l);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_LD_LQ_DIFF, c->foc_motor_ld_lq_diff);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_R, c->foc_motor_r);
    put_auto_at(w, VESC6_MC_OFF_FOC_MOTOR_FLUX_LINKAGE, c->foc_motor_flux_linkage);
    put_auto_at(w, VESC6_MC_OFF_FOC_OBSERVER_GAIN, c->foc_observer_gain);
    put_auto_at(w, VESC6_MC_OFF_FOC_OBSERVER_GAIN_SLOW, c->foc_observer_gain_slow);
    put_f16_at(w, VESC6_MC_OFF_FOC_OBSERVER_OFFSET, c->foc_observer_offset, 1000.0f);
    put_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KP, c->foc_duty_dowmramp_kp);
    put_auto_at(w, VESC6_MC_OFF_FOC_DUTY_DOWNRAMP_KI, c->foc_duty_dowmramp_ki);
    put_f16_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC, c->foc_start_curr_dec, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_FOC_START_CURR_DEC_RPM, c->foc_start_curr_dec_rpm);
    put_auto_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM, c->foc_openloop_rpm);
    put_f16_at(w, VESC6_MC_OFF_FOC_OPENLOOP_RPM_LOW, c->foc_openloop_rpm_low, 1000.0f);
    /* Keep typed serialization faithful even for fields whose runtime backend
       is intentionally disabled. SET_MCCONF ownership validation will reject
       such changes instead of silently accepting them. */
    put_f16_at(w, 211U, c->foc_sl_openloop_hyst, 100.0f);
    put_f16_at(w, 213U, c->foc_sl_openloop_time_lock, 100.0f);
    put_f16_at(w, 215U, c->foc_sl_openloop_time_ramp, 100.0f);
    put_f16_at(w, 217U, c->foc_sl_openloop_time, 100.0f);
    put_f16_at(w, 219U, c->foc_sl_openloop_boost_q, 100.0f);
    put_f16_at(w, 221U, c->foc_sl_openloop_max_q, 100.0f);
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0; k < 8; k++)
        w[VESC6_MC_OFF_FOC_HALL_TABLE+k] = c->foc_hall_table[k];
    put_auto_at(w, VESC6_MC_OFF_FOC_HALL_INTERP_ERPM, c->foc_hall_interp_erpm);
    put_auto_at(w, VESC6_MC_OFF_FOC_SL_ERPM, c->foc_sl_erpm);
    w[VESC6_MC_OFF_FOC_SAMPLE_V0_V7] = c->foc_sample_v0_v7 ? 1U : 0U;
    w[VESC6_MC_OFF_FOC_SAMPLE_HIGH_CURRENT] = c->foc_sample_high_current ? 1U : 0U;
    w[VESC6_MC_OFF_FOC_SAT_COMP_MODE] = (uint8_t)c->foc_sat_comp_mode;
    put_f16_at(w, VESC6_MC_OFF_FOC_SAT_COMP, c->foc_sat_comp, 1000.0f);
    put_f16_at(w, VESC6_MC_OFF_FOC_CURRENT_FILTER_CONST, c->foc_current_filter_const, 10000.0f);
    w[VESC6_MC_OFF_FOC_CC_DECOUPLING] = (uint8_t)c->foc_cc_decoupling;
    w[VESC6_MC_OFF_FOC_OBSERVER_TYPE] = (uint8_t)c->foc_observer_type;
    w[VESC6_MC_OFF_FOC_MTPA_MODE] = (uint8_t)c->foc_mtpa_mode;
    put_auto_at(w, VESC6_MC_OFF_FOC_FW_CURRENT_MAX, c->foc_fw_current_max);
    put_f16_at(w, VESC6_MC_OFF_FOC_FW_DUTY_START, c->foc_fw_duty_start, 10000.0f);
    put_f16_at(w, VESC6_MC_OFF_FOC_FW_RAMP_TIME, c->foc_fw_ramp_time, 1000.0f);
    put_f16_at(w, VESC6_MC_OFF_FOC_FW_Q_CURRENT_FACTOR, c->foc_fw_q_current_factor, 10000.0f);
    w[VESC6_MC_OFF_FOC_SPEED_SOURCE] = (uint8_t)c->foc_speed_source;
    put_auto_at(w, VESC6_MC_OFF_S_PID_KP, c->s_pid_kp);
    put_auto_at(w, VESC6_MC_OFF_S_PID_KI, c->s_pid_ki);
    put_auto_at(w, VESC6_MC_OFF_S_PID_KD, c->s_pid_kd);
    put_f16_at(w, VESC6_MC_OFF_S_PID_KD_FILTER, c->s_pid_kd_filter, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_S_PID_MIN_ERPM, c->s_pid_min_erpm);
    w[VESC6_MC_OFF_S_PID_ALLOW_BRAKING] = c->s_pid_allow_braking ? 1U : 0U;
    put_auto_at(w, VESC6_MC_OFF_S_PID_RAMP_ERPMS_S, c->s_pid_ramp_erpms_s);
    put_auto_at(w, VESC6_MC_OFF_P_PID_KP, c->p_pid_kp);
    put_auto_at(w, VESC6_MC_OFF_P_PID_KI, c->p_pid_ki);
    put_auto_at(w, VESC6_MC_OFF_P_PID_KD, c->p_pid_kd);
    put_auto_at(w, VESC6_MC_OFF_P_PID_KD_PROC, c->p_pid_kd_proc);
    put_f16_at(w, VESC6_MC_OFF_P_PID_KD_FILTER, c->p_pid_kd_filter, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_P_PID_ANG_DIV, c->p_pid_ang_div);
    put_f16_at(w, VESC6_MC_OFF_P_PID_GAIN_DEC_ANGLE, c->p_pid_gain_dec_angle, 10.0f);
    put_auto_at(w, VESC6_MC_OFF_P_PID_OFFSET, c->p_pid_offset);
    put_f16_at(w, VESC6_MC_OFF_CC_STARTUP_BOOST_DUTY, c->cc_startup_boost_duty, 10000.0f);
    put_auto_at(w, VESC6_MC_OFF_CC_MIN_CURRENT, c->cc_min_current);
    put_auto_at(w, VESC6_MC_OFF_CC_GAIN, c->cc_gain);
    put_f16_at(w, VESC6_MC_OFF_CC_RAMP_STEP_MAX, c->cc_ramp_step_max, 10000.0f);
    put_u32_at(w, VESC6_MC_OFF_M_ENCODER_COUNTS, c->m_encoder_counts);
    w[VESC6_MC_OFF_M_SENSOR_PORT_MODE] = (uint8_t)c->m_sensor_port_mode;
    w[VESC6_MC_OFF_M_INVERT_DIRECTION] = c->m_invert_direction ? 1U : 0U;
    w[VESC6_MC_OFF_SI_MOTOR_POLES] = c->si_motor_poles;
    put_auto_at(w, VESC6_MC_OFF_SI_GEAR_RATIO, c->si_gear_ratio);
    put_auto_at(w, VESC6_MC_OFF_SI_WHEEL_DIAMETER, c->si_wheel_diameter);
    w[VESC6_MC_OFF_SI_BATTERY_TYPE] = c->si_battery_type;
    w[VESC6_MC_OFF_SI_BATTERY_CELLS] = c->si_battery_cells;
    put_auto_at(w, VESC6_MC_OFF_SI_BATTERY_AH, c->si_battery_ah);
    put_auto_at(w, VESC6_MC_OFF_SI_MOTOR_NL_CURRENT, c->si_motor_nl_current);
}

// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi confgenerator_deserialize_mcconf: mengurai confgenerator deserialize mcconf dari buffer komunikasi
// menjadi data internal setelah format dan batas input diperiksa.
bool confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf) {
    if (!buffer || !conf || !sig_ok(buffer, VESC6_MCCONF_SIGNATURE))
        return false;
    mcconf_decode_wire(buffer, conf);
    return true;
}
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi confgenerator_serialize_mcconf_motor: menyusun confgenerator serialize mcconf motor ke buffer/wire
// format dengan urutan field, skala, dan batas data yang konsisten.
int32_t confgenerator_serialize_mcconf_motor(uint8_t *buffer, const mc_configuration *conf, motor_id_t id) {
    if (!buffer || !conf || (id != MOTOR_LEFT && id != MOTOR_RIGHT))
        return -1;
    /* VESC6 has no s_pid_speed_source field. Refuse to persist a runtime-only
       FAST/FASTER selection instead of silently serializing it as PLL. */
    if (conf->s_pid_speed_source != S_PID_SPEED_SRC_PLL)
        return -1;
    if (fabsf(conf->l_in_current_map_start-MCCONF_L_IN_CURRENT_MAP_START_DEFAULT) > 1.0e-6f ||
       fabsf(conf->l_in_current_map_filter-MCCONF_L_IN_CURRENT_MAP_FILTER_DEFAULT) > 1.0e-6f)
       return -1;
    if (conf->l_additional_faults != MCCONF_L_ADDITIONAL_FAULTS_DEFAULT)
        return -1;
    if (conf->foc_short_ls_on_zero_duty != MCCONF_FOC_SHORT_LS_ON_ZERO_DUTY_DEFAULT)
        return -1;
    vesc_config_init_defaults();
    memcpy(buffer, s_mc_active[id], VESC6_MCCONF_WIRE_SIZE);
    mcconf_patch_wire(buffer, conf);
    return (int32_t)VESC6_MCCONF_WIRE_SIZE;
}
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi confgenerator_serialize_mcconf: menyusun confgenerator serialize mcconf ke buffer/wire format dengan
// urutan field, skala, dan batas data yang konsisten.
int32_t confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf) {
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    motor_id_t id = mc_interface_get_motor_thread() == 2 ? MOTOR_RIGHT : MOTOR_LEFT;
    return confgenerator_serialize_mcconf_motor(buffer, conf, id);
}
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi confgenerator_set_defaults_mcconf: mengatur confgenerator set defaults mcconf setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void confgenerator_set_defaults_mcconf(mc_configuration *conf) {
    if (!conf)
        return;
    vesc_config_init_defaults();
    mcconf_decode_wire(s_mc_factory[MOTOR_LEFT], conf);
}

// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi confgenerator_deserialize_appconf: mengurai confgenerator deserialize appconf dari buffer komunikasi
// menjadi data internal setelah format dan batas input diperiksa.
bool confgenerator_deserialize_appconf(const uint8_t *buffer, app_configuration *conf) {
    if (!buffer || !conf || !sig_ok(buffer, VESC6_APPCONF_SIGNATURE))
        return false;
    memset(conf, 0, sizeof(*conf));
    conf->controller_id = buffer[VESC6_APP_OFF_CONTROLLER_ID];
    conf->timeout_msec = get_u32_at(buffer, VESC6_APP_OFF_TIMEOUT_MSEC);
    conf->timeout_brake_current = get_auto_at(buffer, VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT);
    conf->permanent_uart_enabled = true;
    conf->app_to_use = (app_use)buffer[VESC6_APP_OFF_APP_TO_USE];
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    adc_config *a = &conf->app_adc_conf;
    a->ctrl_type = (adc_control_type)buffer[VESC6_APP_OFF_ADC_CTRL_TYPE];
    a->hyst = get_auto_at(buffer, VESC6_APP_OFF_ADC_HYST);
    a->voltage_start = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_START, 1000.0f);
    a->voltage_end = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_END, 1000.0f);
    a->voltage_min = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_MIN, 1000.0f);
    a->voltage_max = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_MAX, 1000.0f);
    a->voltage_center = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_CENTER, 1000.0f);
    a->voltage2_start = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE2_START, 1000.0f);
    a->voltage2_end = get_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE2_END, 1000.0f);
    a->use_filter = buffer[VESC6_APP_OFF_ADC_USE_FILTER] != 0U;
    a->safe_start = (SAFE_START_MODE)buffer[VESC6_APP_OFF_ADC_SAFE_START];
    a->buttons = buffer[VESC6_APP_OFF_ADC_BUTTONS];
    a->voltage_inverted = buffer[VESC6_APP_OFF_ADC_VOLTAGE_INVERTED] != 0U;
    a->voltage2_inverted = buffer[VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED] != 0U;
    a->throttle_exp = get_auto_at(buffer, VESC6_APP_OFF_ADC_THROTTLE_EXP);
    a->throttle_exp_brake = get_auto_at(buffer, VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE);
    a->throttle_exp_mode = (thr_exp_mode)buffer[VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE];
    a->ramp_time_pos = get_auto_at(buffer, VESC6_APP_OFF_ADC_RAMP_TIME_POS);
    a->ramp_time_neg = get_auto_at(buffer, VESC6_APP_OFF_ADC_RAMP_TIME_NEG);
    a->multi_esc = buffer[VESC6_APP_OFF_ADC_MULTI_ESC] != 0U;
    a->tc = buffer[VESC6_APP_OFF_ADC_TC] != 0U;
    a->tc_max_diff = get_auto_at(buffer, VESC6_APP_OFF_ADC_TC_MAX_DIFF);
    a->update_rate_hz = get_u16_at(buffer, VESC6_APP_OFF_ADC_UPDATE_RATE_HZ);
    conf->app_uart_baudrate = get_u32_at(buffer, VESC6_APP_OFF_UART_BAUD);
    return true;
}
// Parameter buffer: buffer sementara untuk serialisasi, komunikasi, atau pengolahan data.
// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi confgenerator_serialize_appconf: menyusun confgenerator serialize appconf ke buffer/wire format dengan
// urutan field, skala, dan batas data yang konsisten.
int32_t confgenerator_serialize_appconf(uint8_t *buffer, const app_configuration *conf) {
    if (!buffer || !conf)
        return -1;
    vesc_config_init_defaults();
    memcpy(buffer, s_app_active, VESC6_APPCONF_WIRE_SIZE);
    buffer[VESC6_APP_OFF_CONTROLLER_ID] = conf->controller_id;
    put_u32_at(buffer, VESC6_APP_OFF_TIMEOUT_MSEC, conf->timeout_msec);
    put_auto_at(buffer, VESC6_APP_OFF_TIMEOUT_BRAKE_CURRENT, conf->timeout_brake_current);
    buffer[VESC6_APP_OFF_APP_TO_USE] = (uint8_t)conf->app_to_use;
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const adc_config *a = &conf->app_adc_conf;
    buffer[VESC6_APP_OFF_ADC_CTRL_TYPE] = (uint8_t)a->ctrl_type;
    put_auto_at(buffer, VESC6_APP_OFF_ADC_HYST, a->hyst);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_START, a->voltage_start, 1000.0f);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_END, a->voltage_end, 1000.0f);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_MIN, a->voltage_min, 1000.0f);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_MAX, a->voltage_max, 1000.0f);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE_CENTER, a->voltage_center, 1000.0f);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE2_START, a->voltage2_start, 1000.0f);
    put_f16_at(buffer, VESC6_APP_OFF_ADC_VOLTAGE2_END, a->voltage2_end, 1000.0f);
    buffer[VESC6_APP_OFF_ADC_USE_FILTER] = a->use_filter ? 1U : 0U;
    buffer[VESC6_APP_OFF_ADC_SAFE_START] = (uint8_t)a->safe_start;
    buffer[VESC6_APP_OFF_ADC_BUTTONS] = a->buttons;
    buffer[VESC6_APP_OFF_ADC_VOLTAGE_INVERTED] = a->voltage_inverted ? 1U : 0U;
    buffer[VESC6_APP_OFF_ADC_VOLTAGE2_INVERTED] = a->voltage2_inverted ? 1U : 0U;
    put_auto_at(buffer, VESC6_APP_OFF_ADC_THROTTLE_EXP, a->throttle_exp);
    put_auto_at(buffer, VESC6_APP_OFF_ADC_THROTTLE_EXP_BRAKE, a->throttle_exp_brake);
    buffer[VESC6_APP_OFF_ADC_THROTTLE_EXP_MODE] = (uint8_t)a->throttle_exp_mode;
    put_auto_at(buffer, VESC6_APP_OFF_ADC_RAMP_TIME_POS, a->ramp_time_pos);
    put_auto_at(buffer, VESC6_APP_OFF_ADC_RAMP_TIME_NEG, a->ramp_time_neg);
    buffer[VESC6_APP_OFF_ADC_MULTI_ESC] = a->multi_esc ? 1U : 0U;
    buffer[VESC6_APP_OFF_ADC_TC] = a->tc ? 1U : 0U;
    put_auto_at(buffer, VESC6_APP_OFF_ADC_TC_MAX_DIFF, a->tc_max_diff);
    put_u16_at(buffer, VESC6_APP_OFF_ADC_UPDATE_RATE_HZ, a->update_rate_hz);
    put_u32_at(buffer, VESC6_APP_OFF_UART_BAUD, conf->app_uart_baudrate);
    return (int32_t)VESC6_APPCONF_WIRE_SIZE;
}

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi confgenerator_set_defaults_appconf: mengatur confgenerator set defaults appconf setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void confgenerator_set_defaults_appconf(app_configuration *conf) {
    if (!conf)
        return;
    vesc_config_init_defaults();
    (void)confgenerator_deserialize_appconf(s_app_factory, conf);
}
