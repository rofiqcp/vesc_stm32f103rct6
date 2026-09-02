#include "motor/mcpwm_foc.h"
#include "motor/foc_math.h"
#include "motor/mc_interface.h"
#include "motor/mc_math.h"
#include "hwconf/hw.h"
#include "motor/mcconf_default.h"
#include "encoder/encoder.h"
#include "timeout.h"
#include "comm/commands.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>

// Variabel s_cal_done: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_cal_done = false;
// Variabel s_isr_motor: state atau parameter motor yang sedang diproses.
static volatile int s_isr_motor = 0;
// Variabel s_foc_init_done: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_foc_init_done = false;
// Variabel s_cal_valid: penanda validitas hasil pengukuran atau konfigurasi.
static volatile bool s_cal_valid = false;
// Variabel s_cal_request: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_cal_request = false;
// Variabel s_cal_stage: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile foc_cal_stage_t s_cal_stage = FOC_CAL_STAGE_UNDRIVEN;
/* When true, boot calibration is skipped and the bridge is considered
 * offset-calibrated using stored/gross-default offsets. Set from
 * mc_interface_init() based on mc_configuration.foc_calibrate_on_boot. */
// Variabel s_calibration_skip: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_calibration_skip = false;
// Variabel s_cal_count: pencacah kejadian atau sampel.
static uint32_t s_cal_count = 0U; /* sample count in current stage */
// Variabel s_cal_progress: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_cal_progress = 0U; /* total accepted calibration samples */
// Variabel s_cal_warmup: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_cal_warmup = 0U;
// Variabel s_cal_vbus_stable_ticks: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static uint16_t s_cal_vbus_stable_ticks = 0U;
// Variabel s_cal_vbus_last_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_cal_vbus_last_q15 = 0;
// Variabel s_cal_vbus_last_valid: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static bool s_cal_vbus_last_valid = false;
// Variabel s_min_ldc: batas atau nilai minimum untuk validasi dan proteksi.
// Variabel s_min_lu: batas atau nilai minimum untuk validasi dan proteksi.
// Variabel s_min_lv: batas atau nilai minimum untuk validasi dan proteksi.
// Variabel s_min_rdc: batas atau nilai minimum untuk validasi dan proteksi.
// Variabel s_min_ru: batas atau nilai minimum untuk validasi dan proteksi.
// Variabel s_min_rv: batas atau nilai minimum untuk validasi dan proteksi.
static uint16_t s_min_lu, s_min_lv, s_min_ldc, s_min_ru, s_min_rv, s_min_rdc;
// Variabel s_max_ldc: batas atau nilai maksimum untuk validasi dan proteksi.
// Variabel s_max_lu: batas atau nilai maksimum untuk validasi dan proteksi.
// Variabel s_max_lv: batas atau nilai maksimum untuk validasi dan proteksi.
// Variabel s_max_rdc: batas atau nilai maksimum untuk validasi dan proteksi.
// Variabel s_max_ru: batas atau nilai maksimum untuk validasi dan proteksi.
// Variabel s_max_rv: batas atau nilai maksimum untuk validasi dan proteksi.
static uint16_t s_max_lu, s_max_lv, s_max_ldc, s_max_ru, s_max_rv, s_max_rdc;
// Variabel s_cal_outlier_count: pencacah kejadian atau sampel.
static uint16_t s_cal_outlier_count[6];
// Variabel s_cal_moe_wait_events: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint16_t s_cal_moe_wait_events[2];
// Variabel s_cal_moe_live_bdtr: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint8_t s_cal_moe_live_bdtr[2] = {
    0U, 0U
}
;
// Variabel s_cal_moe_fail_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_cal_moe_fail_mask;
// Variabel s_cal_moe_confirmed_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_cal_moe_confirmed_mask;
// Variabel s_cal_moe_request_adc: nilai atau state ADC pada jalur pengukuran.
static uint32_t s_cal_moe_request_adc[2];
// Variabel s_cal_moe_confirm_adc: nilai atau state ADC pada jalur pengukuran.
static uint32_t s_cal_moe_confirm_adc[2];
// Variabel s_cal_first_sample_adc: nilai atau state ADC pada jalur pengukuran.
static uint32_t s_cal_first_sample_adc[2];
/* VESC keeps driven and undriven calibration states distinct. The current PI
 * uses driven offsets. Undriven values are retained only for diagnostics and
 * for gross zero-vector safety during the driven-calibration transition. */
// Variabel s_undriven_mean: state internal modul yang dipertahankan antar pemanggilan fungsi.
static int32_t s_undriven_mean[6] = {
    0
}
;
// Variabel s_driven_mean: state internal modul yang dipertahankan antar pemanggilan fungsi.
static int32_t s_driven_mean[6] = {
    0
}
;
// Variabel s_cal_diag_channels: state internal modul yang dipertahankan antar pemanggilan fungsi.
static foc_cal_channel_diag_t s_cal_diag_channels[6];
// Variabel s_cal_shift_warn_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_cal_shift_warn_mask = 0U;
// Variabel s_fault_snapshot: status atau data gangguan untuk sistem proteksi.
static volatile foc_fault_snapshot_t s_fault_snapshot;
// Variabel s_overrun_consecutive: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_overrun_consecutive[2] = {
    0U, 0U
}
;
// Variabel s_hard_current_trip_consecutive: nilai arus untuk pengukuran, kendali, atau proteksi.
static uint8_t s_hard_current_trip_consecutive[2] = {
    0U, 0U
}
;
// Variabel s_startup_dc_trip_consecutive: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_startup_dc_trip_consecutive[2] = {
    0U, 0U
}
;
// Variabel s_inv_vbus_q30: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_inv_vbus_q30 = 0;
// Variabel s_inv_vbus_last_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_inv_vbus_last_q15 = 0;
// Variabel s_inv_vbus_age: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static uint8_t s_inv_vbus_age = 0U;
// Variabel s_adc_isr_count: nilai atau state ADC pada jalur pengukuran.
static volatile uint32_t s_adc_isr_count = 0U;
/* Stage-3 hard real-time instrumentation. These counters are updated only by
 * the ADC/FOC ISR and read task-side, so no lock is required for 32-bit reads
 * on Cortex-M3. They measure the complete dual-motor ISR, not just one motor. */
// Variabel s_isr_total_max_cycles: batas atau nilai maksimum untuk validasi dan proteksi.
static volatile uint32_t s_isr_total_max_cycles = 0U;
// Variabel s_isr_near_deadline_count: pencacah kejadian atau sampel.
static volatile uint32_t s_isr_near_deadline_count = 0U;
// Variabel s_isr_period_min_cycles: periode pengukuran atau eksekusi modul.
static volatile uint32_t s_isr_period_min_cycles = UINT32_MAX;
// Variabel s_isr_period_max_cycles: periode pengukuran atau eksekusi modul.
static volatile uint32_t s_isr_period_max_cycles = 0U;
// Variabel s_isr_last_entry_cycle: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_isr_last_entry_cycle = 0U;
/* Last measured dual-motor ISR duration in seconds, for the VESC-compatible
 * mc_interface_get_last_inj_adc_isr_duration() telemetry getter. This F103
 * target uses the DMA ADC path, not the upstream injected-ADC ISR, but the
 * same DWT cycle-count instrumentation is the honest source for this value. */
// Variabel s_isr_last_cycles: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_isr_last_cycles = 0U;
// Variabel s_vbus_dma_stale_count: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static uint8_t s_vbus_dma_stale_count = 0U;
// Variabel s_vbus_dma_stale_events: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static volatile uint32_t s_vbus_dma_stale_events = 0U;
// Variabel s_cal_warn_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_cal_warn_mask = 0U;
// Variabel s_cal_fail_range_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_cal_fail_range_mask = 0U;
// Variabel s_cal_fail_noise_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint16_t s_cal_fail_noise_mask = 0U;

// Variabel s_vbus_scale_q16: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_vbus_scale_q16;
// Variabel s_vbus_min_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_vbus_min_q15;
// Variabel s_vbus_max_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_vbus_max_q15;
// Variabel s_vbus_hard_min_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_vbus_hard_min_q15;
// Variabel s_vbus_hard_max_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
static int32_t s_vbus_hard_max_q15;
// Variabel s_current_trip_q15: nilai arus untuk pengukuran, kendali, atau proteksi.
static int32_t s_current_trip_q15;
// Variabel s_startup_dc_trip_q15: state internal modul yang dipertahankan antar pemanggilan fungsi.
static int32_t s_startup_dc_trip_q15;

// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi low16: menjalankan operasi low16 sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static inline uint16_t low16(uint32_t w) {
    return (uint16_t)(w & 0xFFFFU);
}
// Parameter w: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi high16: menjalankan operasi high16 sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static inline uint16_t high16(uint32_t w) {
    return (uint16_t)(w >> 16);
}
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi iabs32: menjalankan operasi iabs32 sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static inline int32_t iabs32(int32_t x) {
    if (x >= 0)
        return x;
    return x == INT32_MIN ? INT32_MAX : -x;
}
// Parameter a_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b_q16: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi q15_mul_q16: menjalankan operasi q15 mul q16 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static inline int32_t q15_mul_q16(int32_t a_q15, int32_t b_q16) {
    return (int32_t)(((int64_t)a_q15 * (int64_t)b_q16) >> 16);
}

/* Integer square-root for Q15 voltage-vector saturation. The operand is at
 * most about 2^30, so this bounded bit-wise implementation is deterministic
 * on Cortex-M3 and keeps sqrtf out of the 16-kHz hard ISR. */
// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi isqrt_u32: menjalankan operasi isqrt u32 sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static inline uint32_t isqrt_u32(uint32_t x) {
    // Variabel op: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t op = x;
    // Variabel res: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t res = 0U;
    // Variabel one: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t one = 1UL << 30;
    while (one > op)
        one >>= 2;
    while (one != 0U) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        }
        else {
            res >>= 1;
        }
        one >>= 2;
    }
    return res;
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi clamp_i32: membatasi clamp i32 ke rentang yang diizinkan agar pengendali dan perangkat keras tetap
// aman.
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi abs_i32_sat_local: menjalankan operasi abs i32 sat local sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static inline int32_t abs_i32_sat_local(int32_t v) {
    if (v >= 0)
        return v;
    if (v == (-2147483647 - 1))
        return 2147483647;
    return -v;
}

/* VESC 7.x moved field weakening into the fast current-control loop and
 * composes it with MTPA by selecting the d-axis request with the largest
 * magnitude instead of summing two negative Id requests. This STM32F103 port
 * keeps that behavior entirely fixed-point. Configuration-dependent divisions
 * are precalculated by foc_precalc_values(). */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_prev_duty_abs_q15: menjalankan bagian foc prev duty abs q15 pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
static inline int32_t foc_prev_duty_abs_q15(const MotorRuntime *m) {
    // Variabel du: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t du = (int32_t)m->duty_u_q15 - FOC_Q15_HALF;
    // Variabel dv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dv = (int32_t)m->duty_v_q15 - FOC_Q15_HALF;
    // Variabel dw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dw = (int32_t)m->duty_w_q15 - FOC_Q15_HALF;
    // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t a = abs_i32_sat_local(du);
    // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t b = abs_i32_sat_local(dv);
    if (b > a)
        a = b;
    b = abs_i32_sat_local(dw);
    if (b > a)
        a = b;
    a <<= 1;
    return a > 32767 ? 32767 : a;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_fw_control_mode_fast: menjalankan bagian foc fw control mode fast pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
static inline bool foc_fw_control_mode_fast(const MotorRuntime *m) {
    return m->control_mode == MOTOR_CTRL_CURRENT ||
           m->control_mode == MOTOR_CTRL_BRAKE_CURRENT ||
           m->control_mode == MOTOR_CTRL_SPEED;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id_target: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Parameter iq_target: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi foc_apply_fast_fw_targets_isr: menangani foc apply fast fw targets isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
static inline void foc_apply_fast_fw_targets_isr(MotorRuntime *m,
                                                  int32_t *id_target,
                                                  int32_t *iq_target) {
    // Variabel id_base: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    int32_t id_base = m->id_target_base_q15;
    // Variabel iq_base: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t iq_base = m->iq_target_base_q15;
    // Variabel fw_now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fw_now = m->foc_fw_current_q15;
    // Variabel fw_target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t fw_target = 0;
    // Variabel fw_max: batas atau nilai maksimum untuk validasi dan proteksi.
    const int32_t fw_max = m->foc_fw_max_q15;

    // Variabel keep_ramping: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool keep_ramping = fw_now > 0;
    // Variabel eligible: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool eligible = m->pwm_enabled && m->command_active && !m->detect.busy &&
                          (foc_fw_control_mode_fast(m) || keep_ramping);

    /* VESC uses a 0.01 fast low-pass on |duty| before FW thresholding. Keep
       the same noise rejection with integer math (328/32768 ~= 0.0100). */
    // Variabel duty_raw: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    const int32_t duty_raw = foc_prev_duty_abs_q15(m);
    m->foc_fw_duty_filter_q15 += (int32_t)(((int64_t)(duty_raw - m->foc_fw_duty_filter_q15) * 328) >> 15);

    if (eligible && fw_max > 0) {
        // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
        int32_t duty = m->foc_fw_duty_filter_q15;
        duty = (int32_t)(((int64_t)duty * (int64_t)m->foc_fw_duty_norm_scale_q16) >> 16);
        if (duty > m->foc_fw_duty_end_q15)
            duty = m->foc_fw_duty_end_q15;

        if (duty > m->foc_fw_duty_start_q15 &&
            m->foc_fw_duty_end_q15 > m->foc_fw_duty_start_q15) {
            // Variabel fw_max_now: batas atau nilai maksimum untuk validasi dan proteksi.
            int32_t fw_max_now = fw_max;

            /* Upstream FW backoff uses Iq tracking error and electrical-speed
               direction. Use filtered Iq on this board to suppress switching
               noise while retaining the same sign semantics. */
            if (m->foc_fw_backoff_per_current_q16 > 0 && fw_max_now > 0) {
                /* iq_target_q15 still contains the previous frame's effective
                   target here, matching upstream's backoff against the active
                   target rather than the new MTPA-only base request. */
                // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                int32_t err = m->iq_filter_q15 - m->iq_target_q15;
                if (m->speed_est_fast_erpm_q16 < 0)
                    err = -err;
                // Variabel backoff_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                int32_t backoff_q15 = (int32_t)(((int64_t)err *
                                                (int64_t)m->foc_fw_backoff_per_current_q16) >> 16);
                backoff_q15 = clamp_i32(backoff_q15, 0, 32767);
                fw_max_now = foc_q15_mul(fw_max_now, 32768 - backoff_q15);
            }

            // Variabel ratio_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int32_t ratio_q15 = (int32_t)(((int64_t)(duty - m->foc_fw_duty_start_q15) *
                                           (int64_t)m->foc_fw_duty_span_inv_q30) >> 15);
            ratio_q15 = clamp_i32(ratio_q15, 0, 32767);
            fw_target = foc_q15_mul(fw_max_now, ratio_q15);
            if (fw_target > 0)
                m->foc_fw_hold_request = true;
        }
    }

    // Variabel fw_override: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool fw_override = m->pwm_enabled && m->command_active && m->fw_override_current_q15 > 6;
    if (fw_override)
        fw_target = m->fw_override_current_q15;
    // Variabel current_lim: nilai arus untuk pengukuran, kendali, atau proteksi.
    int32_t current_lim = m->foc_current_limit_q15;
    if (current_lim <= 0 || current_lim > 32767)
        current_lim = 32767;
    if (fw_target > current_lim)
        fw_target = current_lim;

    /* Fractional Q31 accumulator preserves smooth long ramp times even when a
       single 16-kHz step is below one current-Q15 LSB. */
    // Variabel target_q31: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t target_q31 = (int32_t)((int64_t)fw_target * 65536LL);
    // Variabel acc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t acc = m->foc_fw_current_acc_q31;
    // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t step = m->foc_fw_ramp_step_q31;
    /* Upstream FW override writes the FW setpoint directly rather than
       passing it through the automatic FW ramp. */
    if (fw_override || step <= 0 || m->foc_fw_ramp_direct) {
        acc = target_q31;
    }
    else if (acc < target_q31) {
        acc += step;
        if (acc > target_q31)
            acc = target_q31;
    }
    else if (acc > target_q31) {
        acc -= step;
        if (acc < target_q31)
            acc = target_q31;
    }
    if (acc < 0)
        acc = 0;
    // Variabel max_acc: batas atau nilai maksimum untuk validasi dan proteksi.
    const int32_t max_acc = (int32_t)((int64_t)current_lim * 65536LL);
    if (acc > max_acc)
        acc = max_acc;
    m->foc_fw_current_acc_q31 = acc;
    fw_now = (int32_t)(acc >> 16);
    m->foc_fw_current_q15 = fw_now;
    m->foc_fw_fast_active = fw_now > 0;

    /* VESC 7.00: choose the highest-magnitude d-axis request. For normal MTPA
       both values are negative; the absolute comparison is kept general. */
    // Variabel fw_id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    int32_t fw_id = -fw_now;
    // Variabel id_eff: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    int32_t id_eff = abs_i32_sat_local(fw_id) > abs_i32_sat_local(id_base) ? fw_id : id_base;

    /* VESC FW q-current compensation follows modulation/Vq sign. The previous
       applied Vq is the freshest deterministic sign available before this
       frame's PI output is calculated. */
    // Variabel q_corr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t q_corr = foc_q15_mul(fw_now, m->foc_fw_q_factor_q15);
    // Variabel mod_sign: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t mod_sign = (m->vq_q15 < 0) ? -1 : ((m->vq_q15 > 0) ? 1 : ((iq_base < 0) ? -1 : 1));
    // Variabel iq_eff: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t iq_eff = iq_base - mod_sign * q_corr;

    id_eff = clamp_i32(id_eff, -current_lim, current_lim);
    // Variabel rem: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t rem = (int64_t)current_lim * current_lim - (int64_t)id_eff * id_eff;
    if (rem < 0)
        rem = 0;
    if (rem > UINT32_MAX)
        rem = UINT32_MAX;
    // Variabel iq_lim: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t iq_lim = (int32_t)isqrt_u32((uint32_t)rem);
    iq_eff = clamp_i32(iq_eff, -iq_lim, iq_lim);

    m->id_target_q15 = id_eff;
    m->iq_target_q15 = iq_eff;
    *id_target = id_eff;
    *iq_target = iq_eff;
}

/* VESC-style dead-time compensation for the applied-voltage model. This does
   not alter CCR timing: the advanced timers already generate the proven board
   dead-time. It corrects only the voltage fed to the flux observer, using the
   sign of the filtered phase currents and a task-side Q15 value of
   foc_dt_us * foc_f_zv. All arithmetic remains fixed-point in the hard ISR. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter vbus_q15: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter sn_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter cs_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter v_alpha_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter v_beta_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_deadtime_applied_voltage_q15: menjalankan bagian foc deadtime applied voltage q15 pada algoritma
// FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
static inline void foc_deadtime_applied_voltage_q15(
        const MotorRuntime *m, int32_t vbus_q15, int32_t sn_q15, int32_t cs_q15,
        int32_t *v_alpha_q15, int32_t *v_beta_q15) {
    if (!m || !v_alpha_q15 || !v_beta_q15 || m->deadtime_comp_q15 <= 0)
        return;

    // Variabel idf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t idf = m->id_filter_q15;
    // Variabel iqf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t iqf = m->iq_filter_q15;
    // Variabel i_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t i_alpha = foc_q15_mul(cs_q15, idf) - foc_q15_mul(sn_q15, iqf);
    // Variabel i_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t i_beta = foc_q15_mul(sn_q15, idf) + foc_q15_mul(cs_q15, iqf);
    // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t ia = i_alpha;
    // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t ib = -(i_alpha / 2) + foc_q15_mul(FOC_Q15_SQRT3_BY_2, i_beta);
    // Variabel ic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t ic = -(i_alpha / 2) - foc_q15_mul(FOC_Q15_SQRT3_BY_2, i_beta);

    foc_deadtime_compensate_voltage_q15(ia, ib, ic, vbus_q15,
                                         m->deadtime_comp_q15,
                                         v_alpha_q15, v_beta_q15);
}

// Fungsi cal_acc_reset: menangani kalibrasi cal acc reset agar offset atau parameter hasil ukur valid sebelum
// dipakai kendali.
static void cal_acc_reset(void) {
    s_cal_count = 0U;
    s_cal_warmup = 0U;
    s_min_lu = s_min_lv = s_min_ldc = s_min_ru = s_min_rv = s_min_rdc = UINT16_MAX;
    s_max_lu = s_max_lv = s_max_ldc = s_max_ru = s_max_rv = s_max_rdc = 0U;
}


// Parameter stage: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi cal_task_reset_acc_and_stage: menangani kalibrasi cal task reset acc and stage agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
static void cal_task_reset_acc_and_stage(foc_cal_stage_t stage) {
    /* The accumulators are written by DMA1_CH1 ISR. Reset them atomically so
       the ISR can never observe a half-reset sum/count set. The critical
       section is only register/RAM writes and contains no HAL/RTOS calls. */
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cal_acc_reset();
    if (stage == FOC_CAL_STAGE_LEFT_WARMUP) {
        // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (uint8_t i = 0U; i < 3U; i++) {
            s_cal_outlier_count[i] = 0U;
        }
        s_cal_moe_wait_events[0] = 0U;
        s_cal_moe_request_adc[0] = s_adc_isr_count;
        s_cal_moe_confirm_adc[0] = 0U;
        s_cal_first_sample_adc[0] = 0U;
    }
    else if (stage == FOC_CAL_STAGE_RIGHT_WARMUP) {
        // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (uint8_t i = 3U; i < 6U; i++) {
            s_cal_outlier_count[i] = 0U;
        }
        s_cal_moe_wait_events[1] = 0U;
        s_cal_moe_request_adc[1] = s_adc_isr_count;
        s_cal_moe_confirm_adc[1] = 0U;
        s_cal_first_sample_adc[1] = 0U;
    }
    s_cal_stage = stage;
    if (!primask)
        __enable_irq();
}

// Fungsi cal_reset_isr: menangani cal reset isr pada konteks interrupt dengan pekerjaan minimum agar timing FOC
// tetap deterministik.
static void cal_reset_isr(void) {
    /* NOTE: s_cal_done is NOT cleared here. foc_request_recalibration()
     * sets s_cal_done = false BEFORE setting s_cal_request = true so that the
     * gate in motor_hw_service_pwm_enable_from_isr() (which checks !s_cal_done)
     * cannot block MOE during the one-ISR window between the request and the
     * ISR's cal_reset_isr() call. The task-side calibration_service_task()
     * transition from UNDRIVEN to WAIT_LEFT_DRIVEN happens at the next 200 Hz
     * tick, not in this ISR. */
    s_cal_valid = false;
    s_cal_request = false;
    s_cal_stage = FOC_CAL_STAGE_WAIT_LEFT_DRIVEN;
    s_cal_progress = 0U;
    s_cal_warn_mask = 0U;
    s_cal_fail_range_mask = 0U;
    s_cal_fail_noise_mask = 0U;
    s_cal_shift_warn_mask = 0U;
    s_cal_moe_fail_mask = 0U;
    s_cal_moe_confirmed_mask = 0U;
    s_cal_vbus_stable_ticks = 0U;
    s_cal_vbus_last_q15 = 0;
    s_cal_vbus_last_valid = false;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t i = 0U; i < 6U; i++) {
        s_undriven_mean[i] = 0;
        s_driven_mean[i] = 0;
        s_cal_outlier_count[i] = 0U;
        s_cal_diag_channels[i].mean = 0;
        s_cal_diag_channels[i].min = UINT16_MAX;
        s_cal_diag_channels[i].max = 0U;
        s_cal_diag_channels[i].variance_x100 = 0U;
    }
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t i = 0U; i < 2U; i++) {
        s_cal_moe_wait_events[i] = 0U;
        s_cal_moe_request_adc[i] = 0U;
        s_cal_moe_confirm_adc[i] = 0U;
        s_cal_first_sample_adc[i] = 0U;
    }
    s_fault_snapshot.valid = 0U;
    cal_acc_reset();
    /* Drop any stale bridge-enable handshake from a previous calibration or
     * from a stopped motor so the next driven stage can always re-arm MOE.
     * Without this, a motor left with pwm_enable_pending_events == 0 (e.g.
     * after the task-side stop that decrements it to zero) would block the
     * new calibration forever inside cal_moe_ready_isr's 128-event timeout. */
    g_motor_left.pwm_enable_pending_events = 0U;
    g_motor_left.pwm_enabled = false;
    g_motor_right.pwm_enable_pending_events = 0U;
    g_motor_right.pwm_enabled = false;
    /* Jangan pernah menghapus fault dari ISR kalibrasi. foc_request_recalibration()
     * sudah membersihkan hanya fault software yang recoverable di task context;
     * PVD/BKIN/BREAK/MCU-UV tetap latched dan motor_hw_set_pwm_enabled() akan
     * menolak MOE jika hardware belum aman. */
}

// Fungsi foc_calibration_done: menangani kalibrasi foc calibration done agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
bool foc_calibration_done(void) {
    return s_cal_done;
}
// Fungsi foc_calibration_valid: menangani kalibrasi foc calibration valid agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
bool foc_calibration_valid(void) {
    return s_cal_done && s_cal_valid;
}
// Fungsi foc_calibration_in_progress: menangani kalibrasi foc calibration in progress agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
bool foc_calibration_in_progress(void) {
    return !s_cal_done;
}
// Fungsi foc_calibration_stage: menangani kalibrasi foc calibration stage agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
foc_cal_stage_t foc_calibration_stage(void) {
    return s_cal_stage;
}

// Fungsi foc_adc_isr_count: menangani foc adc isr count pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
uint32_t foc_adc_isr_count(void) {
    return s_adc_isr_count;
}
// Fungsi foc_isr_total_max_cycles: menangani foc isr total max cycles pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_total_max_cycles(void) {
    return s_isr_total_max_cycles;
}
/* Mengubah jumlah cycle ISR terakhir menjadi detik di konteks task/getter.
 * ISR 16 kHz hanya menyimpan cycle mentah agar tidak melakukan pembagian float. */
// Fungsi foc_last_isr_duration_s: menangani foc last isr duration s pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
float foc_last_isr_duration_s(void) {
    return (float)s_isr_last_cycles / (float)CPU_CLOCK_HZ;
}
// Fungsi foc_isr_near_deadline_count: menangani foc isr near deadline count pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_near_deadline_count(void) {
    return s_isr_near_deadline_count;
}
// Fungsi foc_isr_period_min_cycles: menangani foc isr period min cycles pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_period_min_cycles(void) {
    return s_isr_period_min_cycles == UINT32_MAX ? 0U : s_isr_period_min_cycles;
}
// Fungsi foc_isr_period_max_cycles: menangani foc isr period max cycles pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
uint32_t foc_isr_period_max_cycles(void) {
    return s_isr_period_max_cycles;
}
// Fungsi foc_vbus_dma_stale_events: menjalankan bagian foc vbus dma stale events pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
uint32_t foc_vbus_dma_stale_events(void) {
    return s_vbus_dma_stale_events;
}
// Fungsi foc_vbus_dma_stale_count: menjalankan bagian foc vbus dma stale count pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
uint8_t foc_vbus_dma_stale_count(void) {
    return s_vbus_dma_stale_count;
}

// Parameter count: pencacah kejadian, elemen, atau sampel.
// Parameter target: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi foc_get_calibration_progress: membaca foc get calibration progress tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
void foc_get_calibration_progress(uint32_t *count, uint32_t *target) {
    if (count != NULL)
        *count = s_cal_progress;
    if (target != NULL)
        *target = ADC_NEUTRAL_CAL_SAMPLES;
}

// Fungsi foc_request_recalibration: menjalankan bagian foc request recalibration pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
void foc_request_recalibration(void) {
    /* Clear recoverable software faults before the driven zero-vector stage.
     * motor_clear_fault_for_cal() clears even a config-flash fault (raised when
     * the VESC config record is blank/corrupt), while still refusing PVD/BKIN/
     * break/under-voltage. A stale fault must not permanently prevent a safe
     * stopped recalibration from arming its 50% zero vector. */
    if (motor_hw_clear_recoverable_powerstage_faults()) {
        motor_clear_fault_for_cal(&g_motor_left);
        motor_clear_fault_for_cal(&g_motor_right);
    }
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_cal_request = true;
    s_cal_done = false;
    if (!primask)
        __enable_irq();
    hw_status_notify_calibration_start();
}

// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_get_fault_snapshot: membaca foc get fault snapshot tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void foc_get_fault_snapshot(foc_fault_snapshot_t *out) {
    if (out == NULL)
        return;
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_fault_snapshot;
    if (!primask)
        __enable_irq();
}

// Parameter skip: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_calibration_set_skip: mengatur foc calibration set skip setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void foc_calibration_set_skip(bool skip) {
    /* Called once from mc_interface_init() before mcpwm_foc_init_hw().
     * When skip is set, the boot offset-calibration pipeline is bypassed and
     * the bridge is treated as already calibrated using the stored/gross
     * default offsets. The DMA ISR still accumulates samples harmlessly; the
     * task-side state machine simply never advances from DONE. */
    s_calibration_skip = skip;
}

// Fungsi mcpwm_foc_init_hw: menginisialisasi mcpwm foc init hw sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void mcpwm_foc_init_hw(void) {
    s_foc_init_done = true;
    if (s_calibration_skip) {
        /* Bypass the entire calibration pipeline. Mark done+valid so
         * motor_slow_update_1khz() can enable the bridge immediately and
         * foc_calibration_in_progress() returns false. Offsets remain at
         * their stored/gross-default values (set by conf_general or
         * cal_set_runtime_offsets during a prior calibrated boot). */
        // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_cal_done = true;
        s_cal_valid = true;
        s_cal_stage = FOC_CAL_STAGE_DONE;
        s_cal_request = false;
        if (!primask)
            __enable_irq();
    }
    else {
        /* Boot dan recalibration manual memakai state machine yang sama.
         * Offset tidak pernah lagi diambil saat bridge OFF lalu dipakai saat
         * bridge aktif, karena common-mode analog stock board dapat bergeser
         * ratusan count. Task 200 Hz akan menunggu Vbus stabil, menyalakan
         * kedua bridge pada zero-vector 50%, lalu ISR mengambil 2000 frame. */
        cal_reset_isr();
        s_cal_done = false;
        s_cal_valid = false;
        s_cal_request = false;
    }
    s_overrun_consecutive[0] = 0U;
    s_overrun_consecutive[1] = 0U;
    s_inv_vbus_q30 = 0;
    s_inv_vbus_last_q15 = 0;
    s_inv_vbus_age = 0U;
    s_adc_isr_count = 0U;
    s_vbus_dma_stale_count = 0U;
    s_vbus_dma_stale_events = 0U;
    s_vbus_scale_q16 = (int32_t)((DCLINK_V_PER_COUNT / FOC_VOLTAGE_Q_BASE_V) * 32768.0f * 65536.0f);
    s_vbus_min_q15 = (int32_t)((VBUS_MIN_RUN_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    s_vbus_max_q15 = (int32_t)((VBUS_MAX_RUN_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    s_vbus_hard_min_q15 = (int32_t)((FOC_VBUS_HARD_MIN_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    s_vbus_hard_max_q15 = (int32_t)((FOC_VBUS_HARD_MAX_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    s_current_trip_q15 = (int32_t)((FOC_ABS_CURRENT_TRIP_A / FOC_CURRENT_Q_BASE_A) * 32768.0f);
    s_startup_dc_trip_q15 = (int32_t)((PWM_STARTUP_DC_TRIP_A / FOC_CURRENT_Q_BASE_A) * 32768.0f);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi foc_enter_control_mode: menjalankan bagian foc enter control mode pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
static void foc_enter_control_mode(MotorRuntime *m, motor_control_mode_t mode) {
    if (!m)
        return;
    if (m->control_mode != mode) {
        m->speed_pid.integrator = 0.0f;
        m->speed_pid.prev_error = 0.0f;
        m->speed_derivative_filtered = 0.0f;
        m->position_pid.integrator = 0.0f;
        m->position_pid.prev_error = 0.0f;
        m->position_derivative_filtered = 0.0f;
        m->position_derivative_proc_filtered = 0.0f;
        m->position_dt_integrator = 0.0f;
        m->position_dt_process_integrator = 0.0f;
        m->position_prev_process_deg = m->position_deg+m->position_offset_deg;
        m->duty_pid.integrator = 0.0f;
        m->duty_pid.prev_error = 0.0f;
        m->duty_was_pi = false;
        m->duty_pi_duty_last = 0.0f;
        m->force_zero_modulation = false;
        if (mode == MOTOR_CTRL_SPEED) {
            m->speed_pid_set_erpm = (float)m->pll_erpm_q16/65536.0f;
        }
        if (mode != MOTOR_CTRL_BRAKE_CURRENT) {
            m->brake_zero_active = false;
            m->brake_zero_hold_ticks = 1U;
        }
    }
    if (mode != MOTOR_CTRL_HANDBRAKE && mode != MOTOR_CTRL_DETECT)
        m->detect_force_angle = false;
    m->control_mode = mode;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_current_motor: mengatur mcpwm foc set current motor setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_motor(MotorRuntime *m, float current) {
    if (m == NULL) {
        return;
    }

    m->current_command_a = foc_clampf(current, m->current_min_a, m->current_max_a);
    foc_enter_control_mode(m, MOTOR_CTRL_CURRENT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_brake_current_motor: mengatur mcpwm foc set brake current motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_brake_current_motor(MotorRuntime *m, float current) {
    if (m == NULL) {
        return;
    }

    // Variabel lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lim = fmaxf(fabsf(m->current_min_a), fabsf(m->current_max_a));
    m->brake_current_a = fabsf(foc_clampf(current, -lim, lim));
    foc_enter_control_mode(m, MOTOR_CTRL_BRAKE_CURRENT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_handbrake_motor: mengatur mcpwm foc set handbrake motor setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_handbrake_motor(MotorRuntime *m, float current) {
    if (m == NULL) {
        return;
    }

    // Variabel lim: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float lim = fmaxf(fabsf(m->current_min_a), fabsf(m->current_max_a));
    m->handbrake_current_a = fabsf(foc_clampf(current, -lim, lim));
    m->detect_phase_u16 = motor_sensor_electrical_phase_u16(m);
    m->detect_force_angle = true;
    foc_enter_control_mode(m, MOTOR_CTRL_HANDBRAKE);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mcpwm_foc_set_pid_speed_motor: mengatur mcpwm foc set pid speed motor setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_speed_motor(MotorRuntime *m, float erpm) {
    if (m == NULL) {
        return;
    }

    m->speed_target_erpm = foc_clampf(erpm, m->min_erpm, m->max_erpm);
    foc_enter_control_mode(m, MOTOR_CTRL_SPEED);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter pos_deg: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi mcpwm_foc_set_pid_pos_motor: mengatur mcpwm foc set pid pos motor setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_pos_motor(MotorRuntime *m, float pos_deg) {
    if (m == NULL) {
        return;
    }

    /* LEFT A/B steering uses an extended, non-wrapped mechanical coordinate.
       Other sensor modes retain normal VESC circular position semantics. */
    if (m->id == MOTOR_LEFT &&
        (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
         m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER)) {
        m->position_target_deg = pos_deg;
    }
    else {
        m->position_target_deg = foc_wrap_deg(pos_deg);
    }
    foc_enter_control_mode(m, MOTOR_CTRL_POSITION);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi foc_clamp_duty_command: menjalankan bagian foc clamp duty command pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
static float foc_clamp_duty_command(const MotorRuntime *m, float duty) {
    if (m == NULL)
        return 0.0f;
    // Variabel max_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float max_duty = foc_clampf(fabsf(m->max_duty), 0.01f, 0.98f);
    // Variabel min_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    float min_duty = foc_clampf(fabsf(m->min_duty), 0.0f, max_duty);
    // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float out = foc_clampf(duty, -max_duty, max_duty);
    /* VESC duty semantics: commands below l_min_duty release/stop duty drive;
       they are not promoted to the minimum non-zero duty. */
    if (fabsf(out) < min_duty)
        out = 0.0f;
    return out;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Fungsi mcpwm_foc_set_duty_motor: mengatur mcpwm foc set duty motor setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_duty_motor(MotorRuntime *m, float duty) {
    if (m == NULL) {
        return;
    }

    m->duty_command = foc_clamp_duty_command(m, duty);
    m->duty_command_q15 = (int32_t)lrintf(m->duty_command * 32768.0f);
    foc_enter_control_mode(m, MOTOR_CTRL_DUTY);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mcpwm_foc_set_openloop_current_motor: mengatur mcpwm foc set openloop current motor setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_current_motor(MotorRuntime *m, float current, float erpm) {
    if (m == NULL) {
        return;
    }

    m->current_command_a = foc_clampf(current, m->current_min_a, m->current_max_a);
    m->openloop_command_erpm = foc_clampf(erpm, m->min_erpm, m->max_erpm);
    foc_enter_control_mode(m, MOTOR_CTRL_OPENLOOP);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter phase_deg: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_phase_motor: mengatur mcpwm foc set openloop phase motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_phase_motor(MotorRuntime *m, float current, float phase_deg) {
    if (m == NULL) {
        return;
    }

    m->current_command_a = foc_clampf(current, m->current_min_a, m->current_max_a);
    m->openloop_command_erpm = 0.0f;
    m->openloop_command_phase_u16 = foc_deg_to_u16(phase_deg);
    foc_enter_control_mode(m, MOTOR_CTRL_OPENLOOP_PHASE);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter erpm: kecepatan listrik rotor dalam electrical RPM.
// Fungsi mcpwm_foc_set_openloop_duty_motor: mengatur mcpwm foc set openloop duty motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty_motor(MotorRuntime *m, float duty, float erpm) {
    if (m == NULL) {
        return;
    }

    m->duty_command = foc_clamp_duty_command(m, duty);
    m->duty_command_q15 = (int32_t)lrintf(m->duty_command * 32768.0f);
    m->openloop_command_erpm = foc_clampf(erpm, m->min_erpm, m->max_erpm);
    foc_enter_control_mode(m, MOTOR_CTRL_OPENLOOP_DUTY);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter phase_deg: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_duty_phase_motor: mengatur mcpwm foc set openloop duty phase motor setelah
// nilai masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty_phase_motor(MotorRuntime *m, float duty, float phase_deg) {
    if (m == NULL) {
        return;
    }

    m->duty_command = foc_clamp_duty_command(m, duty);
    m->duty_command_q15 = (int32_t)lrintf(m->duty_command * 32768.0f);
    m->openloop_command_erpm = 0.0f;
    m->openloop_command_phase_u16 = foc_deg_to_u16(phase_deg);
    /* Diagnostik terbatas: aktifkan command agar mc_interface_mc_timer_isr
     * menyalakan MOE (wants_pwm butuh command_active). Keamanan tetap dijaga
     * oleh clamp duty 10–90% di SVPWM dan trip arus absolut. */
    motor_touch_command(m);
    foc_enter_control_mode(m, MOTOR_CTRL_OPENLOOP_DUTY_PHASE);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_release_motor_motor: menjalankan bagian mcpwm foc release motor motor pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void mcpwm_foc_release_motor_motor(MotorRuntime *m) {
    if (m == NULL) {
        return;
    }

    foc_enter_control_mode(m, MOTOR_CTRL_OFF);
    motor_set_foc_targets(m, 0.0f, 0.0f);
    m->detect_force_angle = false;
    m->phase_observer_override = false;
    m->openloop_started = false;
    m->openloop_command_erpm = 0.0f;
    m->encoder.sync_active = false;
    motor_hw_set_pwm_enabled(m, false);
}

/* Apply the same observer-delay idea used by VESC without putting floating
 * point into the 16-kHz path. pll_erpm_q16 is Q16.16 ERPM and the
 * configurable (0.5 + observer_offset) factor is precomputed in signed Q15.
 * One electrical revolution is 65536 phase counts. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi observer_phase_compensated_u16: memperbarui observer phase compensated u16 untuk memperkirakan fase
// atau kecepatan rotor pada operasi sensorless.
static inline uint16_t observer_phase_compensated_u16(const MotorRuntime *m) {
    if (m == NULL)
        return 0U;
    // Variabel num: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t num = (int64_t)m->pll_erpm_q16 *
                        (int64_t)m->observer_offset_factor_q15;
    // Variabel den: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t den = 60LL * (int64_t)FOC_ISR_EVENT_HZ * 32768LL;
    // Variabel adv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t adv = (den != 0LL) ? (num / den) : 0LL;
    if (adv > 32767LL)
        adv = 32767LL;
    if (adv < -32768LL)
        adv = -32768LL;
    return (uint16_t)(m->observer_phase_u16 + (int16_t)adv);
}

/* Current VESC ENCODER_AB uses a 5% hysteresis around foc_sl_erpm to choose
 * the encoder below the threshold and the observer above it. Keep that source
 * decision in the hard loop so it is based on the same fast corrected speed
 * estimator as phase control, while the slower task only performs the ABI
 * counter rebase when observer mode is active. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi encoder_ab_update_source_isr: menangani encoder ab update source isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
static inline void encoder_ab_update_source_isr(MotorRuntime *m) {
    if (m == NULL || m->id != MOTOR_LEFT || !m->encoder.synced ||
        (m->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER_AB &&
         m->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER))
         return;

    // Variabel sw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t sw = m->foc_sl_erpm_q16;
    if (sw <= 0 || !m->observer_valid) {
        m->using_encoder = true;
        return;
    }
    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t h = sw / 20; /* 5 percent */
    // Variabel speed_abs: nilai kecepatan untuk target atau pengukuran.
    const int32_t speed_abs = abs_i32_sat_local(m->speed_est_fast_erpm_q16);
    if (m->using_encoder) {
        if (speed_abs > (sw + h))
            m->using_encoder = false;
    }
    else if (speed_abs < (sw - h)) {
        m->using_encoder = true;
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter target: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi hall_rate_limit_phase_isr: menangani hall rate limit phase isr pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
static inline uint16_t hall_rate_limit_phase_isr(MotorRuntime *m, uint16_t target) {
    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    hall_state_t *h = &m->hall;
    // Variabel frame: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t frame = s_adc_isr_count;
    if (!h->rate_limited_valid) {
        h->rate_limited_phase_u16 = target;
        h->rate_limited_valid = true;
        h->rate_limit_frame = frame;
        return target;
    }
    if (h->rate_limit_frame == frame)
        return h->rate_limited_phase_u16;
    h->rate_limit_frame = frame;

    // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
    uint32_t erpm = m->foc_hall_interp_erpm_u32;
    if (h->period_cycles > 0U) {
        // Variabel measured: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint64_t measured = ((uint64_t)CPU_CLOCK_HZ * 10ULL) / (uint64_t)h->period_cycles;
        if (measured > erpm)
            erpm = measured > UINT32_MAX ? UINT32_MAX : (uint32_t)measured;
    }
    /* VESC Hall rate limit: 1.5 * electrical speed * dt. In u16/rev units:
       step = ERPM * 65536/60/Fs * 1.5. */
    // Variabel num: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t num = (uint64_t)erpm * 98304ULL;
    // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t step = (uint32_t)((num + (60ULL * FOC_ISR_EVENT_HZ - 1ULL)) /
                               (60ULL * FOC_ISR_EVENT_HZ));
    if (step < 1U)
        step = 1U;
    if (step > 32767U)
        step = 32767U;
    // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t diff = (int16_t)(target - h->rate_limited_phase_u16);
    if (diff > (int32_t)step)
        diff = (int32_t)step;
    if (diff < -(int32_t)step)
        diff = -(int32_t)step;
    h->rate_limited_phase_u16 = (uint16_t)(h->rate_limited_phase_u16 + diff);
    return h->rate_limited_phase_u16;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_sensor_electrical_phase_u16: menjalankan operasi motor sensor electrical phase u16 sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m) {
    if (m->detect_force_angle) {
        return m->detect_phase_u16;
    }
    if (m->control_mode == MOTOR_CTRL_OPENLOOP ||
        m->control_mode == MOTOR_CTRL_OPENLOOP_PHASE ||
        m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY ||
        m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE) {
        return m->openloop_command_phase_u16;
    }
    if (m->phase_observer_override) {
        return m->phase_observer_override_u16;
    }

    if (m->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
        return observer_phase_compensated_u16(m);
    }

    if (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB ||
        m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) {
        /* ENCODER_AB follows VESC's hybrid idea: before first sync the phase
           comes from open-loop/observer override. After sync use encoder at
           low speed, and the predicted observer phase above foc_sl_erpm. */
        if (!m->encoder.synced) {
            /* Upstream FOC_SENSOR_MODE_ENCODER_AB: before software sync use
               observer (or the open-loop observer override handled above),
               never the arbitrary cold-boot ABI counter origin. */
            return observer_phase_compensated_u16(m);
        }
        if (m->observer_valid && !m->using_encoder) {
            // Variabel cyc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t cyc = DWT->CYCCNT - m->observer_update_cycle;
            /* First compensate the fixed PWM/ADC sample delay using the VESC
               observer-offset factor, then extrapolate only the few CPU cycles
               elapsed since the observer update in this ISR. */
            // Variabel compensated: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint16_t compensated = observer_phase_compensated_u16(m);
            // Variabel adv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int32_t adv = (int32_t)(((int64_t)m->pll_erpm_q16 * (int64_t)cyc) /
                                    ((int64_t)CPU_CLOCK_HZ * 60LL));
            return (uint16_t)(compensated + adv);
        }
        /* After software sync, the ABI counter is the low-speed source. */
        // Variabel ext: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t ext = m->encoder.turns * (int32_t)m->encoder.cpr + (int32_t)motor_hw_encoder_cnt();
        // Variabel rel: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t rel = ext - m->encoder.session_zero_count;
        // Variabel phase64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int64_t phase64 = ((int64_t)rel * (int64_t)m->encoder.phase_per_count_q16) >> 16;
        // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t p = (uint16_t)phase64;
        if (m->encoder.inverted)
            p = (uint16_t)(0U-p);
        /* Match VESC mcpwm_foc: encoder electrical phase is
           signed(mechanical * ratio) - foc_encoder_offset. */
        return (uint16_t)(p - m->encoder.elec_offset_u16);
    }

    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    hall_state_t *h = &m->hall;
    if (!h->valid)
        return m->hall_offset_u16;

    // Variabel base: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t base = h->base_phase_u16;
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    uint8_t raw = h->raw_state & 7U;
    if (m->foc_hall_table[raw] != 255U) {
        base = m->hall_angle_u16[raw];
    }

    /* VESC foc_hall_interp_erpm semantics: below the configured electrical
       speed threshold use the detected Hall phase directly; above it, advance
       continuously between edges. Convert edge period to ERPM without float:
       one Hall edge is 1/6 electrical revolution -> edge_hz * 10 = ERPM. */
    // Variabel hall_phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint16_t hall_phase = (uint16_t)((int32_t)base + (int32_t)m->hall_offset_u16);
    if (m->foc_hall_interp_erpm_u32 > 0U && h->period_cycles > 0U) {
        // Variabel erpm_num: kecepatan listrik motor dalam electrical RPM.
        const uint64_t erpm_num = (uint64_t)CPU_CLOCK_HZ * 10ULL;
        // Variabel threshold_num: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint64_t threshold_num = (uint64_t)m->foc_hall_interp_erpm_u32 * (uint64_t)h->period_cycles;
        if (erpm_num >= threshold_num) {
            // Variabel elapsed: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t elapsed = DWT->CYCCNT - h->edge_cycle;
            // Variabel advance: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int64_t advance = ((int64_t)(int32_t)elapsed * (int64_t)h->phase_per_cycle_q16) >> 16;
            // Variabel sector_phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
            const int32_t sector_phase = 65536 / 6;
            if (advance > sector_phase)
                advance = sector_phase;
            if (advance < -sector_phase)
                advance = -sector_phase;
            hall_phase = (uint16_t)((int32_t)base + (int32_t)advance +
                                    (int32_t)m->hall_offset_u16);
        }
    }

    hall_phase = hall_rate_limit_phase_isr(m, hall_phase);

    /* VESC Hall mode transitions smoothly to the observer between
       foc_sl_erpm_start and foc_sl_erpm. Do the same with integer phase math
       so Hall remains the low-speed source while the observer carries the
       high-speed phase without adding float to the 16-kHz path. */
    if (m->observer_valid && m->foc_sl_erpm_q16 > m->foc_sl_erpm_start_q16) {
        // Variabel speed_abs: nilai kecepatan untuk target atau pengukuran.
        const int32_t speed_abs = abs_i32_sat_local(m->speed_est_fast_erpm_q16);
        // Variabel obs: fase observer yang sudah dikompensasi delay sampling PWM/ADC.
        const uint16_t obs = observer_phase_compensated_u16(m);
        // Variabel phase_error: selisih fase signed terpendek antara observer dan Hall.
        const int16_t phase_error = (int16_t)(obs - hall_phase);
        // Variabel phase_error_abs: besar selisih fase untuk memvalidasi koherensi sebelum transisi sumber sudut.
        const int32_t phase_error_abs = phase_error < 0 ? -(int32_t)phase_error : (int32_t)phase_error;

        /* Jangan pernah meninggalkan Hall bila observer belum koheren. Ini
           mencegah loncatan sudut pada motor yang R/L/flux-nya belum tepat;
           Hall tetap bisa menjalankan motor sampai observer benar-benar sejajar. */
        if (phase_error_abs <= 10923) { /* 60 electrical degrees */
            if (speed_abs >= m->foc_sl_erpm_q16)
                return obs;

            if (speed_abs > m->foc_sl_erpm_start_q16) {
                const int32_t span = m->foc_sl_erpm_q16 - m->foc_sl_erpm_start_q16;
                const int32_t pos = speed_abs - m->foc_sl_erpm_start_q16;
                const int32_t t_q15 = (int32_t)(((int64_t)pos * 32768LL) / span);
                return (uint16_t)(hall_phase +
                                  (int16_t)(((int32_t)phase_error * t_q15) >> 15));
            }
        }
    }
    return hall_phase;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi hall_update_raw_fast: memperbarui hall update raw fast menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
static void hall_update_raw_fast(MotorRuntime *m, uint8_t raw) {
    if (m == NULL || m->sensor_mode != SENSOR_MODE_HALL)
        return;

    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    hall_state_t *h = &m->hall;
    raw &= 7U;
    h->raw_state = raw;
    // Variabel sector: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int8_t sector = m->hall_table[raw];

    /* Hall 000/111 or an unmapped table entry is not a usable rotor phase.
       Keep a very short fast-loop validation counter rather than a millisecond
       debounce, because phase information must not be delayed. */
    if (raw == 0U || raw == 7U || sector < 0) {
        if (h->invalid_count < UINT16_MAX)
            h->invalid_count++;
        h->valid = false;
        h->rate_limited_valid = false;
        return;
    }
    h->invalid_count = 0U;

    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t now = DWT->CYCCNT;
    // Variabel old: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int8_t old = h->sector;
    if (old < 0) {
        h->sector = sector;
        h->direction = (h->direction == 0) ? 1 : h->direction;
        h->base_phase_u16 = m->hall_angle_u16[raw];
        h->edge_cycle = now;
        h->phase_per_cycle_q16 = 0;
        h->valid = true;
        return;
    }

    /* Polling happens on every FOC update. Re-reading the same sector must
       NOT refresh edge_cycle; otherwise interpolation speed collapses to zero. */
    if (sector == old) {
        /* A stable valid sector proves that a previous non-neighbour glitch is
           gone. Do not leave sequence_error_count latched forever, otherwise
           the next PWM enable immediately recreates HALL_INVALID. */
        h->sequence_error_count = 0U;
        h->valid = true;
        return;
    }

    // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int8_t diff = (int8_t)(sector - old);
    // Variabel dir: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int8_t dir = h->direction;
    // Variabel neighbor: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool neighbor = false;
    if (diff == 1 || diff == -5) {
        dir = 1;
        neighbor = true;
    }
    else if (diff == -1 || diff == 5) {
        dir = -1;
        neighbor = true;
    }

    if (!neighbor) {
        if (h->sequence_error_count < UINT16_MAX)
            h->sequence_error_count++;
        h->valid = false;
        return;
    }

    h->sequence_error_count = 0U;
    // Variabel period: periode pengukuran atau eksekusi modul.
    uint32_t period = now - h->edge_cycle;
    if (period > 100U) {
        h->period_cycles = period;
        // Variabel step: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int64_t step = (((int64_t)(65536 / 6)) << 16) / (int64_t)period;
        h->phase_per_cycle_q16 = (int32_t)(dir > 0 ? step : -step);
    }
    h->edge_count += dir;
    h->direction = dir;
    h->sector = sector;
    h->base_phase_u16 = m->hall_angle_u16[raw];
    h->edge_cycle = now;
    h->valid = true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hall_edge_isr: menangani motor hall edge isr pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
void motor_hall_edge_isr(MotorRuntime *m) {
    if (m == NULL || m->sensor_mode != SENSOR_MODE_HALL)
        return;
    hall_update_raw_fast(m, motor_hw_read_hall_raw(m->id));
}

// Parameter raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter scale_q16: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi adc_current_to_q15: menjalankan operasi adc current to q15 sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static inline int32_t adc_current_to_q15(uint16_t raw, int32_t offset, int32_t scale_q16) {
    /* Stock hoverboard analog front-end polarity (EFeru): phase/DC current is
       offset - ADC. Using raw-offset turns the current PI into positive
       feedback on this PCB and can produce the growl/current spike seen during
       forced-angle detection. */
    return (int32_t)(((int64_t)(offset - (int32_t)raw) * (int64_t)scale_q16) >> 16);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter fault: status atau data gangguan yang digunakan sistem proteksi.
// Parameter raw_u: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter raw_v: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter raw_dc: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter ia: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ib: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ic: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter trip: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi capture_current_fault_snapshot: menangani capture current fault snapshot dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
static void capture_current_fault_snapshot(MotorRuntime *m, motor_fault_t fault,
                                           uint16_t raw_u, uint16_t raw_v, uint16_t raw_dc,
                                           int32_t ia, int32_t ib, int32_t ic, int32_t trip) {
    if (s_fault_snapshot.valid)
        return; /* Pertahankan sampel akar penyebab pertama. */
    s_fault_snapshot.valid = 1U;
    s_fault_snapshot.motor = (uint8_t)m->id;
    s_fault_snapshot.fault = (uint8_t)fault;
    s_fault_snapshot.cal_stage = (uint8_t)s_cal_stage;
    s_fault_snapshot.raw_u = raw_u;
    s_fault_snapshot.raw_v = raw_v;
    s_fault_snapshot.raw_dc = raw_dc;
    s_fault_snapshot.offset_u = m->current_offset_u_counts;
    s_fault_snapshot.offset_v = m->current_offset_v_counts;
    s_fault_snapshot.offset_dc = m->dc_current_offset_counts;
    s_fault_snapshot.ia_q15 = ia;
    s_fault_snapshot.ib_q15 = ib;
    s_fault_snapshot.ic_q15 = ic;
    s_fault_snapshot.trip_q15 = trip;
    s_fault_snapshot.id_target_q15 = m->id_target_q15;
    s_fault_snapshot.iq_target_q15 = m->iq_target_q15;
    s_fault_snapshot.ccr1 = (uint16_t)m->pwm_tim->CCR1;
    s_fault_snapshot.ccr2 = (uint16_t)m->pwm_tim->CCR2;
    s_fault_snapshot.ccr3 = (uint16_t)m->pwm_tim->CCR3;
    s_fault_snapshot.tim_cnt = (uint16_t)m->pwm_tim->CNT;
    s_fault_snapshot.dma_cndtr = (uint16_t)DMA1_Channel1->CNDTR;
    s_fault_snapshot.adc_isr_count = s_adc_isr_count;
    s_fault_snapshot.blank_cycles = m->pwm_enable_blank_cycles;
    s_fault_snapshot.pwm_enabled = m->pwm_enabled ? 1U : 0U;
    s_fault_snapshot.moe = (m->pwm_tim->BDTR & TIM_BDTR_MOE) ? 1U : 0U;
    s_fault_snapshot.pending_events = m->pwm_enable_pending_events;
    s_fault_snapshot.reserved = 0U;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter id_q15: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Parameter iq_q15: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi detect_inductance_capture_isr: menangani detect inductance capture isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
static inline void detect_inductance_capture_isr(MotorRuntime *m, int32_t id_q15, int32_t iq_q15) {
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_detect_state_t *d = &m->detect;
    if (!d->l_capture_active)
        return;

    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    uint16_t n = d->l_capture_count;
    if (n >= FOC_DETECT_L_CAPTURE_MAX) {
        d->l_capture_active = false;
        d->l_capture_done = true;
        return;
    }

    /* vd/vq still belong to the preceding PWM interval. Select the axis in
       task context before arming capture so the 16-kHz ISR only stores the
       current sample and its causally preceding applied voltage. */
    if (d->l_capture_axis == 0U) {
        d->l_capture_i_q15[n] = id_q15;
        d->l_capture_v_prev_q15[n] = m->vd_q15;
    }
    else {
        d->l_capture_i_q15[n] = iq_q15;
        d->l_capture_v_prev_q15[n] = m->vq_q15;
    }
    n++;
    d->l_capture_count = n;
    if (n >= FOC_DETECT_L_CAPTURE_MAX) {
        d->l_capture_active = false;
        d->l_capture_done = true;
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi snapshot_encoder_phase_u16: menyiapkan snapshot encoder phase u16 secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
static inline uint16_t snapshot_encoder_phase_u16(const MotorRuntime *m) {
    if (m == NULL || m->id != MOTOR_LEFT || !m->encoder.synced || m->encoder.cpr < 4U)
        return 0U;
    // Variabel ext: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ext = m->encoder.turns * (int32_t)m->encoder.cpr + (int32_t)motor_hw_encoder_cnt();
    // Variabel rel: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t rel = ext - m->encoder.session_zero_count;
    // Variabel phase64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t phase64 = ((int64_t)rel * (int64_t)m->encoder.phase_per_count_q16) >> 16;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t p = (uint16_t)phase64;
    if (m->encoder.inverted)
        p = (uint16_t)(0U - p);
    return (uint16_t)(p - m->encoder.elec_offset_u16);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter phase_control: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi publish_rt_snapshot_isr: menangani publish rt snapshot isr pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
static inline void publish_rt_snapshot_isr(MotorRuntime *m, uint16_t phase_control) {
    // Variabel seq: nomor urut untuk konsistensi snapshot atau record.
    uint32_t seq = m->rt_snapshot_seq + 1U;
    if ((seq & 1U) == 0U)
        seq++;
    m->rt_snapshot_seq = seq;
    __DMB();
    m->rt_snapshot.ia_q15 = m->ia_q15;
    m->rt_snapshot.ib_q15 = m->ib_q15;
    m->rt_snapshot.ic_q15 = m->ic_q15;
    m->rt_snapshot.id_q15 = m->id_q15;
    m->rt_snapshot.iq_q15 = m->iq_q15;
    m->rt_snapshot.id_filter_q15 = m->id_filter_q15;
    m->rt_snapshot.iq_filter_q15 = m->iq_filter_q15;
    m->rt_snapshot.id_target_q15 = m->id_target_q15;
    m->rt_snapshot.iq_target_q15 = m->iq_target_q15;
    m->rt_snapshot.vd_q15 = m->vd_q15;
    m->rt_snapshot.vq_q15 = m->vq_q15;
    m->rt_snapshot.vbus_q15 = m->vbus_q15;
    m->rt_snapshot.dc_current_q15 = m->dc_current_q15;
    m->rt_snapshot.erpm_fast_q16 = m->speed_est_fast_erpm_q16;
    m->rt_snapshot.duty_u_q15 = m->duty_u_q15;
    m->rt_snapshot.duty_v_q15 = m->duty_v_q15;
    m->rt_snapshot.duty_w_q15 = m->duty_w_q15;
    m->rt_snapshot.phase_control_u16 = phase_control;
    m->rt_snapshot.phase_observer_u16 = m->observer_phase_u16;
    m->rt_snapshot.phase_encoder_u16 = snapshot_encoder_phase_u16(m);
    m->rt_snapshot.phase_hall_u16 = m->hall.valid ? m->hall.base_phase_u16 : m->hall_offset_u16;
    m->rt_snapshot.adc_frame = s_adc_isr_count;
    m->rt_snapshot.cycle_counter = DWT->CYCCNT;
    __DMB();
    m->rt_snapshot_seq = seq + 1U;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter raw_u: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter raw_v: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter raw_dc: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter vbus_q15: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Parameter inv_vbus_q30: tegangan DC bus untuk normalisasi PWM dan pemeriksaan batas tegangan.
// Fungsi foc_one_motor_isr: menangani foc one motor isr pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
static void foc_one_motor_isr(MotorRuntime *m, uint16_t raw_u, uint16_t raw_v, uint16_t raw_dc,
                              int32_t vbus_q15, int32_t inv_vbus_q30) {
    m->current_raw_u = raw_u;
    m->current_raw_v = raw_v;
    // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ia, ib, ic;
    if (m->id == MOTOR_LEFT) {
        /* PCB LEFT shunts measure phase A and phase B. */
        ia = adc_current_to_q15(raw_u, m->current_offset_u_counts, m->current_scale_q16);
        ib = adc_current_to_q15(raw_v, m->current_offset_v_counts, m->current_scale_q16);
        ic = -(ia + ib);
    }
    else {
        /* PCB RIGHT shunts are phase B (PC4) and phase C (PC5), not A/B.
           Reconstruct A from Kirchhoff: ia + ib + ic = 0. */
        ib = adc_current_to_q15(raw_u, m->current_offset_u_counts, m->current_scale_q16);
        ic = adc_current_to_q15(raw_v, m->current_offset_v_counts, m->current_scale_q16);
        ia = -(ib + ic);
    }

    m->ia_q15 = ia;
    m->ib_q15 = ib;
    m->ic_q15 = ic;
    m->dc_current_raw = raw_dc;
    m->vbus_q15 = vbus_q15;
    m->dc_current_q15 = adc_current_to_q15(raw_dc, m->dc_current_offset_counts, m->dc_current_scale_q16);
    // Variabel mi: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t mi = (m->id == MOTOR_RIGHT) ? 1U : 0U;

    /* Hall GPIO is the fast source of truth. EXTI is only an optional early
       timestamp hint; polling here ensures phase validity is not dependent on
       an RTOS task or on edge-IRQ delivery. During forced-angle detection the
       detector reads Hall separately and Hall validity must not gate SVPWM. */
    if (m->foc_sensor_mode == FOC_SENSOR_MODE_HALL &&
        m->sensor_mode == SENSOR_MODE_HALL && !m->detect_force_angle) {
        hall_update_raw_fast(m, motor_hw_read_hall_raw(m->id));
        if (m->pwm_enabled && m->command_active &&
            (m->hall.invalid_count >= 32U || m->hall.sequence_error_count >= 4U)) {
            motor_request_fault_from_isr(m, MOTOR_FAULT_HALL_INVALID);
            return;
        }
    }

    // Variabel trip: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t trip = (m->abs_current_trip_q15 > 0) ? m->abs_current_trip_q15 : s_current_trip_q15;

    /* Hard bus-voltage limits must remain active even during the short PWM
       startup current-blanking window. Configured limits are evaluated later
       with debounce; these wider limits are first-sample protection. */
    // Variabel vbus_max_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    int32_t vbus_max_q15 = m->max_vin_q15 > 0 ? m->max_vin_q15 : s_vbus_max_q15;
    // Variabel vbus_min_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    int32_t vbus_min_q15 = m->min_vin_q15 > 0 ? m->min_vin_q15 : s_vbus_min_q15;
    // Variabel hard_max_q15: batas atau nilai maksimum untuk validasi dan proteksi.
    int32_t hard_max_q15 = m->hard_max_vin_q15 > 0 ? m->hard_max_vin_q15 : s_vbus_hard_max_q15;
    // Variabel hard_min_q15: batas atau nilai minimum untuk validasi dan proteksi.
    int32_t hard_min_q15 = m->hard_min_vin_q15 > 0 ? m->hard_min_vin_q15 : s_vbus_hard_min_q15;
    if (m->pwm_enabled && vbus_q15 > hard_max_q15) {
        motor_request_fault_from_isr(m, MOTOR_FAULT_OVER_VOLTAGE);
        return;
    }
    if (m->pwm_enabled && vbus_q15 < hard_min_q15) {
        motor_request_fault_from_isr(m, MOTOR_FAULT_UNDER_VOLTAGE);
        return;
    }

    // Variabel abs_phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    int32_t abs_phase = iabs32(ia);
    // Variabel tmp_abs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t tmp_abs = iabs32(ib);
    if (tmp_abs > abs_phase)
        abs_phase = tmp_abs;
    tmp_abs = iabs32(ic);
    if (tmp_abs > abs_phase)
        abs_phase = tmp_abs;
    if (abs_phase > m->abs_current_peak_q15)
        m->abs_current_peak_q15 = abs_phase;

    /* The first few PWM-synchronous samples after MOE is asserted are held at
       the exact 50% zero vector. Phase-current protection intentionally uses
       the proven startup DC-current guard during this brief window because the
       phase ADCs can contain switching-settling transients. Hard VIN above is
       never blanked. */
    if (m->pwm_enabled && m->pwm_enable_blank_cycles > 0U) {
        // Variabel dc_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t dc_q15 = m->dc_current_q15;
        if (iabs32(dc_q15) > s_startup_dc_trip_q15) {
            if (s_startup_dc_trip_consecutive[mi] < 255U)
                s_startup_dc_trip_consecutive[mi]++;
        }
        else {
            s_startup_dc_trip_consecutive[mi] = 0U;
        }
        if (s_startup_dc_trip_consecutive[mi] >= PWM_STARTUP_DC_TRIP_DEBOUNCE_SAMPLES) {
            capture_current_fault_snapshot(m, MOTOR_FAULT_ABS_OVER_CURRENT,
                                           raw_u, raw_v, raw_dc, ia, ib, ic, s_startup_dc_trip_q15);
            motor_request_fault_from_isr(m, MOTOR_FAULT_ABS_OVER_CURRENT);
            return;
        }
        m->pwm_enable_blank_cycles--;
        m->vd_int_q31 = 0;
        m->vq_int_q31 = 0;
        m->vd_int_q15 = 0;
        m->vq_int_q15 = 0;
        motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
        publish_rt_snapshot_isr(m, m->phase_before_speed_est_u16);
        return;
    }

    s_startup_dc_trip_consecutive[mi] = 0U;

    /* Preserve two current-protection layers. Outside startup blanking the
       physical board ceiling remains the hard upper threshold, but it is
       qualified for a few consecutive 16-kHz samples so a single switching
       transient cannot latch ABS_OVER_CURRENT. l_slow_abs_current only changes
       the configurable lower threshold and can never mask the board ceiling. */
    m->abs_phase_current_filter_q15 +=
        foc_q15_mul(FOC_ABS_CURRENT_FILTER_ALPHA_Q15,
                    abs_phase - m->abs_phase_current_filter_q15);

    /* Proteksi arus fase aktif pada operasi normal. Startup tetap memakai
       guard arus DC selama blanking singkat agar transien switching pertama
       tidak menimbulkan fault palsu. Setelah blanking, batas hardware dan
       batas MCCONF kembali menjadi lapisan proteksi utama seperti VESC. */
    if (m->pwm_enabled && abs_phase > s_current_trip_q15) {
        if (s_hard_current_trip_consecutive[mi] < 255U)
            s_hard_current_trip_consecutive[mi]++;
    }
    else {
        s_hard_current_trip_consecutive[mi] = 0U;
    }
    if (m->pwm_enabled &&
        s_hard_current_trip_consecutive[mi] >= FOC_HARD_CURRENT_FAULT_DEBOUNCE_SAMPLES) {
        capture_current_fault_snapshot(m, MOTOR_FAULT_ABS_OVER_CURRENT,
                                       raw_u, raw_v, raw_dc, ia, ib, ic, s_current_trip_q15);
        motor_request_fault_from_isr(m, MOTOR_FAULT_ABS_OVER_CURRENT);
        return;
    }
    if (m->pwm_enabled) {
        // Variabel configured_over: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        bool configured_over = m->slow_abs_current ?
            (m->abs_phase_current_filter_q15 > trip) : (abs_phase > trip);
        if (configured_over) {
            if (m->abs_current_fault_count < 255U)
                m->abs_current_fault_count++;
        }
        else {
            m->abs_current_fault_count = 0U;
        }
        // Variabel required: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint8_t required = m->slow_abs_current ? FOC_ABS_CURRENT_FAULT_DEBOUNCE_SAMPLES : FOC_HARD_CURRENT_FAULT_DEBOUNCE_SAMPLES;
        if (m->abs_current_fault_count >= required) {
            capture_current_fault_snapshot(m, MOTOR_FAULT_ABS_OVER_CURRENT,
                                           raw_u, raw_v, raw_dc, ia, ib, ic, trip);
            motor_request_fault_from_isr(m, MOTOR_FAULT_ABS_OVER_CURRENT);
            return;
        }
    }
    else {
        m->abs_current_fault_count = 0U;
        s_hard_current_trip_consecutive[mi] = 0U;
        s_startup_dc_trip_consecutive[mi] = 0U;
    }

    /* Configured VIN thresholds get a short consecutive-sample debounce to
       reject switching/ADC spikes. The wider hard envelope was already checked
       before startup blanking, so catastrophic OV/UV is never delayed. */
    if (m->pwm_enabled) {
        if (vbus_q15 > vbus_max_q15) {
            if (m->over_voltage_fault_count < 255U)
                m->over_voltage_fault_count++;
        }
        else {
            m->over_voltage_fault_count = 0U;
        }
        if (vbus_q15 < vbus_min_q15) {
            if (m->under_voltage_fault_count < 255U)
                m->under_voltage_fault_count++;
        }
        else {
            m->under_voltage_fault_count = 0U;
        }
        if (m->over_voltage_fault_count >= FOC_VBUS_FAULT_DEBOUNCE_SAMPLES) {
            motor_request_fault_from_isr(m, MOTOR_FAULT_OVER_VOLTAGE);
            return;
        }
        if (m->under_voltage_fault_count >= FOC_VBUS_FAULT_DEBOUNCE_SAMPLES) {
            motor_request_fault_from_isr(m, MOTOR_FAULT_UNDER_VOLTAGE);
            return;
        }
    }
    else {
        m->over_voltage_fault_count = 0U;
        m->under_voltage_fault_count = 0U;
    }

    if (!m->pwm_enabled || m->fault != MOTOR_FAULT_NONE || inv_vbus_q30 <= 0) {
        m->vd_int_q31 = 0;
        m->vq_int_q31 = 0;
        m->vd_int_q15 = 0;
        m->vq_int_q15 = 0;
        motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
        publish_rt_snapshot_isr(m, m->phase_before_speed_est_u16);
        return;
    }

    /* Eferu-style lightweight open-loop path: ADC/current/DC/VIN guards above
       have already run; do not execute observer, PLL, speed estimator, Park, or
       PI for a fixed-phase diagnostic pulse. */
    if (m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE) {
        // Variabel cs_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel sn_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t sn_ol, cs_ol;
        foc_fast_sincos_u16_q15(m->openloop_command_phase_u16, &sn_ol, &cs_ol);
        // Variabel vmax_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t vmax_ol = foc_q15_mul(vbus_q15, m->vmax_coeff_q15);
        if (vmax_ol < 256)
            vmax_ol = 256;
        // Variabel vd_ol: tegangan sumbu-d keluaran regulator FOC.
        int32_t vd_ol = foc_q15_mul(vmax_ol, m->duty_command_q15);
        // Variabel va_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t va_ol = foc_q15_mul(cs_ol, vd_ol);
        // Variabel vb_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t vb_ol = foc_q15_mul(sn_ol, vd_ol);
        // Variabel du_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel dv_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel dw_ol: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t du_ol, dv_ol, dw_ol;
        foc_svm_q15(va_ol, vb_ol, inv_vbus_q30, &du_ol, &dv_ol, &dw_ol);
        m->vd_q15 = vd_ol;
        m->vq_q15 = 0;
        m->duty_u_q15 = du_ol;
        m->duty_v_q15 = dv_ol;
        m->duty_w_q15 = dw_ol;
        motor_hw_set_pwm_q15(m, du_ol, dv_ol, dw_ol);
        publish_rt_snapshot_isr(m, m->openloop_command_phase_u16);
        return;
    }

    // Variabel i_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t i_alpha = ia;
    // Variabel i_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t i_beta = foc_q15_mul(ia + (ib * 2), FOC_Q15_INV_SQRT3);

    /* Forced Hall/encoder detection owns the electrical angle directly and
       only needs Clarke/Park + current PI + inverse Park/SVPWM. Observer, PLL,
       encoder source update, and fast speed estimation are not part of the
       measurement and previously pushed Hall detect above the 4000-cycle slot. */
    // Variabel direct_openloop_phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    const bool direct_openloop_phase =
        m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE;
    // Variabel forced_detect_phase: true saat deteksi memakai sudut listrik yang dipaksa task-side.
    const bool forced_detect_phase = m->detect_force_angle && m->detect.busy;
    // Variabel fast_phase_only: jalur fase eksplisit yang tidak memerlukan observer/PLL.
    const bool fast_phase_only = direct_openloop_phase || forced_detect_phase;
    if (!fast_phase_only) {
        /* VESC keeps the flux observer in the fast FOC path during normal FOC. */
        foc_observer_update_fixed(m, m->observer_v_alpha_q15_prev,
                            m->observer_v_beta_q15_prev, i_alpha, i_beta);
        encoder_ab_update_source_isr(m);
    }

    // Variabel phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint16_t phase = direct_openloop_phase ? m->openloop_command_phase_u16 :
                                             motor_sensor_electrical_phase_u16(m);
    // Variabel speed_phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    uint16_t speed_phase = phase;
    if (!fast_phase_only &&
        m->foc_speed_source == FOC_SPEED_SRC_OBSERVER && m->observer_valid) {
        speed_phase = observer_phase_compensated_u16(m);
    }

    /* Current VESC can source PLL/fast speed from corrected/control phase or
       directly from observer phase. The default remains corrected. */
    if (!fast_phase_only)
        foc_pll_run_fixed(m, speed_phase);

    /* Also retain a corrected-phase fast estimate independent of the selected
       source for diagnostics and sensor-transition plausibility. */
    if (!fast_phase_only && m->speed_est_corrected_valid) {
        // Variabel dc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t dc = (int16_t)(phase - m->phase_before_speed_est_corrected_u16);
        // Variabel md: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t md = 65536 / 6;
        if (dc > md)
            dc = md;
        if (dc < -md)
            dc = -md;
        // Variabel inst: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int64_t inst = (int64_t)dc * (int64_t)FOC_ISR_EVENT_HZ * 60LL;
        if (inst > INT32_MAX)
            inst = INT32_MAX;
        if (inst < INT32_MIN)
            inst = INT32_MIN;
        m->speed_est_fast_corrected_erpm_q16 += (int32_t)(((int64_t)((int32_t)inst - m->speed_est_fast_corrected_erpm_q16) * 328) >> 15);
    }
    else if (!fast_phase_only) {
        m->speed_est_corrected_valid = true;
        m->speed_est_fast_corrected_erpm_q16 = 0;
    }
    if (!fast_phase_only)
        m->phase_before_speed_est_corrected_u16 = phase;

    /* VESC low-latency speed estimators are based on phase delta in the fast
       FOC loop. Keep the same behavior without floating point: for a phase
       delta in u16/rev units, ERPM_Q16 = delta * Fs * 60 exactly because the
       65536 angle denominator cancels the Q16.16 scale factor. Clamp delta to
       +/-60 electrical degrees as upstream does before low-pass filtering. */
    if (!fast_phase_only && m->speed_est_phase_valid) {
        // Variabel dphase: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t dphase = (int16_t)(speed_phase - m->phase_before_speed_est_u16);
        // Variabel max_dphase: batas atau nilai maksimum untuk validasi dan proteksi.
        const int32_t max_dphase = (65536 / 6);
        if (dphase > max_dphase)
            dphase = max_dphase;
        if (dphase < -max_dphase)
            dphase = -max_dphase;
        // Variabel inst64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int64_t inst64 = (int64_t)dphase * (int64_t)FOC_ISR_EVENT_HZ * 60LL;
        if (inst64 > INT32_MAX)
            inst64 = INT32_MAX;
        if (inst64 < INT32_MIN)
            inst64 = INT32_MIN;
        // Variabel inst_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t inst_q16 = (int32_t)inst64;
        /* Upstream uses LP gains 0.01 (fast) and 0.2 (faster). */
        // Variabel alpha_fast_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t alpha_fast_q15 = 328; /* round(0.01 * 32768) */
        // Variabel alpha_faster_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int32_t alpha_faster_q15 = 6554; /* round(0.2 * 32768) */
        m->speed_est_fast_erpm_q16 += (int32_t)(((int64_t)(inst_q16 - m->speed_est_fast_erpm_q16) * alpha_fast_q15) >> 15);
        m->speed_est_faster_erpm_q16 += (int32_t)(((int64_t)(inst_q16 - m->speed_est_faster_erpm_q16) * alpha_faster_q15) >> 15);
    }
    else if (!fast_phase_only) {
        m->speed_est_phase_valid = true;
        m->speed_est_fast_erpm_q16 = 0;
        m->speed_est_faster_erpm_q16 = 0;
    }
    if (!fast_phase_only)
        m->phase_before_speed_est_u16 = speed_phase;

    if (!fast_phase_only) {
        // Variabel fast_abs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int64_t fast_abs = (int64_t)abs_i32_sat_local(m->speed_est_fast_erpm_q16);
        // Variabel pll_cap: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int64_t pll_cap = fast_abs * 3LL;
        // Variabel board_cap: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const int64_t board_cap = (int64_t)((int32_t)MOTOR_DEFAULT_MAX_ERPM) * 65536LL;
        if (pll_cap > board_cap)
            pll_cap = board_cap;
        if ((int64_t)m->pll_erpm_q16 > pll_cap)
            m->pll_erpm_q16 = (int32_t)pll_cap;
        if ((int64_t)m->pll_erpm_q16 < -pll_cap)
            m->pll_erpm_q16 = (int32_t)(-pll_cap);
    }


    // Variabel cs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel sn: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sn, cs;
    foc_fast_sincos_u16_q15(phase, &sn, &cs);

    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    int32_t id = foc_q15_mul(cs, i_alpha) + foc_q15_mul(sn, i_beta);
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t iq = foc_q15_mul(cs, i_beta) - foc_q15_mul(sn, i_alpha);
    m->id_q15 = id;
    m->iq_q15 = iq;
    detect_inductance_capture_isr(m, id, iq);

    /* VESC-style current filter. The coefficient comes from MCCONF but is
       precomputed to Q15 in task context; raw id/iq remain PI feedback. */
    // Variabel filter_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t filter_q15 = m->foc_current_filter_q15;
    m->id_filter_q15 += foc_q15_mul(filter_q15, id - m->id_filter_q15);
    m->iq_filter_q15 += foc_q15_mul(filter_q15, iq - m->iq_filter_q15);

    // Variabel id_target_eff: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    int32_t id_target_eff = m->id_target_q15;
    // Variabel iq_target_eff: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int32_t iq_target_eff = m->iq_target_q15;
    /* Forced-angle detection tidak memerlukan field-weakening/MTPA fast path.
       Target Id/Iq berasal langsung dari worker deteksi untuk menjaga ISR ringan. */
    if (!forced_detect_phase)
        foc_apply_fast_fw_targets_isr(m, &id_target_eff, &iq_target_eff);
    // Variabel err_d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t err_d = id_target_eff - id;
    // Variabel err_q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t err_q = iq_target_eff - iq;
    // Variabel vmax_coeff_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vmax_coeff_q15 = m->vmax_coeff_q15;
    if (vmax_coeff_q15 <= 0) {
        /* Defensive boot fallback only; normal startup/config apply calls
           foc_precalc_values() before PWM can be enabled. */
        vmax_coeff_q15 = (int32_t)(0.8f * 0.5773502691896258f * 32768.0f);
    }
    // Variabel vmax_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vmax_q15 = foc_q15_mul(vbus_q15, vmax_coeff_q15);
    if (vmax_q15 < 256)
        vmax_q15 = 256;

    // Variabel direct_duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    bool direct_duty = (m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY ||
                        m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE);
    // Variabel vd: tegangan sumbu-d keluaran regulator FOC.
    // Variabel vq: tegangan sumbu-q keluaran regulator FOC.
    int32_t vd = 0, vq = 0;
    if (direct_duty) {
        /* Explicit open-loop duty stays fully fixed-point in the 16-kHz ISR.
           Rotating mode uses q-axis voltage; fixed-phase mode uses d-axis. */
        // Variabel direct: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t direct = foc_q15_mul(vmax_q15, m->duty_command_q15);
        if (m->control_mode == MOTOR_CTRL_OPENLOOP_DUTY_PHASE)
            vd = direct;
        else vq = direct;
        m->vd_int_q31 = m->vq_int_q31 = 0;
        m->vd_int_q15 = m->vq_int_q15 = 0;
    }
    else {
    /* Q15 error x Q16.16 Ki*dt produces a Q31 increment. Temperature
       compensation scales Ki (current_ki_temp_comp) so the integrator tracks
       copper drift; fall back to the untouched value when comp is disabled.
       The Ki*dt Q16 conversion mirrors motor_set_current_pi_gains(). */
    // Variabel ki_dt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    /* current_ki_dt_q16 sudah dihitung task-side, termasuk temperature
       compensation. Hard ISR tidak melakukan float multiply/conversion. */
    const int32_t ki_dt = m->current_ki_dt_q16;
    m->vd_int_q31 += (int64_t)err_d * (int64_t)ki_dt;
    m->vq_int_q31 += (int64_t)err_q * (int64_t)ki_dt;
    // Variabel int_lim_q31: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t int_lim_q31 = (int64_t)vmax_q15 << 16;
    if (m->vd_int_q31 > int_lim_q31)
        m->vd_int_q31 = int_lim_q31;
    if (m->vd_int_q31 < -int_lim_q31)
        m->vd_int_q31 = -int_lim_q31;
    if (m->vq_int_q31 > int_lim_q31)
        m->vq_int_q31 = int_lim_q31;
    if (m->vq_int_q31 < -int_lim_q31)
        m->vq_int_q31 = -int_lim_q31;
    m->vd_int_q15 = (int32_t)(m->vd_int_q31 >> 16);
    m->vq_int_q15 = (int32_t)(m->vq_int_q31 >> 16);

    // Variabel prop_d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t prop_d = q15_mul_q16(err_d, m->current_kp_q16);
    // Variabel prop_q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t prop_q = q15_mul_q16(err_q, m->current_kp_q16);
    vd = m->vd_int_q15 + prop_d;
    vq = m->vq_int_q15 + prop_q;

    /* VESC dq decoupling in fixed point.
       vd -= w_e * iq * Lq
       vq += w_e * id * Ld + w_e * flux
       speed_est_fast is Q16.16 ERPM; use its integer part here so all
       arithmetic stays bounded int64 and deterministic in the hard ISR. */
    // Variabel decouple_allowed: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool decouple_allowed = (m->control_mode == MOTOR_CTRL_CURRENT ||
                                   m->control_mode == MOTOR_CTRL_BRAKE_CURRENT ||
                                   m->control_mode == MOTOR_CTRL_SPEED ||
                                   m->control_mode == MOTOR_CTRL_POSITION ||
                                   m->control_mode == MOTOR_CTRL_DUTY);
    if (!fast_phase_only && decouple_allowed &&
        m->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED) {
        // Variabel erpm_fast: kecepatan listrik motor dalam electrical RPM.
        const int32_t erpm_fast = m->speed_est_fast_erpm_q16 >> 16;
        // Variabel dec_vd: tegangan sumbu-d keluaran regulator FOC.
        int32_t dec_vd = 0;
        // Variabel dec_vq: tegangan sumbu-q keluaran regulator FOC.
        int32_t dec_vq = 0;
        // Variabel dec_bemf: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        int32_t dec_bemf = 0;
        if (m->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS ||
            m->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
            // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int64_t a = (int64_t)m->iq_filter_q15 * (int64_t)erpm_fast *
                        (int64_t)m->decouple_lq_coeff_q30;
            // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int64_t b = (int64_t)m->id_filter_q15 * (int64_t)erpm_fast *
                        (int64_t)m->decouple_ld_coeff_q30;
            dec_vd = (int32_t)(a >> 30);
            dec_vq = (int32_t)(b >> 30);
        }
        if (m->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
            m->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
            dec_bemf = (int32_t)(((int64_t)erpm_fast *
                                  (int64_t)m->bemf_flux_coeff_q30) >> 15);
        }
        vd -= dec_vd;
        vq += dec_vq + dec_bemf;
    }

    /* VESC-style voltage saturation with d-axis priority. This is important
       for field weakening: Vd is clamped first, then Vq gets the remaining
       circle. Saturation error is fed back into the Q31 PI integrators. */
    // Variabel vd_axis_factor_q15: tegangan sumbu-d keluaran regulator FOC.
    int32_t vd_axis_factor_q15 = m->foc_mag_vd_max_q15;
    if (vd_axis_factor_q15 <= 0 || vd_axis_factor_q15 > 32767)
        vd_axis_factor_q15 = 32767;
    // Variabel vd_lim: tegangan sumbu-d keluaran regulator FOC.
    int32_t vd_lim = foc_q15_mul(vmax_q15, vd_axis_factor_q15);
    if (vd_lim < 1)
        vd_lim = 1;

    // Variabel vd_presat: tegangan sumbu-d keluaran regulator FOC.
    const int32_t vd_presat = vd;
    vd = clamp_i32(vd, -vd_lim, vd_lim);
    if (vd != vd_presat) {
        m->vd_int_q31 += (int64_t)(vd - vd_presat) * 65536LL;
    }

    // Variabel rem64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t rem64 = (int64_t)vmax_q15 * (int64_t)vmax_q15 -
                    (int64_t)vd * (int64_t)vd;
    if (rem64 < 0)
        rem64 = 0;
    if (rem64 > UINT32_MAX)
        rem64 = UINT32_MAX;
    // Variabel vq_lim: tegangan sumbu-q keluaran regulator FOC.
    const int32_t vq_lim = (int32_t)isqrt_u32((uint32_t)rem64);
    // Variabel vq_presat: tegangan sumbu-q keluaran regulator FOC.
    const int32_t vq_presat = vq;
    vq = clamp_i32(vq, -vq_lim, vq_lim);
    if (vq != vq_presat) {
        m->vq_int_q31 += (int64_t)(vq - vq_presat) * 65536LL;
    }

    // Variabel int_lim2_q31: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int64_t int_lim2_q31 = (int64_t)vmax_q15 << 16;
    if (m->vd_int_q31 > int_lim2_q31)
        m->vd_int_q31 = int_lim2_q31;
    if (m->vd_int_q31 < -int_lim2_q31)
        m->vd_int_q31 = -int_lim2_q31;
    if (m->vq_int_q31 > int_lim2_q31)
        m->vq_int_q31 = int_lim2_q31;
    if (m->vq_int_q31 < -int_lim2_q31)
        m->vq_int_q31 = -int_lim2_q31;
    m->vd_int_q15 = (int32_t)(m->vd_int_q31 >> 16);
    m->vq_int_q15 = (int32_t)(m->vq_int_q31 >> 16);
    } /* current PI */
    m->vd_q15 = vd;
    m->vq_q15 = vq;

    // Variabel v_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t v_alpha = foc_q15_mul(cs, vd) - foc_q15_mul(sn, vq);
    // Variabel v_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t v_beta = foc_q15_mul(sn, vd) + foc_q15_mul(cs, vq);

    /* During the VESC brake zero-cross guard, zero modulation is intentional:
       keep the bridge at a centered zero vector instead of allowing the current
       PI to actively drive through a sign change. One 1-kHz hold tick is 16 FOC
       samples on this board, exceeding upstream's minimum ten samples. */
    if (m->force_zero_modulation) {
        v_alpha = 0;
        v_beta = 0;
        m->vd_int_q31 = m->vq_int_q31 = 0;
        m->vd_int_q15 = m->vq_int_q15 = 0;
    }



    // Variabel du: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel dv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel dw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t du, dv, dw;
    foc_svm_q15(v_alpha, v_beta, inv_vbus_q30, &du, &dv, &dw);

    /* Sampling-window integrity instrumentation. This board has one shared
       coherent current sample per PWM period, so reaching either 10/90% edge
       is recorded rather than enabling unsafe V0/V7 or high-current retiming. */
    // Variabel margin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t margin = (uint16_t)(du - PWM_MIN_DUTY_Q15);
    // Variabel t: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t t = (uint16_t)(PWM_MAX_DUTY_Q15 - du);
    if (t < margin)
        margin = t;
    t = (uint16_t)(dv - PWM_MIN_DUTY_Q15);
    if (t < margin)
        margin = t;
    t = (uint16_t)(PWM_MAX_DUTY_Q15 - dv);
    if (t < margin)
        margin = t;
    t = (uint16_t)(dw - PWM_MIN_DUTY_Q15);
    if (t < margin)
        margin = t;
    t = (uint16_t)(PWM_MAX_DUTY_Q15 - dw);
    if (t < margin)
        margin = t;
    if (margin < m->sampling_margin_min_q15)
        m->sampling_margin_min_q15 = margin;
    if (margin == 0U && (v_alpha != 0 || v_beta != 0))
        m->sampling_window_clamp_count++;

    /* Reconstruct the voltage from the duties that will actually reach the
       timers. This includes Batch-1 vector scaling for the 10..90% sampling
       window. Apply VESC dead-time compensation to that applied-voltage model,
       not to an unclipped command that the bridge never generated. */
    // Variabel obs_v_alpha: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t obs_v_alpha = 0;
    // Variabel obs_v_beta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t obs_v_beta = 0;
    foc_pwm_applied_voltage_q15(du, dv, dw, vbus_q15, &obs_v_alpha, &obs_v_beta);
    foc_deadtime_applied_voltage_q15(m, vbus_q15, sn, cs, &obs_v_alpha, &obs_v_beta);
    m->observer_v_alpha_q15_prev = obs_v_alpha;
    m->observer_v_beta_q15_prev = obs_v_beta;
    m->duty_u_q15 = du;
    m->duty_v_q15 = dv;
    m->duty_w_q15 = dw;

    /* Current VESC can replace an exactly-equal zero-vector PWM triplet with
       static all-low-side conduction. The electrical line-line voltage is
       still zero, so the applied-voltage observer model above remains valid.
       This STM32F103 backend is real but disabled by default until the stock
       hoverboard gate-driver/bootstrap path is validated for continuous LS. */
    // Variabel exact_zero_vector: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool exact_zero_vector = (du == dv) && (dv == dw);
    if (m->foc_short_ls_on_zero_duty && exact_zero_vector) {
        motor_hw_set_low_side_brake(m, true);
    }
    else {
        if (m->full_brake_active)
            motor_hw_set_low_side_brake(m, false);
        motor_hw_set_pwm_q15(m, du, dv, dw);
    }
    publish_rt_snapshot_isr(m, phase);
}

// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mn: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mx: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi cal_track_minmax: menangani kalibrasi cal track minmax agar offset atau parameter hasil ukur valid
// sebelum dipakai kendali.
static inline void cal_track_minmax(uint16_t v, uint16_t *mn, uint16_t *mx) {
    if (v < *mn) {
        *mn = v;
    }
    if (v > *mx) {
        *mx = v;
    }
}

// Parameter sum: akumulator penjumlahan untuk averaging atau statistik.
// Parameter sum_sq: akumulator penjumlahan untuk averaging atau statistik.
// Parameter idx: indeks elemen yang sedang diproses.
// Parameter mean: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mn: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mx: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sum: akumulator penjumlahan untuk averaging atau statistik.
// Parameter sum_sq: akumulator penjumlahan untuk averaging atau statistik.
// Parameter inliers: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter outliers: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi validate_cal_channel: menangani kalibrasi validate cal channel agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
static void validate_cal_channel(uint8_t idx, int32_t mean, uint16_t mn,
                                 uint16_t mx, uint16_t outliers) {
    const uint16_t bit = (uint16_t)(1U << idx);
    const uint32_t spread = (uint32_t)(mx - mn);

    /* Midpoint min/max adalah estimator offset final. Validasi hard hanya
     * memeriksa rail, spread ekstrem, dan jumlah outlier sehingga ISR tidak
     * membutuhkan sum/square 64-bit yang mahal pada STM32F103. */
    if (mean <= ADC_OFFSET_HARD_MIN_COUNT || mean >= ADC_OFFSET_HARD_MAX_COUNT) {
        s_cal_fail_range_mask |= bit;
    }
    if (spread > ADC_OFFSET_HARD_SPREAD_COUNT ||
        outliers > ADC_OFFSET_HARD_OUTLIER_COUNT) {
        s_cal_fail_noise_mask |= bit;
    }
    else if (outliers != 0U || spread > ADC_OFFSET_WARN_SPREAD_COUNT) {
        s_cal_warn_mask |= bit;
    }
}

// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_get_calibration_diag: membaca foc get calibration diag tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void foc_get_calibration_diag(foc_cal_diag_t *out) {
    if (out == NULL)
        return;
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->stage = (uint8_t)s_cal_stage;
    out->reserved = 0U;
    out->shift_warn_mask = s_cal_shift_warn_mask;
    out->warn_mask = s_cal_warn_mask;
    out->fail_range_mask = s_cal_fail_range_mask;
    out->fail_noise_mask = s_cal_fail_noise_mask;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 6U; k++) {
        out->ch[k] = s_cal_diag_channels[k];
        out->undriven_mean[k] = s_undriven_mean[k];
        out->driven_mean[k] = s_driven_mean[k];
        out->outlier_count[k] = s_cal_outlier_count[k];
    }
    out->moe_fail_mask = s_cal_moe_fail_mask;
    out->moe_confirmed_mask = s_cal_moe_confirmed_mask;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint8_t k = 0U; k < 2U; k++) {
        out->moe_request_adc[k] = s_cal_moe_request_adc[k];
        out->moe_confirm_adc[k] = s_cal_moe_confirm_adc[k];
        out->first_sample_adc[k] = s_cal_first_sample_adc[k];
    }
    if (!primask)
        __enable_irq();
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi calibration_zero_fast_states: menangani kalibrasi calibration zero fast states agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
static void calibration_zero_fast_states(MotorRuntime *m) {
    m->ia_q15 = 0;
    m->ib_q15 = 0;
    m->ic_q15 = 0;
    m->id_q15 = 0;
    m->iq_q15 = 0;
    m->id_filter_q15 = 0;
    m->iq_filter_q15 = 0;
    m->vd_q15 = 0;
    m->vq_q15 = 0;
    m->vd_int_q31 = 0;
    m->vq_int_q31 = 0;
    m->vd_int_q15 = 0;
    m->vq_int_q15 = 0;
    m->dc_current_raw = (uint16_t)m->dc_current_offset_counts;
}


// Parameter idx: indeks elemen yang sedang diproses.
// Parameter mean: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mn: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mx: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sum: akumulator penjumlahan untuk averaging atau statistik.
// Parameter sum_sq: akumulator penjumlahan untuk averaging atau statistik.
// Parameter inliers: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi cal_store_diag: menangani kalibrasi cal store diag agar offset atau parameter hasil ukur valid sebelum
// dipakai kendali.
static void cal_store_diag(uint8_t idx, int32_t mean, uint16_t mn, uint16_t mx) {
    s_cal_diag_channels[idx].mean = mean;
    s_cal_diag_channels[idx].min = mn;
    s_cal_diag_channels[idx].max = mx;
    /* Variance tidak dihitung di hard ISR. Nilai nol menandakan estimator
     * midpoint-only; min/max/spread tetap dikirim lengkap ke debug host. */
    s_cal_diag_channels[idx].variance_x100 = 0U;
    validate_cal_channel(idx, mean, mn, mx, s_cal_outlier_count[idx]);
    int32_t shift = mean - s_undriven_mean[idx];
    if (shift < 0) {
        shift = -shift;
    }
    if ((uint32_t)shift > ADC_OFFSET_WARN_SPREAD_COUNT) {
        s_cal_shift_warn_mask |= (uint16_t)(1U << idx);
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter u: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi cal_set_runtime_offsets: mengatur cal set runtime offsets setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
static void cal_set_runtime_offsets(MotorRuntime *m, int32_t u, int32_t v, int32_t dc) {
    m->current_offset_u_counts = u;
    m->current_offset_v_counts = v;
    m->dc_current_offset_counts = dc;
}

// Fungsi cal_vbus_ready_task: menangani kalibrasi cal vbus ready task agar offset atau parameter hasil ukur
// valid sebelum dipakai kendali.
static bool cal_vbus_ready_task(void) {
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t v = g_motor_left.vbus_q15;
    // Variabel vmin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vmin = g_motor_left.min_vin_q15 > 0 ? g_motor_left.min_vin_q15 : s_vbus_min_q15;
    // Variabel vmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t vmax = g_motor_left.max_vin_q15 > 0 ? g_motor_left.max_vin_q15 : s_vbus_max_q15;
    if (v < vmin || v > vmax) {
        s_cal_vbus_stable_ticks = 0U;
        s_cal_vbus_last_valid = false;
        return false;
    }

    /* VESC waits for input voltage to settle before DC calibration and
       restarts the settle timer after a >1.5 V movement. This task runs at
       200 Hz, so 400 ticks reproduces the 2.0 s stable window. */
    // Variabel delta_limit_q15: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t delta_limit_q15 = (int32_t)((ADC_CAL_VBUS_STABLE_DELTA_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    if (!s_cal_vbus_last_valid) {
        s_cal_vbus_last_q15 = v;
        s_cal_vbus_last_valid = true;
        s_cal_vbus_stable_ticks = 0U;
        return false;
    }
    // Variabel dv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dv = v - s_cal_vbus_last_q15;
    if (dv < 0)
        dv = -dv;
    if (dv > delta_limit_q15) {
        s_cal_vbus_last_q15 = v;
        s_cal_vbus_stable_ticks = 0U;
        return false;
    }

    if (s_cal_vbus_stable_ticks < ADC_CAL_VBUS_STABLE_5MS_TICKS)
        s_cal_vbus_stable_ticks++;
    return s_cal_vbus_stable_ticks >= ADC_CAL_VBUS_STABLE_5MS_TICKS;
}

// Parameter bit: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi cal_fail_from_isr: menangani cal fail from isr pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
static void cal_fail_from_isr(uint16_t bit) {
    s_cal_fail_noise_mask |= bit;
    s_cal_valid = false;
    s_cal_done = true;
    s_cal_stage = FOC_CAL_STAGE_FAILED;
    motor_hw_emergency_all_off();
}

// Parameter motor: objek runtime motor yang sedang dikendalikan atau dibaca.
// Parameter index: indeks elemen yang sedang diproses.
// Fungsi cal_moe_ready_isr: menangani cal moe ready isr pada konteks interrupt dengan pekerjaan minimum agar
// timing FOC tetap deterministik.
static bool cal_moe_ready_isr(MotorRuntime *motor, uint8_t index) {
    // Variabel mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t mask = (uint8_t)(1U << index);
    // Variabel moe: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool moe = (motor->pwm_tim->BDTR & TIM_BDTR_MOE) != 0U;
    s_cal_moe_live_bdtr[index] = (uint8_t)(motor->pwm_tim->BDTR & 0xFFU);
    if (motor->pwm_enabled && moe) {
        if ((s_cal_moe_confirmed_mask & mask) == 0U) {
            s_cal_moe_confirmed_mask |= mask;
            s_cal_moe_confirm_adc[index] = s_adc_isr_count;
        }
        return true;
    }

    if (s_cal_moe_wait_events[index] < UINT16_MAX) {
        s_cal_moe_wait_events[index]++;
    }
    if (s_cal_moe_wait_events[index] >= ADC_OFFSET_MOE_WAIT_EVENTS) {
        s_cal_moe_fail_mask |= mask;
        cal_fail_from_isr(index == 0U ? 0x0003U : 0x0018U);
    }
    return false;
}

// Parameter l_u: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter l_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter l_dc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r_u: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r_dc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi cal_accumulate_all: menangani kalibrasi cal accumulate all agar offset atau parameter hasil ukur valid
// sebelum dipakai kendali.
static void cal_accumulate_all(uint16_t l_u, uint16_t l_v, uint16_t l_dc,
                               uint16_t r_u, uint16_t r_v, uint16_t r_dc) {
    cal_track_minmax(l_u, &s_min_lu, &s_max_lu);
    cal_track_minmax(l_v, &s_min_lv, &s_max_lv);
    cal_track_minmax(l_dc, &s_min_ldc, &s_max_ldc);
    cal_track_minmax(r_u, &s_min_ru, &s_max_ru);
    cal_track_minmax(r_v, &s_min_rv, &s_max_rv);
    cal_track_minmax(r_dc, &s_min_rdc, &s_max_rdc);
}

/* Task-side portion of current calibration: both bridges use the same
 * 50%/50%/50% zero-vector common-mode. The ISR only updates six min/max pairs;
 * midpoint validation and offset commit stay in the 1-kHz task. */
// Fungsi foc_calibration_service_task: menangani kalibrasi foc calibration service task agar offset atau
// parameter hasil ukur valid sebelum dipakai kendali.
void foc_calibration_service_task(void) {
    switch (s_cal_stage) {
    case FOC_CAL_STAGE_WAIT_LEFT_DRIVEN:
        /* Satu-satunya sumber offset: kedua inverter berada pada kondisi
         * zero-vector 50% yang sama dengan common-mode saat FOC aktif. */
        if (!cal_vbus_ready_task())
            break;
        motor_hw_set_pwm_q15(&g_motor_left, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
        motor_hw_set_pwm_q15(&g_motor_right, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
        calibration_zero_fast_states(&g_motor_left);
        calibration_zero_fast_states(&g_motor_right);
        cal_task_reset_acc_and_stage(FOC_CAL_STAGE_LEFT_WARMUP);
        s_cal_moe_request_adc[1] = s_adc_isr_count;
        motor_hw_set_pwm_enabled(&g_motor_left, true);
        motor_hw_set_pwm_enabled(&g_motor_right, true);
        break;

    case FOC_CAL_STAGE_WAIT_FINALIZE:
    {
        /* Finalisasi statistik dipindah ke task 1 kHz. ISR hanya mengumpulkan
         * min/max/sum agar frame ke-2000 tidak pernah menghabiskan ribuan cycle
         * tambahan dan melampaui slot 4000 cycle. */
        const int32_t midpoint[6] = {
            ((int32_t)s_min_lu + (int32_t)s_max_lu) / 2,
            ((int32_t)s_min_lv + (int32_t)s_max_lv) / 2,
            ((int32_t)s_min_ldc + (int32_t)s_max_ldc) / 2,
            ((int32_t)s_min_ru + (int32_t)s_max_ru) / 2,
            ((int32_t)s_min_rv + (int32_t)s_max_rv) / 2,
            ((int32_t)s_min_rdc + (int32_t)s_max_rdc) / 2
        };
        const uint16_t mins[6] = {s_min_lu, s_min_lv, s_min_ldc, s_min_ru, s_min_rv, s_min_rdc};
        const uint16_t maxs[6] = {s_max_lu, s_max_lv, s_max_ldc, s_max_ru, s_max_rv, s_max_rdc};

        for (uint8_t i = 0U; i < 6U; i++) {
            s_undriven_mean[i] = midpoint[i];
            s_driven_mean[i] = midpoint[i];
            cal_store_diag(i, midpoint[i], mins[i], maxs[i]);
        }
        cal_set_runtime_offsets(&g_motor_left, midpoint[0], midpoint[1], midpoint[2]);
        cal_set_runtime_offsets(&g_motor_right, midpoint[3], midpoint[4], midpoint[5]);

        motor_hw_set_pwm_enabled(&g_motor_left, false);
        motor_hw_set_pwm_enabled(&g_motor_right, false);
        calibration_zero_fast_states(&g_motor_left);
        calibration_zero_fast_states(&g_motor_right);
        s_cal_valid = (s_cal_fail_range_mask == 0U) &&
                      (s_cal_fail_noise_mask == 0U) &&
                      (s_cal_moe_fail_mask == 0U);
        s_cal_done = true;
        s_cal_stage = s_cal_valid ? FOC_CAL_STAGE_DONE : FOC_CAL_STAGE_FAILED;
        hw_status_notify_calibration_done(s_cal_valid);
        break;
    }

    case FOC_CAL_STAGE_FAILED:
        motor_hw_set_pwm_enabled(&g_motor_left, false);
        motor_hw_set_pwm_enabled(&g_motor_right, false);
        s_cal_valid = false;
        s_cal_done = true;
        break;

    default:
        break;
    }
}


// Parameter l_u: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter l_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter l_dc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r_u: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r_v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r_dc: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi calibration_process_isr: menangani calibration process isr pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
static bool calibration_process_isr(uint16_t l_u, uint16_t l_v, uint16_t l_dc,
                                    uint16_t r_u, uint16_t r_v, uint16_t r_dc) {
    if (s_cal_stage == FOC_CAL_STAGE_DONE)
        return false;
    if (s_cal_stage == FOC_CAL_STAGE_FAILED)
        return true;

    switch (s_cal_stage) {
    case FOC_CAL_STAGE_LEFT_WARMUP:
    {
        /* Tunggu kedua MOE benar-benar aktif. Frame sebelum konfirmasi MOE tidak
         * boleh masuk statistik karena common-mode-nya masih bridge-OFF. */
        const bool left_warmup_ready = cal_moe_ready_isr(&g_motor_left, 0U);
        const bool right_warmup_ready = cal_moe_ready_isr(&g_motor_right, 1U);
        if (!left_warmup_ready || !right_warmup_ready)
            return true;
        if (++s_cal_warmup >= ADC_NEUTRAL_CAL_WARMUP_EVENTS) {
            cal_acc_reset();
            s_cal_first_sample_adc[0] = s_adc_isr_count + 1U;
            s_cal_first_sample_adc[1] = s_adc_isr_count + 1U;
            s_cal_stage = FOC_CAL_STAGE_LEFT_DRIVEN;
        }
        return true;
    }

    case FOC_CAL_STAGE_LEFT_DRIVEN:
    {
        const bool left_sample_ready = cal_moe_ready_isr(&g_motor_left, 0U);
        const bool right_sample_ready = cal_moe_ready_isr(&g_motor_right, 1U);
        if (!left_sample_ready || !right_sample_ready)
            return true;
        cal_accumulate_all(l_u, l_v, l_dc, r_u, r_v, r_dc);
        s_cal_count++;
        s_cal_progress = s_cal_count;
        if (s_cal_count >= ADC_NEUTRAL_CAL_SAMPLES) {
            /* Jangan hitung midpoint atau mengubah offset di hard ISR.
             * Freeze min/max dan serahkan finalisasi ke timer task. */
            s_cal_progress = ADC_NEUTRAL_CAL_SAMPLES;
            s_cal_stage = FOC_CAL_STAGE_WAIT_FINALIZE;
        }
        return true;
    }

    case FOC_CAL_STAGE_UNDRIVEN:
    case FOC_CAL_STAGE_WAIT_RIGHT_DRIVEN:
    case FOC_CAL_STAGE_RIGHT_WARMUP:
    case FOC_CAL_STAGE_RIGHT_DRIVEN:
        /* Legacy stage IDs dipertahankan untuk kompatibilitas diagnostik,
         * tetapi pipeline baru tidak pernah membedakan driven/undriven. */
        s_cal_stage = FOC_CAL_STAGE_WAIT_LEFT_DRIVEN;
        return true;

    case FOC_CAL_STAGE_WAIT_LEFT_DRIVEN:
    case FOC_CAL_STAGE_WAIT_FINALIZE:
        return true;

    default:
        return true;
    }
}


// Parameter start_cycle: jumlah cycle CPU atau siklus algoritma untuk diagnostik timing.
// Fungsi foc_isr_finish_timing: menangani foc isr finish timing pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
static inline void foc_isr_finish_timing(uint32_t start_cycle) {
    // Variabel cycles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t cycles = DWT->CYCCNT - start_cycle;
    // Variabel slot_cycles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t slot_cycles = FOC_ISR_SLOT_CYCLES;

    if (cycles > s_isr_total_max_cycles) {
        s_isr_total_max_cycles = cycles;
    }
    if (cycles > (slot_cycles * 85U) / 100U) {
        s_isr_near_deadline_count++;
    }

    /* 85% is only an early warning. A true overrun means the complete Run31
       control update exceeded its three-PWM-period (12,000-cycle) slot. */
    if (cycles > slot_cycles) {
        g_motor_left.isr_overruns++;
        g_motor_right.isr_overruns++;
        if (s_overrun_consecutive[0] < UINT8_MAX)
            s_overrun_consecutive[0]++;
        if (s_overrun_consecutive[1] < UINT8_MAX)
            s_overrun_consecutive[1]++;
        if (s_overrun_consecutive[0] >= 8U) {
            motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_FOC_ISR_OVERRUN);
        }
        if (s_overrun_consecutive[1] >= 8U) {
            motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_FOC_ISR_OVERRUN);
        }
    }
    else {
        s_overrun_consecutive[0] = 0U;
        s_overrun_consecutive[1] = 0U;
    }

    /* Simpan raw cycle saja; konversi floating-point dilakukan task-side. */
    s_isr_last_cycles = cycles;
    s_isr_motor = 0;
}

// Parameter start_cycle: cycle CPU yang ditangkap pada awal IRQ DMA untuk menghitung waktu end-to-end ISR.
// Fungsi mcpwm_foc_finish_irq_timing: menutup pencatatan waktu ISR pada jalur keluar dini seperti transfer-error DMA.
void mcpwm_foc_finish_irq_timing(uint32_t start_cycle) {
    foc_isr_finish_timing(start_cycle);
}

// Parameter adc_words: lima word dari frame dual-ADC V15 terbaru.
// Parameter start_cycle: cycle CPU yang ditangkap pada instruksi awal IRQ DMA sebelum pemeriksaan flag.
// Fungsi mcpwm_foc_adc_words_isr_timed: menjalankan FOC dari sampel DMA dan mengukur seluruh jalur IRQ sampai selesai.
void mcpwm_foc_adc_words_isr_timed(const volatile uint32_t adc_words[ADC_WORDS_PER_FRAME], uint32_t start_cycle) {
    timeout_heartbeat_from_isr(TIMEOUT_HEARTBEAT_FOC);
    s_isr_motor = 0;
    // Variabel start: timestamp awal IRQ DMA yang menjadi acuan seluruh pengukuran cycle FOC.
    const uint32_t start = start_cycle;
    if (s_isr_last_entry_cycle != 0U) {
        // Variabel period: periode pengukuran atau eksekusi modul.
        uint32_t period = start - s_isr_last_entry_cycle;
        /* Ignore debugger-sized gaps; Run35 control IRQ cadence is 16k/3. */
        if (period < (FOC_ISR_SLOT_CYCLES * 4U)) {
            if (period < s_isr_period_min_cycles)
                s_isr_period_min_cycles = period;
            if (period > s_isr_period_max_cycles)
                s_isr_period_max_cycles = period;
        }
    }
    s_isr_last_entry_cycle = start;
    s_adc_isr_count++;
    if (s_cal_request)
        cal_reset_isr();

    /* Raw current is read from the newest complete 5-rank V15 frame. ADC
       itself still samples every 16-kHz PWM period; this full FOC handler is
       entered once per three-frame DMA batch (about 5.333 kHz):
         word0: ADC1 R_DC (PC1) | ADC2 L_DC (PC0)   -> {dcr, dcl}
         word1: ADC1 L_A  (PA0) | ADC2 L_B  (PC3)   -> {rlA, rlB}
         word2: ADC1 R_B  (PC4) | ADC2 R_C  (PC5)   -> {rrB, rrC}
       These are the six raw current channels the F103 hoverboard board exposes,
       named to match the reference current sense layout. */
    // Variabel idcr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t idcr = low16(adc_words[0]); /* RIGHT DC  (ADC1 PC1) */
    // Variabel idcl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t idcl = high16(adc_words[0]); /* LEFT  DC  (ADC2 PC0) */
    // Variabel irla: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t irla = low16(adc_words[1]); /* LEFT  A   (ADC1 PA0) */
    // Variabel irlb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t irlb = high16(adc_words[1]); /* LEFT  B   (ADC2 PC3) */
    // Variabel irrb: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t irrb = low16(adc_words[2]); /* RIGHT B   (ADC1 PC4) */
    // Variabel irrc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t irrc = high16(adc_words[2]); /* RIGHT C   (ADC2 PC5) */
    /* Convenience aliases used by the rest of the FOC path (matches prior
       internal names so the downstream math is unchanged). */
    // Variabel l_dc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel r_dc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t r_dc = idcr, l_dc = idcl;
    // Variabel l_u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel l_v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t l_u = irla, l_v = irlb;
    // Variabel r_u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel r_v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t r_u = irrb, r_v = irrc;

    /* Boot dan manual calibration ditangani oleh calibration_process_isr()
       yang sama setelah Vbus/MOE siap. Tidak ada lagi boot-only averaging. */

    /* VBUS is ADC1 rank-4 PC2 from the same newest complete V15 frame. */
    uint16_t vraw = low16(adc_words[3]);
    s_vbus_dma_stale_count = 0U;

    /* Voltage is independent of current-offset validity. Keep it alive during
       calibration so the task can enforce the VESC-style stable-Vbus gate and
       so early telemetry does not report an artificial 0 V. */
    // Variabel vbus_q15: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    int32_t vbus_q15 = (int32_t)(((int64_t)vraw * (int64_t)s_vbus_scale_q16) >> 16);
    g_motor_left.vbus_q15 = vbus_q15;
    g_motor_right.vbus_q15 = vbus_q15;

    /* During calibration the ISR still owns synchronized MOE enable. Service
       the pending bridge only after consuming this already-acquired sample so
       the first sample after MOE assertion is guaranteed to be a later frame. */
    if (calibration_process_isr(l_u, l_v, l_dc, r_u, r_v, r_dc)) {
        motor_hw_service_pwm_enable_from_isr(&g_motor_left);
        motor_hw_service_pwm_enable_from_isr(&g_motor_right);
        foc_isr_finish_timing(start);
        return;
    }

    // Variabel dv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t dv = vbus_q15 - s_inv_vbus_last_q15;
    if (dv < 0)
        dv = -dv;
    if (vbus_q15 <= 256) {
        s_inv_vbus_q30 = 0;
        s_inv_vbus_last_q15 = vbus_q15;
        s_inv_vbus_age = 0U;
    }
    else if (s_inv_vbus_q30 == 0 || dv > 48 || ++s_inv_vbus_age >= 32U) {
        s_inv_vbus_q30 = (int32_t)(((int64_t)1<<30)/vbus_q15);
        s_inv_vbus_last_q15 = vbus_q15;
        s_inv_vbus_age = 0U;
    }
    // Variabel inv_vbus_q30: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    int32_t inv_vbus_q30 = s_inv_vbus_q30;

    /* One coherent V15-format frame contains both motors. The complete Run31
       FOC loops execute at the DMA-batch rate (16k/3); PWM and raw ADC
       acquisition remain 16 kHz. */
    // Variabel left_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t left_start = DWT->CYCCNT;
    s_isr_motor = 1;
    foc_one_motor_isr(&g_motor_left, l_u, l_v, l_dc, vbus_q15, inv_vbus_q30);
    mc_interface_mc_timer_isr(false, 1.0f / (float)FOC_ISR_EVENT_HZ);
    mc_interface_sample_capture_isr(&g_motor_left);
    // Variabel left_cycles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t left_cycles = DWT->CYCCNT - left_start;
    if (left_cycles > g_motor_left.isr_max_cycles)
        g_motor_left.isr_max_cycles = left_cycles;

    // Variabel right_start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t right_start = DWT->CYCCNT;
    s_isr_motor = 2;
    foc_one_motor_isr(&g_motor_right, r_u, r_v, r_dc, vbus_q15, inv_vbus_q30);
    mc_interface_mc_timer_isr(true, 1.0f / (float)FOC_ISR_EVENT_HZ);
    mc_interface_sample_capture_isr(&g_motor_right);
    // Variabel right_cycles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t right_cycles = DWT->CYCCNT - right_start;
    if (right_cycles > g_motor_right.isr_max_cycles)
        g_motor_right.isr_max_cycles = right_cycles;

    /* Assert MOE only after this sample has been processed. This makes the
       following ADC frame the first physically-driven sample and gives the
       startup blanking counter exact sample semantics. */
    motor_hw_service_pwm_enable_from_isr(&g_motor_left);
    motor_hw_service_pwm_enable_from_isr(&g_motor_right);

    /* Semua jalur keluar ISR, termasuk jalur normal ini, menggunakan satu
       finalizer sehingga statistik cycle dan overrun tidak dapat berbeda. */
    foc_isr_finish_timing(start);
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================================
 * VESC-style blocking FOC detection
 * =========================================================================
 * Upstream keeps encoder/Hall/R/L detection in mcpwm_foc.c. follows that
 * ownership so commands.c only dispatches COMM_DETECT_* and mc_interface.c
 * remains the high-level motor API. The STM32F103 dual-motor port passes an
 * explicit MotorRuntime pointer where upstream selects the motor thread-local.
 *
 * HFI is deliberately absent. Current control during detection still runs in
 * the fixed-point FOC control ISR; these routines only move setpoints and average
 * task-side measurements.
 */

// Variabel s_detect_owner: state internal modul yang dipertahankan antar pemanggilan fungsi.
static MotorRuntime *s_detect_owner = NULL;

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_wrap_deg: menjalankan deteksi detect wrap deg dengan proteksi motor dan memvalidasi hasil
// sebelum parameter diterapkan.
static float detect_wrap_deg(float x) {
    while (x >= 360.0f)
        x -= 360.0f;
    while (x < 0.0f)
        x += 360.0f;
    return x;
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_angle_diff_deg: menjalankan deteksi detect angle diff deg dengan proteksi motor dan memvalidasi
// hasil sebelum parameter diterapkan.
static float detect_angle_diff_deg(float a, float b) {
    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float d = detect_wrap_deg(a) - detect_wrap_deg(b);
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

// Parameter x: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter lo: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter hi: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_finite_positive: menjalankan deteksi detect finite positive dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
static bool detect_finite_positive(float x, float lo, float hi) {
    return isfinite(x) && x >= lo && x <= hi;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_clear_acc: menjalankan deteksi detect clear acc dengan proteksi motor dan memvalidasi hasil
// sebelum parameter diterapkan.
static void detect_clear_acc(MotorRuntime *m) {
    memset(m->detect.hall_sin_sum, 0, sizeof(m->detect.hall_sin_sum));
    memset(m->detect.hall_cos_sum, 0, sizeof(m->detect.hall_cos_sum));
    memset(m->detect.hall_samples, 0, sizeof(m->detect.hall_samples));
    memset(m->detect.result_hall_table, 255, sizeof(m->detect.result_hall_table));
    m->detect.hall_valid_states = 0U;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter sensor_mode: mode operasi yang menentukan jalur algoritma aktif.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi detect_begin: menjalankan deteksi detect begin dengan proteksi motor dan memvalidasi hasil sebelum
// parameter diterapkan.
static bool detect_begin(MotorRuntime *m, uint8_t sensor_mode, float current_a) {
    if (m == NULL || s_detect_owner != NULL || m->detect.busy ||
        m->fault != MOTOR_FAULT_NONE || !foc_calibration_valid()) {
        return false;
    }
    if (sensor_mode == SENSOR_MODE_ENCODER && m->id != MOTOR_LEFT)
        return false;

    s_detect_owner = m;
    motor_stop(m);

    // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_detect_state_t *d = &m->detect;
    d->saved_sensor_mode = m->sensor_mode;
    d->saved_sensor_request_mode = m->sensor_request_mode;
    d->saved_current_kp = m->current_kp;
    d->saved_current_ki = m->current_ki;
    d->saved_timeout_ms = timeout_get_timeout_ms();
    d->saved_timeout_brake_a = timeout_get_brake_current();
    d->saved_encoder_coordinate_valid = (m->id == MOTOR_LEFT &&
                                         m->sensor_mode == SENSOR_MODE_ENCODER);
    if (d->saved_encoder_coordinate_valid) {
        d->saved_encoder_hw_count = motor_hw_encoder_cnt();
        d->saved_encoder_turns = m->encoder.turns;
        d->saved_encoder_last_cnt = m->encoder.last_cnt;
        d->saved_encoder_extended_count = m->encoder.extended_count;
        d->saved_encoder_prev_extended_count = m->encoder.prev_extended_count;
        d->saved_encoder_session_zero_count = m->encoder.session_zero_count;
        d->saved_encoder_mechanical_zero_count = m->encoder.mechanical_zero_count;
        d->saved_encoder_speed_sample_valid = m->encoder.speed_sample_valid;
        d->saved_encoder_synced = m->encoder.synced;
        d->saved_encoder_motion_proved = m->encoder.motion_proved;
        d->saved_encoder_sync_active = m->encoder.sync_active;
        d->saved_encoder_sync_start_tick = m->encoder.sync_start_tick;
    }
    d->drive_current_a = fabsf(current_a);
    if (d->drive_current_a < 0.2f)
        d->drive_current_a = 0.2f;
    if (d->drive_current_a > FOC_DETECT_MAX_CURRENT_A)
        d->drive_current_a = FOC_DETECT_MAX_CURRENT_A;
    if (m->current_max_a > 0.0f && d->drive_current_a > m->current_max_a)
        d->drive_current_a = m->current_max_a;
    d->busy = true;
    d->requested = false;
    d->success = false;
    d->result_mode = SENSOR_MODE_AUTO;
    d->state = SENSOR_DETECT_PREPARE;
    d->result_encoder_offset_deg = 0.0f;
    d->result_encoder_ratio = 0.0f;
    d->result_encoder_inverted = false;
    d->l_capture_active = false;
    d->l_capture_done = false;
    d->l_capture_count = 0U;
    d->l_capture_axis = 0U;
    detect_clear_acc(m);

    m->sensor_request_mode = sensor_mode;
    /* Re-entering the sensor mode that is already active must not reinitialize
       its peripheral. On incremental A/B, motor_hw_configure_sensor() rebases
       TIM4; doing that during standard encoder detection would destroy the
       caller's multi-turn coordinate. */
    if (m->sensor_mode != sensor_mode)
        motor_hw_configure_sensor(m, sensor_mode);
    if (sensor_mode == SENSOR_MODE_ENCODER) {
        /* Jangan nolkan counter quadrature di sini. Deteksi encoder boleh
           menggerakkan poros tetapi koordinat multi-turn pemanggil harus tetap
           dipertahankan saat transaksi selesai. */
        m->encoder.synced = false;
        m->encoder.motion_proved = false;
    }
    else if (sensor_mode == SENSOR_MODE_HALL) {
        /* Hanya mode Hall yang membutuhkan pembacaan edge awal. Sensorless
           tidak menyentuh GPIO Hall sehingga benar-benar bebas sensor. */
        motor_hall_edge_isr(m);
    }

    /* Siapkan output dalam keadaan nol sebelum metode detect memilih eksitasi.
       Hall detect kemudian mengganti mode menjadi CURRENT dengan forced angle +
       Id seperti VESC; deteksi R/L/flux lain tetap boleh memakai helper open-loop. */
    m->control_mode = MOTOR_CTRL_OPENLOOP_DUTY_PHASE;
    m->command_active = true;
    m->detect_force_angle = false;
    m->detect_phase_u16 = 0U;
    m->openloop_command_phase_u16 = 0U;
    motor_touch_command(m);

    /* Detection can legitimately take tens of seconds. Upstream disables the
       normal command timeout while the blocking detect transaction owns motor. */
    timeout_configure(180000U, 0.0f);
    timeout_reset();
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Fungsi detect_wait_pwm: menjalankan deteksi detect wait pwm dengan proteksi motor dan memvalidasi hasil
// sebelum parameter diterapkan.
static bool detect_wait_pwm(MotorRuntime *m, uint32_t timeout_ms) {
    // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t start = xTaskGetTickCount();
    while ((uint32_t)(xTaskGetTickCount() - start) < timeout_ms) {
        if (m->fault != MOTOR_FAULT_NONE)
            return false;
        if (m->pwm_enabled && m->pwm_enable_pending_events == 0U &&
            m->pwm_enable_blank_cycles == 0U)
            return true;
        timeout_reset();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_restore_sensor: menjalankan deteksi detect restore sensor dengan proteksi motor dan memvalidasi
// hasil sebelum parameter diterapkan.
static void detect_restore_sensor(MotorRuntime *m) {
    // Variabel mode: mode operasi yang menentukan jalur algoritma aktif.
    uint8_t mode = m->detect.saved_sensor_mode;
    if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER)
        mode = SENSOR_MODE_HALL;
    /* Avoid a destructive encoder peripheral re-init when the detect transaction
       already used the same mode. Only switch GPIO/timer ownership when the
       saved mode is actually different from the temporary detect mode. */
    if (m->sensor_mode != mode)
        motor_hw_configure_sensor(m, mode);
    if (mode == SENSOR_MODE_ENCODER && m->detect.saved_encoder_coordinate_valid) {
        /* motor_hw_configure_sensor() necessarily resets TIM4 when taking the
           shared pins back from Hall mode. Restore the entry coordinate after
           the peripheral is running; no torque is enabled at this point. */
        motor_hw_encoder_set_count(m, m->detect.saved_encoder_hw_count);
        m->encoder.turns = m->detect.saved_encoder_turns;
        m->encoder.last_cnt = m->detect.saved_encoder_last_cnt;
        m->encoder.extended_count = m->detect.saved_encoder_extended_count;
        m->encoder.prev_extended_count = m->detect.saved_encoder_prev_extended_count;
        m->encoder.session_zero_count = m->detect.saved_encoder_session_zero_count;
        m->encoder.mechanical_zero_count = m->detect.saved_encoder_mechanical_zero_count;
        m->encoder.speed_sample_valid = m->detect.saved_encoder_speed_sample_valid;
        m->encoder.synced = m->detect.saved_encoder_synced;
        m->encoder.motion_proved = m->detect.saved_encoder_motion_proved;
        m->encoder.sync_active = m->detect.saved_encoder_sync_active;
        m->encoder.sync_start_tick = m->detect.saved_encoder_sync_start_tick;
    }
    m->sensor_request_mode = m->detect.saved_sensor_request_mode;
    if (mode == SENSOR_MODE_HALL)
        motor_hall_edge_isr(m);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ok: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_end: menjalankan deteksi detect end dengan proteksi motor dan memvalidasi hasil sebelum
// parameter diterapkan.
static void detect_end(MotorRuntime *m, bool ok) {
    m->detect.l_capture_active = false;
    m->detect.l_capture_done = false;
    m->detect.l_capture_axis = 0U;
    /* A Hall sweep can be in OPENLOOP_DUTY_PHASE, where Id/Iq targets are
     * intentionally ignored. Explicitly command a zero duty vector before the
     * settling delay; otherwise the last SVPWM triplet remains energized. */
    mcpwm_foc_set_openloop_duty_phase_motor(m, 0.0f,
                                            (float)m->detect_phase_u16 * (360.0f / 65536.0f));
    vTaskDelay(pdMS_TO_TICKS(20U));
    m->detect_force_angle = false;
    m->phase_observer_override = false;
    m->openloop_started = false;
    m->control_mode = MOTOR_CTRL_OFF;
    m->command_active = false;
    motor_hw_set_pwm_enabled(m, false);

    motor_set_current_pi_gains(m, m->detect.saved_current_kp, m->detect.saved_current_ki);
    timeout_configure(m->detect.saved_timeout_ms, m->detect.saved_timeout_brake_a);
    timeout_reset();
    detect_restore_sensor(m);

    m->detect.busy = false;
    m->detect.requested = false;
    m->detect.success = ok;
    m->detect.state = ok ? SENSOR_DETECT_DONE : SENSOR_DETECT_FAILED;
    if (s_detect_owner == m)
        s_detect_owner = NULL;
}

// Parameter m: motor yang sedang menjalankan deteksi Hall.
// Parameter ramp_ms: durasi ramp arus Id dari nol sampai arus deteksi.
// Fungsi detect_ramp_hall_id: mengunci rotor dengan Id pada sudut paksa seperti VESC sebelum sweep Hall.
static bool detect_ramp_hall_id(MotorRuntime *m, uint32_t ramp_ms) {
    if (m == NULL)
        return false;
    if (!detect_wait_pwm(m, 500U))
        return false;
    if (ramp_ms == 0U)
        ramp_ms = 1U;

    /* Hall detect harus memakai parameter current yang diberikan pengguna.
       Paksa fase listrik dan jalankan regulator Id normal; motor-service tidak
       akan menimpa target karena m->detect.busy masih aktif. */
    m->detect_force_angle = true;
    m->detect_phase_u16 = 0U;
    m->control_mode = MOTOR_CTRL_CURRENT;
    m->command_active = true;
    for (uint32_t k = 0U; k <= ramp_ms; k++) {
        if (m->fault != MOTOR_FAULT_NONE)
            return false;
        const float id_cmd = m->detect.drive_current_a * ((float)k / (float)ramp_ms);
        motor_set_foc_targets(m, id_cmd, 0.0f);
        timeout_reset();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    /* Pastikan current loop benar-benar menghasilkan arus sebelum sweep.
       Batas 15% cukup longgar untuk motor tanpa beban tetapi mendeteksi wiring
       arus/PWM yang sama sekali tidak bekerja. */
    vTaskDelay(pdMS_TO_TICKS(100U));
    const float measured = fabsf(m->id_filter);
    return isfinite(measured) && measured >= (0.15f * m->detect.drive_current_a);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ramp_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi detect_ramp_id: menjalankan deteksi detect ramp id dengan proteksi motor dan memvalidasi hasil sebelum
// parameter diterapkan.
static bool detect_ramp_id(MotorRuntime *m, uint32_t ramp_ms) {
    if (!detect_wait_pwm(m, 500U))
        return false;
    if (ramp_ms == 0U)
        ramp_ms = 1U;
    /* Open-loop SVPWM excitation (no current PI): ramp the modulation from 0
     * to SENSOR_DETECT_DUTY over the ramp window. The F103 closed-loop current
     * PI is not used here because it is unstable for sub-2A requests. */
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t k = 0U; k <= ramp_ms; k++) {
        if (m->fault != MOTOR_FAULT_NONE)
            return false;
        m->detect_phase_u16 = 0U;
        // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
        float duty = SENSOR_DETECT_DUTY * ((float)k / (float)ramp_ms);
        mcpwm_foc_set_openloop_duty_phase_motor(m, duty, 0.0f);
        timeout_reset();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_hall_sample: menjalankan deteksi detect hall sample dengan proteksi motor dan memvalidasi hasil
// sebelum parameter diterapkan.
static void detect_hall_sample(MotorRuntime *m) {
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    uint8_t raw = motor_hw_read_hall_raw(m->id) & 7U;
    // Variabel cs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel sn: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t sn, cs;
    foc_fast_sincos_u16_q15(m->detect_phase_u16, &sn, &cs);
    m->detect.hall_sin_sum[raw] += sn;
    m->detect.hall_cos_sum[raw] += cs;
    m->detect.hall_samples[raw]++;
}

// Parameter table: tabel Hall VESC 0..200 yang akan divalidasi sebelum diterapkan.
// Fungsi detect_hall_table_topology_valid: memastikan enam state Hall valid membentuk siklus Gray-code dengan jarak sudut masuk akal.
static bool detect_hall_table_topology_valid(const uint8_t table[8]) {
    if (table == NULL || table[0] != 255U || table[7] != 255U)
        return false;

    uint8_t raw_sorted[6];
    uint8_t angle_sorted[6];
    uint8_t n = 0U;
    for (uint8_t raw = 1U; raw <= 6U; raw++) {
        if (table[raw] >= 200U)
            return false;
        uint8_t pos = n;
        while (pos > 0U && angle_sorted[pos - 1U] > table[raw]) {
            angle_sorted[pos] = angle_sorted[pos - 1U];
            raw_sorted[pos] = raw_sorted[pos - 1U];
            pos--;
        }
        angle_sorted[pos] = table[raw];
        raw_sorted[pos] = raw;
        n++;
    }

    for (uint8_t i = 0U; i < 6U; i++) {
        const uint8_t j = (uint8_t)((i + 1U) % 6U);
        const uint8_t changed = (uint8_t)(raw_sorted[i] ^ raw_sorted[j]);
        if (changed == 0U || (changed & (uint8_t)(changed - 1U)) != 0U)
            return false;
        const uint16_t next = (j == 0U) ? (uint16_t)angle_sorted[0] + 200U : angle_sorted[j];
        const uint16_t spacing = next - angle_sorted[i];
        /* Ideal 60 derajat = 33.3 unit VESC. Toleransi 27..99 derajat
           menerima mekanik/Hall nyata tetapi menolak tabel terlipat/duplikat. */
        if (spacing < 15U || spacing > 55U)
            return false;
    }
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi detect_hall_evaluate: menjalankan deteksi detect hall evaluate dengan proteksi motor dan memvalidasi
// hasil sebelum parameter diterapkan.
static bool detect_hall_evaluate(MotorRuntime *m) {
    // Variabel fails: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel valid: penanda validitas hasil pengukuran atau konfigurasi.
    uint8_t fails = 0U, valid = 0U;
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    for (uint8_t raw = 0U; raw < 8U; raw++) {
        if (m->detect.hall_samples[raw] > SENSOR_DETECT_HALL_MIN_SAMPLES) {
            // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float a = atan2f((float)m->detect.hall_sin_sum[raw],
                             (float)m->detect.hall_cos_sum[raw]);
            if (a < 0.0f)
                a += 2.0f * (float)M_PI;
            // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t v = (uint32_t)lroundf(a * (200.0f / (2.0f * (float)M_PI)));
            if (v >= 200U)
                v -= 200U;
            m->detect.result_hall_table[raw] = (uint8_t)v;
            valid++;
        }
        else {
            m->detect.result_hall_table[raw] = 255U;
            fails++;
        }
    }
    m->detect.hall_valid_states = valid;
    const bool count_ok = fails == 2U && valid == 6U &&
                          m->detect.result_hall_table[0] == 255U &&
                          m->detect.result_hall_table[7] == 255U;
    return count_ok && detect_hall_table_topology_valid(m->detect.result_hall_table);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter hall_table: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Parameter result: hasil sementara atau akhir dari operasi yang sedang dijalankan.
// Fungsi mcpwm_foc_hall_detect_motor: menjalankan deteksi mcpwm foc hall detect motor dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
int mcpwm_foc_hall_detect_motor(MotorRuntime *m, float current, uint8_t *hall_table, bool *result) {
    if (result)
        *result = false;
    if (hall_table)
        memset(hall_table, 255, 8U);
    if (!detect_begin(m, SENSOR_MODE_HALL, current))
        return MOTOR_FAULT_SENSOR_DETECT;

    m->detect.state = SENSOR_DETECT_HALL_LOCK;
    /* Keep the calibrated runtime PI gains. The historical 0.01/10.0 detect
       pair is incompatible with this port's Q16.16 Ki*dt scaling: it drives
       the integrator into saturation and can produce tens of amps from a
       0.5-A request. detect_end() still restores the saved gains defensively. */
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = detect_ramp_hall_id(m, SENSOR_DETECT_CURRENT_RAMP_MS);

    if (ok) {
        detect_clear_acc(m);
        m->detect.state = SENSOR_DETECT_HALL_FWD;
        /* Three forward electrical revolutions, 1 degree per 5 ms. */
        // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (uint32_t k = 0U; k < 360U * SENSOR_DETECT_SWEEPS && ok; k++) {
            // Variabel deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float deg = (float)(k % 360U);
            m->detect_phase_u16 = foc_deg_to_u16(deg);
            motor_set_foc_targets(m, m->detect.drive_current_a, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(SENSOR_DETECT_STEP_MS));
            detect_hall_sample(m);
            timeout_reset();
            if (m->fault != MOTOR_FAULT_NONE)
                ok = false;
        }
    }

    if (ok) {
        m->detect.state = SENSOR_DETECT_HALL_REV;
        // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (uint32_t k = 0U; k < 360U * SENSOR_DETECT_SWEEPS && ok; k++) {
            // Variabel deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float deg = 360.0f - (float)(k % 360U);
            m->detect_phase_u16 = foc_deg_to_u16(deg);
            motor_set_foc_targets(m, m->detect.drive_current_a, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(SENSOR_DETECT_STEP_MS));
            detect_hall_sample(m);
            timeout_reset();
            if (m->fault != MOTOR_FAULT_NONE)
                ok = false;
        }
    }

    m->detect.state = SENSOR_DETECT_HALL_EVAL;
    ok = ok && detect_hall_evaluate(m);
    if (hall_table)
        memcpy(hall_table, m->detect.result_hall_table, 8U);
    if (result)
        *result = ok;
    m->detect.result_mode = SENSOR_MODE_HALL;
    detect_end(m, ok);
    return ok ? MOTOR_FAULT_NONE :
           (m->fault != MOTOR_FAULT_NONE ? (int)m->fault : MOTOR_FAULT_SENSOR_DETECT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter phase_deg: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Parameter delta_deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter step_deg: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter delay_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi detect_rotate_phase: menjalankan deteksi detect rotate phase dengan proteksi motor dan memvalidasi
// hasil sebelum parameter diterapkan.
static bool detect_rotate_phase(MotorRuntime *m, float *phase_deg, float delta_deg,
                                float current_a, float step_deg, uint32_t delay_ms) {
    if (step_deg <= 0.0f)
        step_deg = 0.72f;
    // Variabel dir: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float dir = delta_deg >= 0.0f ? 1.0f : -1.0f;
    // Variabel left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float left = fabsf(delta_deg);
    while (left > 1.0e-4f) {
        if (m->fault != MOTOR_FAULT_NONE)
            return false;
        // Variabel d: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float d = fminf(step_deg, left) * dir;
        *phase_deg += d;
        m->detect_phase_u16 = foc_deg_to_u16(*phase_deg);
        motor_set_foc_targets(m, current_a, 0.0f);
        timeout_reset();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        left -= fabsf(d);
    }
    return true;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi detect_encoder_electrical_deg: menjalankan deteksi detect encoder electrical deg dengan proteksi motor
// dan memvalidasi hasil sebelum parameter diterapkan.
static float detect_encoder_electrical_deg(MotorRuntime *m, float ratio, bool inverted) {
    // Variabel deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float deg = encoder_read_deg(m);
    if (inverted)
        deg = 360.0f - deg;
    return detect_wrap_deg(deg * ratio);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter print: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_encoder_detect_motor: menjalankan deteksi mcpwm foc encoder detect motor dengan proteksi
// motor dan memvalidasi hasil sebelum parameter diterapkan.
int mcpwm_foc_encoder_detect_motor(MotorRuntime *m, float current, bool print,
                             float *offset, float *ratio, bool *inverted) {
    (void)print;
    if (offset)
        *offset = 0.0f;
    if (ratio)
        *ratio = 0.0f;
    if (inverted)
        *inverted = false;
    if (m == NULL || m->id != MOTOR_LEFT)
        return MOTOR_FAULT_SENSOR_DETECT;

    /* Standard COMM_DETECT_ENCODER is a measurement transaction: it must not
       destroy the active incremental A/B coordinate. Save the encoder transform
       and synchronization state before detect_begin() temporarily switches the
       sensor path. In particular, do not reset TIM4/turns: physical motion during
       detection must remain visible in the existing multi-turn coordinate. */
    // Variabel old_ratio: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float old_ratio = m->encoder.electrical_ratio;
    // Variabel old_ratio_q16: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t old_ratio_q16 = m->encoder.electrical_ratio_q16;
    // Variabel old_ppc: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t old_ppc = m->encoder.phase_per_count_q16;
    // Variabel old_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    uint16_t old_offset = m->encoder.elec_offset_u16;
    // Variabel old_inv: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool old_inv = m->encoder.inverted;
    // Variabel old_session_zero: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t old_session_zero = m->encoder.session_zero_count;
    // Variabel old_synced: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool old_synced = m->encoder.synced;
    // Variabel old_motion: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool old_motion = m->encoder.motion_proved;
    // Variabel old_speed_valid: nilai kecepatan untuk target atau pengukuran.
    bool old_speed_valid = m->encoder.speed_sample_valid;

    if (!detect_begin(m, SENSOR_MODE_ENCODER, current))
        return MOTOR_FAULT_SENSOR_DETECT;

    /* VESC detect temporarily makes encoder transform identity. For A/B-only
       there is no physical I/Z index to find, so the useless generic index
       search is intentionally skipped; all ratio/direction/offset sweeps below
       follow the upstream forced-phase method. */
    m->encoder.electrical_ratio = 1.0f;
    m->encoder.electrical_ratio_q16 = 65536U;
    m->encoder.phase_per_count_q16 = (uint32_t)(((uint64_t)65536U << 16) /
                                                (uint64_t)m->encoder.cpr);
    m->encoder.elec_offset_u16 = 0U;
    m->encoder.inverted = false;
    m->encoder.session_zero_count = motor_encoder_extended_count(m);
    m->encoder.synced = false;

    m->detect.state = SENSOR_DETECT_ENCODER_LOCK0;
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = detect_ramp_id(m, SENSOR_DETECT_CURRENT_RAMP_MS);
    // Variabel phase_deg: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    float phase_deg = 0.0f;

    if (ok) {
        /* One full electrical rotation for mechanical settling/synchronizing. */
        ok = detect_rotate_phase(m, &phase_deg, 360.0f, m->detect.drive_current_a, 0.72f, 1U);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }

    /* Direction and electrical/mechanical ratio. Upstream repeats 120-degree
       forward/backward motions and circular-averages the mechanical delta. */
    // Variabel cos_sum: akumulator penjumlahan untuk averaging atau statistik.
    // Variabel sin_sum: akumulator penjumlahan untuk averaging atau statistik.
    float sin_sum = 0.0f, cos_sum = 0.0f;
    // Variabel ratio_iterations: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const unsigned ratio_iterations = 10U;
    if (ok)
        m->detect.state = SENSOR_DETECT_ENCODER_SWEEP;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0U; i < ratio_iterations && ok; i++) {
        // Variabel before: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float before = encoder_read_deg(m);
        ok = detect_rotate_phase(m, &phase_deg, 120.0f, m->detect.drive_current_a, 0.72f, 1U);
        vTaskDelay(pdMS_TO_TICKS(150U));
        // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float diff = detect_angle_diff_deg(encoder_read_deg(m), before);
        // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float r = diff * ((float)M_PI / 180.0f);
        sin_sum += sinf(r);
        cos_sum += cosf(r);
    }
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0U; i < ratio_iterations && ok; i++) {
        // Variabel before: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float before = encoder_read_deg(m);
        ok = detect_rotate_phase(m, &phase_deg, -120.0f, m->detect.drive_current_a, 0.72f, 1U);
        vTaskDelay(pdMS_TO_TICKS(150U));
        /* Reverse traversal is accumulated with the same positive convention. */
        // Variabel diff: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float diff = detect_angle_diff_deg(before, encoder_read_deg(m));
        // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float r = diff * ((float)M_PI / 180.0f);
        sin_sum += sinf(r);
        cos_sum += cosf(r);
    }

    // Variabel ratio_result: hasil sementara atau akhir suatu operasi.
    float ratio_result = 0.0f;
    // Variabel inv_result: hasil sementara atau akhir suatu operasi.
    bool inv_result = false;
    if (ok) {
        // Variabel diff_deg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float diff_deg = atan2f(sin_sum, cos_sum) * (180.0f / (float)M_PI);
        if (fabsf(diff_deg) < 0.25f)
            ok = false;
        else {
            inv_result = diff_deg < 0.0f;
            ratio_result = roundf(120.0f / fabsf(diff_deg));
            if (!detect_finite_positive(ratio_result, 1.0f,
                                        (float)SENSOR_DETECT_MAX_POLE_PAIRS)) ok = false;
        }
    }

    /* Offset: compare encoder-derived electrical phase against forced phase
       around several electrical revolutions using circular averaging. */
    // Variabel offset_result: offset kalibrasi untuk mengoreksi bias pengukuran.
    float offset_result = 0.0f;
    if (ok) {
        m->detect.state = SENSOR_DETECT_ENCODER_EVAL;
        // Variabel it_ofs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        unsigned it_ofs = (unsigned)lroundf(ratio_result * 3.0f);
        if (it_ofs < 6U)
            it_ofs = 6U;
        if (it_ofs > 120U)
            it_ofs = 120U;
        sin_sum = 0.0f;
        cos_sum = 0.0f;

        // Variabel pass: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (unsigned pass = 0U; pass < 2U && ok; pass++) {
            // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
            for (unsigned n = 0U; n < it_ofs && ok; n++) {
                // Variabel idx: indeks elemen yang sedang diproses.
                unsigned idx = pass == 0U ? n : (it_ofs - 1U - n);
                // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float target = ((float)idx / (float)it_ofs) * 360.0f * ratio_result;
                // Variabel delta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float delta = target - phase_deg;
                ok = detect_rotate_phase(m, &phase_deg, delta, m->detect.drive_current_a,
                                         fmaxf(fabsf(delta) / 40.0f, 0.72f), 2U);
                vTaskDelay(pdMS_TO_TICKS(40U));
                // Variabel enc_e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float enc_e = detect_encoder_electrical_deg(m, ratio_result, inv_result);
                // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float err = detect_angle_diff_deg(enc_e, detect_wrap_deg(phase_deg));
                // Variabel er: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
                float er = err * ((float)M_PI / 180.0f);
                sin_sum += sinf(er);
                cos_sum += cosf(er);
            }
        }
        if (ok)
            offset_result = detect_wrap_deg(atan2f(sin_sum, cos_sum) *
                                                 (180.0f / (float)M_PI));
    }

    m->detect.result_encoder_ratio = ratio_result;
    m->detect.result_encoder_inverted = inv_result;
    m->detect.result_encoder_offset_deg = offset_result;
    m->detect.result_mode = SENSOR_MODE_ENCODER;
    if (offset)
        *offset = offset_result;
    if (ratio)
        *ratio = ratio_result;
    if (inverted)
        *inverted = inv_result;

    /* Standard VESC detection returns values and restores the active config. */
    m->encoder.electrical_ratio = old_ratio;
    m->encoder.electrical_ratio_q16 = old_ratio_q16;
    m->encoder.phase_per_count_q16 = old_ppc;
    m->encoder.elec_offset_u16 = old_offset;
    m->encoder.inverted = old_inv;
    m->encoder.session_zero_count = old_session_zero;
    m->encoder.synced = old_synced;
    m->encoder.motion_proved = old_motion;
    m->encoder.speed_sample_valid = old_speed_valid;

    detect_end(m, ok);
    return ok ? MOTOR_FAULT_NONE :
           (m->fault != MOTOR_FAULT_NONE ? (int)m->fault : MOTOR_FAULT_SENSOR_DETECT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter stop_after: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter resistance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_resistance_motor: menjalankan bagian mcpwm foc measure resistance motor pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_resistance_motor(MotorRuntime *m, float current, int samples,
                                 bool stop_after, float *resistance) {
    if (resistance)
        *resistance = 0.0f;
    if (samples < 10)
        samples = 10;
    if (!detect_begin(m, m ? m->sensor_mode : SENSOR_MODE_HALL, current))
        return MOTOR_FAULT_SENSOR_DETECT;

    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = detect_ramp_id(m, 250U);
    if (ok)
        vTaskDelay(pdMS_TO_TICKS(FOC_DETECT_SETTLE_MS));
    // Variabel sum: akumulator penjumlahan untuk averaging atau statistik.
    float sum = 0.0f;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    unsigned n = 0U;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (int k = 0; k < samples && ok; k++) {
        // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
        float id = m->id_meas;
        // Variabel vd: tegangan sumbu-d keluaran regulator FOC.
        float vd = m->vd;
        if (fabsf(id) > m->detect.drive_current_a * 0.35f) {
            // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float r = fabsf(vd / id);
            if (detect_finite_positive(r, 0.0001f, 10.0f)) {
                sum += r;
                n++;
            }
        }
        if (m->fault != MOTOR_FAULT_NONE)
            ok = false;
        timeout_reset();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    if (n < (unsigned)(samples / 4))
        ok = false;
    // Variabel out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float out = ok ? (sum / (float)n) : 0.0f;
    if (resistance)
        *resistance = out;
    if (ok) {
        m->detect_resistance_ohm = out;
        m->detect_rl_valid = true;
    }

    /* This port always closes the detection transaction here. The upstream
       stop_after=false optimization is not needed because R+L has a combined
       helper below and leaving a blocking caller energized is unsafe. */
    (void)stop_after;
    detect_end(m, ok);
    return ok ? MOTOR_FAULT_NONE :
           (m->fault != MOTOR_FAULT_NONE ? (int)m->fault : MOTOR_FAULT_SENSOR_DETECT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance_motor: menjalankan bagian mcpwm foc measure inductance motor pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance_motor(MotorRuntime *m, float duty, int samples,
                                 float *curr, float *ld_lq_diff, float *inductance) {
    if (!m || !curr || !inductance)
        return MOTOR_FAULT_SENSOR_DETECT;
    /* Upstream API specifies excitation as duty. On the current-limited F103
       power stage derive a bounded equivalent current goal from Vbus/R, then
       use the same PWM-synchronous fixed-point current loop for the step. */
    // Variabel vbus: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
    float vbus = fmaxf(m->vbus_filter, m->vbus);
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = fmaxf(m->foc_motor_r, 0.01f);
    // Variabel goal: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float goal = fabsf(duty) * fmaxf(vbus, 8.0f) / r;
    goal = foc_clampf(goal, 0.20f, FOC_DETECT_MAX_CURRENT_A);
    return mcpwm_foc_measure_inductance_current_motor(m, goal, samples, curr, ld_lq_diff, inductance);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter curr_goal: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance_current_motor: menjalankan bagian mcpwm foc measure inductance current
// motor pada algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance_current_motor(MotorRuntime *m, float curr_goal,
                                         int samples, float *curr,
                                         float *ld_lq_diff, float *inductance) {
    if (m == NULL)
        return MOTOR_FAULT_SENSOR_DETECT;
    if (curr)
        *curr = 0.0f;
    if (ld_lq_diff)
        *ld_lq_diff = 0.0f;
    if (inductance)
        *inductance = 0.0f;
    if (samples < 10)
        samples = 10;
    if (!detect_begin(m, m ? m->sensor_mode : SENSOR_MODE_HALL, curr_goal))
        return MOTOR_FAULT_SENSOR_DETECT;

    // Variabel goal: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float goal = m->detect.drive_current_a;
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float r = m->detect_rl_valid ? m->detect_resistance_ohm : fmaxf(m->foc_motor_r, 0.0f);
    // Variabel lock_current: nilai arus untuk pengukuran, kendali, atau proteksi.
    const float lock_current = fminf(goal, foc_clampf(goal * 0.35f, 0.20f, 0.75f));
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = detect_ramp_id(m, 120U);
    if (ok)
        vTaskDelay(pdMS_TO_TICKS(80U));

    // Variabel l_axis: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l_axis[2] = {
        0.0f, 0.0f
    }
    ;
    // Variabel i_axis: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float i_axis[2] = {
        0.0f, 0.0f
    }
    ;
    // Variabel valid_axis: penanda validitas hasil pengukuran atau konfigurasi.
    uint32_t valid_axis[2] = {
        0U, 0U
    }
    ;
    // Variabel pulses: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    unsigned pulses = (unsigned)((samples + 23) / 24);
    if (pulses < 2U)
        pulses = 2U;
    if (pulses > 8U)
        pulses = 8U;

    /* Detection-only saliency measurement. The rotor stays electrically locked
       by Id. D-axis pulses identify Ld. Short alternating +Iq/-Iq pulses identify
       Lq while cancelling average torque. No PWM frequency, ADC trigger, HFI
       runtime mode or ISR floating point is introduced. */
    // Variabel axis: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (uint8_t axis = 0U; axis < 2U && ok; axis++) {
        // Variabel l_weighted: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float l_weighted = 0.0f;
        // Variabel i_weighted: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float i_weighted = 0.0f;
        // Variabel valid_total: penanda validitas hasil pengukuran atau konfigurasi.
        uint32_t valid_total = 0U;
        // Variabel pulse: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (unsigned pulse = 0U; pulse < pulses && ok; pulse++) {
            // Variabel q_sign: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const float q_sign = (pulse & 1U) ? -1.0f : 1.0f;
            motor_set_foc_targets(m, lock_current, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(axis == 0U ? 8U : 12U));

            m->detect.l_capture_active = false;
            m->detect.l_capture_done = false;
            m->detect.l_capture_count = 0U;
            m->detect.l_capture_axis = axis;
            __DMB();
            m->detect.l_capture_active = true;
            if (axis == 0U) {
                motor_set_foc_targets(m, goal, 0.0f);
            }
            else {
                motor_set_foc_targets(m, lock_current, q_sign * goal);
            }

            // Variabel start_tick: nilai tick scheduler untuk pengukuran waktu.
            uint32_t start_tick = xTaskGetTickCount();
            while (!m->detect.l_capture_done &&
                   (uint32_t)(xTaskGetTickCount() - start_tick) < 20U) {
                if (m->fault != MOTOR_FAULT_NONE) {
                    ok = false;
                    break;
                }
                /* Lq probing is intended for a locked rotor. Abort if the short
                   torque pulse manages to accelerate the shaft materially. */
                if (axis == 1U &&
                    abs_i32_sat_local(m->speed_est_fast_erpm_q16) > (250 * 65536)) {
                    ok = false;
                    break;
                }
                timeout_reset();
                vTaskDelay(pdMS_TO_TICKS(1U));
            }
            m->detect.l_capture_active = false;
            motor_set_foc_targets(m, lock_current, 0.0f);
            if (!m->detect.l_capture_done || m->detect.l_capture_count < 3U) {
                ok = false;
                break;
            }

            // Variabel l_pulse: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float l_pulse = 0.0f;
            // Variabel i_pulse: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float i_pulse = 0.0f;
            // Variabel n_pulse: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint16_t n_pulse = 0U;
            if (mc_math_estimate_inductance_q15(m->detect.l_capture_i_q15,
                                                m->detect.l_capture_v_prev_q15,
                                                m->detect.l_capture_count,
                                                FOC_CURRENT_Q_BASE_A,
                                                FOC_VOLTAGE_Q_BASE_V,
                                                (float)FOC_ISR_EVENT_HZ,
                                                r,
                                                &l_pulse,
                                                &i_pulse,
                                                &n_pulse)) {
                l_weighted += l_pulse * (float)n_pulse;
                i_weighted += i_pulse * (float)n_pulse;
                valid_total += n_pulse;
            }
            timeout_reset();
        }
        if (valid_total < 6U)
            ok = false;
        if (ok) {
            l_axis[axis] = 0.90f * (l_weighted / (float)valid_total);
            i_axis[axis] = i_weighted / (float)valid_total;
            valid_axis[axis] = valid_total;
            if (!detect_finite_positive(l_axis[axis], 0.2e-6f, 0.02f))
                ok = false;
        }
    }

    // Variabel ld: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ld = ok ? l_axis[0] : 0.0f;
    // Variabel lq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float lq = ok ? l_axis[1] : 0.0f;
    // Variabel l_out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float l_out = ok ? 0.5f * (ld + lq) : 0.0f;
    // Variabel ldq_out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float ldq_out = ok ? (lq - ld) : 0.0f;
    // Variabel i_out: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float i_out = ok ? 0.5f * (i_axis[0] + i_axis[1]) : 0.0f;
    (void)valid_axis;

    if (ok && (!detect_finite_positive(l_out, 0.2e-6f, 0.02f) ||
               !isfinite(ldq_out) || fabsf(ldq_out) > (1.8f * l_out))) {
        ok = false;
    }
    if (curr)
        *curr = ok ? i_out : 0.0f;
    if (inductance)
        *inductance = ok ? l_out : 0.0f;
    if (ld_lq_diff)
        *ld_lq_diff = ok ? ldq_out : 0.0f;
    if (ok) {
        m->detect_inductance_h = l_out;
        m->detect_ld_lq_diff_h = ldq_out;
        m->detect_rl_valid = true;
    }
    detect_end(m, ok);
    return ok ? MOTOR_FAULT_NONE :
           (m->fault != MOTOR_FAULT_NONE ? (int)m->fault : MOTOR_FAULT_SENSOR_DETECT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter res: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ind: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi measure_res_ind_ex: menjalankan operasi measure res ind ex sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static int measure_res_ind_ex(MotorRuntime *m, float current, int samples,
                              float *res, float *ind, float *ld_lq_diff) {
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = 0.0f;
    // Variabel fault: status atau data gangguan untuk sistem proteksi.
    int fault = mcpwm_foc_measure_resistance_motor(m, current, samples, true, &r);
    if (fault != MOTOR_FAULT_NONE)
        return fault;
    /* Preserve measured R across the separate L transaction. */
    m->detect_resistance_ohm = r;
    m->detect_rl_valid = true;
    // Variabel i_meas: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ldq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float i_meas = 0.0f, l = 0.0f, ldq = 0.0f;
    fault = mcpwm_foc_measure_inductance_current_motor(m, current, samples,
                                                 &i_meas, &ldq, &l);
    (void)i_meas;
    if (fault == MOTOR_FAULT_NONE) {
        m->detect_resistance_ohm = r;
        m->detect_inductance_h = l;
        m->detect_ld_lq_diff_h = ldq;
        m->detect_rl_valid = true;
        if (res)
            *res = r;
        if (ind)
            *ind = l;
        if (ld_lq_diff)
            *ld_lq_diff = ldq;
    }
    return fault;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter res: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ind: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_res_ind_motor: menjalankan bagian mcpwm foc measure res ind motor pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_res_ind_motor(MotorRuntime *m, float *res, float *ind, float *ld_lq_diff) {
    return measure_res_ind_ex(m, FOC_DETECT_CURRENT_A, 200, res, ind, ld_lq_diff);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter target_erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter erpm_per_sec: kecepatan listrik rotor dalam electrical RPM.
// Parameter max_duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter resistance_ohm: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter inductance_h: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter flux_wb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_flux_linkage_motor_bounded: menjalankan bagian mcpwm foc measure flux linkage motor
// bounded pada algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_flux_linkage_motor_bounded(MotorRuntime *m, float current_a,
                                   float target_erpm, float erpm_per_sec,
                                   float max_duty, float resistance_ohm,
                                   float inductance_h, float *flux_wb) {
    if (flux_wb)
        *flux_wb = 0.0f;
    if (m == NULL)
        return MOTOR_FAULT_SENSOR_DETECT;

    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = resistance_ohm;
    if (!detect_finite_positive(r, 0.00001f, 10.0f)) {
        r = m->detect_rl_valid ? m->detect_resistance_ohm : m->foc_motor_r;
    }
    if (!detect_finite_positive(r, 0.00001f, 10.0f))
        return MOTOR_FAULT_SENSOR_DETECT;

    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l = inductance_h;
    if (!isfinite(l) || l < 0.0f || l > 0.1f) {
        l = m->detect_rl_valid ? m->detect_inductance_h : m->foc_motor_l;
    }
    if (!isfinite(l) || l < 0.0f || l > 0.1f)
        l = 0.0f;

    if (!detect_begin(m, m->sensor_mode, current_a))
        return MOTOR_FAULT_SENSOR_DETECT;

    target_erpm = fminf(fmaxf(fabsf(target_erpm), 400.0f), 5000.0f);
    erpm_per_sec = fminf(fmaxf(fabsf(erpm_per_sec), 100.0f), 20000.0f);
    max_duty = fminf(fmaxf(fabsf(max_duty), 0.02f), 0.95f);

    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = detect_wait_pwm(m, 500U);
    // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
    // Variabel phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    float phase = 0.0f, erpm = 0.0f;
    motor_set_foc_targets(m, 0.0f, m->detect.drive_current_a);

    /* Ramp forced electrical speed until the requested ERPM or the command's
       duty ceiling is reached. This makes the standard flux-detect duty field
       a real safety/measurement constraint instead of a parsed-but-ignored
       parameter. */
    // Variabel ramp_limit_ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t ramp_limit_ms = (uint32_t)ceilf(target_erpm / erpm_per_sec * 1000.0f) + 1000U;
    if (ramp_limit_ms > 30000U)
        ramp_limit_ms = 30000U;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t k = 0U; k < ramp_limit_ms && ok; k++) {
        if (erpm < target_erpm) {
            erpm += erpm_per_sec * 0.001f;
            if (erpm > target_erpm)
                erpm = target_erpm;
        }
        phase += erpm * 6.0f * 0.001f; /* ERPM -> electrical deg/s */
        m->detect_phase_u16 = foc_deg_to_u16(phase);
        motor_set_foc_targets(m, 0.0f, m->detect.drive_current_a);
        vTaskDelay(pdMS_TO_TICKS(1U));
        timeout_reset();
        if (m->fault != MOTOR_FAULT_NONE)
            ok = false;
        if (ok && erpm >= 400.0f && fabsf(m->duty_now) >= max_duty)
            break;
        if (ok && erpm >= target_erpm)
            break;
    }
    if (erpm < 400.0f)
        ok = false;

    /* Hold before sampling so dIq/dt is close to zero. At steady state the
       q-axis PMSM equation is Vq = R*Iq + omega*(Ld*Id + lambda). */
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t k = 0U; k < FOC_DETECT_FLUX_SETTLE_MS && ok; k++) {
        phase += erpm * 6.0f * 0.001f;
        m->detect_phase_u16 = foc_deg_to_u16(phase);
        motor_set_foc_targets(m, 0.0f, m->detect.drive_current_a);
        vTaskDelay(pdMS_TO_TICKS(1U));
        timeout_reset();
        if (m->fault != MOTOR_FAULT_NONE)
            ok = false;
    }

    // Variabel sum: akumulator penjumlahan untuk averaging atau statistik.
    float sum = 0.0f;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    unsigned n = 0U;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t k = 0U; k < 1000U && ok; k++) {
        phase += erpm * 6.0f * 0.001f;
        m->detect_phase_u16 = foc_deg_to_u16(phase);
        motor_set_foc_targets(m, 0.0f, m->detect.drive_current_a);
        vTaskDelay(pdMS_TO_TICKS(1U));
        timeout_reset();
        if (m->fault != MOTOR_FAULT_NONE)
            ok = false;
        if (ok) {
            // Variabel omega: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float omega = erpm * (2.0f * (float)M_PI / 60.0f);
            // Variabel lambda: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float lambda = (m->vq - r * m->iq_meas) / fmaxf(omega, 1.0f) - l * m->id_meas;
            // Variabel fl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            float fl = fabsf(lambda);
            if (detect_finite_positive(fl, 1.0e-5f, 0.5f)) {
                sum += fl;
                n++;
            }
        }
    }
    if (n < 100U)
        ok = false;
    // Variabel fl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float fl = ok ? (sum / (float)n) : 0.0f;
    if (ok) {
        m->detect_flux_linkage_wb = fl;
        m->detect_flux_valid = true;
        if (flux_wb)
            *flux_wb = fl;
    }
    detect_end(m, ok);
    return ok ? MOTOR_FAULT_NONE :
           (m->fault != MOTOR_FAULT_NONE ? (int)m->fault : MOTOR_FAULT_SENSOR_DETECT);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter target_erpm: kecepatan listrik rotor dalam electrical RPM.
// Parameter erpm_per_sec: kecepatan listrik rotor dalam electrical RPM.
// Parameter flux_wb: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_flux_linkage_motor: menjalankan bagian mcpwm foc measure flux linkage motor pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_flux_linkage_motor(MotorRuntime *m, float current_a,
                                   float target_erpm, float erpm_per_sec,
                                   float *flux_wb) {
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = (m && m->detect_rl_valid) ? m->detect_resistance_ohm : (m ? m->foc_motor_r : 0.0f);
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float l = (m && m->detect_rl_valid) ? m->detect_inductance_h : (m ? m->foc_motor_l : 0.0f);
    return mcpwm_foc_measure_flux_linkage_motor_bounded(m, current_a, target_erpm,
            erpm_per_sec, 0.95f, r, l, flux_wb);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_detect_apply_all_motor: menjalankan deteksi mcpwm foc detect apply all motor dengan proteksi
// motor dan memvalidasi hasil sebelum parameter diterapkan.
int16_t mcpwm_foc_detect_apply_all_motor(MotorRuntime *m, float current_a) {
    if (m == NULL)
        return -1;
    // Variabel flux: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel ldq: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float r = 0.0f, l = 0.0f, ldq = 0.0f, flux = 0.0f;
    // Variabel fault: status atau data gangguan untuk sistem proteksi.
    int fault = measure_res_ind_ex(m, current_a, 200, &r, &l, &ldq);
    if (fault != MOTOR_FAULT_NONE)
        return -2;

    m->detect_resistance_ohm = r;
    m->detect_rl_valid = true;
    // Variabel flux_target_erpm: kecepatan listrik motor dalam electrical RPM.
    float flux_target_erpm = fabsf(m->foc_openloop_rpm);
    if (flux_target_erpm < 400.0f)
        flux_target_erpm = FOC_DETECT_FLUX_ERPM;
    fault = mcpwm_foc_measure_flux_linkage_motor(m,
            fminf(fabsf(current_a), FOC_DETECT_FLUX_CURRENT_A),
            flux_target_erpm, 3000.0f, &flux);
    if (fault != MOTOR_FAULT_NONE)
        return -10;

    m->foc_motor_r = r;
    m->res_est_state_ohm = r;
    m->res_est_ohm = r;
    m->res_est_valid = false;
    m->foc_motor_l = l;
    m->foc_motor_ld_lq_diff = ldq;
    m->foc_motor_flux_linkage = flux;

    /* Same VESC principle: synthesize current PI from R/L after detection.
       The fast controller stores these coefficients in Q16.16. */
    // Variabel bw: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float bw = 1000.0f;
    motor_set_current_pi_gains(m, l * bw, r * bw);
    foc_observer_reset(m, 0U);
    return 0;
}


/* ================= VESC-style FOC telemetry getters ================= */
// Parameter q: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi q15_current_to_a: menjalankan operasi q15 current to a sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float q15_current_to_a(int32_t q) {
    return (float)q * (FOC_CURRENT_Q_BASE_A / 32768.0f);
}

/* VESC 6.00 current_motor = SIGN(Vq * Iq) * |I_dq|. Fungsi task-side
 * ini menjaga getter API kompatibel tanpa menambahkan kerja ke ISR FOC. */
// Parameter id_q15: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Parameter iq_q15: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Parameter vq_q15: tegangan sumbu-q hasil regulator FOC.
// Fungsi vesc_signed_current_q15: menjalankan operasi vesc signed current q15 sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static float vesc_signed_current_q15(int32_t id_q15, int32_t iq_q15, int32_t vq_q15) {
    // Variabel id_a: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    const float id_a = q15_current_to_a(id_q15);
    // Variabel iq_a: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq_a = q15_current_to_a(iq_q15);
    // Variabel magnitude: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float magnitude = sqrtf(id_a * id_a + iq_a * iq_a);
    // Variabel negative: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool negative = ((vq_q15 < 0) != (iq_q15 < 0));
    return negative ? -magnitude : magnitude;
}
// Parameter q: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi q15_voltage_to_v: menjalankan operasi q15 voltage to v sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static float q15_voltage_to_v(int32_t q) {
    return (float)q * (FOC_VOLTAGE_Q_BASE_V / 32768.0f);
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_duty_cycle_now_rt: membaca mcpwm foc get duty cycle now rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_now_rt(const MotorRuntime *m) {
    return m ? m->duty_now : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_rpm_rt: membaca mcpwm foc get rpm rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_rt(const MotorRuntime *m) {
    return m ? m->erpm : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_rt: membaca mcpwm foc get tot current rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_rt(const MotorRuntime *m) {
    if (!m)
        return 0.0f;
    return vesc_signed_current_q15(m->id_q15, m->iq_q15, m->vq_q15);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_in_rt: membaca mcpwm foc get tot current in rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_rt(const MotorRuntime *m) {
    return m ? m->input_current : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_id_rt: membaca mcpwm foc get id rt tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float mcpwm_foc_get_id_rt(const MotorRuntime *m) {
    return m ? q15_current_to_a(m->id_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_iq_rt: membaca mcpwm foc get iq rt tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float mcpwm_foc_get_iq_rt(const MotorRuntime *m) {
    return m ? q15_current_to_a(m->iq_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_id_filter_rt: membaca mcpwm foc get id filter rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_filter_rt(const MotorRuntime *m) {
    return m ? q15_current_to_a(m->id_filter_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_iq_filter_rt: membaca mcpwm foc get iq filter rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_filter_rt(const MotorRuntime *m) {
    return m ? q15_current_to_a(m->iq_filter_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_id_target_rt: membaca mcpwm foc get id target rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_target_rt(const MotorRuntime *m) {
    return m ? q15_current_to_a(m->id_target_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_iq_target_rt: membaca mcpwm foc get iq target rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_target_rt(const MotorRuntime *m) {
    return m ? q15_current_to_a(m->iq_target_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_rt: membaca mcpwm foc get phase rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_rt(const MotorRuntime *m) {
    return m ? ((float)motor_sensor_electrical_phase_u16((MotorRuntime*)m) * 360.0f / 65536.0f) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_observer_rt: membaca mcpwm foc get phase observer rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_observer_rt(const MotorRuntime *m) {
    return m ? ((float)m->observer_phase_u16 * 360.0f / 65536.0f) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_encoder_rt: membaca mcpwm foc get phase encoder rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_encoder_rt(const MotorRuntime *m) {
    if (!m || m->id != MOTOR_LEFT || m->encoder.cpr == 0U)
        return 0.0f;
    // Variabel ext: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t ext = motor_encoder_extended_count((MotorRuntime*)m);
    // Variabel rel: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t rel = ext - m->encoder.session_zero_count;
    // Variabel ph: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t ph = ((int64_t)rel * m->encoder.phase_per_count_q16) >> 16;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t p = (uint16_t)ph;
    if (m->encoder.inverted)
        p = (uint16_t)(0U-p);
    p = (uint16_t)(p-m->encoder.elec_offset_u16);
    return (float)p * 360.0f / 65536.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_vd_rt: membaca mcpwm foc get vd rt tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float mcpwm_foc_get_vd_rt(const MotorRuntime *m) {
    return m ? q15_voltage_to_v(m->vd_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_vq_rt: membaca mcpwm foc get vq rt tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float mcpwm_foc_get_vq_rt(const MotorRuntime *m) {
    return m ? q15_voltage_to_v(m->vq_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_v_alpha_rt: membaca mcpwm foc get v alpha rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_v_alpha_rt(const MotorRuntime *m) {
    return m ? q15_voltage_to_v(m->observer_v_alpha_q15_prev) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_v_beta_rt: membaca mcpwm foc get v beta rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_v_beta_rt(const MotorRuntime *m) {
    return m ? q15_voltage_to_v(m->observer_v_beta_q15_prev) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_est_lambda_rt: membaca mcpwm foc get est lambda rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_est_lambda_rt(const MotorRuntime *m) {
    if (m == NULL)
        return 0.0f;
    /* Upstream reports the observer lambda estimate. This fixed-point observer
       exposes its rotor-flux vector; its magnitude is the equivalent estimate. */
    return sqrtf(m->observer_flux_alpha * m->observer_flux_alpha +
                 m->observer_flux_beta * m->observer_flux_beta);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_est_res_rt: membaca mcpwm foc get est res rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_res_rt(const MotorRuntime *m) {
    if (m == NULL)
        return 0.0f;
    if (m->res_est_valid && isfinite(m->res_est_ohm))
        return m->res_est_ohm;
    return m->detect_rl_valid ? m->detect_resistance_ohm : m->foc_motor_r;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_est_ind_rt: membaca mcpwm foc get est ind rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_ind_rt(const MotorRuntime *m) {
    if (m == NULL)
        return 0.0f;
    /* Upstream's live inductance estimator depends on HFI. HFI is explicitly
       absent here, so return the latest non-HFI R/L detection result when
       available, otherwise the configured motor inductance. */
    return m->detect_rl_valid ? m->detect_inductance_h : m->foc_motor_l;
}
// Fungsi mcpwm_foc_get_sampling_frequency_now: membaca mcpwm foc get sampling frequency now tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_sampling_frequency_now(void) {
    return (float)FOC_ISR_EVENT_HZ;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_is_using_encoder_rt: menjalankan bagian mcpwm foc is using encoder rt pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
bool mcpwm_foc_is_using_encoder_rt(const MotorRuntime *m) {
    return m && (m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB || m->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) &&
           m->sensor_mode == SENSOR_MODE_ENCODER && m->encoder.synced && m->using_encoder;
}

/* ========================================================================
 * VESC public-API parity helpers (non-HFI)
 * ======================================================================== */

// Fungsi mcpwm_foc_deinit: melepas atau menonaktifkan resource mcpwm foc deinit dengan urutan yang aman.
void mcpwm_foc_deinit(void) {
    mcpwm_foc_release_motor_motor(&g_motor_left);
    mcpwm_foc_release_motor_motor(&g_motor_right);
    s_foc_init_done = false;
}
// Fungsi mcpwm_foc_init_done: menginisialisasi mcpwm foc init done sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
bool mcpwm_foc_init_done(void) {
    return s_foc_init_done;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_configuration_rt: mengatur mcpwm foc set configuration rt setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_configuration_rt(MotorRuntime*m) {
    if (!m)
        return;
    motor_set_current_pi_gains(m, m->current_kp, m->current_ki);
    foc_observer_precalc(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi foc_state_of: menjalankan bagian foc state of pada algoritma FOC dengan skala, konvensi tanda, dan
// batas numerik yang konsisten.
static mc_state foc_state_of(const MotorRuntime*m) {
    if (!m)
        return MC_STATE_OFF;
    if (m->detect.busy)
        return MC_STATE_DETECTING;
    if (m->pwm_enabled)
        return MC_STATE_RUNNING;
    return MC_STATE_OFF;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_state_rt: membaca mcpwm foc get state rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_state mcpwm_foc_get_state_rt(const MotorRuntime*m) {
    return foc_state_of(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_control_mode_rt: menjalankan bagian mcpwm foc control mode rt pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
mc_control_mode mcpwm_foc_control_mode_rt(const MotorRuntime*m) {
    if (!m)
        return CONTROL_MODE_NONE;
    switch (m->control_mode) {
        case MOTOR_CTRL_DUTY :
            return CONTROL_MODE_DUTY;
        case MOTOR_CTRL_SPEED :
            return CONTROL_MODE_SPEED;
        case MOTOR_CTRL_CURRENT :
            return CONTROL_MODE_CURRENT;
        case MOTOR_CTRL_BRAKE_CURRENT :
            return CONTROL_MODE_CURRENT_BRAKE;
        case MOTOR_CTRL_POSITION :
            return CONTROL_MODE_POS;
        case MOTOR_CTRL_HANDBRAKE :
            return CONTROL_MODE_HANDBRAKE;
        case MOTOR_CTRL_OPENLOOP :
            return CONTROL_MODE_OPENLOOP;
        case MOTOR_CTRL_OPENLOOP_PHASE :
            return CONTROL_MODE_OPENLOOP_PHASE;
        case MOTOR_CTRL_OPENLOOP_DUTY :
            return CONTROL_MODE_OPENLOOP_DUTY;
        case MOTOR_CTRL_OPENLOOP_DUTY_PHASE :
            return CONTROL_MODE_OPENLOOP_DUTY_PHASE;
        default:
            return CONTROL_MODE_NONE;
    }
}
// Fungsi mcpwm_foc_is_dccal_done: menjalankan bagian mcpwm foc is dccal done pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
bool mcpwm_foc_is_dccal_done(void) {
    return foc_calibration_done();
}
// Fungsi mcpwm_foc_isr_motor: menangani mcpwm foc isr motor pada konteks interrupt dengan pekerjaan minimum
// agar timing FOC tetap deterministik.
int mcpwm_foc_isr_motor(void) {
    return s_isr_motor;
}
// Parameter is_second_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi mcpwm_foc_stop_pwm: menjalankan bagian mcpwm foc stop pwm pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
void mcpwm_foc_stop_pwm(bool is_second_motor) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime*m = motor_get(is_second_motor ? MOTOR_RIGHT : MOTOR_LEFT);
    motor_hw_set_pwm_enabled(m, false);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter d: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_duty_noramp_motor: mengatur mcpwm foc set duty noramp motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_duty_noramp_motor(MotorRuntime*m, float d) {
    mcpwm_foc_set_duty_motor(m, d);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_fw_override_motor: mengatur mcpwm foc set fw override motor setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_fw_override_motor(MotorRuntime*m, float current) {
    if (m) {
        m->fw_override_current_a = fabsf(current);
        // Variabel q: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        float q = (m->fw_override_current_a/FOC_CURRENT_Q_BASE_A)*32768.0f;
        if (q < 0.0f)
            q = 0.0f;
        if (q > 32767.0f)
            q = 32767.0f;
        m->fw_override_current_q15 = (int32_t)lrintf(q);
    }
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_tachometer_value_motor: mengatur mcpwm foc set tachometer value motor setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
int mcpwm_foc_set_tachometer_value_motor(MotorRuntime*m, int steps) {
    if (!m)
        return 0;
    m->stats.tachometer = steps;
    m->stats.tachometer_abs = steps < 0 ? -steps : steps;
    return steps;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_duty_cycle_set_rt: membaca mcpwm foc get duty cycle set rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_set_rt(const MotorRuntime*m) {
    return m ? m->duty_command : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_duty_cycle_abs_filter_rt: membaca mcpwm foc get duty cycle abs filter rt tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_abs_filter_rt(const MotorRuntime*m) {
    return m ? fabsf(m->duty_now) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_pid_speed_set_rt: membaca mcpwm foc get pid speed set rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_speed_set_rt(const MotorRuntime*m) {
    return m ? m->speed_target_erpm : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_pid_pos_set_rt: membaca mcpwm foc get pid pos set rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_pos_set_rt(const MotorRuntime*m) {
    return m ? m->position_target_deg : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_pid_pos_now_rt: membaca mcpwm foc get pid pos now rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_pos_now_rt(const MotorRuntime*m) {
    return m ? m->position_deg : 0.0f;
}
// Fungsi mcpwm_foc_get_switching_frequency_now: membaca mcpwm foc get switching frequency now tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_switching_frequency_now(void) {
    return (float)VESC_FOC_F_ZV_HZ;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_rpm_fast_rt: membaca mcpwm foc get rpm fast rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_fast_rt(const MotorRuntime*m) {
    return m ? (float)m->speed_est_fast_erpm_q16/65536.0f : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_rpm_faster_rt: membaca mcpwm foc get rpm faster rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_faster_rt(const MotorRuntime*m) {
    return m ? (float)m->speed_est_faster_erpm_q16/65536.0f : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_filtered_rt: membaca mcpwm foc get tot current filtered rt tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_filtered_rt(const MotorRuntime*m) {
    if (!m)
        return 0.0f;
    return vesc_signed_current_q15(m->id_filter_q15, m->iq_filter_q15, m->vq_q15);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_abs_motor_current_rt: membaca mcpwm foc get abs motor current rt tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_rt(const MotorRuntime*m) {
    if (!m)
        return 0.0f;
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    float id = q15_current_to_a(m->id_q15);
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq = q15_current_to_a(m->iq_q15);
    return sqrtf(id*id+iq*iq);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_abs_motor_current_unbalance_rt: membaca mcpwm foc get abs motor current unbalance rt
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_unbalance_rt(const MotorRuntime*m) {
    return m ? fabsf(m->ia+m->ib+m->ic) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_abs_motor_voltage_rt: membaca mcpwm foc get abs motor voltage rt tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_voltage_rt(const MotorRuntime*m) {
    if (!m)
        return 0.0f;
    return sqrtf(m->vd*m->vd+m->vq*m->vq);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_abs_motor_current_filtered_rt: membaca mcpwm foc get abs motor current filtered rt tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_filtered_rt(const MotorRuntime*m) {
    if (!m)
        return 0.0f;
    return sqrtf(q15_current_to_a(m->id_filter_q15)*q15_current_to_a(m->id_filter_q15)+
        q15_current_to_a(m->iq_filter_q15)*q15_current_to_a(m->iq_filter_q15));
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_directional_rt: membaca mcpwm foc get tot current directional rt tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_directional_rt(const MotorRuntime*m) {
    return m ? q15_current_to_a(m->iq_q15) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_directional_filtered_rt: membaca mcpwm foc get tot current directional
// filtered rt tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_directional_filtered_rt(const MotorRuntime*m) {
    if (!m)
        return 0.0f;
    /* iq_filter_q15 carries the signed PI output. Use it directly for the
       directional filtered current so the sign follows the actual output, not
       the unfiltered current sign that may have changed between samples. */
    // Variabel iq_a: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    float iq_a = q15_current_to_a(m->iq_filter_q15);
    return iq_a;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_id_set_rt: membaca mcpwm foc get id set rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_set_rt(const MotorRuntime*m) {
    return m ? m->id_target : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_iq_set_rt: membaca mcpwm foc get iq set rt tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_set_rt(const MotorRuntime*m) {
    return m ? m->iq_target : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_in_filtered_rt: membaca mcpwm foc get tot current in filtered rt tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_filtered_rt(const MotorRuntime*m) {
    return mcpwm_foc_get_tot_current_in_rt(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tachometer_value_rt: membaca mcpwm foc get tachometer value rt tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
int mcpwm_foc_get_tachometer_value_rt(MotorRuntime*m, bool reset) {
    if (!m)
        return 0;
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int v = m->stats.tachometer;
    if (reset)
        m->stats.tachometer = 0;
    return v;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tachometer_abs_value_rt: membaca mcpwm foc get tachometer abs value rt tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
int mcpwm_foc_get_tachometer_abs_value_rt(MotorRuntime*m, bool reset) {
    if (!m)
        return 0;
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int v = m->stats.tachometer_abs;
    if (reset)
        m->stats.tachometer_abs = 0;
    return v;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_bemf_rt: membaca mcpwm foc get phase bemf rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_bemf_rt(const MotorRuntime *m) {
    if (m == NULL)
        return 0.0f;
    // Variabel phase_bemf: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
    float phase_bemf = atan2f(mcpwm_foc_get_v_beta_rt(m), mcpwm_foc_get_v_alpha_rt(m)) * (180.0f / (float)M_PI);
    // Variabel speed: nilai kecepatan untuk target atau pengukuran.
    float speed = mcpwm_foc_get_rpm_fast_rt(m);
    if (speed > 0.0f)
        phase_bemf -= 90.0f;
    else if (speed < 0.0f) {
        phase_bemf += 90.0f;
    }
    while (phase_bemf >= 360.0f)
        phase_bemf -= 360.0f;
    while (phase_bemf < 0.0f)
        phase_bemf += 360.0f;
    return phase_bemf;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_phase_hall_rt: membaca mcpwm foc get phase hall rt tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_hall_rt(const MotorRuntime*m) {
    if (!m || !m->hall.valid)
        return 0.0f;
    return (float)m->hall.base_phase_u16*360.0f/65536.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi norm_mod: menjalankan operasi norm mod sesuai tanggung jawab modul dengan input tervalidasi dan state
// yang konsisten.
static float norm_mod(const MotorRuntime*m, float v) {
    // Variabel bus: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float bus = m ? fmaxf(m->vbus_filter, 1.0f) : 1.0f;
    return v/bus;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_mod_alpha_raw_rt: membaca mcpwm foc get mod alpha raw rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_alpha_raw_rt(const MotorRuntime*m) {
    return m ? norm_mod(m, mcpwm_foc_get_v_alpha_rt(m)) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_mod_beta_raw_rt: membaca mcpwm foc get mod beta raw rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_beta_raw_rt(const MotorRuntime*m) {
    return m ? norm_mod(m, mcpwm_foc_get_v_beta_rt(m)) : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_mod_alpha_measured_rt: membaca mcpwm foc get mod alpha measured rt tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_alpha_measured_rt(const MotorRuntime*m) {
    return mcpwm_foc_get_mod_alpha_raw_rt(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_mod_beta_measured_rt: membaca mcpwm foc get mod beta measured rt tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_beta_measured_rt(const MotorRuntime*m) {
    return mcpwm_foc_get_mod_beta_raw_rt(m);
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter current_q15: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi current_q15_to_adc_delta: menjalankan operasi current q15 to adc delta sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static int32_t current_q15_to_adc_delta(const MotorRuntime *m, int32_t current_q15) {
    if (m == NULL || m->current_scale_q16 == 0)
        return 0;
    return (int32_t)(((int64_t)current_q15 * 65536LL) / (int64_t)m->current_scale_q16);
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_current_offsets: membaca mcpwm foc get current offsets tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_current_offsets(volatile float *a, volatile float *b, volatile float *c, bool second) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT);
    if (m == NULL)
        return;
    // Variabel synthetic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t synthetic = (m->current_offset_u_counts + m->current_offset_v_counts) / 2;
    if (m->id == MOTOR_LEFT) {
        if (a)
            *a = (float)m->current_offset_u_counts; /* phase A */
        if (b)
            *b = (float)m->current_offset_v_counts; /* phase B */
        if (c)
            *c = (float)synthetic; /* reconstructed C */
    }
    else {
        if (a)
            *a = (float)synthetic; /* reconstructed A */
        if (b)
            *b = (float)m->current_offset_u_counts; /* physical phase B */
        if (c)
            *c = (float)m->current_offset_v_counts; /* physical phase C */
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_current_offsets_motor: mengatur mcpwm foc set current offsets motor setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_offsets_motor(MotorRuntime *m, volatile float a, volatile float b, volatile float c) {
    if (m == NULL)
        return;
    if (m->id == MOTOR_LEFT) {
        m->current_offset_u_counts = (int32_t)lrintf(a);
        m->current_offset_v_counts = (int32_t)lrintf(b);
        m->current_offset_u = a;
        m->current_offset_v = b;
    }
    else {
        m->current_offset_u_counts = (int32_t)lrintf(b);
        m->current_offset_v_counts = (int32_t)lrintf(c);
        m->current_offset_u = b;
        m->current_offset_v = c;
    }
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_voltage_offsets: membaca mcpwm foc get voltage offsets tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_voltage_offsets(float *a, float *b, float *c, bool second) {
    /* This hoverboard board has no three phase-voltage ADC channels. Keep the
       VESC API deterministic rather than inventing measurements. */
    (void)second;
    if (a)
        *a = 0.0f;
    if (b)
        *b = 0.0f;
    if (c)
        *c = 0.0f;
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_voltage_offsets_undriven: membaca mcpwm foc get voltage offsets undriven tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_voltage_offsets_undriven(float *a, float *b, float *c, bool second) {
    mcpwm_foc_get_voltage_offsets(a, b, c, second);
}

// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_currents_adc: membaca mcpwm foc get currents adc tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void mcpwm_foc_get_currents_adc(float *a, float *b, float *c, bool second) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT);
    if (m == NULL)
        return;
    // Variabel synthetic_offset: offset kalibrasi untuk mengoreksi bias pengukuran.
    int32_t synthetic_offset = (m->current_offset_u_counts + m->current_offset_v_counts) / 2;
    if (m->id == MOTOR_LEFT) {
        if (a)
            *a = (float)m->current_raw_u;
        if (b)
            *b = (float)m->current_raw_v;
        if (c)
            *c = (float)(synthetic_offset - current_q15_to_adc_delta(m, m->ic_q15));
    }
    else {
        if (a)
            *a = (float)(synthetic_offset - current_q15_to_adc_delta(m, m->ia_q15));
        if (b)
            *b = (float)m->current_raw_u;
        if (c)
            *c = (float)m->current_raw_v;
    }
}
// Fungsi mcpwm_foc_get_ts: membaca mcpwm foc get ts tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_ts(void) {
    return 1.0f/(float)VESC_FOC_F_ZV_HZ;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter x1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter x2: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_observer_state_rt: membaca mcpwm foc get observer state rt tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
void mcpwm_foc_get_observer_state_rt(const MotorRuntime*m, float*x1, float*x2) {
    if (x1)
        *x1 = m ? m->observer_flux_alpha : 0.0f;
    if (x2)
        *x2 = m ? m->observer_flux_beta : 0.0f;
}
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter delay_s: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_set_current_off_delay_motor: mengatur mcpwm foc set current off delay motor setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_off_delay_motor(MotorRuntime *m, float delay_s) {
    if (m == NULL)
        return;
    if (delay_s < 0.0f)
        delay_s = 0.0f;
    if (m->current_off_delay_s < delay_s)
        m->current_off_delay_s = delay_s;
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_motor: membaca mcpwm foc get tot current motor tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_motor(bool second) {
    return mcpwm_foc_get_tot_current_rt(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_filtered_motor: membaca mcpwm foc get tot current filtered motor tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_filtered_motor(bool second) {
    return mcpwm_foc_get_tot_current_filtered_rt(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_in_motor: membaca mcpwm foc get tot current in motor tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_motor(bool second) {
    return mcpwm_foc_get_tot_current_in_rt(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_tot_current_in_filtered_motor: membaca mcpwm foc get tot current in filtered motor tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_filtered_motor(bool second) {
    return mcpwm_foc_get_tot_current_in_filtered_rt(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_abs_motor_current_motor: membaca mcpwm foc get abs motor current motor tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_motor(bool second) {
    return mcpwm_foc_get_abs_motor_current_rt(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_abs_motor_current_filtered_motor: membaca mcpwm foc get abs motor current filtered motor
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_filtered_motor(bool second) {
    return mcpwm_foc_get_abs_motor_current_filtered_rt(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}
// Parameter second: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_get_state_motor: membaca mcpwm foc get state motor tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
mc_state mcpwm_foc_get_state_motor(bool second) {
    return foc_state_of(motor_get(second ? MOTOR_RIGHT : MOTOR_LEFT));
}

/* This board has a dedicated PA4 buzzer, so the VESC motor-audio API is
 * mapped to that output instead of injecting acoustic voltage into a motor. */
// Parameter cal_undriven: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mcpwm_foc_dc_cal: menangani kalibrasi mcpwm foc dc cal agar offset atau parameter hasil ukur valid
// sebelum dipakai kendali.
int mcpwm_foc_dc_cal(bool cal_undriven) {
    (void)cal_undriven;
    foc_request_recalibration();
    // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t st = xTaskGetTickCount();
    while (!foc_calibration_done() && (uint32_t)(xTaskGetTickCount()-st) < 10000U)
        vTaskDelay(pdMS_TO_TICKS(1));
    return foc_calibration_valid() ? MOTOR_FAULT_NONE : MOTOR_FAULT_CURRENT_OFFSET;
}
// Fungsi mcpwm_foc_print_state: menjalankan bagian mcpwm foc print state pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void mcpwm_foc_print_state(void) {
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = mc_interface_motor_runtime_now();
    if (m == NULL) {
        commands_send_print("FOC state: no selected motor");
        return;
    }

    // Variabel msg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    char msg[128];
    // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
    int erpm = (int)lrintf(m->erpm);
    // Variabel id_ma: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
    int id_ma = (int)lrintf(m->id_meas * 1000.0f);
    // Variabel iq_ma: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    int iq_ma = (int)lrintf(m->iq_meas * 1000.0f);
    // Variabel duty_milli: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
    int duty_milli = (int)lrintf(m->duty_now * 1000.0f);
    (void)snprintf(msg, sizeof(msg),
                   "M%u state=%d mode=%d erpm=%d duty_m=%d id_mA=%d iq_mA=%d fault=%d",
                   (unsigned)m->id + 1U, (int)foc_state_of(m),
                   (int)mcpwm_foc_control_mode_rt(m), erpm, duty_milli,
                   id_ma, iq_ma, (int)m->fault);
    commands_send_print(msg);
}

/* ========================================================================
 * VESC master public API wrappers.
 *
 * The fixed-point F103 backend keeps explicit MotorRuntime helpers for the
 * dual-motor fast path. Public functions intentionally keep the same calling
 * convention as vedderb/bldc and resolve the selected motor through
 * mc_interface_select_motor_thread(), exactly where upstream relies on its
 * motor-thread selection context.
 * ======================================================================== */
// Fungsi foc_selected_motor: menjalankan bagian foc selected motor pada algoritma FOC dengan skala, konvensi
// tanda, dan batas numerik yang konsisten.
static inline MotorRuntime *foc_selected_motor(void) {
    return mc_interface_motor_runtime_now();
}

// Parameter conf_m1: data konfigurasi yang menentukan perilaku firmware.
// Parameter conf_m2: data konfigurasi yang menentukan perilaku firmware.
// Fungsi mcpwm_foc_init: menginisialisasi mcpwm foc init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2) {
    mcpwm_foc_init_hw();
    // Variabel old: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int old = mc_interface_get_motor_thread();
    if (conf_m1) {
        mc_interface_select_motor_thread(1);
        mc_interface_set_configuration(conf_m1);
    }
    if (conf_m2) {
        mc_interface_select_motor_thread(2);
        mc_interface_set_configuration(conf_m2);
    }
    mc_interface_select_motor_thread(old);
}
// Parameter configuration: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi mcpwm_foc_set_configuration: mengatur mcpwm foc set configuration setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_configuration(mc_configuration *configuration) {
    if (!configuration || configuration->motor_type != MOTOR_TYPE_FOC)
        return;
    mc_interface_set_configuration(configuration);
    mcpwm_foc_set_configuration_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_state: membaca mcpwm foc get state tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
mc_state mcpwm_foc_get_state(void) {
    return mcpwm_foc_get_state_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_control_mode: menjalankan bagian mcpwm foc control mode pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
mc_control_mode mcpwm_foc_control_mode(void) {
    return mcpwm_foc_control_mode_rt(foc_selected_motor());
}
// Parameter d: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_duty: mengatur mcpwm foc set duty setelah nilai masukan divalidasi dan dibatasi sesuai
// aturan keselamatan modul.
void mcpwm_foc_set_duty(float d) {
    mcpwm_foc_set_duty_motor(foc_selected_motor(), d);
}
// Parameter d: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_duty_noramp: mengatur mcpwm foc set duty noramp setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_duty_noramp(float d) {
    mcpwm_foc_set_duty_noramp_motor(foc_selected_motor(), d);
}
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mcpwm_foc_set_pid_speed: mengatur mcpwm foc set pid speed setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_speed(float rpm) {
    mcpwm_foc_set_pid_speed_motor(foc_selected_motor(), rpm);
}
// Parameter pos: nilai posisi rotor/aktuator yang diukur atau dijadikan target.
// Fungsi mcpwm_foc_set_pid_pos: mengatur mcpwm foc set pid pos setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void mcpwm_foc_set_pid_pos(float pos) {
    mcpwm_foc_set_pid_pos_motor(foc_selected_motor(), pos);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_current: mengatur mcpwm foc set current setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void mcpwm_foc_set_current(float current) {
    mcpwm_foc_set_current_motor(foc_selected_motor(), current);
}
// Fungsi mcpwm_foc_release_motor: menjalankan bagian mcpwm foc release motor pada algoritma FOC dengan skala,
// konvensi tanda, dan batas numerik yang konsisten.
void mcpwm_foc_release_motor(void) {
    mcpwm_foc_release_motor_motor(foc_selected_motor());
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_brake_current: mengatur mcpwm foc set brake current setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_brake_current(float current) {
    mcpwm_foc_set_brake_current_motor(foc_selected_motor(), current);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_handbrake: mengatur mcpwm foc set handbrake setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_handbrake(float current) {
    mcpwm_foc_set_handbrake_motor(foc_selected_motor(), current);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mcpwm_foc_set_openloop_current: mengatur mcpwm foc set openloop current setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_current(float current, float rpm) {
    mcpwm_foc_set_openloop_current_motor(foc_selected_motor(), current, rpm);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_phase: mengatur mcpwm foc set openloop phase setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_phase(float current, float phase) {
    mcpwm_foc_set_openloop_phase_motor(foc_selected_motor(), current, phase);
}
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter rpm: kecepatan putar yang digunakan sebagai target atau hasil pengukuran.
// Fungsi mcpwm_foc_set_openloop_duty: mengatur mcpwm foc set openloop duty setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty(float duty, float rpm) {
    mcpwm_foc_set_openloop_duty_motor(foc_selected_motor(), duty, rpm);
}
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter phase: sudut atau data fasa listrik untuk transformasi dan komutasi FOC.
// Fungsi mcpwm_foc_set_openloop_duty_phase: mengatur mcpwm foc set openloop duty phase setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_openloop_duty_phase(float duty, float phase) {
    mcpwm_foc_set_openloop_duty_phase_motor(foc_selected_motor(), duty, phase);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi mcpwm_foc_set_fw_override: mengatur mcpwm foc set fw override setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_fw_override(float current) {
    mcpwm_foc_set_fw_override_motor(foc_selected_motor(), current);
}
// Parameter steps: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_tachometer_value: mengatur mcpwm foc set tachometer value setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
int mcpwm_foc_set_tachometer_value(int steps) {
    return mcpwm_foc_set_tachometer_value_motor(foc_selected_motor(), steps);
}
// Fungsi mcpwm_foc_get_duty_cycle_set: membaca mcpwm foc get duty cycle set tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_set(void) {
    return mcpwm_foc_get_duty_cycle_set_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_duty_cycle_now: membaca mcpwm foc get duty cycle now tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_now(void) {
    return mcpwm_foc_get_duty_cycle_now_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_duty_cycle_abs_filter: membaca mcpwm foc get duty cycle abs filter tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_duty_cycle_abs_filter(void) {
    return mcpwm_foc_get_duty_cycle_abs_filter_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_pid_speed_set: membaca mcpwm foc get pid speed set tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_speed_set(void) {
    return mcpwm_foc_get_pid_speed_set_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_pid_pos_set: membaca mcpwm foc get pid pos set tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_pos_set(void) {
    return mcpwm_foc_get_pid_pos_set_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_pid_pos_now: membaca mcpwm foc get pid pos now tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_pid_pos_now(void) {
    return mcpwm_foc_get_pid_pos_now_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_rpm: membaca mcpwm foc get rpm tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_rpm(void) {
    return mcpwm_foc_get_rpm_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_rpm_fast: membaca mcpwm foc get rpm fast tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_fast(void) {
    return mcpwm_foc_get_rpm_fast_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_rpm_faster: membaca mcpwm foc get rpm faster tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_rpm_faster(void) {
    return mcpwm_foc_get_rpm_faster_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_tot_current: membaca mcpwm foc get tot current tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current(void) {
    return mcpwm_foc_get_tot_current_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_tot_current_filtered: membaca mcpwm foc get tot current filtered tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_filtered(void) {
    return mcpwm_foc_get_tot_current_filtered_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_abs_motor_current: membaca mcpwm foc get abs motor current tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current(void) {
    return mcpwm_foc_get_abs_motor_current_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_abs_motor_current_unbalance: membaca mcpwm foc get abs motor current unbalance tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_unbalance(void) {
    return mcpwm_foc_get_abs_motor_current_unbalance_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_abs_motor_voltage: membaca mcpwm foc get abs motor voltage tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_voltage(void) {
    return mcpwm_foc_get_abs_motor_voltage_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_abs_motor_current_filtered: membaca mcpwm foc get abs motor current filtered tanpa
// mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_abs_motor_current_filtered(void) {
    return mcpwm_foc_get_abs_motor_current_filtered_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_tot_current_directional: membaca mcpwm foc get tot current directional tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_directional(void) {
    return mcpwm_foc_get_tot_current_directional_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_tot_current_directional_filtered: membaca mcpwm foc get tot current directional filtered
// tanpa mengubah state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_directional_filtered(void) {
    return mcpwm_foc_get_tot_current_directional_filtered_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_id: membaca mcpwm foc get id tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_id(void) {
    return mcpwm_foc_get_id_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_iq: membaca mcpwm foc get iq tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_iq(void) {
    return mcpwm_foc_get_iq_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_id_set: membaca mcpwm foc get id set tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_set(void) {
    return mcpwm_foc_get_id_set_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_iq_set: membaca mcpwm foc get iq set tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_set(void) {
    return mcpwm_foc_get_iq_set_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_id_target: membaca mcpwm foc get id target tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_target(void) {
    return mcpwm_foc_get_id_target_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_iq_target: membaca mcpwm foc get iq target tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_target(void) {
    return mcpwm_foc_get_iq_target_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_id_filter: membaca mcpwm foc get id filter tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_id_filter(void) {
    return mcpwm_foc_get_id_filter_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_iq_filter: membaca mcpwm foc get iq filter tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_iq_filter(void) {
    return mcpwm_foc_get_iq_filter_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_tot_current_in: membaca mcpwm foc get tot current in tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in(void) {
    return mcpwm_foc_get_tot_current_in_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_tot_current_in_filtered: membaca mcpwm foc get tot current in filtered tanpa mengubah
// state kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_tot_current_in_filtered(void) {
    return mcpwm_foc_get_tot_current_in_filtered_rt(foc_selected_motor());
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tachometer_value: membaca mcpwm foc get tachometer value tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
int mcpwm_foc_get_tachometer_value(bool reset) {
    return mcpwm_foc_get_tachometer_value_rt(foc_selected_motor(), reset);
}
// Parameter reset: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_tachometer_abs_value: membaca mcpwm foc get tachometer abs value tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
int mcpwm_foc_get_tachometer_abs_value(bool reset) {
    return mcpwm_foc_get_tachometer_abs_value_rt(foc_selected_motor(), reset);
}
// Fungsi mcpwm_foc_get_phase: membaca mcpwm foc get phase tanpa mengubah state kendali utama dan mengembalikan
// data yang konsisten.
float mcpwm_foc_get_phase(void) {
    return mcpwm_foc_get_phase_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_phase_observer: membaca mcpwm foc get phase observer tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_observer(void) {
    return mcpwm_foc_get_phase_observer_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_phase_bemf: membaca mcpwm foc get phase bemf tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_bemf(void) {
    return mcpwm_foc_get_phase_bemf_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_phase_encoder: membaca mcpwm foc get phase encoder tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_encoder(void) {
    return mcpwm_foc_get_phase_encoder_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_phase_hall: membaca mcpwm foc get phase hall tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_phase_hall(void) {
    return mcpwm_foc_get_phase_hall_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_vd: membaca mcpwm foc get vd tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_vd(void) {
    return mcpwm_foc_get_vd_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_vq: membaca mcpwm foc get vq tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
float mcpwm_foc_get_vq(void) {
    return mcpwm_foc_get_vq_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_mod_alpha_raw: membaca mcpwm foc get mod alpha raw tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_alpha_raw(void) {
    return mcpwm_foc_get_mod_alpha_raw_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_mod_beta_raw: membaca mcpwm foc get mod beta raw tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_beta_raw(void) {
    return mcpwm_foc_get_mod_beta_raw_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_mod_alpha_measured: membaca mcpwm foc get mod alpha measured tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_alpha_measured(void) {
    return mcpwm_foc_get_mod_alpha_measured_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_mod_beta_measured: membaca mcpwm foc get mod beta measured tanpa mengubah state kendali
// utama dan mengembalikan data yang konsisten.
float mcpwm_foc_get_mod_beta_measured(void) {
    return mcpwm_foc_get_mod_beta_measured_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_v_alpha: membaca mcpwm foc get v alpha tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_v_alpha(void) {
    return mcpwm_foc_get_v_alpha_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_v_beta: membaca mcpwm foc get v beta tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_v_beta(void) {
    return mcpwm_foc_get_v_beta_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_est_lambda: membaca mcpwm foc get est lambda tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_lambda(void) {
    return mcpwm_foc_get_est_lambda_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_est_res: membaca mcpwm foc get est res tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_res(void) {
    return mcpwm_foc_get_est_res_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_get_est_ind: membaca mcpwm foc get est ind tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float mcpwm_foc_get_est_ind(void) {
    return mcpwm_foc_get_est_ind_rt(foc_selected_motor());
}
// Fungsi mcpwm_foc_is_using_encoder: menjalankan bagian mcpwm foc is using encoder pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
bool mcpwm_foc_is_using_encoder(void) {
    return mcpwm_foc_is_using_encoder_rt(foc_selected_motor());
}

// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter print: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter offset: offset kalibrasi untuk menghilangkan bias pembacaan sensor.
// Parameter ratio: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter inverted: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_encoder_detect: menjalankan deteksi mcpwm foc encoder detect dengan proteksi motor dan
// memvalidasi hasil sebelum parameter diterapkan.
int mcpwm_foc_encoder_detect(float current, bool print, float *offset, float *ratio, bool *inverted) {
    return mcpwm_foc_encoder_detect_motor(foc_selected_motor(), current, print, offset, ratio, inverted);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter stop_after: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter resistance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_resistance: menjalankan bagian mcpwm foc measure resistance pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_resistance(float current, int samples, bool stop_after, float *resistance) {
    return mcpwm_foc_measure_resistance_motor(foc_selected_motor(), current, samples, stop_after, resistance);
}
// Parameter duty: rasio duty PWM yang digunakan atau dilaporkan inverter.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance: menjalankan bagian mcpwm foc measure inductance pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff, float *inductance) {
    return mcpwm_foc_measure_inductance_motor(foc_selected_motor(), duty, samples, curr, ld_lq_diff, inductance);
}
// Parameter goal: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter samples: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter curr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter inductance: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_inductance_current: menjalankan bagian mcpwm foc measure inductance current pada
// algoritma FOC dengan skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_inductance_current(float goal, int samples, float *curr, float *ld_lq_diff, float *inductance) {
    return mcpwm_foc_measure_inductance_current_motor(foc_selected_motor(), goal, samples, curr, ld_lq_diff, inductance);
}
// Parameter res: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ind: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter ld_lq_diff: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_measure_res_ind: menjalankan bagian mcpwm foc measure res ind pada algoritma FOC dengan
// skala, konvensi tanda, dan batas numerik yang konsisten.
int mcpwm_foc_measure_res_ind(float *res, float *ind, float *ld_lq_diff) {
    return mcpwm_foc_measure_res_ind_motor(foc_selected_motor(), res, ind, ld_lq_diff);
}
// Parameter current: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Parameter hall_table: data sensor Hall untuk menentukan sektor dan posisi rotor.
// Parameter result: hasil sementara atau akhir dari operasi yang sedang dijalankan.
// Fungsi mcpwm_foc_hall_detect: menjalankan deteksi mcpwm foc hall detect dengan proteksi motor dan memvalidasi
// hasil sebelum parameter diterapkan.
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result) {
    return mcpwm_foc_hall_detect_motor(foc_selected_motor(), current, hall_table, result);
}
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_set_current_offsets: mengatur mcpwm foc set current offsets setelah nilai masukan divalidasi
// dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_offsets(volatile float a, volatile float b, volatile float c) {
    mcpwm_foc_set_current_offsets_motor(foc_selected_motor(), a, b, c);
}
// Parameter x1: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter x2: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_get_observer_state: membaca mcpwm foc get observer state tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
void mcpwm_foc_get_observer_state(float *x1, float *x2) {
    mcpwm_foc_get_observer_state_rt(foc_selected_motor(), x1, x2);
}
// Parameter delay_sec: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi mcpwm_foc_set_current_off_delay: mengatur mcpwm foc set current off delay setelah nilai masukan
// divalidasi dan dibatasi sesuai aturan keselamatan modul.
void mcpwm_foc_set_current_off_delay(float delay_sec) {
    mcpwm_foc_set_current_off_delay_motor(foc_selected_motor(), delay_sec);
}
// Parameter adc_words: lima word dari frame dual-ADC V15 terbaru.
// Fungsi mcpwm_foc_adc_words_isr: entry kompatibilitas untuk test/simulasi yang menangkap timestamp saat dipanggil.
void mcpwm_foc_adc_words_isr(const volatile uint32_t adc_words[ADC_WORDS_PER_FRAME]) {
    // Variabel start_cycle: timestamp lokal untuk pemanggil non-DMA seperti test dan simulasi.
    const uint32_t start_cycle = DWT->CYCCNT;
    mcpwm_foc_adc_words_isr_timed(adc_words, start_cycle);
}

// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter flags: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi mcpwm_foc_adc_int_handler: menangani mcpwm foc adc int handler pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
void mcpwm_foc_adc_int_handler(void *p, uint32_t flags) {
    (void)flags;
    if (p)
        mcpwm_foc_adc_words_isr((const volatile uint32_t *)p);
}
