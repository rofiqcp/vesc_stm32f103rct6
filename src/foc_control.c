#include "foc_control.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_math.h"
#include "app_config.h"
#include "debug_sample.h"
#include <limits.h>
#include <stddef.h>

static volatile bool s_cal_done = false;
static volatile bool s_cal_valid = false;
static volatile bool s_cal_request = false;
static uint32_t s_cal_count = 0U;
static uint32_t s_cal_warmup = 0U;
static uint64_t s_sum_lu = 0U, s_sum_lv = 0U, s_sum_ldc = 0U;
static uint64_t s_sum_ru = 0U, s_sum_rv = 0U, s_sum_rdc = 0U;
static uint64_t s_sq_lu = 0U, s_sq_lv = 0U, s_sq_ldc = 0U;
static uint64_t s_sq_ru = 0U, s_sq_rv = 0U, s_sq_rdc = 0U;
static uint16_t s_min_lu, s_min_lv, s_min_ldc, s_min_ru, s_min_rv, s_min_rdc;
static uint16_t s_max_lu, s_max_lv, s_max_ldc, s_max_ru, s_max_rv, s_max_rdc;
static uint8_t s_overrun_consecutive[2] = {0U, 0U};
static int32_t s_inv_vbus_q30 = 0;
static int32_t s_inv_vbus_last_q15 = 0;
static uint8_t s_inv_vbus_age = 0U;
static volatile uint32_t s_adc_isr_count = 0U;
static volatile uint16_t s_cal_warn_mask = 0U;
static volatile uint16_t s_cal_fail_range_mask = 0U;
static volatile uint16_t s_cal_fail_noise_mask = 0U;

static int32_t s_vbus_scale_q16;
static int32_t s_vbus_min_q15;
static int32_t s_vbus_max_q15;
static int32_t s_current_trip_q15;
static int32_t s_mod_limit_coeff_q15;

static inline uint16_t low16(uint32_t w)  { return (uint16_t)(w & 0xFFFFU); }
static inline uint16_t high16(uint32_t w) { return (uint16_t)(w >> 16); }
static inline int32_t iabs32(int32_t x)   { return (x < 0) ? -x : x; }
static inline int32_t q15_mul_q16(int32_t a_q15, int32_t b_q16) {
    return (int32_t)(((int64_t)a_q15 * (int64_t)b_q16) >> 16);
}

static void cal_reset_isr(void) {
    s_cal_done = false;
    s_cal_valid = false;
    s_cal_request = false;
    s_cal_count = 0U;
    s_cal_warmup = 0U;
    s_sum_lu = s_sum_lv = s_sum_ldc = 0U;
    s_sum_ru = s_sum_rv = s_sum_rdc = 0U;
    s_sq_lu = s_sq_lv = s_sq_ldc = 0U;
    s_sq_ru = s_sq_rv = s_sq_rdc = 0U;
    s_min_lu = s_min_lv = s_min_ldc = s_min_ru = s_min_rv = s_min_rdc = UINT16_MAX;
    s_max_lu = s_max_lv = s_max_ldc = s_max_ru = s_max_rv = s_max_rdc = 0U;
    s_cal_warn_mask = 0U;
    s_cal_fail_range_mask = 0U;
    s_cal_fail_noise_mask = 0U;
}

bool foc_calibration_done(void) { return s_cal_done; }
bool foc_calibration_valid(void) { return s_cal_done && s_cal_valid; }

uint32_t foc_adc_isr_count(void) { return s_adc_isr_count; }

void foc_get_calibration_progress(uint32_t *count, uint32_t *target) {
    if (count != NULL) *count = s_cal_count;
    if (target != NULL) *target = ADC_OFFSET_CAL_SAMPLES;
}

void foc_request_recalibration(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_cal_request = true;
    if (!primask) __enable_irq();
}

void foc_control_init(void) {
    cal_reset_isr();
    s_overrun_consecutive[0] = 0U;
    s_overrun_consecutive[1] = 0U;
    s_inv_vbus_q30 = 0; s_inv_vbus_last_q15 = 0; s_inv_vbus_age = 0U;
    s_adc_isr_count = 0U;
    s_vbus_scale_q16 = (int32_t)((DCLINK_V_PER_COUNT / FOC_VOLTAGE_Q_BASE_V) * 32768.0f * 65536.0f);
    s_vbus_min_q15 = (int32_t)((VBUS_MIN_RUN_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    s_vbus_max_q15 = (int32_t)((VBUS_MAX_RUN_V / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    s_current_trip_q15 = (int32_t)((FOC_ABS_CURRENT_TRIP_A / FOC_CURRENT_Q_BASE_A) * 32768.0f);
    s_mod_limit_coeff_q15 = (int32_t)(FOC_MAX_VOLTAGE_MODULATION * 0.5773502691896258f * 32768.0f);
}

uint16_t motor_sensor_electrical_phase_u16(MotorRuntime *m) {
    if (m->detect_force_angle) {
        return m->detect_phase_u16;
    }

    if (m->sensor_mode == SENSOR_MODE_ENCODER) {
        /* AB has no index: the electrical phase is relative to the controlled
           phase-0 alignment captured this boot, not to TIM4 CNT=0 at reset. */
        int32_t ext = m->encoder.turns * (int32_t)m->encoder.cpr + (int32_t)motor_hw_encoder_cnt();
        int32_t rel = ext - m->encoder.session_zero_count;
        int64_t phase64 = ((int64_t)rel * (int64_t)m->encoder.phase_per_count_q16) >> 16;
        uint16_t p=(uint16_t)phase64;
        if (m->encoder.inverted) p=(uint16_t)(0U-p);
        return (uint16_t)(p + m->encoder.elec_offset_u16);
    }

    hall_state_t *h = &m->hall;
    if (!h->valid) return m->hall_offset_u16;

    uint16_t base = h->base_phase_u16;
    uint8_t raw = h->raw_state & 7U;
    if (m->foc_hall_table[raw] != 255U) {
        base = m->hall_angle_u16[raw];
    }

    uint32_t elapsed = DWT->CYCCNT - h->edge_cycle;
    int64_t advance = ((int64_t)(int32_t)elapsed * (int64_t)h->phase_per_cycle_q16) >> 16;
    const int32_t sector_phase = 65536 / 6;
    if (advance > sector_phase) advance = sector_phase;
    if (advance < -sector_phase) advance = -sector_phase;
    return (uint16_t)((int32_t)base + (int32_t)advance + (int32_t)m->hall_offset_u16);
}

static void hall_update_raw_fast(MotorRuntime *m, uint8_t raw) {
    if (m == NULL || m->sensor_mode != SENSOR_MODE_HALL) return;

    hall_state_t *h = &m->hall;
    raw &= 7U;
    h->raw_state = raw;
    int8_t sector = m->hall_table[raw];

    /* Hall 000/111 or an unmapped table entry is not a usable rotor phase.
       Keep a very short fast-loop validation counter rather than a millisecond
       debounce, because phase information must not be delayed. */
    if (raw == 0U || raw == 7U || sector < 0) {
        if (h->invalid_count < UINT16_MAX) h->invalid_count++;
        h->valid = false;
        return;
    }
    h->invalid_count = 0U;

    uint32_t now = DWT->CYCCNT;
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
        h->valid = true;
        return;
    }

    int8_t diff = (int8_t)(sector - old);
    int8_t dir = h->direction;
    bool neighbor = false;
    if (diff == 1 || diff == -5) { dir = 1; neighbor = true; }
    else if (diff == -1 || diff == 5) { dir = -1; neighbor = true; }

    if (!neighbor) {
        if (h->sequence_error_count < UINT16_MAX) h->sequence_error_count++;
        h->valid = false;
        return;
    }

    h->sequence_error_count = 0U;
    uint32_t period = now - h->edge_cycle;
    if (period > 100U) {
        h->period_cycles = period;
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

void motor_hall_edge_isr(MotorRuntime *m) {
    if (m == NULL || m->sensor_mode != SENSOR_MODE_HALL) return;
    hall_update_raw_fast(m, motor_hw_read_hall_raw(m->id));
}

static inline int32_t adc_current_to_q15(uint16_t raw, int32_t offset, int32_t scale_q16) {
    /* Stock hoverboard analog front-end polarity (EFeru): phase/DC current is
       offset - ADC. Using raw-offset turns the current PI into positive
       feedback on this PCB and can produce the growl/current spike seen during
       forced-angle detection. */
    return (int32_t)(((int64_t)(offset - (int32_t)raw) * (int64_t)scale_q16) >> 16);
}

static inline void offset_track_isr(MotorRuntime *m, uint16_t raw_u, uint16_t raw_v, uint16_t raw_dc) {
    if (m->pwm_enabled || m->command_active || m->detect.busy) return;
    int64_t tu = ((int64_t)raw_u) << 16;
    int64_t tv = ((int64_t)raw_v) << 16;
    int64_t td = ((int64_t)raw_dc) << 16;
    m->current_offset_u_acc_q16 += (tu - m->current_offset_u_acc_q16) >> ADC_OFFSET_TRACK_SHIFT;
    m->current_offset_v_acc_q16 += (tv - m->current_offset_v_acc_q16) >> ADC_OFFSET_TRACK_SHIFT;
    m->dc_current_offset_acc_q16 += (td - m->dc_current_offset_acc_q16) >> ADC_OFFSET_TRACK_SHIFT;
    m->current_offset_u_counts = (int32_t)(m->current_offset_u_acc_q16 >> 16);
    m->current_offset_v_counts = (int32_t)(m->current_offset_v_acc_q16 >> 16);
    m->dc_current_offset_counts = (int32_t)(m->dc_current_offset_acc_q16 >> 16);
}

static void foc_one_motor_isr(MotorRuntime *m, uint16_t raw_u, uint16_t raw_v, uint16_t raw_dc,
                              int32_t vbus_q15, int32_t inv_vbus_q30) {
    m->current_raw_u = raw_u;
    m->current_raw_v = raw_v;
    offset_track_isr(m, raw_u, raw_v, raw_dc);

    int32_t ia, ib, ic;
    if (m->id == MOTOR_LEFT) {
        /* PCB LEFT shunts measure phase A and phase B. */
        ia = adc_current_to_q15(raw_u, m->current_offset_u_counts, m->current_scale_q16);
        ib = adc_current_to_q15(raw_v, m->current_offset_v_counts, m->current_scale_q16);
        ic = -(ia + ib);
    } else {
        /* PCB RIGHT shunts are phase B (PC4) and phase C (PC5), not A/B.
           Reconstruct A from Kirchhoff: ia + ib + ic = 0. */
        ib = adc_current_to_q15(raw_u, m->current_offset_u_counts, m->current_scale_q16);
        ic = adc_current_to_q15(raw_v, m->current_offset_v_counts, m->current_scale_q16);
        ia = -(ib + ic);
    }

    m->ia_q15 = ia; m->ib_q15 = ib; m->ic_q15 = ic;
    m->dc_current_raw = raw_dc; m->vbus_q15 = vbus_q15;

    /* Hall GPIO is the fast source of truth. EXTI is only an optional early
       timestamp hint; polling here ensures phase validity is not dependent on
       an RTOS task or on edge-IRQ delivery. During forced-angle detection the
       detector reads Hall separately and Hall validity must not gate SVPWM. */
    if (m->sensor_mode == SENSOR_MODE_HALL && !m->detect_force_angle) {
        hall_update_raw_fast(m, motor_hw_read_hall_raw(m->id));
        if (m->pwm_enabled && m->command_active &&
            (m->hall.invalid_count >= 32U || m->hall.sequence_error_count >= 4U)) {
            motor_request_fault_from_isr(m, MOTOR_FAULT_HALL_INVALID);
            return;
        }
    }

    int32_t trip = (m->abs_current_trip_q15 > 0) ? m->abs_current_trip_q15 : s_current_trip_q15;
    /* With MOE already OFF an externally back-driven motor can generate
       current through the bridge diodes. Latching ABS_OVER_CURRENT in that
       state cannot protect the bridge and caused false 3-beep faults when the
       wheel was rotated by hand. The hard software trip is meaningful while
       this firmware is actively driving (MOE/pwm_enabled). */
    if (m->pwm_enabled && (iabs32(ia) > trip || iabs32(ib) > trip || iabs32(ic) > trip)) {
        motor_request_fault_from_isr(m, MOTOR_FAULT_ABS_OVER_CURRENT); return;
    }
    if (m->pwm_enabled && vbus_q15 > s_vbus_max_q15) { motor_request_fault_from_isr(m, MOTOR_FAULT_OVER_VOLTAGE); return; }
    if (m->pwm_enabled && vbus_q15 < s_vbus_min_q15) { motor_request_fault_from_isr(m, MOTOR_FAULT_UNDER_VOLTAGE); return; }

    if (!m->pwm_enabled || m->fault != MOTOR_FAULT_NONE || inv_vbus_q30 <= 0) {
        m->vd_int_q31 = 0; m->vq_int_q31 = 0;
        m->vd_int_q15 = 0; m->vq_int_q15 = 0;
        motor_hw_set_pwm_q15(m, FOC_Q15_HALF, FOC_Q15_HALF, FOC_Q15_HALF);
        return;
    }

    uint16_t phase = motor_sensor_electrical_phase_u16(m);
    int32_t sn, cs;
    foc_fast_sincos_u16_q15(phase, &sn, &cs);

    int32_t i_alpha = ia;
    int32_t i_beta = foc_q15_mul(ia + (ib * 2), FOC_Q15_INV_SQRT3);
    int32_t id = foc_q15_mul(cs, i_alpha) + foc_q15_mul(sn, i_beta);
    int32_t iq = foc_q15_mul(cs, i_beta) - foc_q15_mul(sn, i_alpha);
    m->id_q15 = id; m->iq_q15 = iq;

    /* VESC-style: filtered currents for telemetry/slow logic only; the raw id/iq above remain PI feedback. */
    m->id_filter_q15 += foc_q15_mul(FOC_CURRENT_FILTER_Q15, id - m->id_filter_q15);
    m->iq_filter_q15 += foc_q15_mul(FOC_CURRENT_FILTER_Q15, iq - m->iq_filter_q15);

    int32_t err_d = m->id_target_q15 - id;
    int32_t err_q = m->iq_target_q15 - iq;
    int32_t vmax_q15 = foc_q15_mul(vbus_q15, s_mod_limit_coeff_q15);
    if (vmax_q15 < 256) vmax_q15 = 256;

    /* Q15 error x Q16.16 Ki*dt produces a Q31 increment. Keep that full
       precision in the integrator instead of truncating every 62.5 us. */
    m->vd_int_q31 += (int64_t)err_d * (int64_t)m->current_ki_dt_q16;
    m->vq_int_q31 += (int64_t)err_q * (int64_t)m->current_ki_dt_q16;
    int64_t int_lim_q31 = (int64_t)vmax_q15 << 16;
    if (m->vd_int_q31 > int_lim_q31) m->vd_int_q31 = int_lim_q31;
    if (m->vd_int_q31 < -int_lim_q31) m->vd_int_q31 = -int_lim_q31;
    if (m->vq_int_q31 > int_lim_q31) m->vq_int_q31 = int_lim_q31;
    if (m->vq_int_q31 < -int_lim_q31) m->vq_int_q31 = -int_lim_q31;
    m->vd_int_q15 = (int32_t)(m->vd_int_q31 >> 16);
    m->vq_int_q15 = (int32_t)(m->vq_int_q31 >> 16);

    int32_t prop_d = q15_mul_q16(err_d, m->current_kp_q16);
    int32_t prop_q = q15_mul_q16(err_q, m->current_kp_q16);
    int32_t vd = m->vd_int_q15 + prop_d;
    int32_t vq = m->vq_int_q15 + prop_q;

    int32_t avd = iabs32(vd), avq = iabs32(vq);
    int32_t hi = (avd > avq) ? avd : avq;
    int32_t lo = (avd > avq) ? avq : avd;
    int32_t mag_approx = hi + ((lo * 3) >> 3);
    if (mag_approx > vmax_q15 && mag_approx > 0) {
        int32_t scale_q15 = (int32_t)(((int64_t)vmax_q15 << 15) / mag_approx);
        vd = foc_q15_mul(vd, scale_q15); vq = foc_q15_mul(vq, scale_q15);
        m->vd_int_q15 = vd - prop_d;
        m->vq_int_q15 = vq - prop_q;
        m->vd_int_q31 = (int64_t)m->vd_int_q15 << 16;
        m->vq_int_q31 = (int64_t)m->vq_int_q15 << 16;
    }
    m->vd_q15 = vd; m->vq_q15 = vq;

    int32_t v_alpha = foc_q15_mul(cs, vd) - foc_q15_mul(sn, vq);
    int32_t v_beta  = foc_q15_mul(sn, vd) + foc_q15_mul(cs, vq);
    uint16_t du, dv, dw;
    foc_svm_q15(v_alpha, v_beta, inv_vbus_q30, &du, &dv, &dw);
    m->duty_u_q15 = du; m->duty_v_q15 = dv; m->duty_w_q15 = dw;
    motor_hw_set_pwm_q15(m, du, dv, dw);
}

static inline void cal_track_minmax(uint16_t v, uint16_t *mn, uint16_t *mx) {
    if (v < *mn) {
        *mn = v;
    }
    if (v > *mx) {
        *mx = v;
    }
}

static uint64_t offset_variance_num(uint64_t sum, uint64_t sum_sq, uint32_t n) {
    if (n == 0U) return UINT64_MAX;
    uint64_t nn = (uint64_t)n;
    uint64_t a = sum_sq * nn;
    uint64_t b = sum * sum;
    return (a >= b) ? (a - b) : 0U;
}

static bool variance_below_sigma(uint64_t sum, uint64_t sum_sq, uint32_t n, uint32_t sigma) {
    if (n == 0U) return false;
    uint64_t nn = (uint64_t)n;
    uint64_t lhs = offset_variance_num(sum, sum_sq, n);
    uint64_t rhs = (uint64_t)sigma * (uint64_t)sigma * nn * nn;
    return lhs <= rhs;
}

static uint32_t variance_x100(uint64_t sum, uint64_t sum_sq, uint32_t n) {
    if (n == 0U) return UINT32_MAX;
    uint64_t num = offset_variance_num(sum, sum_sq, n);
    uint64_t den = (uint64_t)n * (uint64_t)n;
    /* num/den is ADC-count^2. x100 retains two decimal places for host sqrt. */
    uint64_t v = (num * 100ULL + den / 2ULL) / den;
    return (v > UINT32_MAX) ? UINT32_MAX : (uint32_t)v;
}

static void validate_cal_channel(uint8_t idx, int32_t mean, uint16_t mn, uint16_t mx,
                                 uint64_t sum, uint64_t sum_sq, uint32_t n, bool phase_channel) {
    uint16_t bit = (uint16_t)(1U << idx);
    uint32_t spread = (uint32_t)(mx - mn);

    /* The phase amplifiers must not be near either ADC rail. The DC-current
       auxiliary channels on this PCB can use a different bias point, but they
       still must not be effectively railed. Use the same broad 12-bit guard;
       unlike V11, do not assume every valid zero-current offset is near 2048. */
    (void)phase_channel;
    if (mean < ADC_OFFSET_HARD_MIN_COUNT || mean > ADC_OFFSET_HARD_MAX_COUNT) {
        s_cal_fail_range_mask |= bit;
    }

    if (spread > ADC_OFFSET_HARD_SPREAD_COUNT ||
        !variance_below_sigma(sum, sum_sq, n, ADC_OFFSET_HARD_STDDEV_COUNT)) {
        s_cal_fail_noise_mask |= bit;
    } else if (spread > ADC_OFFSET_WARN_SPREAD_COUNT ||
               !variance_below_sigma(sum, sum_sq, n, ADC_OFFSET_WARN_STDDEV_COUNT)) {
        s_cal_warn_mask |= bit;
    }
}

void foc_get_calibration_diag(foc_cal_diag_t *out) {
    if (out == NULL) return;
    /* Snapshot ISR-owned counters atomically, but do the 64-bit divisions only
       after IRQs are restored. Holding off the 32 kHz sampling IRQ across six
       software 64-bit divisions would itself perturb the control timing. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t n = s_cal_count;
    uint16_t warn = s_cal_warn_mask, fail_range = s_cal_fail_range_mask, fail_noise = s_cal_fail_noise_mask;
    uint64_t sums[6] = {s_sum_lu,s_sum_lv,s_sum_ldc,s_sum_ru,s_sum_rv,s_sum_rdc};
    uint64_t sqs[6]  = {s_sq_lu,s_sq_lv,s_sq_ldc,s_sq_ru,s_sq_rv,s_sq_rdc};
    uint16_t mins[6] = {s_min_lu,s_min_lv,s_min_ldc,s_min_ru,s_min_rv,s_min_rdc};
    uint16_t maxs[6] = {s_max_lu,s_max_lv,s_max_ldc,s_max_ru,s_max_rv,s_max_rdc};
    if (!primask) __enable_irq();

    out->warn_mask = warn;
    out->fail_range_mask = fail_range;
    out->fail_noise_mask = fail_noise;
    for (uint8_t k=0U;k<6U;k++) {
        out->ch[k].mean = (n > 0U) ? (int32_t)(sums[k] / n) : 0;
        out->ch[k].min = mins[k];
        out->ch[k].max = maxs[k];
        out->ch[k].variance_x100 = variance_x100(sums[k], sqs[k], n);
    }
}

static void calibration_zero_fast_states(MotorRuntime *m) {
    m->ia_q15 = 0; m->ib_q15 = 0; m->ic_q15 = 0;
    m->id_q15 = 0; m->iq_q15 = 0;
    m->id_filter_q15 = 0; m->iq_filter_q15 = 0;
    m->vd_q15 = 0; m->vq_q15 = 0;
    m->vd_int_q31 = 0; m->vq_int_q31 = 0;
    m->vd_int_q15 = 0; m->vq_int_q15 = 0;
    m->dc_current_raw = (uint16_t)m->dc_current_offset_counts;
}


void foc_adc_dma_isr(const volatile uint32_t adc_words[6]) {
    uint32_t start = DWT->CYCCNT;
    s_adc_isr_count++;
    if (s_cal_request) cal_reset_isr();

    uint16_t l_u = low16(adc_words[0]), l_v = high16(adc_words[0]);
    uint16_t r_u = low16(adc_words[1]), r_v = high16(adc_words[1]);
    uint16_t l_dc = low16(adc_words[2]), r_dc = high16(adc_words[2]);
    uint16_t vraw = low16(adc_words[3]);

    if (!s_cal_done) {
        /* Ignore startup pipeline content; then average thousands of PWM-synchronous samples with both MOE off. */
        if (s_cal_warmup < 64U) { s_cal_warmup++; return; }
        s_sum_lu += l_u; s_sum_lv += l_v; s_sum_ldc += l_dc;
        s_sum_ru += r_u; s_sum_rv += r_v; s_sum_rdc += r_dc;
        s_sq_lu += (uint64_t)l_u * l_u; s_sq_lv += (uint64_t)l_v * l_v; s_sq_ldc += (uint64_t)l_dc * l_dc;
        s_sq_ru += (uint64_t)r_u * r_u; s_sq_rv += (uint64_t)r_v * r_v; s_sq_rdc += (uint64_t)r_dc * r_dc;
        cal_track_minmax(l_u,&s_min_lu,&s_max_lu); cal_track_minmax(l_v,&s_min_lv,&s_max_lv); cal_track_minmax(l_dc,&s_min_ldc,&s_max_ldc);
        cal_track_minmax(r_u,&s_min_ru,&s_max_ru); cal_track_minmax(r_v,&s_min_rv,&s_max_rv); cal_track_minmax(r_dc,&s_min_rdc,&s_max_rdc);
        s_cal_count++;
        if (s_cal_count >= ADC_OFFSET_CAL_SAMPLES) {
            MotorRuntime *l=&g_motor_left, *r=&g_motor_right;
            l->current_offset_u_counts=(int32_t)(s_sum_lu/s_cal_count); l->current_offset_v_counts=(int32_t)(s_sum_lv/s_cal_count); l->dc_current_offset_counts=(int32_t)(s_sum_ldc/s_cal_count);
            r->current_offset_u_counts=(int32_t)(s_sum_ru/s_cal_count); r->current_offset_v_counts=(int32_t)(s_sum_rv/s_cal_count); r->dc_current_offset_counts=(int32_t)(s_sum_rdc/s_cal_count);
            l->current_offset_u_acc_q16=((int64_t)l->current_offset_u_counts)<<16; l->current_offset_v_acc_q16=((int64_t)l->current_offset_v_counts)<<16; l->dc_current_offset_acc_q16=((int64_t)l->dc_current_offset_counts)<<16;
            r->current_offset_u_acc_q16=((int64_t)r->current_offset_u_counts)<<16; r->current_offset_v_acc_q16=((int64_t)r->current_offset_v_counts)<<16; r->dc_current_offset_acc_q16=((int64_t)r->dc_current_offset_counts)<<16;
            /* VESC-like acceptance: the offset is the measured mean. Noise is
               diagnostic unless it is extreme enough to make current feedback
               unsafe. V11's <=12-count stddev hard gate was not upstream VESC
               behavior and falsely rejected this hoverboard analog front-end. */
            s_cal_warn_mask = s_cal_fail_range_mask = s_cal_fail_noise_mask = 0U;
            validate_cal_channel(0U,l->current_offset_u_counts,s_min_lu,s_max_lu,s_sum_lu,s_sq_lu,s_cal_count,true);
            validate_cal_channel(1U,l->current_offset_v_counts,s_min_lv,s_max_lv,s_sum_lv,s_sq_lv,s_cal_count,true);
            validate_cal_channel(2U,l->dc_current_offset_counts,s_min_ldc,s_max_ldc,s_sum_ldc,s_sq_ldc,s_cal_count,false);
            validate_cal_channel(3U,r->current_offset_u_counts,s_min_ru,s_max_ru,s_sum_ru,s_sq_ru,s_cal_count,true);
            validate_cal_channel(4U,r->current_offset_v_counts,s_min_rv,s_max_rv,s_sum_rv,s_sq_rv,s_cal_count,true);
            validate_cal_channel(5U,r->dc_current_offset_counts,s_min_rdc,s_max_rdc,s_sum_rdc,s_sq_rdc,s_cal_count,false);
            bool ok = (s_cal_fail_range_mask == 0U) && (s_cal_fail_noise_mask == 0U);
            calibration_zero_fast_states(l);
            calibration_zero_fast_states(r);
            s_cal_valid = ok; s_cal_done = true;
            if (!ok) {
                motor_request_fault_from_isr(l,MOTOR_FAULT_CURRENT_OFFSET);
                motor_request_fault_from_isr(r,MOTOR_FAULT_CURRENT_OFFSET);
            }
        }
        return;
    }

    int32_t vbus_q15 = (int32_t)(((int64_t)vraw * (int64_t)s_vbus_scale_q16) >> 16);
    /* Cortex-M3 integer divide is expensive. Vbus moves slowly relative to
       the current loop, so cache its reciprocal and refresh only when needed. */
    int32_t dv = vbus_q15 - s_inv_vbus_last_q15; if (dv < 0) dv=-dv;
    if (vbus_q15 <= 256) { s_inv_vbus_q30=0; s_inv_vbus_last_q15=vbus_q15; s_inv_vbus_age=0U; }
    else if (s_inv_vbus_q30==0 || dv > 48 || ++s_inv_vbus_age >= 32U) {
        s_inv_vbus_q30=(int32_t)(((int64_t)1<<30)/vbus_q15);
        s_inv_vbus_last_q15=vbus_q15; s_inv_vbus_age=0U;
    }
    int32_t inv_vbus_q30=s_inv_vbus_q30;

    /* Current VESC dual-motor semantics: ADC fires in both V0/V7 halves and
       only one motor executes the full FOC path per event. Upstream derives
       the active motor from TIM1 direction. In this board TIM1=RIGHT and
       TIM8=LEFT, therefore DIR=0 selects LEFT/TIM8 and DIR=1 RIGHT/TIM1. */
    bool sample_left = (TIM1->CR1 & TIM_CR1_DIR) == 0U;
    MotorRuntime *active;
    if (sample_left) {
        active = &g_motor_left;
        foc_one_motor_isr(active,l_u,l_v,l_dc,vbus_q15,inv_vbus_q30);
    } else {
        active = &g_motor_right;
        foc_one_motor_isr(active,r_u,r_v,r_dc,vbus_q15,inv_vbus_q30);
    }
    debug_sample_capture_isr(active);

    uint32_t cycles = DWT->CYCCNT - start;
    if (cycles > active->isr_max_cycles) active->isr_max_cycles = cycles;

    /* There are two ADC/FOC events per 16-kHz center-aligned PWM period, so
       the hard return budget is 64MHz/32kHz = 2000 cycles. Each individual
       motor is still updated at 16 kHz. */
    const uint32_t slot_cycles = FOC_ISR_SLOT_CYCLES;
    uint8_t mi = (active->id == MOTOR_RIGHT) ? 1U : 0U;
    if (cycles > (slot_cycles * 85U) / 100U) {
        active->isr_overruns++;
    }

    /* Never intentionally skip a current-control event. Persistent >slot
       execution is a real deadline fault, not something to hide with a shed
       pass. Keep a short consecutive filter for occasional IRQ jitter. */
    if (cycles > slot_cycles) {
        if (s_overrun_consecutive[mi] < 255U) s_overrun_consecutive[mi]++;
        if (s_overrun_consecutive[mi] >= 8U) {
            motor_request_fault_from_isr(active, MOTOR_FAULT_FOC_ISR_OVERRUN);
        }
    } else {
        s_overrun_consecutive[mi] = 0U;
    }
}
