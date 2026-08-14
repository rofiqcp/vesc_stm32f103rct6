#include "foc_control.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_math.h"
#include "app_config.h"
#include "debug_sample.h"
#include <limits.h>

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
static uint32_t s_overrun_consecutive = 0U;
static int32_t s_inv_vbus_q30 = 0;
static int32_t s_inv_vbus_last_q15 = 0;
static uint8_t s_inv_vbus_age = 0U;

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
}

bool foc_calibration_done(void) { return s_cal_done; }
bool foc_calibration_valid(void) { return s_cal_done && s_cal_valid; }

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
    s_overrun_consecutive = 0U;
    s_inv_vbus_q30 = 0; s_inv_vbus_last_q15 = 0; s_inv_vbus_age = 0U;
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

void motor_hall_edge_isr(MotorRuntime *m) {
    if (m == NULL || m->sensor_mode != SENSOR_MODE_HALL) return;
    uint8_t raw = motor_hw_read_hall_raw(m->id);
    int8_t sector = m->hall_table[raw & 7U];
    m->hall.raw_state = raw;
    if (sector < 0 || raw == 0U || raw == 7U) {
        m->hall.valid = false;
        return;
    }

    uint32_t now = DWT->CYCCNT;
    int8_t old = m->hall.sector;
    int8_t dir = m->hall.direction;
    bool valid_transition = false;

    if (old >= 0 && sector != old) {
        int8_t diff = (int8_t)(sector - old);
        if (diff == 1 || diff == -5) { dir = 1; valid_transition = true; }
        else if (diff == -1 || diff == 5) { dir = -1; valid_transition = true; }
    }

    if (valid_transition) {
        uint32_t period = now - m->hall.edge_cycle;
        if (period > 100U) {
            m->hall.period_cycles = period;
            int64_t step = (((int64_t)(65536 / 6)) << 16) / (int64_t)period;
            m->hall.phase_per_cycle_q16 = (int32_t)(dir > 0 ? step : -step);
        }
        m->hall.edge_count += dir;
    } else if (old < 0) {
        m->hall.phase_per_cycle_q16 = 0;
    }

    m->hall.direction = dir;
    m->hall.sector = sector;
    m->hall.base_phase_u16 = (uint16_t)((uint32_t)sector * (65536U / 6U));
    m->hall.edge_cycle = now;
    m->hall.valid = true;
}

static inline int32_t adc_current_to_q15(uint16_t raw, int32_t offset, int32_t scale_q16) {
    return (int32_t)(((int64_t)((int32_t)raw - offset) * (int64_t)scale_q16) >> 16);
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

    int32_t ia = adc_current_to_q15(raw_u, m->current_offset_u_counts, m->current_scale_q16);
    int32_t ib = adc_current_to_q15(raw_v, m->current_offset_v_counts, m->current_scale_q16);
    int32_t ic = -(ia + ib);

    m->ia_q15 = ia; m->ib_q15 = ib; m->ic_q15 = ic;
    m->dc_current_raw = raw_dc; m->vbus_q15 = vbus_q15;

    int32_t trip = (m->abs_current_trip_q15 > 0) ? m->abs_current_trip_q15 : s_current_trip_q15;
    if (iabs32(ia) > trip || iabs32(ib) > trip || iabs32(ic) > trip) {
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

static bool offset_noise_valid(uint64_t sum, uint64_t sum_sq, uint32_t n,
                               uint16_t mn, uint16_t mx) {
    if (n == 0U || (uint32_t)(mx - mn) > ADC_OFFSET_MAX_SPREAD_COUNT) {
        return false;
    }

    /* Integer form of population variance:
       variance = (sum_sq*n - sum*sum) / (n*n).
       Avoid sqrt/float inside the FOC ISR. */
    uint64_t nn = (uint64_t)n;
    uint64_t lhs = sum_sq * nn - sum * sum;
    uint64_t sigma = (uint64_t)ADC_OFFSET_MAX_STDDEV_COUNT;
    uint64_t rhs = sigma * sigma * nn * nn;
    return lhs <= rhs;
}

static bool offset_valid_phase(int32_t v, uint16_t mn, uint16_t mx,
                               uint64_t sum, uint64_t sum_sq, uint32_t n) {
    if (v < ADC_OFFSET_VALID_MIN_COUNT || v > ADC_OFFSET_VALID_MAX_COUNT) {
        return false;
    }
    return offset_noise_valid(sum, sum_sq, n, mn, mx);
}

static bool offset_valid_aux(uint16_t mn, uint16_t mx, uint64_t sum,
                             uint64_t sum_sq, uint32_t n) {
    /* DC-link current amplifiers may be either mid-biased or near ground at
       zero current, so only stability is enforced for the auxiliary channel. */
    return offset_noise_valid(sum, sum_sq, n, mn, mx);
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


void foc_adc_dma_quick_guard_isr(const volatile uint32_t adc_words[4]) {
    /* Liveness-shed path: keep only hard electrical protection and leave the
     * previous PWM vector in place for one sample. No trigonometry, PI or
     * telemetry is executed here. */
    if (!s_cal_done || !s_cal_valid) return;

    MotorRuntime *l = &g_motor_left;
    MotorRuntime *r = &g_motor_right;
    uint16_t l_u = low16(adc_words[0]), l_v = high16(adc_words[0]);
    uint16_t r_u = low16(adc_words[1]), r_v = high16(adc_words[1]);
    uint16_t vraw = low16(adc_words[3]);

    int32_t lia = adc_current_to_q15(l_u, l->current_offset_u_counts, l->current_scale_q16);
    int32_t lib = adc_current_to_q15(l_v, l->current_offset_v_counts, l->current_scale_q16);
    int32_t lic = -(lia + lib);
    int32_t ria = adc_current_to_q15(r_u, r->current_offset_u_counts, r->current_scale_q16);
    int32_t rib = adc_current_to_q15(r_v, r->current_offset_v_counts, r->current_scale_q16);
    int32_t ric = -(ria + rib);

    int32_t l_trip = (l->abs_current_trip_q15 > 0) ? l->abs_current_trip_q15 : s_current_trip_q15;
    int32_t r_trip = (r->abs_current_trip_q15 > 0) ? r->abs_current_trip_q15 : s_current_trip_q15;
    if (iabs32(lia) > l_trip || iabs32(lib) > l_trip ||
        iabs32(lic) > l_trip) {
        motor_request_fault_from_isr(l, MOTOR_FAULT_ABS_OVER_CURRENT);
    }
    if (iabs32(ria) > r_trip || iabs32(rib) > r_trip ||
        iabs32(ric) > r_trip) {
        motor_request_fault_from_isr(r, MOTOR_FAULT_ABS_OVER_CURRENT);
    }

    int32_t vbus_q15 = (int32_t)(((int64_t)vraw * (int64_t)s_vbus_scale_q16) >> 16);
    if (l->pwm_enabled && vbus_q15 > s_vbus_max_q15) motor_request_fault_from_isr(l, MOTOR_FAULT_OVER_VOLTAGE);
    if (l->pwm_enabled && vbus_q15 < s_vbus_min_q15) motor_request_fault_from_isr(l, MOTOR_FAULT_UNDER_VOLTAGE);
    if (r->pwm_enabled && vbus_q15 > s_vbus_max_q15) motor_request_fault_from_isr(r, MOTOR_FAULT_OVER_VOLTAGE);
    if (r->pwm_enabled && vbus_q15 < s_vbus_min_q15) motor_request_fault_from_isr(r, MOTOR_FAULT_UNDER_VOLTAGE);
}

void foc_adc_dma_isr(const volatile uint32_t adc_words[4]) {
    uint32_t start = DWT->CYCCNT;
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
            bool ok =
                offset_valid_phase(l->current_offset_u_counts,s_min_lu,s_max_lu,s_sum_lu,s_sq_lu,s_cal_count) &&
                offset_valid_phase(l->current_offset_v_counts,s_min_lv,s_max_lv,s_sum_lv,s_sq_lv,s_cal_count) &&
                offset_valid_aux(s_min_ldc,s_max_ldc,s_sum_ldc,s_sq_ldc,s_cal_count) &&
                offset_valid_phase(r->current_offset_u_counts,s_min_ru,s_max_ru,s_sum_ru,s_sq_ru,s_cal_count) &&
                offset_valid_phase(r->current_offset_v_counts,s_min_rv,s_max_rv,s_sum_rv,s_sq_rv,s_cal_count) &&
                offset_valid_aux(s_min_rdc,s_max_rdc,s_sum_rdc,s_sq_rdc,s_cal_count);
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
       16 kHz, so update its reciprocal only on meaningful change or timeout. */
    int32_t dv = vbus_q15 - s_inv_vbus_last_q15; if (dv < 0) dv=-dv;
    if (vbus_q15 <= 256) { s_inv_vbus_q30=0; s_inv_vbus_last_q15=vbus_q15; s_inv_vbus_age=0U; }
    else if (s_inv_vbus_q30==0 || dv > 48 || ++s_inv_vbus_age >= 32U) {
        s_inv_vbus_q30=(int32_t)(((int64_t)1<<30)/vbus_q15);
        s_inv_vbus_last_q15=vbus_q15; s_inv_vbus_age=0U;
    }
    int32_t inv_vbus_q30=s_inv_vbus_q30;
    foc_one_motor_isr(&g_motor_left,l_u,l_v,l_dc,vbus_q15,inv_vbus_q30);
    foc_one_motor_isr(&g_motor_right,r_u,r_v,r_dc,vbus_q15,inv_vbus_q30);
    debug_sample_capture_isr(&g_motor_left,&g_motor_right);

    uint32_t cycles = DWT->CYCCNT - start;
    if (cycles > g_motor_left.isr_max_cycles) g_motor_left.isr_max_cycles = cycles;
    if (cycles > g_motor_right.isr_max_cycles) g_motor_right.isr_max_cycles = cycles;
    /* One 16 kHz ADC/FOC update has CPU_CLOCK/PWM cycles available. At
       64 MHz that is 4000 cycles. Previous revisions incorrectly divided by
       two again and could report overrun at only ~2000 cycles. */
    const uint32_t deadline_cycles = CPU_CLOCK_HZ / PWM_FREQUENCY_HZ;
    if (cycles > (deadline_cycles * 85U) / 100U) {
        g_motor_left.isr_overruns++;
        g_motor_right.isr_overruns++;
    }

    /* The caller provides a one-sample liveness shed when the next DMA event
       is already pending. Only an extreme >2-period execution is promoted to
       a sticky internal fault. */
    if (cycles > (deadline_cycles * 2U)) {
        if (++s_overrun_consecutive >= 8U) {
            motor_request_fault_from_isr(&g_motor_left, MOTOR_FAULT_FOC_ISR_OVERRUN);
            motor_request_fault_from_isr(&g_motor_right, MOTOR_FAULT_FOC_ISR_OVERRUN);
        }
    } else {
        s_overrun_consecutive = 0U;
    }
}
