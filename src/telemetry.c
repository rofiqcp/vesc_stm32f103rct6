#include "telemetry.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "applications/appconf_default.h"
#include "motor/foc_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

static motor_telemetry_t s_telem[2];

typedef struct {
    float sum[6];
    uint32_t count[6];
} telemetry_avg_acc_t;

static telemetry_avg_acc_t s_avg[2];
/* Seqlock ringan untuk cache telemetry task-side. Writer hanya 100 Hz dan
 * reader tidak pernah menunggu mutex, sehingga polling VESC Tool tidak dapat
 * menahan timer task atau jalur kontrol motor. */
static volatile uint32_t s_telem_seq[2];

static bool read_rt_snapshot(const MotorRuntime *m, foc_rt_snapshot_t *out);

/* Inisialisasi cache telemetry tanpa alokasi heap.
 * Tidak ada mutex yang dapat memblokir task timer maupun packet-process. */
bool telemetry_init(void){
    memset(s_telem,0,sizeof(s_telem));
    memset(s_avg,0,sizeof(s_avg));
    s_telem_seq[0]=0U;
    s_telem_seq[1]=0U;
    return true;
}

/* Bangun enam nilai average dari SATU frame FOC koheren. Semua operasi float
 * termasuk sqrtf dilakukan di luar critical section agar ADC/FOC ISR 16 kHz
 * tidak tertunda oleh pekerjaan telemetry 100 Hz. */
static bool build_avg_values(const MotorRuntime *m, float v[6]) {
    foc_rt_snapshot_t rt;
    if (m == NULL || v == NULL || !read_rt_snapshot(m, &rt)) return false;

    const float is = FOC_CURRENT_Q_BASE_A / 32768.0f;
    const float vs = FOC_VOLTAGE_Q_BASE_V / 32768.0f;
    const float id = (float)rt.id_filter_q15 * is;
    const float iq = (float)rt.iq_filter_q15 * is;
    const float imotor_mag = sqrtf(id * id + iq * iq);
    const float iin = (float)rt.dc_current_q15 * is;
    v[0] = iin < 0.0f ? -imotor_mag : imotor_mag;
    v[1] = iin;
    v[2] = id;
    v[3] = iq;
    v[4] = (float)rt.vd_q15 * vs;
    v[5] = (float)rt.vq_q15 * vs;
    return true;
}

/* Tambahkan hasil average ke accumulator. Fungsi ini dipanggil saat scheduler
 * disuspend (IRQ tetap aktif): maksimal 12 penjumlahan/counter untuk dua motor,
 * tanpa sqrtf dan tanpa snapshot copy. */
static void accumulate_avg_values(unsigned idx, const float v[6]) {
    if (idx >= 2U || v == NULL) return;
    for (unsigned k = 0U; k < 6U; k++) {
        s_avg[idx].sum[k] += v[k];
        if (s_avg[idx].count[k] != UINT32_MAX) s_avg[idx].count[k]++;
    }
}

static void update_stats(MotorRuntime *m,float dt_s){
    float ih=m->input_current*dt_s/3600.0f;
    float wh=m->vbus_filter*m->input_current*dt_s/3600.0f;
    if(ih>=0.0f)m->stats.amp_hours+=ih;else m->stats.amp_hours_charged+=-ih;
    if(wh>=0.0f)m->stats.watt_hours+=wh;else m->stats.watt_hours_charged+=-wh;
    /* Treat position-derived tach as the raw motion source, not as the public
       counter value. VESC APIs can set/reset tachometer; the next statistics
       tick must add only new shaft motion instead of reconstructing the old
       absolute position and undoing that set/reset. */
    int32_t tach_raw=(int32_t)lroundf((m->position_deg/360.0f)*(6.0f*(float)m->pole_pairs));
    int32_t delta=0;
    if (m->stats.tachometer_source_valid) {
        delta=tach_raw-m->stats.tachometer_last;
    } else {
        /* A configuration/sensor-coordinate change may change position scale
           or sign without physical shaft motion. Rebase the private motion
           source once so public tachometer/distance/odometer cannot jump. */
        m->stats.tachometer_source_valid=true;
    }
    m->stats.tachometer += delta;
    int64_t delta64=(int64_t)delta;
    uint32_t abs_delta=(uint32_t)(delta64<0?-delta64:delta64);
    m->stats.tachometer_abs += (int32_t)abs_delta;
    mc_interface_odometer_add_tach_delta(m->id,abs_delta);
    m->stats.tachometer_last=tach_raw;
    float ac=fabsf(m->motor_current);if(ac>m->stats.max_current)m->stats.max_current=ac;
    float ai=fabsf(m->input_current);if(ai>m->stats.max_input_current)m->stats.max_input_current=ai;
    float ar=fabsf(m->erpm);if(ar>m->stats.max_erpm)m->stats.max_erpm=ar;
    m->stats.runtime_ms+=STAT_PERIOD_MS;
}

static bool read_rt_snapshot(const MotorRuntime *m, foc_rt_snapshot_t *out) {
    if (m == NULL || out == NULL) return false;
    for (unsigned retry = 0U; retry < 8U; retry++) {
        uint32_t s1 = m->rt_snapshot_seq;
        if ((s1 & 1U) != 0U) continue;
        __DMB();
#define CPY(f) out->f = m->rt_snapshot.f
        CPY(ia_q15); CPY(ib_q15); CPY(ic_q15);
        CPY(id_q15); CPY(iq_q15); CPY(id_filter_q15); CPY(iq_filter_q15);
        CPY(id_target_q15); CPY(iq_target_q15); CPY(vd_q15); CPY(vq_q15);
        CPY(vbus_q15); CPY(dc_current_q15); CPY(erpm_fast_q16);
        CPY(duty_u_q15); CPY(duty_v_q15); CPY(duty_w_q15);
        CPY(phase_control_u16); CPY(phase_observer_u16);
        CPY(phase_encoder_u16); CPY(phase_hall_u16);
        CPY(adc_frame); CPY(cycle_counter);
#undef CPY
        __DMB();
        uint32_t s2 = m->rt_snapshot_seq;
        if (s1 == s2 && (s2 & 1U) == 0U) return true;
    }
    return false;
}

static void snapshot(MotorRuntime *m,motor_telemetry_t *t){
    foc_rt_snapshot_t rt;
    const bool have_rt = read_rt_snapshot(m, &rt);
    if (have_rt) {
        const float is = FOC_CURRENT_Q_BASE_A / 32768.0f;
        const float vs = FOC_VOLTAGE_Q_BASE_V / 32768.0f;
        t->phase_current_a = (float)rt.ia_q15 * is;
        t->phase_current_b = (float)rt.ib_q15 * is;
        t->phase_current_c = (float)rt.ic_q15 * is;
        t->id = (float)rt.id_q15 * is;
        t->iq = (float)rt.iq_q15 * is;
        t->id_filter = (float)rt.id_filter_q15 * is;
        t->iq_filter = (float)rt.iq_filter_q15 * is;
        t->vd = (float)rt.vd_q15 * vs;
        t->vq = (float)rt.vq_q15 * vs;
        float imag = sqrtf(t->id_filter*t->id_filter + t->iq_filter*t->iq_filter);
        const float rt_iin = (float)rt.dc_current_q15 * is;
        t->current_motor = rt_iin < 0.0f ? -imag : imag;
        t->current_in = rt_iin;
        t->erpm = (float)rt.erpm_fast_q16 / 65536.0f;
        t->mech_rpm = t->erpm / fmaxf((float)m->pole_pairs, 1.0f);
        t->vbus = (float)rt.vbus_q15 * vs;
        t->rotor_elec_deg = (float)rt.phase_control_u16 * (360.0f/65536.0f);
        t->duty = m->duty_now;
    } else {
        /* Startup before the first FOC frame only. */
        t->phase_current_a=m->ia;t->phase_current_b=m->ib;t->phase_current_c=m->ic;
        t->id=m->id_meas;t->iq=m->iq_meas;t->id_filter=m->id_filter;t->iq_filter=m->iq_filter;
        t->vd=m->vd_filter;t->vq=m->vq_filter;t->current_motor=m->motor_current;t->current_in=m->input_current;
        t->duty=m->duty_now;t->erpm=m->erpm;t->mech_rpm=m->mech_rpm;t->vbus=m->vbus_filter;t->rotor_elec_deg=m->rotor_elec_deg;
    }
    t->position_deg=foc_wrap_deg(m->position_deg);
    t->current_offset_u=(float)m->current_offset_u_counts;t->current_offset_v=(float)m->current_offset_v_counts;t->dc_current_offset=(float)m->dc_current_offset_counts;
    t->amp_hours=m->stats.amp_hours;t->amp_hours_charged=m->stats.amp_hours_charged;t->watt_hours=m->stats.watt_hours;t->watt_hours_charged=m->stats.watt_hours_charged;
    t->tachometer=m->stats.tachometer;t->tachometer_abs=m->stats.tachometer_abs;
    t->fault=(uint8_t)m->fault;t->controller_id=(m->id==MOTOR_LEFT)?VESC_CONTROLLER_ID_LEFT:VESC_CONTROLLER_ID_RIGHT;
    t->sensor_mode=m->sensor_mode; /* physical pin/timer mux */
    t->foc_sensor_mode=(uint8_t)m->foc_sensor_mode; /* logical FOC source */
    t->sensor_detect_state=(uint8_t)m->detect.state;
    t->calibration_done=foc_calibration_done()?1U:0U;t->calibration_valid=foc_calibration_valid()?1U:0U;t->timeout_active=m->timeout_active?1U:0U;
    t->observer_valid=m->observer_valid?1U:0U;t->using_encoder=m->using_encoder?1U:0U;t->encoder_synced=m->encoder.synced?1U:0U;
    t->observer_phase_deg=m->observer_phase_deg;t->observer_erpm=m->observer_erpm;t->observer_quality=m->observer_quality;
    t->foc_motor_r=m->foc_motor_r;t->foc_motor_l=m->foc_motor_l;t->foc_motor_ld_lq_diff=m->foc_motor_ld_lq_diff;t->foc_motor_flux_linkage=m->foc_motor_flux_linkage;
    t->foc_sl_erpm_start=m->foc_sl_erpm_start;t->foc_sl_erpm=m->foc_sl_erpm;t->foc_openloop_rpm=m->foc_openloop_rpm;t->foc_openloop_rpm_low=m->foc_openloop_rpm_low;
    t->current_loop_hz=FOC_ISR_EVENT_HZ;t->telemetry_snapshot_hz=(STAT_PERIOD_MS>0U)?(1000U/STAT_PERIOD_MS):0U;
    t->isr_max_cycles=m->isr_max_cycles;t->isr_overruns=m->isr_overruns;
    t->abs_current_filtered=(float)m->abs_phase_current_filter_q15*(FOC_CURRENT_Q_BASE_A/32768.0f);
    t->abs_current_peak=(float)m->abs_current_peak_q15*(FOC_CURRENT_Q_BASE_A/32768.0f);
    t->over_voltage_fault_count=m->over_voltage_fault_count;
    t->under_voltage_fault_count=m->under_voltage_fault_count;
}

/* Update statistik dan average 100 Hz. Matematika FOC selesai sebelum scheduler
 * disuspend; interrupt prioritas tinggi termasuk ADC/DMA tetap dapat berjalan. */
void telemetry_stats_update_100hz(void){
    float left_v[6];
    float right_v[6];
    const bool have_left = build_avg_values(&g_motor_left,left_v);
    const bool have_right = build_avg_values(&g_motor_right,right_v);

    /* Hanya task yang mengakses accumulator. Suspend scheduler menjaga
     * read-reset atomik TANPA mematikan ADC/DMA interrupt prioritas tinggi. */
    vTaskSuspendAll();
    if (have_left) accumulate_avg_values(0U,left_v);
    if (have_right) accumulate_avg_values(1U,right_v);
    (void)xTaskResumeAll();

    update_stats(&g_motor_left,STAT_PERIOD_MS/1000.0f);
    update_stats(&g_motor_right,STAT_PERIOD_MS/1000.0f);
}

/* Tulis cache telemetry langsung di bawah seqlock. Tidak memakai temporary
 * motor_telemetry_t di stack timer 1 KiB dan tidak memakai mutex/heap. Pada
 * single-core F103 packet task tidak dapat berjalan bersamaan dengan writer;
 * sequence tetap melindungi pembacaan jika ada perubahan scheduling kelak. */
static void update_cached_snapshot(unsigned idx, MotorRuntime *m) {
    if (idx >= 2U || m == NULL) return;
    uint32_t seq = s_telem_seq[idx];
    s_telem_seq[idx] = seq + 1U;
    __DMB();
    memset(&s_telem[idx],0,sizeof(s_telem[idx]));
    snapshot(m,&s_telem[idx]);
    __DMB();
    s_telem_seq[idx] = seq + 2U;
}

/* Ambil snapshot 100 Hz tanpa mutex/heap dan tanpa menambah stack besar pada
 * timer_thread. */
void telemetry_snapshot_100hz(void){
    update_cached_snapshot(0U,&g_motor_left);
    update_cached_snapshot(1U,&g_motor_right);
}

void telemetry_update_100hz(void){
    telemetry_stats_update_100hz();
    telemetry_snapshot_100hz();
}

/* Baca cache telemetry secara lock-free. Jika writer bertepatan terus dengan
 * pembacaan, fallback ke snapshot realtime agar komunikasi tidak pernah macet. */
void telemetry_get(motor_id_t id,motor_telemetry_t *out){
    if(out==NULL)return;
    const uint32_t idx=(id==MOTOR_RIGHT)?1U:0U;
    for (unsigned retry=0U; retry<8U; retry++) {
        const uint32_t s1=s_telem_seq[idx];
        if ((s1 & 1U) != 0U) continue;
        __DMB();
        *out=s_telem[idx];
        __DMB();
        const uint32_t s2=s_telem_seq[idx];
        if (s1==s2 && (s2 & 1U)==0U) return;
    }

    memset(out,0,sizeof(*out));
    snapshot(motor_get(id),out);
}

void telemetry_get_realtime(motor_id_t id, motor_telemetry_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    MotorRuntime *m = motor_get(id);
    if (m != NULL) snapshot(m, out);
}

/* Baca lalu reset average VESC untuk I_motor/I_in/Id/Iq/Vd/Vq. Scheduler
 * disuspend sebentar agar transaksi atomik, sementara ADC/FOC IRQ tetap aktif. */
void telemetry_read_reset_avg(motor_id_t id, uint32_t mask, motor_telemetry_avg_t *out) {
    if (out == NULL) return;
    const unsigned idx = (id == MOTOR_RIGHT) ? 1U : 0U;
    memset(out, 0, sizeof(*out));

    /* COMM_FORWARD_CAN reaches this function one call level deeper than the
     * local GET_VALUES path. Do not put the large motor_telemetry_t aggregate
     * on packet_process' stack here. Read only the six fallback fields from one
     * coherent FOC seqlock frame. This keeps VESC's read-reset-average semantics
     * while making local and virtual motor-2 use the same bounded stack depth. */
    float fallback[6] = {0};
    MotorRuntime *m = motor_get(id);
    foc_rt_snapshot_t rt;
    if (m != NULL && read_rt_snapshot(m, &rt)) {
        const float is = FOC_CURRENT_Q_BASE_A / 32768.0f;
        const float vs = FOC_VOLTAGE_Q_BASE_V / 32768.0f;
        const float id_a = (float)rt.id_filter_q15 * is;
        const float iq_a = (float)rt.iq_filter_q15 * is;
        const float imag = sqrtf(id_a * id_a + iq_a * iq_a);
        const float rt_iin = (float)rt.dc_current_q15 * is;
        fallback[0] = rt_iin < 0.0f ? -imag : imag;
        fallback[1] = rt_iin;
        fallback[2] = id_a;
        fallback[3] = iq_a;
        fallback[4] = (float)rt.vd_q15 * vs;
        fallback[5] = (float)rt.vq_q15 * vs;
    } else if (m != NULL) {
        const float imag = sqrtf(m->id_filter * m->id_filter + m->iq_filter * m->iq_filter);
        fallback[0] = m->input_current < 0.0f ? -imag : imag;
        fallback[1] = m->input_current;
        fallback[2] = m->id_filter;
        fallback[3] = m->iq_filter;
        fallback[4] = m->vd_filter;
        fallback[5] = m->vq_filter;
    }
    const uint8_t bits[6] = {2U, 3U, 4U, 5U, 19U, 20U};
    float sum[6] = {0};
    uint32_t count[6] = {0};

    /* Packet task dan timer task diserialisasi oleh scheduler, bukan global
     * IRQ mask. ADC/FOC ISR tetap dapat preempt selama copy/reset ini. */
    vTaskSuspendAll();
    for (unsigned k = 0U; k < 6U; k++) {
        if ((mask & (1UL << bits[k])) != 0U) {
            sum[k] = s_avg[idx].sum[k];
            count[k] = s_avg[idx].count[k];
            s_avg[idx].sum[k] = 0.0f;
            s_avg[idx].count[k] = 0U;
        }
    }
    (void)xTaskResumeAll();

    float value[6];
    for (unsigned k = 0U; k < 6U; k++) {
        value[k] = count[k] ? (sum[k] / (float)count[k]) : fallback[k];
    }
    out->current_motor = value[0]; out->current_in = value[1];
    out->id = value[2]; out->iq = value[3]; out->vd = value[4]; out->vq = value[5];
}
