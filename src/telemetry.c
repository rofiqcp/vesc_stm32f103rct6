#include "telemetry.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "applications/appconf_default.h"
#include "motor/foc_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>

// Variabel s_telem: state internal modul yang dipertahankan antar pemanggilan fungsi.
static motor_telemetry_t s_telem[2];

typedef struct {
    // Variabel sum: akumulator penjumlahan untuk averaging atau statistik.
    float sum[6];
    // Variabel count: pencacah kejadian atau sampel.
    uint32_t count[6];
} telemetry_avg_acc_t;

// Variabel s_avg: state internal modul yang dipertahankan antar pemanggilan fungsi.
static telemetry_avg_acc_t s_avg[2];
/* Seqlock ringan untuk cache telemetry task-side. Writer hanya 100 Hz dan
 * reader tidak pernah menunggu mutex, sehingga polling VESC Tool tidak dapat
 * menahan timer task atau jalur kontrol motor. */
// Variabel s_telem_seq: nomor urut untuk konsistensi snapshot atau record.
static volatile uint32_t s_telem_seq[2];

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi read_rt_snapshot: membaca read rt snapshot tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
static bool read_rt_snapshot(const MotorRuntime *m, foc_rt_snapshot_t *out);

/* VESC 6.00 mendefinisikan current_motor sebagai magnitudo arus dq dengan
 * tanda SIGN(Vq * Iq): positif saat daya mengalir ke motor dan negatif saat
 * regenerasi. Jangan memakai tanda current_in karena shunt DC memiliki jalur
 * filter/offset terpisah dan dapat berbeda sesaat dari state FOC yang sama. */
// Parameter id_a: arus sumbu-d FOC yang mengatur komponen fluks motor.
// Parameter iq_a: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Parameter vq_q15: tegangan sumbu-q hasil regulator FOC.
// Parameter iq_q15: arus sumbu-q FOC yang berkaitan dengan pembentukan torsi motor.
// Fungsi vesc_motor_current_signed: menjalankan operasi vesc motor current signed sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
static float vesc_motor_current_signed(float id_a, float iq_a, int32_t vq_q15, int32_t iq_q15) {
    // Variabel magnitude: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float magnitude = sqrtf(id_a * id_a + iq_a * iq_a);
    // Variabel negative: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool negative = ((vq_q15 < 0) != (iq_q15 < 0));
    return negative ? -magnitude : magnitude;
}

/* Inisialisasi cache telemetry tanpa alokasi heap.
 * Tidak ada mutex yang dapat memblokir task timer maupun packet-process. */
// Fungsi telemetry_init: menginisialisasi telemetry init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
bool telemetry_init(void) {
    memset(s_telem, 0, sizeof(s_telem));
    memset(s_avg, 0, sizeof(s_avg));
    s_telem_seq[0] = 0U;
    s_telem_seq[1] = 0U;
    return true;
}

/* Bangun enam nilai average dari SATU frame FOC koheren. Semua operasi float
 * termasuk sqrtf dilakukan di luar critical section agar ADC/FOC ISR 16 kHz
 * tidak tertunda oleh pekerjaan telemetry 100 Hz. */
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi build_avg_values: menjalankan operasi build avg values sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool build_avg_values(const MotorRuntime *m, float v[6]) {
    // Variabel rt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_rt_snapshot_t rt;
    if (m == NULL || v == NULL || !read_rt_snapshot(m, &rt))
        return false;

    // Variabel is: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float is = FOC_CURRENT_Q_BASE_A / 32768.0f;
    // Variabel vs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float vs = FOC_VOLTAGE_Q_BASE_V / 32768.0f;
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    const float id = (float)rt.id_filter_q15 * is;
    // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
    const float iq = (float)rt.iq_filter_q15 * is;
    // Variabel iin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const float iin = (float)rt.dc_current_q15 * is;
    v[0] = vesc_motor_current_signed(id, iq, rt.vq_q15, rt.iq_filter_q15);
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
// Parameter idx: indeks elemen yang sedang diproses.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi accumulate_avg_values: menjalankan operasi accumulate avg values sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static void accumulate_avg_values(unsigned idx, const float v[6]) {
    if (idx >= 2U || v == NULL)
        return;
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0U; k < 6U; k++) {
        s_avg[idx].sum[k] += v[k];
        if (s_avg[idx].count[k] != UINT32_MAX)
            s_avg[idx].count[k]++;
    }
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dt_s: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_stats: memperbarui update stats menggunakan state terbaru dengan urutan yang konsisten dan
// aman.
static void update_stats(MotorRuntime *m, float dt_s) {
    // Variabel ih: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ih = m->input_current*dt_s/3600.0f;
    // Variabel wh: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float wh = m->vbus_filter*m->input_current*dt_s/3600.0f;
    if (ih >= 0.0f)
        m->stats.amp_hours += ih;
    else m->stats.amp_hours_charged += -ih;
    if (wh >= 0.0f)
        m->stats.watt_hours += wh;
    else m->stats.watt_hours_charged += -wh;
    /* Treat position-derived tach as the raw motion source, not as the public
       counter value. VESC APIs can set/reset tachometer; the next statistics
       tick must add only new shaft motion instead of reconstructing the old
       absolute position and undoing that set/reset. */
    // Variabel tach_raw: nilai mentah sebelum konversi ke satuan fisik.
    int32_t tach_raw = (int32_t)lroundf((m->position_deg/360.0f)*(6.0f*(float)m->pole_pairs));
    // Variabel delta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t delta = 0;
    if (m->stats.tachometer_source_valid) {
        delta = tach_raw-m->stats.tachometer_last;
    }
    else {
        /* A configuration/sensor-coordinate change may change position scale
           or sign without physical shaft motion. Rebase the private motion
           source once so public tachometer/distance/odometer cannot jump. */
        m->stats.tachometer_source_valid = true;
    }
    m->stats.tachometer += delta;
    // Variabel delta64: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int64_t delta64 = (int64_t)delta;
    // Variabel abs_delta: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t abs_delta = (uint32_t)(delta64 < 0 ? -delta64 : delta64);
    m->stats.tachometer_abs += (int32_t)abs_delta;
    mc_interface_odometer_add_tach_delta(m->id, abs_delta);
    m->stats.tachometer_last = tach_raw;
    // Variabel ac: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ac = fabsf(m->motor_current);
    if (ac > m->stats.max_current)
        m->stats.max_current = ac;
    // Variabel ai: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ai = fabsf(m->input_current);
    if (ai > m->stats.max_input_current)
        m->stats.max_input_current = ai;
    // Variabel ar: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float ar = fabsf(m->erpm);
    if (ar > m->stats.max_erpm)
        m->stats.max_erpm = ar;
    m->stats.runtime_ms += STAT_PERIOD_MS;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi read_rt_snapshot: membaca read rt snapshot tanpa mengubah state kendali utama dan mengembalikan data
// yang konsisten.
static bool read_rt_snapshot(const MotorRuntime *m, foc_rt_snapshot_t *out) {
    if (m == NULL || out == NULL)
        return false;
    // Variabel retry: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (unsigned retry = 0U; retry < 8U; retry++) {
        // Variabel s1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t s1 = m->rt_snapshot_seq;
        if ((s1 & 1U) != 0U)
            continue;
        __DMB();
#define CPY(f) out->f = m->rt_snapshot.f
        CPY(ia_q15);
        CPY(ib_q15);
        CPY(ic_q15);
        CPY(id_q15);
        CPY(iq_q15);
        CPY(id_filter_q15);
        CPY(iq_filter_q15);
        CPY(id_target_q15);
        CPY(iq_target_q15);
        CPY(vd_q15);
        CPY(vq_q15);
        CPY(vbus_q15);
        CPY(dc_current_q15);
        CPY(erpm_fast_q16);
        CPY(duty_u_q15);
        CPY(duty_v_q15);
        CPY(duty_w_q15);
        CPY(phase_control_u16);
        CPY(phase_observer_u16);
        CPY(phase_encoder_u16);
        CPY(phase_hall_u16);
        CPY(adc_frame);
        CPY(cycle_counter);
#undef CPY
        __DMB();
        // Variabel s2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t s2 = m->rt_snapshot_seq;
        if (s1 == s2 && (s2 & 1U) == 0U)
            return true;
    }
    return false;
}

// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter t: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi snapshot: menyiapkan snapshot secara koheren untuk telemetri tanpa memblokir jalur ISR FOC.
static void snapshot(MotorRuntime *m, motor_telemetry_t *t) {
    // Variabel rt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_rt_snapshot_t rt;
    // Variabel have_rt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool have_rt = read_rt_snapshot(m, &rt);
    if (have_rt) {
        // Variabel is: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float is = FOC_CURRENT_Q_BASE_A / 32768.0f;
        // Variabel vs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float vs = FOC_VOLTAGE_Q_BASE_V / 32768.0f;
        t->phase_current_a = (float)rt.ia_q15 * is;
        t->phase_current_b = (float)rt.ib_q15 * is;
        t->phase_current_c = (float)rt.ic_q15 * is;
        t->id = (float)rt.id_q15 * is;
        t->iq = (float)rt.iq_q15 * is;
        t->id_filter = (float)rt.id_filter_q15 * is;
        t->iq_filter = (float)rt.iq_filter_q15 * is;
        t->id_target = (float)rt.id_target_q15 * is;
        t->iq_target = (float)rt.iq_target_q15 * is;
        t->vd = (float)rt.vd_q15 * vs;
        t->vq = (float)rt.vq_q15 * vs;
        // Variabel rt_iin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float rt_iin = (float)rt.dc_current_q15 * is;
        t->current_motor = vesc_motor_current_signed(t->id_filter, t->iq_filter,
                                                     rt.vq_q15, rt.iq_filter_q15);
        t->current_in = rt_iin;
        t->erpm = (float)rt.erpm_fast_q16 / 65536.0f;
        t->mech_rpm = t->erpm / fmaxf((float)m->pole_pairs, 1.0f);
        t->vbus = (float)rt.vbus_q15 * vs;
        t->rotor_elec_deg = (float)rt.phase_control_u16 * (360.0f/65536.0f);
        /* Duty VESC diambil dari frame FOC yang sama: magnitudo adalah deviasi
           terbesar phase duty terhadap 50%, sedangkan tandanya mengikuti Vq. */
        // Variabel duty_dev_u: deviasi duty phase-U terhadap titik tengah 50% dalam skala Q15.
        int32_t duty_dev_u = (int32_t)rt.duty_u_q15 - 16384;
        // Variabel duty_dev_v: deviasi duty phase-V terhadap titik tengah 50% dalam skala Q15.
        int32_t duty_dev_v = (int32_t)rt.duty_v_q15 - 16384;
        // Variabel duty_dev_w: deviasi duty phase-W terhadap titik tengah 50% dalam skala Q15.
        int32_t duty_dev_w = (int32_t)rt.duty_w_q15 - 16384;
        if (duty_dev_u < 0)
            duty_dev_u = -duty_dev_u;
        if (duty_dev_v < 0)
            duty_dev_v = -duty_dev_v;
        if (duty_dev_w < 0)
            duty_dev_w = -duty_dev_w;
        // Variabel duty_dev_max: deviasi phase terbesar yang mewakili magnitudo modulasi aktual.
        int32_t duty_dev_max = duty_dev_u;
        if (duty_dev_v > duty_dev_max)
            duty_dev_max = duty_dev_v;
        if (duty_dev_w > duty_dev_max)
            duty_dev_max = duty_dev_w;
        t->duty = (float)duty_dev_max / 16384.0f;
        if (rt.vq_q15 < 0)
            t->duty = -t->duty;
    }
    else {
        /* Startup before the first FOC frame only. */
        t->phase_current_a = m->ia;
        t->phase_current_b = m->ib;
        t->phase_current_c = m->ic;
        t->id = m->id_meas;
        t->iq = m->iq_meas;
        t->id_filter = m->id_filter;
        t->iq_filter = m->iq_filter;
        t->id_target = (float)m->id_target_q15 * (FOC_CURRENT_Q_BASE_A / 32768.0f);
        t->iq_target = (float)m->iq_target_q15 * (FOC_CURRENT_Q_BASE_A / 32768.0f);
        t->vd = m->vd_filter;
        t->vq = m->vq_filter;
        t->current_motor = m->motor_current;
        t->current_in = m->input_current;
        t->duty = m->duty_now;
        t->erpm = m->erpm;
        t->mech_rpm = m->mech_rpm;
        t->vbus = m->vbus_filter;
        t->rotor_elec_deg = m->rotor_elec_deg;
    }
    t->position_deg = foc_wrap_deg(m->position_deg);
    t->current_offset_u = (float)m->current_offset_u_counts;
    t->current_offset_v = (float)m->current_offset_v_counts;
    t->dc_current_offset = (float)m->dc_current_offset_counts;
    t->amp_hours = m->stats.amp_hours;
    t->amp_hours_charged = m->stats.amp_hours_charged;
    t->watt_hours = m->stats.watt_hours;
    t->watt_hours_charged = m->stats.watt_hours_charged;
    t->tachometer = m->stats.tachometer;
    t->tachometer_abs = m->stats.tachometer_abs;
    t->fault = (uint8_t)m->fault;
    t->controller_id = (m->id == MOTOR_LEFT) ? VESC_CONTROLLER_ID_LEFT : VESC_CONTROLLER_ID_RIGHT;
    t->sensor_mode=m->sensor_mode; /* physical pin/timer mux */
    t->foc_sensor_mode=(uint8_t)m->foc_sensor_mode; /* logical FOC source */
    t->sensor_detect_state = (uint8_t)m->detect.state;
    t->calibration_done = foc_calibration_done() ? 1U : 0U;
    t->calibration_valid = foc_calibration_valid() ? 1U : 0U;
    t->timeout_active = m->timeout_active ? 1U : 0U;
    t->observer_valid = m->observer_valid ? 1U : 0U;
    t->using_encoder = m->using_encoder ? 1U : 0U;
    t->encoder_synced = m->encoder.synced ? 1U : 0U;
    t->observer_phase_deg = m->observer_phase_deg;
    t->observer_erpm = m->observer_erpm;
    t->observer_quality = m->observer_quality;
    t->foc_motor_r = m->foc_motor_r;
    t->foc_motor_l = m->foc_motor_l;
    t->foc_motor_ld_lq_diff = m->foc_motor_ld_lq_diff;
    t->foc_motor_flux_linkage = m->foc_motor_flux_linkage;
    t->foc_sl_erpm_start = m->foc_sl_erpm_start;
    t->foc_sl_erpm = m->foc_sl_erpm;
    t->foc_openloop_rpm = m->foc_openloop_rpm;
    t->foc_openloop_rpm_low = m->foc_openloop_rpm_low;
    t->current_loop_hz = FOC_ISR_EVENT_HZ;
    t->telemetry_snapshot_hz = (STAT_PERIOD_MS > 0U) ? (1000U/STAT_PERIOD_MS) : 0U;
    t->isr_max_cycles = m->isr_max_cycles;
    t->isr_overruns = m->isr_overruns;
    t->abs_current_filtered = (float)m->abs_phase_current_filter_q15*(FOC_CURRENT_Q_BASE_A/32768.0f);
    t->abs_current_peak = (float)m->abs_current_peak_q15*(FOC_CURRENT_Q_BASE_A/32768.0f);
    t->over_voltage_fault_count = m->over_voltage_fault_count;
    t->under_voltage_fault_count = m->under_voltage_fault_count;
}

/* Update statistik dan average 100 Hz. Matematika FOC selesai sebelum scheduler
 * disuspend; interrupt prioritas tinggi termasuk ADC/DMA tetap dapat berjalan. */
// Fungsi telemetry_stats_update_100hz: menyiapkan telemetry stats update 100hz secara koheren untuk telemetri
// tanpa memblokir jalur ISR FOC.
void telemetry_stats_update_100hz(void) {
    // Variabel left_v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float left_v[6];
    // Variabel right_v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float right_v[6];
    // Variabel have_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool have_left = build_avg_values(&g_motor_left, left_v);
    // Variabel have_right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool have_right = build_avg_values(&g_motor_right, right_v);

    /* Hanya task yang mengakses accumulator. Suspend scheduler menjaga
     * read-reset atomik TANPA mematikan ADC/DMA interrupt prioritas tinggi. */
    vTaskSuspendAll();
    if (have_left)
        accumulate_avg_values(0U, left_v);
    if (have_right)
        accumulate_avg_values(1U, right_v);
    (void)xTaskResumeAll();

    update_stats(&g_motor_left, STAT_PERIOD_MS/1000.0f);
    update_stats(&g_motor_right, STAT_PERIOD_MS/1000.0f);
}

/* Tulis cache telemetry langsung di bawah seqlock. Tidak memakai temporary
 * motor_telemetry_t di stack timer 1 KiB dan tidak memakai mutex/heap. Pada
 * single-core F103 packet task tidak dapat berjalan bersamaan dengan writer;
 * sequence tetap melindungi pembacaan jika ada perubahan scheduling kelak. */
// Parameter idx: indeks elemen yang sedang diproses.
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi update_cached_snapshot: menyiapkan update cached snapshot secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
static void update_cached_snapshot(unsigned idx, MotorRuntime *m) {
    if (idx >= 2U || m == NULL)
        return;
    // Variabel seq: nomor urut untuk konsistensi snapshot atau record.
    uint32_t seq = s_telem_seq[idx];
    s_telem_seq[idx] = seq + 1U;
    __DMB();
    memset(&s_telem[idx], 0, sizeof(s_telem[idx]));
    snapshot(m, &s_telem[idx]);
    __DMB();
    s_telem_seq[idx] = seq + 2U;
}

/* Ambil snapshot 100 Hz tanpa mutex/heap dan tanpa menambah stack besar pada
 * timer_thread. */
// Fungsi telemetry_snapshot_100hz: menyiapkan telemetry snapshot 100hz secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
void telemetry_snapshot_100hz(void) {
    update_cached_snapshot(0U, &g_motor_left);
    update_cached_snapshot(1U, &g_motor_right);
}

// Fungsi telemetry_update_100hz: menyiapkan telemetry update 100hz secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
void telemetry_update_100hz(void) {
    telemetry_stats_update_100hz();
    telemetry_snapshot_100hz();
}

/* Baca cache telemetry secara lock-free. Jika writer bertepatan terus dengan
 * pembacaan, fallback ke snapshot realtime agar komunikasi tidak pernah macet. */
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi telemetry_get: membaca telemetry get tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
void telemetry_get(motor_id_t id, motor_telemetry_t *out) {
    if (out == NULL)
        return;
    // Variabel idx: indeks elemen yang sedang diproses.
    const uint32_t idx = (id == MOTOR_RIGHT) ? 1U : 0U;
    // Variabel retry: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (unsigned retry = 0U; retry < 8U; retry++) {
        // Variabel s1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint32_t s1 = s_telem_seq[idx];
        if ((s1 & 1U) != 0U)
            continue;
        __DMB();
        *out = s_telem[idx];
        __DMB();
        // Variabel s2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint32_t s2 = s_telem_seq[idx];
        if (s1 == s2 && (s2 & 1U) == 0U)
            return;
    }

    memset(out, 0, sizeof(*out));
    snapshot(motor_get(id), out);
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi telemetry_get_realtime: membaca telemetry get realtime tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
void telemetry_get_realtime(motor_id_t id, motor_telemetry_t *out) {
    if (out == NULL)
        return;
    memset(out, 0, sizeof(*out));
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (m != NULL)
        snapshot(m, out);
}

/* Baca lalu reset average VESC untuk I_motor/I_in/Id/Iq/Vd/Vq. Scheduler
 * disuspend sebentar agar transaksi atomik, sementara ADC/FOC IRQ tetap aktif. */
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter mask: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter out: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi telemetry_read_reset_avg: menyiapkan telemetry read reset avg secara koheren untuk telemetri tanpa
// memblokir jalur ISR FOC.
void telemetry_read_reset_avg(motor_id_t id, uint32_t mask, motor_telemetry_avg_t *out) {
    if (out == NULL)
        return;
    // Variabel idx: indeks elemen yang sedang diproses.
    const unsigned idx = (id == MOTOR_RIGHT) ? 1U : 0U;
    memset(out, 0, sizeof(*out));

    /* COMM_FORWARD_CAN reaches this function one call level deeper than the
     * local GET_VALUES path. Do not put the large motor_telemetry_t aggregate
     * on packet_process' stack here. Read only the six fallback fields from one
     * coherent FOC seqlock frame. This keeps VESC's read-reset-average semantics
     * while making local and virtual motor-2 use the same bounded stack depth. */
    // Variabel fallback: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float fallback[6] = {
        0
    }
    ;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    // Variabel rt: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    foc_rt_snapshot_t rt;
    if (m != NULL && read_rt_snapshot(m, &rt)) {
        // Variabel is: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float is = FOC_CURRENT_Q_BASE_A / 32768.0f;
        // Variabel vs: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float vs = FOC_VOLTAGE_Q_BASE_V / 32768.0f;
        // Variabel id_a: arus sumbu-d FOC yang digunakan untuk pengaturan fluks motor.
        const float id_a = (float)rt.id_filter_q15 * is;
        // Variabel iq_a: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
        const float iq_a = (float)rt.iq_filter_q15 * is;
        // Variabel rt_iin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float rt_iin = (float)rt.dc_current_q15 * is;
        fallback[0] = vesc_motor_current_signed(id_a, iq_a, rt.vq_q15, rt.iq_filter_q15);
        fallback[1] = rt_iin;
        fallback[2] = id_a;
        fallback[3] = iq_a;
        fallback[4] = (float)rt.vd_q15 * vs;
        fallback[5] = (float)rt.vq_q15 * vs;
    }
    else if (m != NULL) {
        // Variabel imag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float imag = sqrtf(m->id_filter * m->id_filter + m->iq_filter * m->iq_filter);
        // Variabel regen: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const bool regen = ((m->vq_filter < 0.0f) != (m->iq_filter < 0.0f));
        fallback[0] = regen ? -imag : imag;
        fallback[1] = m->input_current;
        fallback[2] = m->id_filter;
        fallback[3] = m->iq_filter;
        fallback[4] = m->vd_filter;
        fallback[5] = m->vq_filter;
    }
    // Variabel bits: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t bits[6] = {
        2U, 3U, 4U, 5U, 19U, 20U
    }
    ;
    // Variabel sum: akumulator penjumlahan untuk averaging atau statistik.
    float sum[6] = {
        0
    }
    ;
    // Variabel count: pencacah kejadian atau sampel.
    uint32_t count[6] = {
        0
    }
    ;

    /* Packet task dan timer task diserialisasi oleh scheduler, bukan global
     * IRQ mask. ADC/FOC ISR tetap dapat preempt selama copy/reset ini. */
    vTaskSuspendAll();
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0U; k < 6U; k++) {
        if ((mask & (1UL << bits[k])) != 0U) {
            sum[k] = s_avg[idx].sum[k];
            count[k] = s_avg[idx].count[k];
            s_avg[idx].sum[k] = 0.0f;
            s_avg[idx].count[k] = 0U;
        }
    }
    (void)xTaskResumeAll();

    // Variabel value: nilai kerja sesuai konteks algoritma.
    float value[6];
    // Variabel k: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned k = 0U; k < 6U; k++) {
        value[k] = count[k] ? (sum[k] / (float)count[k]) : fallback[k];
    }
    out->current_motor = value[0];
    out->current_in = value[1];
    out->id = value[2];
    out->iq = value[3];
    out->vd = value[4];
    out->vq = value[5];
}
