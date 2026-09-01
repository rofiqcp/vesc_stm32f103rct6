#include "terminal.h"
#include "comm/commands.h"
#include "motor/mc_interface.h"
#include "telemetry.h"
#include "timeout.h"
#include "motor/mcpwm_foc.h"
#include "applications/app_uartcomm.h"
#include "applications/app.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "conf_general.h"
#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>


// Ini menjaga terminal VESC tetap menampilkan angka pada newlib-nano sekaligus menghindari penambahan
// _printf_float yang boros Flash.
// Parameter dst: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dst_len: panjang data yang sedang diproses atau dikirim.
// Parameter value: nilai kerja yang digunakan oleh algoritma pada konteks tersebut.
// Parameter decimals: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi terminal_format_float: melayani terminal format float sebagai diagnostik terminal tanpa menambah beban
// pada loop kontrol real-time.
static void terminal_format_float(char *dst, size_t dst_len, float value, uint8_t decimals) {
    // Variabel scale_lut: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static const int32_t scale_lut[] = {
        1, 10, 100, 1000, 10000, 100000, 1000000
    }
    ;
    if (dst == NULL || dst_len == 0U)
        return;
    if (!isfinite(value)) {
        (void)snprintf(dst, dst_len, "nan");
        return;
    }
    if (decimals > 6U)
        decimals = 6U;
    // Variabel scale: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const int32_t scale = scale_lut[decimals];
    // Variabel scaled_f: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    float scaled_f = value * (float)scale;
    if (scaled_f > 2147483000.0f)
        scaled_f = 2147483000.0f;
    if (scaled_f < -2147483000.0f)
        scaled_f = -2147483000.0f;
    // Variabel scaled: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int32_t scaled = (int32_t)lrintf(scaled_f);
    // Variabel negative: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool negative = scaled < 0;
    // Variabel mag: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t mag = negative ? (uint32_t)(-(int64_t)scaled) : (uint32_t)scaled;
    // Variabel whole: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t whole = mag / (uint32_t)scale;
    // Variabel frac: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t frac = mag % (uint32_t)scale;
    if (decimals == 0U) {
        (void)snprintf(dst, dst_len, "%s%lu", negative ? "-" : "", (unsigned long)whole);
    }
    else {
        (void)snprintf(dst, dst_len, "%s%lu.%0*lu", negative ? "-" : "", (unsigned long)whole,
                       (int)decimals, (unsigned long)frac);
    }
}

// Fungsi print_help: menjalankan operasi print help sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static void print_help(void) {
    commands_printf("VESC F103 terminal: help, faults, currents, foc, sensor, observer [set N|sat N|speed N], adc, appadc, timing, resources, powerstage, config, integrity, stats, reset_stats, odometer, stop");
}

// Parameter str: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi terminal_process_string: melayani terminal process string sebagai diagnostik terminal tanpa menambah
// beban pada loop kontrol real-time.
void terminal_process_string(char *str) {
    if (str == 0)
        return;
    while (*str == ' ' || *str == '\t')
        str++;
    // Variabel end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    char *end = str + strlen(str);
    while (end > str && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
        *--end = '\0';

    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = mc_interface_motor_runtime_now();
    if (!m)
        return;
    if (*str == '\0' || strcmp(str, "help") == 0) {
        print_help();
        return;
    }

    if (strcmp(str, "faults") == 0) {
        /* Never consume the pending-fault latch from a diagnostic command.
         * The fault worker owns that destructive read. */
        commands_printf("motor=%d fault=%d last=%d recover_ms=%u/1000 pwm=%d state=%d",
                mc_interface_get_motor_thread(), (int)mc_interface_get_fault(),
                (int)motor_fault_to_vesc(m->last_fault), (unsigned)m->fault_recovery_ticks,
                m->pwm_enabled ? 1 : 0, (int)m->state);
    }
    else if (strcmp(str, "currents") == 0) {
        // Variabel ia: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ib: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel ic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
        // Variabel iin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel im: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel iq: arus sumbu-q FOC yang terutama berkaitan dengan pembentukan torsi motor.
        char ia[16], ib[16], ic[16], id[16], iq[16], im[16], iin[16];
        terminal_format_float(ia, sizeof(ia), m->ia, 2);
        terminal_format_float(ib, sizeof(ib), m->ib, 2);
        terminal_format_float(ic, sizeof(ic), m->ic, 2);
        terminal_format_float(id, sizeof(id), m->id_filter, 2);
        terminal_format_float(iq, sizeof(iq), m->iq_filter, 2);
        terminal_format_float(im, sizeof(im), m->motor_current, 2);
        terminal_format_float(iin, sizeof(iin), m->input_current, 2);
        commands_printf("Ia=%s Ib=%s Ic=%s Id=%s Iq=%s Imotor=%s Iin=%s A", ia, ib, ic, id, iq, im, iin);
    }
    else if (strcmp(str, "foc") == 0) {
        // Variabel duty: rasio duty PWM yang diminta, dibatasi, atau dilaporkan.
        // Variabel erpm: kecepatan listrik motor dalam electrical RPM.
        // Variabel phase: sudut atau data fasa listrik untuk komutasi dan transformasi FOC.
        // Variabel vbus: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
        // Variabel vd: tegangan sumbu-d keluaran regulator FOC.
        // Variabel vq: tegangan sumbu-q keluaran regulator FOC.
        char erpm[18], duty[16], vd[16], vq[16], phase[16], vbus[16];
        terminal_format_float(erpm, sizeof(erpm), m->erpm, 1);
        terminal_format_float(duty, sizeof(duty), m->duty_now, 4);
        terminal_format_float(vd, sizeof(vd), m->vd_filter, 2);
        terminal_format_float(vq, sizeof(vq), m->vq_filter, 2);
        terminal_format_float(phase, sizeof(phase), m->rotor_elec_deg, 2);
        terminal_format_float(vbus, sizeof(vbus), m->vbus_filter, 2);
        commands_printf("ERPM=%s duty=%s Vd=%s Vq=%s phase=%s deg Vbus=%s", erpm, duty, vd, vq, phase, vbus);
    }
    else if (strcmp(str, "sensor") == 0 || strcmp(str, "encoder") == 0 || strcmp(str, "hall") == 0) {
        commands_printf("sensor=%d foc_sensor=%d hall=%u hall_valid=%d enc_cnt=%ld enc_sync=%d using_enc=%d",
                (int)m->sensor_mode, (int)m->foc_sensor_mode, (unsigned)m->hall.raw_state, m->hall.valid ? 1 : 0,
                (long)motor_encoder_extended_count(m), m->encoder.synced ? 1 : 0, m->using_encoder ? 1 : 0);
    }
    else if (strncmp(str, "observer", 8) == 0 && (str[8] == '\0' || str[8] == ' ' || str[8] == '\t')) {
        // Variabel arg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char *arg = str + 8;
        while (*arg == ' ' || *arg == '\t')
            arg++;
        if (*arg != '\0') {
            /* VESC 6.00 exposes observer IDs 0..3 in its native schema.
             * Part 2 also implements the later MXV family (4..6), so expose a
             * safe terminal path without changing the advertised wire ABI.
             * Every change still goes through the canonical serializer, motor
             * stopped check, apply path, CRC/transactional flash persistence. */
            // Variabel value_s: nilai kerja sesuai konteks algoritma.
            char *value_s = arg;
            // Variabel field: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int field = 0; /* 1=type, 2=sat-comp, 3=speed-source */
            if (strncmp(arg, "set", 3) == 0 && (arg[3] == ' ' || arg[3] == '\t')) {
                field = 1;
                value_s = arg + 3;
            }
            else if (strncmp(arg, "sat", 3) == 0 && (arg[3] == ' ' || arg[3] == '\t')) {
                field = 2;
                value_s = arg + 3;
            }
            else if (strncmp(arg, "speed", 5) == 0 && (arg[5] == ' ' || arg[5] == '\t')) {
                field = 3;
                value_s = arg + 5;
            }
            while (*value_s == ' ' || *value_s == '\t')
                value_s++;
            // Variabel parse_end: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            char *parse_end = value_s;
            // Variabel value: nilai kerja sesuai konteks algoritma.
            long value = strtol(value_s, &parse_end, 10);
            while (*parse_end == ' ' || *parse_end == '\t')
                parse_end++;
            // Variabel range_ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool range_ok = (field == 1 && value >= 0 && value <= 6) ||
                            (field == 2 && value >= 0 && value <= 3) ||
                            (field == 3 && value >= 0 && value <= 1);
            if (field == 0 || value_s == parse_end || *parse_end != '\0' || !range_ok) {
                commands_printf("usage: observer [set 0..6 | sat 0..3 | speed 0..1]");
                return;
            }
            if (m->pwm_enabled || m->detect.busy) {
                commands_printf("observer config rejected: stop motor/detect first");
                return;
            }
            // Variabel cfg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            mc_configuration cfg;
            conf_general_read_mc_configuration(&cfg, m->id == MOTOR_RIGHT);
            if (field == 1)
                cfg.foc_observer_type = (mc_foc_observer_type)value;
            if (field == 2)
                cfg.foc_sat_comp_mode = (SAT_COMP_MODE)value;
            if (field == 3)
                cfg.foc_speed_source = (FOC_SPEED_SRC)value;
            if (!conf_general_store_mc_configuration(&cfg, m->id == MOTOR_RIGHT)) {
                commands_printf("observer config rejected by validation/store");
                return;
            }
            m = mc_interface_motor_runtime_now();
            commands_printf("observer config stored: motor=%d type=%d sat=%d speed_src=%d",
                    mc_interface_get_motor_thread(), (int)m->foc_observer_type,
                    (int)m->foc_sat_comp_mode, (int)m->foc_speed_source);
            return;
        }
        // Variabel gain: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel lambda: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel obsphase: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel pllerpm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel rcfg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel rest: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel slow: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char lambda[18], obsphase[16], pllerpm[18], gain[18], slow[18], rcfg[18], rest[18];
        terminal_format_float(lambda, sizeof(lambda), (float)m->observer_lambda_est_q30*(FOC_FLUX_Q_BASE_WB/1073741824.0f), 6);
        terminal_format_float(obsphase, sizeof(obsphase), ((float)m->observer_phase_u16*360.0f)/65536.0f, 2);
        terminal_format_float(pllerpm, sizeof(pllerpm), (float)m->pll_erpm_q16/65536.0f, 1);
        terminal_format_float(gain, sizeof(gain), m->foc_observer_gain, 1);
        terminal_format_float(slow, sizeof(slow), m->foc_observer_gain_slow, 3);
        terminal_format_float(rcfg, sizeof(rcfg), m->foc_motor_r, 5);
        terminal_format_float(rest, sizeof(rest), m->res_est_ohm, 5);
        commands_printf("obs_valid=%d type=%d sat=%d lambda=%s obs_phase=%s pll_erpm=%s gain=%s slow=%s Rcfg=%s Rest=%s Rvalid=%d speed_src=%d",
                m->observer_valid ? 1 : 0, (int)m->foc_observer_type, (int)m->foc_sat_comp_mode,
                lambda, obsphase, pllerpm, gain, slow, rcfg, rest, m->res_est_valid ? 1 : 0, (int)m->foc_speed_source);
    }
    else if (strcmp(str, "adc") == 0) {
        // Variabel count: pencacah kejadian atau sampel.
        // Variabel target: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t count = 0U, target = 0U;
        foc_get_calibration_progress(&count, &target);
        commands_printf("ADC motor=%d rawU=%u rawV=%u rawDC=%u ofsU=%ld ofsV=%ld ofsDC=%ld scale=20mA/count cal=%lu/%lu stage=%u valid=%d",
                mc_interface_get_motor_thread(), (unsigned)m->current_raw_u, (unsigned)m->current_raw_v, (unsigned)m->dc_current_raw,
                (long)m->current_offset_u_counts, (long)m->current_offset_v_counts, (long)m->dc_current_offset_counts,
                (unsigned long)count, (unsigned long)target, (unsigned)foc_calibration_stage(), foc_calibration_valid() ? 1 : 0);
    }
    else if (strcmp(str, "appadc") == 0) {
        // Variabel a: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        app_adc_status_t a;
        app_adc_get_status(&a);
        // Variabel cmd: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel d1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel d2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel v1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel v2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char v1[16], d1[16], v2[16], d2[16], cmd[16];
        terminal_format_float(v1, sizeof(v1), a.voltage1, 3);
        terminal_format_float(d1, sizeof(d1), a.decoded1, 3);
        terminal_format_float(v2, sizeof(v2), a.voltage2, 3);
        terminal_format_float(d2, sizeof(d2), a.decoded2, 3);
        terminal_format_float(cmd, sizeof(cmd), a.command, 3);
        commands_printf("APPADC PA2=%u %sV dec=%s PA3=%u %sV dec=%s cmd=%s armed=%d/%d src=%d/%d range=%d fault=0x%02x",
                (unsigned)a.raw1, v1, d1, (unsigned)a.raw2, v2, d2, cmd, a.armed_left ? 1 : 0, a.armed_right ? 1 : 0,
                (int)app_command_get_source(MOTOR_LEFT), (int)app_command_get_source(MOTOR_RIGHT),
                a.range_ok ? 1 : 0, (unsigned)a.fault_flags);
    }
    else if (strcmp(str, "timing") == 0) {
        // Variabel foc_hz: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char foc_hz[16];
        terminal_format_float(foc_hz, sizeof(foc_hz), mc_interface_get_sampling_frequency_now(), 0);
        commands_printf("FOC=%sHz one_max=%lu dual_max=%lu near=%lu slot=%lu period=%lu..%lu contract=0x%lx clamp=%lu margin=%u WDT=%d healthy=%d badmask=0x%lx misses=%lu/%lu/%lu",
                foc_hz,
                (unsigned long)m->isr_max_cycles,
                (unsigned long)foc_isr_total_max_cycles(),
                (unsigned long)foc_isr_near_deadline_count(),
                (unsigned long)FOC_ISR_SLOT_CYCLES,
                (unsigned long)foc_isr_period_min_cycles(),
                (unsigned long)foc_isr_period_max_cycles(),
                (unsigned long)motor_hw_sampling_contract_flags(),
                (unsigned long)m->sampling_window_clamp_count,
                (unsigned)m->sampling_margin_min_q15, timeout_watchdog_started() ? 1 : 0, timeout_watchdog_healthy() ? 1 : 0,
                (unsigned long)timeout_watchdog_unhealthy_mask(),
                (unsigned long)timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_FOC),
                (unsigned long)timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_MOTOR_SERVICE),
                (unsigned long)timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_COMM));
    }
    else if (strcmp(str, "resources") == 0) {
        // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        mc_interface_resource_stats_t r;
        // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        vesc_comm_resource_stats_t c;
        mc_interface_get_resource_stats(&r);
        vesc_comm_get_resource_stats(&c);
        // Variabel u: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const app_uartcomm_stats_t *u = app_uartcomm_get_stats();
        commands_printf("heap=%lu min=%lu stack_free motor/sample/fault/status/packet/block=%lu/%lu/%lu/%lu/%lu/%lu TXq_hwm=%lu busy_drop=%lu",
                (unsigned long)r.heap_free_bytes, (unsigned long)r.heap_min_ever_bytes,
                (unsigned long)r.motor_service_stack_free_bytes,
                (unsigned long)r.sample_sender_stack_free_bytes,
                (unsigned long)r.fault_stack_free_bytes,
                (unsigned long)r.status_stack_free_bytes,
                (unsigned long)c.packet_stack_free_bytes,
                (unsigned long)c.blocking_stack_free_bytes,
                (unsigned long)u->tx_queue_high_water,
                (unsigned long)u->tx_queue_busy_drops);
    }
    else if (strcmp(str, "powerstage") == 0) {
        commands_printf("latch=0x%08lx pvd_low=%d brk1_en=%d brk8_en=%d short_ls=%d full_brake=%d",
                (unsigned long)motor_hw_powerstage_fault_flags(), motor_hw_pvd_low() ? 1 : 0,
                HOVERBOARD_TIM1_BREAK_ENABLE ? 1 : 0, HOVERBOARD_TIM8_BREAK_ENABLE ? 1 : 0,
                m->foc_short_ls_on_zero_duty ? 1 : 0, m->full_brake_active ? 1 : 0);
    }
    else if (strcmp(str, "config") == 0) {
        // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const volatile mc_configuration *c = mc_interface_get_configuration();
        // Variabel dmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel dmin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel emax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel emin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel iinmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel iinmin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel imax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel imin: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char imin[16], imax[16], iinmin[16], iinmax[16], emin[18], emax[18], dmin[16], dmax[16];
        terminal_format_float(imin, sizeof(imin), c->l_current_min, 1);
        terminal_format_float(imax, sizeof(imax), c->l_current_max, 1);
        terminal_format_float(iinmin, sizeof(iinmin), c->l_in_current_min, 1);
        terminal_format_float(iinmax, sizeof(iinmax), c->l_in_current_max, 1);
        terminal_format_float(emin, sizeof(emin), c->l_min_erpm, 0);
        terminal_format_float(emax, sizeof(emax), c->l_max_erpm, 0);
        terminal_format_float(dmin, sizeof(dmin), c->l_min_duty, 3);
        terminal_format_float(dmax, sizeof(dmax), c->l_max_duty, 3);
        commands_printf("I=[%s,%s]A Iin=[%s,%s]A ERPM=[%s,%s] duty=[%s,%s] sensor=%d short_ls=%d",
                imin, imax, iinmin, iinmax, emin, emax, dmin, dmax, (int)c->foc_sensor_mode, c->foc_short_ls_on_zero_duty ? 1 : 0);
    }
    else if (strcmp(str, "integrity") == 0) {
        commands_printf("config_valid=%d integrity=%d saves=%lu checks=%lu failures=%lu",
                conf_general_is_valid() ? 1 : 0, conf_general_integrity_ok() ? 1 : 0,
                (unsigned long)conf_general_get_save_count(),
                (unsigned long)conf_general_get_integrity_checks(),
                (unsigned long)conf_general_get_integrity_failures());
    }
    else if (strcmp(str, "stats") == 0) {
        // Variabel iavg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel imax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel pavg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel pmax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel savg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel smax: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        // Variabel tm: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char savg[16], smax[16], pavg[16], pmax[16], iavg[16], imax[16], tm[16];
        terminal_format_float(savg, sizeof(savg), mc_interface_stat_speed_avg(), 2);
        terminal_format_float(smax, sizeof(smax), mc_interface_stat_speed_max(), 2);
        terminal_format_float(pavg, sizeof(pavg), mc_interface_stat_power_avg(), 1);
        terminal_format_float(pmax, sizeof(pmax), mc_interface_stat_power_max(), 1);
        terminal_format_float(iavg, sizeof(iavg), mc_interface_stat_current_avg(), 2);
        terminal_format_float(imax, sizeof(imax), mc_interface_stat_current_max(), 2);
        terminal_format_float(tm, sizeof(tm), mc_interface_stat_count_time(), 1);
        commands_printf("avg speed=%s max=%s avg P=%sW max=%s avg I=%sA max=%s t=%ss", savg, smax, pavg, pmax, iavg, imax, tm);
    }
    else if (strcmp(str, "reset_stats") == 0) {
        mc_interface_stat_reset();
        commands_printf("stats reset");
    }
    else if (strncmp(str, "odometer", 8) == 0) {
        // Variabel arg: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        char *arg = str+8;
        while (*arg == ' ' || *arg == '\t')
            arg++;
        if (*arg) {
            // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            unsigned long v = strtoul(arg, 0, 10);
            mc_interface_set_odometer((uint64_t)v);
            conf_general_request_aux_store();
        }
        commands_printf("odometer=%lu m", (unsigned long)mc_interface_get_odometer());
    }
    else if (strcmp(str, "stop") == 0) {
        mc_interface_release_motor();
        commands_printf("motor released");
    }
    else {
        commands_printf("Unknown command: %s", str);
        print_help();
    }
}
