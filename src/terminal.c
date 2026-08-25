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

static void print_help(void) {
    commands_printf("VESC F103 terminal: help, faults, currents, foc, sensor, observer [set N|sat N|speed N], appadc, timing, resources, powerstage, config, integrity, stats, reset_stats, odometer, stop");
}

void terminal_process_string(char *str) {
    if (str == 0) return;
    while (*str == ' ' || *str == '\t') str++;
    char *end = str + strlen(str);
    while (end > str && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) *--end = '\0';

    MotorRuntime *m = mc_interface_motor_runtime_now();
    if (!m) return;
    if (*str == '\0' || strcmp(str, "help") == 0) { print_help(); return; }

    if (strcmp(str, "faults") == 0) {
        /* Never consume the pending-fault latch from a diagnostic command.
         * The fault worker owns that destructive read. */
        commands_printf("motor=%d fault=%d pwm=%d state=%d", mc_interface_get_motor_thread(),
                (int)mc_interface_get_fault(), m->pwm_enabled ? 1 : 0, (int)m->state);
    } else if (strcmp(str, "currents") == 0) {
        commands_printf("Ia=%.2f Ib=%.2f Ic=%.2f Id=%.2f Iq=%.2f Imotor=%.2f Iin=%.2f A",
                (double)m->ia,(double)m->ib,(double)m->ic,(double)m->id_filter,(double)m->iq_filter,
                (double)m->motor_current,(double)m->input_current);
    } else if (strcmp(str, "foc") == 0) {
        commands_printf("ERPM=%.1f duty=%.4f Vd=%.2f Vq=%.2f phase=%.2f deg Vbus=%.2f",
                (double)m->erpm,(double)m->duty_now,(double)m->vd_filter,(double)m->vq_filter,
                (double)m->rotor_elec_deg,(double)m->vbus_filter);
    } else if (strcmp(str, "sensor") == 0 || strcmp(str, "encoder") == 0 || strcmp(str, "hall") == 0) {
        commands_printf("sensor=%d foc_sensor=%d hall=%u hall_valid=%d enc_cnt=%ld enc_sync=%d using_enc=%d",
                (int)m->sensor_mode,(int)m->foc_sensor_mode,(unsigned)m->hall.raw_state,m->hall.valid?1:0,
                (long)motor_encoder_extended_count(m),m->encoder.synced?1:0,m->using_encoder?1:0);
    } else if (strncmp(str, "observer", 8) == 0 && (str[8] == '\0' || str[8] == ' ' || str[8] == '\t')) {
        char *arg = str + 8;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg != '\0') {
            /* VESC 6.00 exposes observer IDs 0..3 in its native schema.
             * Part 2 also implements the later MXV family (4..6), so expose a
             * safe terminal path without changing the advertised wire ABI.
             * Every change still goes through the canonical serializer, motor
             * stopped check, apply path, CRC/transactional flash persistence. */
            char *value_s = arg;
            int field = 0; /* 1=type, 2=sat-comp, 3=speed-source */
            if (strncmp(arg, "set", 3) == 0 && (arg[3] == ' ' || arg[3] == '\t')) {
                field = 1; value_s = arg + 3;
            } else if (strncmp(arg, "sat", 3) == 0 && (arg[3] == ' ' || arg[3] == '\t')) {
                field = 2; value_s = arg + 3;
            } else if (strncmp(arg, "speed", 5) == 0 && (arg[5] == ' ' || arg[5] == '\t')) {
                field = 3; value_s = arg + 5;
            }
            while (*value_s == ' ' || *value_s == '\t') value_s++;
            char *parse_end = value_s;
            long value = strtol(value_s, &parse_end, 10);
            while (*parse_end == ' ' || *parse_end == '\t') parse_end++;
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
            mc_configuration cfg;
            conf_general_read_mc_configuration(&cfg, m->id == MOTOR_RIGHT);
            if (field == 1) cfg.foc_observer_type = (mc_foc_observer_type)value;
            if (field == 2) cfg.foc_sat_comp_mode = (SAT_COMP_MODE)value;
            if (field == 3) cfg.foc_speed_source = (FOC_SPEED_SRC)value;
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
        commands_printf("obs_valid=%d type=%d sat=%d lambda=%.6f obs_phase=%.2f pll_erpm=%.1f gain=%.1f slow=%.3f Rcfg=%.5f Rest=%.5f Rvalid=%d speed_src=%d",
                m->observer_valid?1:0,(int)m->foc_observer_type,(int)m->foc_sat_comp_mode,
                (double)((float)m->observer_lambda_est_q30*(FOC_FLUX_Q_BASE_WB/1073741824.0f)),
                (double)(((float)m->observer_phase_u16*360.0f)/65536.0f),
                (double)((float)m->pll_erpm_q16/65536.0f),(double)m->foc_observer_gain,
                (double)m->foc_observer_gain_slow,(double)m->foc_motor_r,(double)m->res_est_ohm,
                m->res_est_valid?1:0,(int)m->foc_speed_source);
    } else if (strcmp(str, "appadc") == 0) {
        app_adc_status_t a;
        app_adc_get_status(&a);
        commands_printf("APPADC PA2=%u %.3fV dec=%.3f PA3=%u %.3fV dec=%.3f cmd=%.3f armed=%d/%d src=%d/%d range=%d fault=0x%02x",
                (unsigned)a.raw1, (double)a.voltage1, (double)a.decoded1,
                (unsigned)a.raw2, (double)a.voltage2, (double)a.decoded2,
                (double)a.command, a.armed_left ? 1 : 0, a.armed_right ? 1 : 0,
                (int)app_command_get_source(MOTOR_LEFT), (int)app_command_get_source(MOTOR_RIGHT),
                a.range_ok ? 1 : 0, (unsigned)a.fault_flags);
    } else if (strcmp(str, "timing") == 0) {
        commands_printf("FOC=%.0fHz one_max=%lu dual_max=%lu near=%lu slot=%lu period=%lu..%lu contract=0x%lx clamp=%lu margin=%u WDT=%d healthy=%d badmask=0x%lx misses=%lu/%lu/%lu",
                (double)mc_interface_get_sampling_frequency_now(),
                (unsigned long)m->isr_max_cycles,
                (unsigned long)foc_isr_total_max_cycles(),
                (unsigned long)foc_isr_near_deadline_count(),
                (unsigned long)FOC_ISR_SLOT_CYCLES,
                (unsigned long)foc_isr_period_min_cycles(),
                (unsigned long)foc_isr_period_max_cycles(),
                (unsigned long)motor_hw_sampling_contract_flags(),
                (unsigned long)m->sampling_window_clamp_count,
                (unsigned)m->sampling_margin_min_q15,timeout_watchdog_started()?1:0,timeout_watchdog_healthy()?1:0,
                (unsigned long)timeout_watchdog_unhealthy_mask(),
                (unsigned long)timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_FOC),
                (unsigned long)timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_MOTOR_SERVICE),
                (unsigned long)timeout_watchdog_miss_count(TIMEOUT_HEARTBEAT_COMM));
    } else if (strcmp(str, "resources") == 0) {
        mc_interface_resource_stats_t r;
        vesc_comm_resource_stats_t c;
        mc_interface_get_resource_stats(&r);
        vesc_comm_get_resource_stats(&c);
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
    } else if (strcmp(str, "powerstage") == 0) {
        commands_printf("latch=0x%08lx pvd_low=%d brk1_en=%d brk8_en=%d short_ls=%d full_brake=%d",
                (unsigned long)motor_hw_powerstage_fault_flags(), motor_hw_pvd_low()?1:0,
                HOVERBOARD_TIM1_BREAK_ENABLE?1:0, HOVERBOARD_TIM8_BREAK_ENABLE?1:0,
                m->foc_short_ls_on_zero_duty?1:0, m->full_brake_active?1:0);
    } else if (strcmp(str, "config") == 0) {
        const volatile mc_configuration *c=mc_interface_get_configuration();
        commands_printf("I=[%.1f,%.1f]A Iin=[%.1f,%.1f]A ERPM=[%.0f,%.0f] duty=[%.3f,%.3f] sensor=%d short_ls=%d",
                (double)c->l_current_min,(double)c->l_current_max,(double)c->l_in_current_min,(double)c->l_in_current_max,
                (double)c->l_min_erpm,(double)c->l_max_erpm,(double)c->l_min_duty,(double)c->l_max_duty,(int)c->foc_sensor_mode,
                c->foc_short_ls_on_zero_duty?1:0);
    } else if (strcmp(str, "integrity") == 0) {
        commands_printf("config_valid=%d integrity=%d saves=%lu checks=%lu failures=%lu",
                conf_general_is_valid()?1:0, conf_general_integrity_ok()?1:0,
                (unsigned long)conf_general_get_save_count(),
                (unsigned long)conf_general_get_integrity_checks(),
                (unsigned long)conf_general_get_integrity_failures());
    } else if (strcmp(str, "stats") == 0) {
        commands_printf("avg speed=%.2f max=%.2f avg P=%.1fW max=%.1f avg I=%.2fA max=%.2f t=%.1fs",
                (double)mc_interface_stat_speed_avg(),(double)mc_interface_stat_speed_max(),(double)mc_interface_stat_power_avg(),
                (double)mc_interface_stat_power_max(),(double)mc_interface_stat_current_avg(),(double)mc_interface_stat_current_max(),
                (double)mc_interface_stat_count_time());
    } else if (strcmp(str, "reset_stats") == 0) {
        mc_interface_stat_reset(); commands_printf("stats reset");
    } else if (strncmp(str, "odometer", 8) == 0) {
        char *arg=str+8; while(*arg==' '||*arg=='\t')arg++;
        if(*arg){ unsigned long v=strtoul(arg,0,10); mc_interface_set_odometer((uint64_t)v); conf_general_request_aux_store(); }
        commands_printf("odometer=%lu m",(unsigned long)mc_interface_get_odometer());
    } else if (strcmp(str, "stop") == 0) {
        mc_interface_release_motor(); commands_printf("motor released");
    } else {
        commands_printf("Unknown command: %s",str); print_help();
    }
}
