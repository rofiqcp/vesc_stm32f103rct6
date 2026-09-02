#include "applications/app_command.h"
#include "applications/app.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

// Variabel s_source: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile app_command_source_t s_source[2];
// Variabel s_uart_last_ms: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_uart_last_ms[2];
// Variabel s_adc_rearm: nilai atau state ADC pada jalur pengukuran.
static volatile bool s_adc_rearm[2];
// Variabel s_uart_reject_reason: alasan penolakan terakhir command UART untuk tiap motor.
static volatile uint8_t s_uart_reject_reason[2];


// Fungsi app_command_init: menginisialisasi app command init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
void app_command_init(void) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0U; i < 2U; i++) {
        s_source[i] = APP_CMD_SRC_NONE;
        s_uart_last_ms[i] = 0U;
        s_adc_rearm[i] = true;
        s_uart_reject_reason[i] = APP_UART_REJECT_NONE;
    }
}

// Fungsi app_command_configuration_changed: menjalankan operasi app command configuration changed sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void app_command_configuration_changed(void) {
    /* APPCONF writes are accepted only while both bridges are off. Drop any
     * stale application lease and require PA2/PA3 to pass neutral safe-start
     * under the new calibration before control can resume. Safe during boot:
     * this function only touches the small arbitration state. */
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0U; i < 2U; i++) {
        s_source[i] = APP_CMD_SRC_NONE;
        s_uart_last_ms[i] = 0U;
        s_adc_rearm[i] = true;
        s_uart_reject_reason[i] = APP_UART_REJECT_NONE;
    }
}

// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi app_command_service_1khz: menjalankan operasi app command service 1khz sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_command_service_1khz(uint32_t now_ms) {
    // Variabel conf: data konfigurasi yang mengatur perilaku firmware.
    const app_configuration *conf = app_get_configuration();
    if (conf == NULL)
        return;

    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0U; i < 2U; i++) {
        // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        MotorRuntime *m = motor_get((motor_id_t)i);

        /* Any application-disable request or motor fault immediately revokes
         * a stale application owner. Hardware fault handling already drops
         * MOE; this also prevents a cleared fault from resuming old throttle. */
        if (app_is_output_disabled() || m->fault != MOTOR_FAULT_NONE) {
            // Variabel old_source: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            const app_command_source_t old_source = s_source[i];
            if (old_source == APP_CMD_SRC_ADC || old_source == APP_CMD_SRC_UART)
                motor_stop(m);
            s_source[i] = APP_CMD_SRC_NONE;
            /* Match VESC SAFE_START_NO_FAULT semantics: a motor fault revokes
             * torque ownership, but does not force a new neutral dwell when
             * the ADC application explicitly selects NO_FAULT. Output-disable
             * requests and UART ownership still force re-arm. */
            if (app_is_output_disabled() || old_source != APP_CMD_SRC_ADC ||
                conf->app_adc_conf.safe_start != SAFE_START_NO_FAULT) {
                s_adc_rearm[i] = true;
            }
            continue;
        }

        /* Detection/calibration own the power stage exclusively. These source
         * states are observational guards only; the existing detection and
         * calibration code keeps its proven direct motor-control path. */
        if (m->detect.busy) {
            s_source[i] = APP_CMD_SRC_DETECTION;
            s_adc_rearm[i] = true;
            continue;
        }
        if (s_source[i] == APP_CMD_SRC_DETECTION) {
            s_source[i] = APP_CMD_SRC_NONE;
            s_adc_rearm[i] = true;
        }

        if (!foc_calibration_done() || !foc_calibration_valid()) {
            s_source[i] = APP_CMD_SRC_CALIBRATION;
            s_adc_rearm[i] = true;
            continue;
        }
        if (s_source[i] == APP_CMD_SRC_CALIBRATION) {
            s_source[i] = APP_CMD_SRC_NONE;
            s_adc_rearm[i] = true;
        }

        /* Per-motor serial lease is necessary even when APP_ADC_UART keeps the
         * legacy global timeout alive. It also makes LEFT/RIGHT command expiry
         * independent rather than stopping both because one host went stale. */
        if (s_source[i] == APP_CMD_SRC_UART && conf->timeout_msec != 0U &&
            (uint32_t)(now_ms - s_uart_last_ms[i]) > conf->timeout_msec) {
            motor_stop(m);
            s_source[i] = APP_CMD_SRC_NONE;
            s_adc_rearm[i] = true;
        }
    }
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_uart_claim: menjalankan operasi app command uart claim sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool app_command_uart_claim(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return false;

    // Variabel m: runtime motor yang akan menerima perintah UART.
    MotorRuntime *m = motor_get(id);
    s_uart_reject_reason[id] = APP_UART_REJECT_NONE;
    if (m == NULL) {
        s_uart_reject_reason[id] = APP_UART_REJECT_INVALID_ID;
        return false;
    }
    if (m->detect.busy) {
        s_uart_reject_reason[id] = APP_UART_REJECT_DETECT_BUSY;
        return false;
    }
    if (app_is_output_disabled()) {
        s_uart_reject_reason[id] = APP_UART_REJECT_OUTPUT_DISABLED;
        return false;
    }
    if (m->fault != MOTOR_FAULT_NONE) {
        s_uart_reject_reason[id] = APP_UART_REJECT_FAULT;
        return false;
    }
    if (!foc_calibration_done()) {
        s_uart_reject_reason[id] = APP_UART_REJECT_CAL_NOT_DONE;
        return false;
    }
    if (!foc_calibration_valid()) {
        s_uart_reject_reason[id] = APP_UART_REJECT_CAL_INVALID;
        return false;
    }

    s_source[id] = APP_CMD_SRC_UART;
    s_uart_last_ms[id] = xTaskGetTickCount();
    s_adc_rearm[id] = true;
    return true;
}

// Parameter id: identitas motor yang ingin diperiksa alasan penolakan UART terakhirnya.
// Fungsi app_command_uart_reject_reason: membaca alasan penolakan UART tanpa mengubah state kendali.
app_uart_reject_reason_t app_command_uart_reject_reason(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return APP_UART_REJECT_INVALID_ID;
    return (app_uart_reject_reason_t)s_uart_reject_reason[id];
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_uart_keepalive: menjalankan operasi app command uart keepalive sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_command_uart_keepalive(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return;
    if (s_source[id] == APP_CMD_SRC_UART)
        s_uart_last_ms[id] = xTaskGetTickCount();
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter neutral_stable: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi app_command_adc_claim: menjalankan operasi app command adc claim sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool app_command_adc_claim(motor_id_t id, bool neutral_stable) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return false;
    // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    MotorRuntime *m = motor_get(id);
    if (m == NULL || m->detect.busy || app_is_output_disabled() || m->fault != MOTOR_FAULT_NONE)
        return false;
    if (!mc_interface_try_input_motor(id)) {
        if (s_source[id] == APP_CMD_SRC_ADC)
            motor_stop(m);
        s_source[id] = APP_CMD_SRC_NONE;
        s_adc_rearm[id] = true;
        return false;
    }
    if (s_source[id] == APP_CMD_SRC_UART)
        return false;
    if (s_adc_rearm[id]) {
        if (!neutral_stable)
            return false;
        s_adc_rearm[id] = false;
    }
    s_source[id] = APP_CMD_SRC_ADC;
    return true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_adc_block: menjalankan operasi app command adc block sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void app_command_adc_block(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return;
    if (s_source[id] == APP_CMD_SRC_ADC)
        s_source[id] = APP_CMD_SRC_NONE;
    s_adc_rearm[id] = true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter stop_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi app_command_adc_release: menjalankan operasi app command adc release sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void app_command_adc_release(motor_id_t id, bool stop_motor) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return;
    if (s_source[id] == APP_CMD_SRC_ADC) {
        if (stop_motor)
            motor_stop(motor_get(id));
        s_source[id] = APP_CMD_SRC_NONE;
    }
    s_adc_rearm[id] = true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_force_adc_rearm: menjalankan operasi app command force adc rearm sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
void app_command_force_adc_rearm(motor_id_t id) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return;
    s_adc_rearm[id] = true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter stop_motor: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi app_command_release: menjalankan operasi app command release sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void app_command_release(motor_id_t id, bool stop_motor) {
    if (id != MOTOR_LEFT && id != MOTOR_RIGHT)
        return;
    if (stop_motor)
        motor_stop(motor_get(id));
    s_source[id] = APP_CMD_SRC_NONE;
    s_adc_rearm[id] = true;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_get_source: membaca app command get source tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
app_command_source_t app_command_get_source(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? s_source[MOTOR_RIGHT] : s_source[MOTOR_LEFT];
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi app_command_adc_rearm_required: menjalankan operasi app command adc rearm required sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
bool app_command_adc_rearm_required(motor_id_t id) {
    return (id == MOTOR_RIGHT) ? s_adc_rearm[MOTOR_RIGHT] : s_adc_rearm[MOTOR_LEFT];
}
