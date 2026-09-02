#include "timeout.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "applications/appconf_default.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include <math.h>

#ifndef VESC_IWDG_ENABLE
#define VESC_IWDG_ENABLE 0
#endif


/* Command timeout state. */
// Variabel s_timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
static volatile uint32_t s_timeout_ms = MOTOR_COMMAND_TIMEOUT_MS;
// Variabel s_last_update_ms: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_last_update_ms = 0U;
// Variabel s_brake_current_a: nilai arus untuk pengukuran, kendali, atau proteksi.
static volatile float s_brake_current_a = 0.0f;
// Variabel s_has_timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
static volatile bool s_has_timeout = false;

/* Reset diagnostics and independent watchdog state. */
// Variabel s_reset_flags: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_reset_flags = 0U;
// Variabel s_hb: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_hb[TIMEOUT_HEARTBEAT_COUNT] = {
    0U
}
;
// Variabel s_hb_last: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_hb_last[TIMEOUT_HEARTBEAT_COUNT] = {
    0U
}
;
// Variabel s_hb_miss: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_hb_miss[TIMEOUT_HEARTBEAT_COUNT] = {
    0U
}
;
// Variabel s_unhealthy_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_unhealthy_mask = 0U;
// Variabel s_required_mask: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_required_mask =
    (1UL << TIMEOUT_HEARTBEAT_MOTOR_SERVICE) |
    (1UL << TIMEOUT_HEARTBEAT_COMM) |
    (1UL << TIMEOUT_HEARTBEAT_FAULT);
// Variabel s_watchdog_started: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_watchdog_started = false;
// Variabel s_watchdog_healthy: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_watchdog_healthy = true;
// Variabel s_health_window_start_ms: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_health_window_start_ms = 0U;
// Variabel s_watchdog_grace_until_ms: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_watchdog_grace_until_ms = 0U;

#define WATCHDOG_HEALTH_WINDOW_MS 100U
#define WATCHDOG_START_GRACE_MS   250U
/* STM32F1 LSI is intentionally imprecise. Prescaler /64 and reload 300 gives
 * roughly a 0.4..1.2 s reset window over the wide documented LSI tolerance.
 * This is much slower than upstream VESC's ~10 ms watchdog, but preserves
 * margin for STM32F103 flash-page operations while still recovering a wedged
 * vehicle controller promptly. */
#define WATCHDOG_IWDG_PRESCALER_BITS 4U
#define WATCHDOG_IWDG_RELOAD         300U

// Fungsi iwdg_feed: menjalankan operasi iwdg feed sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static inline void iwdg_feed(void) {
    IWDG->KR = 0xAAAAU;
}

// Fungsi timeout_capture_reset_reason: mereset timeout capture reset reason ke kondisi awal yang aman tanpa
// meninggalkan state lama yang tidak konsisten.
void timeout_capture_reset_reason(void) {
    // Variabel csr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t csr = RCC->CSR;
    s_reset_flags = csr & (RCC_CSR_LPWRRSTF | RCC_CSR_WWDGRSTF |
                           RCC_CSR_IWDGRSTF | RCC_CSR_SFTRSTF |
                           RCC_CSR_PORRSTF | RCC_CSR_PINRSTF);
    RCC->CSR |= RCC_CSR_RMVF;
}

// Fungsi timeout_get_reset_flags: membaca timeout get reset flags tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
uint32_t timeout_get_reset_flags(void) {
    return s_reset_flags;
}

// Fungsi timeout_had_iwdg_reset: mereset timeout had iwdg reset ke kondisi awal yang aman tanpa meninggalkan
// state lama yang tidak konsisten.
bool timeout_had_iwdg_reset(void) {
    return (s_reset_flags & RCC_CSR_IWDGRSTF) != 0U;
}

// Fungsi timeout_init: menginisialisasi timeout init sehingga resource, konfigurasi awal, dan state modul siap
// digunakan dengan aman.
bool timeout_init(void) {
    s_last_update_ms = xTaskGetTickCount();
    s_has_timeout = false;
    s_watchdog_healthy = true;
    s_required_mask = (1UL << TIMEOUT_HEARTBEAT_MOTOR_SERVICE) |
                      (1UL << TIMEOUT_HEARTBEAT_COMM) |
                      (1UL << TIMEOUT_HEARTBEAT_FAULT);
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t i = 0U; i < TIMEOUT_HEARTBEAT_COUNT; i++) {
        s_hb[i] = 0U;
        s_hb_last[i] = 0U;
        s_hb_miss[i] = 0U;
    }
    s_unhealthy_mask = 0U;
    s_health_window_start_ms = s_last_update_ms;
    s_watchdog_grace_until_ms = s_last_update_ms + WATCHDOG_START_GRACE_MS;
    return true;
}

// Parameter now: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi timeout_update_10ms: memperbarui timeout update 10ms menggunakan state terbaru dengan urutan yang
// konsisten dan aman.
void timeout_update_10ms(uint32_t now) {
    // Variabel timeout: batas atau state waktu untuk pengamanan komunikasi dan kendali.
    const uint32_t timeout = s_timeout_ms;
    // Variabel expired: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool expired = timeout != 0U && (uint32_t)(now - s_last_update_ms) > timeout;

    /* During offset calibration the bridge is deliberately driven with a 50%
     * zero-vector. A command timeout must not tear down that PWM (VESC clears
     * the timeout for the duration of dc_cal), otherwise the driven offset
     * measurement is aborted after the first samples and the MOE handshake
     * fails. Real hardware faults are still handled synchronously elsewhere. */
    if (expired && foc_calibration_in_progress()) {
        g_motor_left.timeout_active = false;
        g_motor_right.timeout_active = false;
        return;
    }

    if (expired) {
        if (!s_has_timeout) {
            s_has_timeout = true;
            g_motor_left.timeout_active = true;
            g_motor_right.timeout_active = true;
        }

        // Variabel brake: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const float brake = s_brake_current_a;
        if (fabsf(brake) > 0.001f) {
            motor_set_brake_current(&g_motor_left, fabsf(brake));
            motor_set_brake_current(&g_motor_right, fabsf(brake));
            g_motor_left.timeout_active = true;
            g_motor_right.timeout_active = true;
        }
        else {
            motor_stop(&g_motor_left);
            motor_stop(&g_motor_right);
            g_motor_left.timeout_active = true;
            g_motor_right.timeout_active = true;
        }
    }
    else {
        s_has_timeout = false;
        g_motor_left.timeout_active = false;
        g_motor_right.timeout_active = false;
    }
}

// Fungsi timeout_reset: mereset timeout reset ke kondisi awal yang aman tanpa meninggalkan state lama yang
// tidak konsisten.
void timeout_reset(void) {
    s_last_update_ms = xTaskGetTickCount();
    s_has_timeout = false;
    g_motor_left.timeout_active = false;
    g_motor_right.timeout_active = false;
}

// Parameter timeout_ms: batas atau state waktu untuk pengamanan komunikasi dan kendali.
// Parameter brake_current_a: nilai arus untuk pengukuran, kendali, atau proteksi motor.
// Fungsi timeout_configure: menjalankan operasi timeout configure sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void timeout_configure(uint32_t timeout_ms, float brake_current_a) {
    s_timeout_ms = timeout_ms;
    s_brake_current_a = brake_current_a;
}

// Fungsi timeout_has_timeout: menjalankan operasi timeout has timeout sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool timeout_has_timeout(void) {
    return s_has_timeout;
}
// Fungsi timeout_get_timeout_ms: membaca timeout get timeout ms tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
uint32_t timeout_get_timeout_ms(void) {
    return s_timeout_ms;
}
// Fungsi timeout_get_brake_current: membaca timeout get brake current tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
float timeout_get_brake_current(void) {
    return s_brake_current_a;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_heartbeat: menjalankan operasi timeout heartbeat sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void timeout_heartbeat(timeout_heartbeat_id_t id) {
    if ((uint32_t)id < TIMEOUT_HEARTBEAT_COUNT)
        s_hb[(uint32_t)id]++;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_heartbeat_from_isr: menangani timeout heartbeat from isr pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void timeout_heartbeat_from_isr(timeout_heartbeat_id_t id) {
    if ((uint32_t)id < TIMEOUT_HEARTBEAT_COUNT)
        s_hb[(uint32_t)id]++;
}

// Fungsi timeout_watchdog_start: memulai timeout watchdog start setelah prasyarat hardware, konfigurasi, dan
// state keselamatan terpenuhi.
void timeout_watchdog_start(void) {
    if (s_watchdog_started)
        return;
#if !VESC_IWDG_ENABLE
    /* Commissioning-safe default: do not start the irreversible hardware IWDG.
     * Motor command timeout and all power-stage fault paths remain active.
     * Re-enable only after UART/FOC commissioning is stable. */
    s_watchdog_healthy = true;
    return;
#else

    /* STM32 IWDG sequence: start it, unlock PR/RLR, program them, wait for
     * synchronization, then reload. Once started it cannot be stopped until
     * reset. The default startup reload is long enough for these writes. */
    IWDG->KR = 0xCCCCU; /* start and enable LSI */
    IWDG->KR = 0x5555U; /* enable PR/RLR write access */
    IWDG->PR = WATCHDOG_IWDG_PRESCALER_BITS;
    IWDG->RLR = WATCHDOG_IWDG_RELOAD;
    // Variabel guard: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (uint32_t guard = 0U; guard < 100000U; guard++) {
        if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0U)
            break;
    }
    iwdg_feed();

    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t now = xTaskGetTickCount();
    s_health_window_start_ms = now;
    s_watchdog_grace_until_ms = now + WATCHDOG_START_GRACE_MS;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t i = 0U; i < TIMEOUT_HEARTBEAT_COUNT; i++)
        s_hb_last[i] = s_hb[i];
    s_watchdog_healthy = true;
    s_watchdog_started = true;
#endif
}

// Parameter required: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi timeout_watchdog_require_foc: menjalankan bagian timeout watchdog require foc pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void timeout_watchdog_require_foc(bool required) {
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (required) {
        s_required_mask |= 1UL << TIMEOUT_HEARTBEAT_FOC;
        s_hb_last[TIMEOUT_HEARTBEAT_FOC] = s_hb[TIMEOUT_HEARTBEAT_FOC];
    }
    else {
        s_required_mask &= ~(1UL << TIMEOUT_HEARTBEAT_FOC);
    }
    if (!primask)
        __enable_irq();
}

// Parameter now: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi timeout_watchdog_update_10ms: memperbarui timeout watchdog update 10ms menggunakan state terbaru
// dengan urutan yang konsisten dan aman.
void timeout_watchdog_update_10ms(uint32_t now) {
    if (!s_watchdog_started)
        return;

    /* Startup grace feeds unconditionally while tasks and sampling settle. */
    if ((int32_t)(now - s_watchdog_grace_until_ms) < 0) {
        iwdg_feed();
        return;
    }

    /* Check liveness at 100 ms cadence. Between checks feed only while the
     * previous health decision remains good. A stalled motor-service thread
     * cannot execute this function at all, so it naturally stops feeding. */
    if ((uint32_t)(now - s_health_window_start_ms) >= WATCHDOG_HEALTH_WINDOW_MS) {
        // Variabel required: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t required = s_required_mask;
        // Variabel unhealthy: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t unhealthy = 0U;
        // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (uint32_t i = 0U; i < TIMEOUT_HEARTBEAT_COUNT; i++) {
            // Variabel cur: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t cur = s_hb[i];
            if ((required & (1UL << i)) != 0U && cur == s_hb_last[i]) {
                unhealthy |= 1UL << i;
                s_hb_miss[i]++;
            }
            s_hb_last[i] = cur;
        }
        s_unhealthy_mask = unhealthy;
        s_watchdog_healthy = unhealthy == 0U;
        s_health_window_start_ms = now;
    }

    if (s_watchdog_healthy)
        iwdg_feed();
}

// Fungsi timeout_watchdog_started: menjalankan operasi timeout watchdog started sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool timeout_watchdog_started(void) {
    return s_watchdog_started;
}
// Fungsi timeout_watchdog_healthy: menjalankan operasi timeout watchdog healthy sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool timeout_watchdog_healthy(void) {
    return s_watchdog_healthy;
}
// Fungsi timeout_watchdog_required_mask: menjalankan operasi timeout watchdog required mask sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_watchdog_required_mask(void) {
    return s_required_mask;
}
// Fungsi timeout_watchdog_unhealthy_mask: menjalankan operasi timeout watchdog unhealthy mask sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_watchdog_unhealthy_mask(void) {
    return s_unhealthy_mask;
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_watchdog_miss_count: menjalankan operasi timeout watchdog miss count sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_watchdog_miss_count(timeout_heartbeat_id_t id) {
    return ((uint32_t)id < TIMEOUT_HEARTBEAT_COUNT) ? s_hb_miss[(uint32_t)id] : 0U;
}
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi timeout_heartbeat_count: menjalankan operasi timeout heartbeat count sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
uint32_t timeout_heartbeat_count(timeout_heartbeat_id_t id) {
    return ((uint32_t)id < TIMEOUT_HEARTBEAT_COUNT) ? s_hb[(uint32_t)id] : 0U;
}
