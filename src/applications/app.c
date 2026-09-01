#include "applications/app.h"
#include "confgenerator.h"
#include "applications/appconf_default.h"
#include "motor/mc_interface.h"
#include "applications/app_adc.h"
#include "applications/app_command.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

// Variabel s_app_conf: data konfigurasi yang mengatur perilaku firmware.
static app_configuration s_app_conf;
// Variabel s_loaded: state internal modul yang dipertahankan antar pemanggilan fungsi.
static bool s_loaded = false;
// Variabel s_disabled_until: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_disabled_until = 0U;
// Variabel s_disabled_indefinite: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_disabled_indefinite = false;
// Variabel s_set_wire: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_set_wire[VESC6_APPCONF_WIRE_SIZE];
// Variabel s_crc_wire: nilai CRC untuk memeriksa integritas data.
static uint8_t s_crc_wire[VESC6_APPCONF_WIRE_SIZE];

// Parameter c: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi app_conf_supported: menjalankan operasi app conf supported sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool app_conf_supported(const app_configuration *c) {
    if (!c)
        return false;
    if (c->controller_id != VESC_CONTROLLER_ID_LEFT)
        return false;
    if (c->timeout_msec > 600000U)
        return false;
    if (c->app_to_use != APP_NONE && c->app_to_use != APP_ADC &&
       c->app_to_use != APP_UART && c->app_to_use != APP_ADC_UART)
       return false;
    if (c->app_uart_baudrate != VESC_UART_BAUD)
        return false;
    /* USART3 is the permanent management/VESC Tool transport on this board. */
    if (!c->permanent_uart_enabled)
        return false;
    return true;
}

// Fungsi app_refresh: menjalankan operasi app refresh sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static void app_refresh(void) {
    // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t *w = vesc_config_app_wire(false);
    if (w && confgenerator_deserialize_appconf(w, &s_app_conf))
        s_loaded = true;
}

// Fungsi app_get_configuration: membaca app get configuration tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
const app_configuration *app_get_configuration(void) {
    if (!s_loaded)
        app_refresh();
    return &s_app_conf;
}

// Fungsi app_notify_configuration_changed: menjalankan operasi app notify configuration changed sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void app_notify_configuration_changed(void) {
    s_loaded = false;
    app_command_configuration_changed();
}

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi app_set_configuration: mengatur app set configuration setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void app_set_configuration(app_configuration *conf) {
    if (!app_conf_supported(conf))
        return;
    if (confgenerator_serialize_appconf(s_set_wire, conf) != (int32_t)VESC6_APPCONF_WIRE_SIZE)
        return;
    if (vesc_config_set_app_wire(s_set_wire, VESC6_APPCONF_WIRE_SIZE, false)) {
        s_app_conf = *conf;
        s_loaded = true;
    }
}

// Parameter time_ms: nilai waktu untuk penjadwalan, timeout, atau pengukuran durasi.
// Fungsi app_disable_output: menjalankan operasi app disable output sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
void app_disable_output(int time_ms) {
    if (time_ms < 0) {
        s_disabled_indefinite = true;
        s_disabled_until = 0U;
        return;
    }
    s_disabled_indefinite = false;
    if (time_ms == 0) {
        s_disabled_until = 0U;
        return;
    }
    s_disabled_until = xTaskGetTickCount()+(uint32_t)time_ms;
}

// Fungsi app_is_output_disabled: menjalankan operasi app is output disabled sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool app_is_output_disabled(void) {
    if (s_disabled_indefinite)
        return true;
    // Variabel until: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t until = s_disabled_until;
    if (until == 0U)
        return false;
    // Variabel now: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t now = xTaskGetTickCount();
    if ((int32_t)(until-now) > 0)
        return true;
    s_disabled_until = 0U;
    return false;
}

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi app_calc_crc: menangani kalibrasi app calc crc agar offset atau parameter hasil ukur valid sebelum
// dipakai kendali.
unsigned short app_calc_crc(app_configuration *conf) {
    if (!conf)
        return 0U;
    if (confgenerator_serialize_appconf(s_crc_wire, conf) != (int32_t)VESC6_APPCONF_WIRE_SIZE)
        return 0U;
    // Variabel crc: nilai CRC untuk memeriksa integritas data.
    uint16_t crc = 0U;
    // Variabel n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t n = 0; n < VESC6_APPCONF_WIRE_SIZE; n++) {
        crc ^= (uint16_t)s_crc_wire[n]<<8;
        // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (unsigned b = 0; b < 8; b++)
            crc = (crc&0x8000U) ? (uint16_t)((crc<<1)^0x1021U) : (uint16_t)(crc<<1);
    }
    return crc;
}
