#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "datatypes.h"

// Variabel hadc1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern ADC_HandleTypeDef hadc1;
// Variabel hadc2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern ADC_HandleTypeDef hadc2;
// Variabel hadc3: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern ADC_HandleTypeDef hadc3;
// Variabel hdma_adc1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern DMA_HandleTypeDef hdma_adc1;
// Variabel hdma_adc3: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern DMA_HandleTypeDef hdma_adc3;
// Variabel htim1: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern TIM_HandleTypeDef htim1;
// Variabel htim8: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern TIM_HandleTypeDef htim8;
// Variabel htim2: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern TIM_HandleTypeDef htim2;
// Variabel htim4: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
extern TIM_HandleTypeDef htim4;
// Variabel g_adc_dual_dma: nilai atau state ADC pada jalur pengukuran.
extern volatile uint32_t g_adc_dual_dma[6];
// Variabel g_adc3_vbus_dma: tegangan DC bus yang digunakan untuk normalisasi modulasi dan proteksi.
extern volatile uint16_t g_adc3_vbus_dma[2];

/* ADC3 DMA2_CH5 uses a two-halfword circular buffer. CNDTR=1 means slot 0
 * has just been written; CNDTR=2 means the second transfer completed and the
 * circular counter reloaded, so slot 1 is newest. Called only from the hard
 * ADC1/DMA1 current ISR, after ADC3 has had enough time to complete DCLINK. */
// Parameter dma_cndtr: sisa transfer DMA2 Channel 5 untuk memeriksa posisi buffer DCLINK dua sampel.
// Parameter transfer_seen: keluaran true bila flag half-transfer/transfer-complete membuktikan ADC3
// menghasilkan sampel baru sejak ISR FOC sebelumnya.
// Fungsi motor_hw_vbus_raw_from_isr: mengambil sampel DCLINK terbaru dan bukti freshness DMA tanpa menambah
// interrupt ADC3 16 kHz.
static inline uint16_t motor_hw_vbus_raw_from_isr(uint16_t *dma_cndtr, bool *transfer_seen) {
    /* DMA2 Channel 5 memakai empat bit status mulai bit 16: GIF5, TCIF5,
       HTIF5, TEIF5. Kita hanya konsumsi TC/HT di ISR FOC; TE tetap ditangani
       handler error DMA sehingga fault transfer tidak pernah tertutup. */
    // Variabel transfer_mask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t transfer_mask = (1UL << 17) | (1UL << 18);
    // Variabel dma_isr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t dma_isr = DMA2->ISR;
    // Variabel seen: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const bool seen = (dma_isr & transfer_mask) != 0U;
    // Variabel rem: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint16_t rem = (uint16_t)DMA2_Channel5->CNDTR;
    __DMB();
    // Variabel raw: nilai mentah sebelum konversi ke satuan fisik.
    const uint16_t raw = (rem == 1U) ? g_adc3_vbus_dma[0] : g_adc3_vbus_dma[1];
    if (seen) {
        DMA2->IFCR = dma_isr & transfer_mask;
    }
    if (dma_cndtr)
        *dma_cndtr = rem;
    if (transfer_seen)
        *transfer_seen = seen;
    return raw;
}

// Fungsi motor_hw_init: menginisialisasi motor hw init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
void motor_hw_init(void);
// Fungsi motor_hw_start_sampling: memulai motor hw start sampling setelah prasyarat hardware, konfigurasi, dan
// state keselamatan terpenuhi.
void motor_hw_start_sampling(void);
enum {
    HW_SAMPLING_CONTRACT_TIM1_MODE = 1UL << 0,
    HW_SAMPLING_CONTRACT_TIM8_MODE = 1UL << 1,
    HW_SAMPLING_CONTRACT_TIM8_TRGO = 1UL << 2,
    HW_SAMPLING_CONTRACT_TIM8_RCR = 1UL << 3,
    HW_SAMPLING_CONTRACT_ADC1_LEN = 1UL << 4,
    HW_SAMPLING_CONTRACT_ADC2_LEN = 1UL << 5,
    HW_SAMPLING_CONTRACT_DMA1_MODE = 1UL << 6,
    HW_SAMPLING_CONTRACT_TIM1_TRGO = 1UL << 7,
    HW_SAMPLING_CONTRACT_TIM8_SLAVE = 1UL << 8,
    HW_SAMPLING_CONTRACT_ADC_DUALMODE = 1UL << 9,
    HW_SAMPLING_CONTRACT_ADC1_TRIGGER = 1UL << 10,
    HW_SAMPLING_CONTRACT_ADC_CHANNELS = 1UL << 11,
    HW_SAMPLING_CONTRACT_DMA1_TRANSFER = 1UL << 12,
    HW_SAMPLING_CONTRACT_ADC3_MODE = 1UL << 13,
    HW_SAMPLING_CONTRACT_DMA2_MODE = 1UL << 14,
    HW_SAMPLING_CONTRACT_DMA2_TRANSFER = 1UL << 15
};
// Fungsi motor_hw_sampling_contract_flags: menjalankan operasi motor hw sampling contract flags sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
uint32_t motor_hw_sampling_contract_flags(void);
// Fungsi motor_hw_sampling_contract_valid: menjalankan operasi motor hw sampling contract valid sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
bool motor_hw_sampling_contract_valid(void);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter enabled: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi motor_hw_set_pwm_enabled: mengatur motor hw set pwm enabled setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_hw_set_pwm_enabled(MotorRuntime *m, bool enabled);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_service_pwm_enable_from_isr: menangani motor hw service pwm enable from isr pada konteks
// interrupt dengan pekerjaan minimum agar timing FOC tetap deterministik.
void motor_hw_service_pwm_enable_from_isr(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter du: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dv: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter dw: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_set_pwm_duty: mengatur motor hw set pwm duty setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_hw_set_pwm_duty(MotorRuntime *m, float du, float dv, float dw);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter du_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter dv_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Parameter dw_q15: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi motor_hw_set_pwm_q15: mengatur motor hw set pwm q15 setelah nilai masukan divalidasi dan dibatasi
// sesuai aturan keselamatan modul.
void motor_hw_set_pwm_q15(MotorRuntime *m, uint16_t du_q15, uint16_t dv_q15, uint16_t dw_q15);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_restore_foc_outputs: menjalankan bagian motor hw restore foc outputs pada algoritma FOC
// dengan skala, konvensi tanda, dan batas numerik yang konsisten.
void motor_hw_restore_foc_outputs(MotorRuntime *m);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter enable: penanda untuk mengaktifkan atau menonaktifkan fitur.
// Fungsi motor_hw_set_low_side_brake: mengatur motor hw set low side brake setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_hw_set_low_side_brake(MotorRuntime *m, bool enable);
// Parameter tim: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_break_irq_handler: menangani motor hw break irq handler pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void motor_hw_break_irq_handler(TIM_TypeDef *tim);
// Fungsi motor_hw_pvd_irq_handler: menangani motor hw pvd irq handler pada konteks interrupt dengan pekerjaan
// minimum agar timing FOC tetap deterministik.
void motor_hw_pvd_irq_handler(void);
// Fungsi motor_hw_powerstage_fault_flags: menangani motor hw powerstage fault flags dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
uint32_t motor_hw_powerstage_fault_flags(void);
// Fungsi motor_hw_powerstage_fault_latched: menangani motor hw powerstage fault latched dengan memprioritaskan
// pemadaman keluaran daya, pencatatan penyebab, dan pemulihan yang aman.
bool motor_hw_powerstage_fault_latched(void);
// Fungsi motor_hw_pvd_low: menjalankan operasi motor hw pvd low sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
bool motor_hw_pvd_low(void);
// Fungsi motor_hw_clear_recoverable_powerstage_faults: mereset motor hw clear recoverable powerstage faults ke
// kondisi awal yang aman tanpa meninggalkan state lama yang tidak konsisten.
bool motor_hw_clear_recoverable_powerstage_faults(void);
// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Fungsi motor_hw_read_hall_raw: menjalankan operasi motor hw read hall raw sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
uint8_t motor_hw_read_hall_raw(motor_id_t id);
// Fungsi motor_hw_encoder_cnt: menjalankan operasi motor hw encoder cnt sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
uint16_t motor_hw_encoder_cnt(void);
// Parameter temp_c: temperatur atau nilai sementara sesuai konteks modul.
// Fungsi motor_hw_board_temperature_c: menjalankan operasi motor hw board temperature c sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
bool motor_hw_board_temperature_c(float *temp_c);
// Fungsi motor_hw_capture_app_adc_from_isr: menangani motor hw capture app adc from isr pada konteks interrupt
// dengan pekerjaan minimum agar timing FOC tetap deterministik.
void motor_hw_capture_app_adc_from_isr(void);
// Parameter ticks: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_set_adc_phase_offset_ticks: mengatur motor hw set adc phase offset ticks setelah nilai
// masukan divalidasi dan dibatasi sesuai aturan keselamatan modul.
void motor_hw_set_adc_phase_offset_ticks(uint16_t ticks);
// Fungsi motor_hw_get_adc_phase_offset_ticks: membaca motor hw get adc phase offset ticks tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
uint16_t motor_hw_get_adc_phase_offset_ticks(void);
// Parameter pa2_raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Parameter pa3_raw: nilai mentah sebelum koreksi offset atau konversi satuan.
// Fungsi motor_hw_get_app_adc_raw: membaca motor hw get app adc raw tanpa mengubah state kendali utama dan
// mengembalikan data yang konsisten.
bool motor_hw_get_app_adc_raw(uint16_t *pa2_raw, uint16_t *pa3_raw);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter count: pencacah kejadian, elemen, atau sampel.
// Fungsi motor_hw_encoder_set_count: mengatur motor hw encoder set count setelah nilai masukan divalidasi dan
// dibatasi sesuai aturan keselamatan modul.
void motor_hw_encoder_set_count(MotorRuntime *m, uint16_t count);
// Parameter m: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter mode: mode operasi yang menentukan jalur algoritma aktif.
// Fungsi motor_hw_configure_sensor: menjalankan operasi motor hw configure sensor sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void motor_hw_configure_sensor(MotorRuntime *m, uint8_t mode);
// Parameter on: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi motor_hw_led: menjalankan operasi motor hw led sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
void motor_hw_led(bool on);
// Fungsi motor_hw_emergency_all_off: menjalankan operasi motor hw emergency all off sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void motor_hw_emergency_all_off(void);

/* Board status/power-latch service. Kept in hwconf because these functions are
 * tied to the hoverboard PCB, not to the generic motor-control interface. */
// Fungsi hw_status_early_init: menginisialisasi hw status early init sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
void hw_status_early_init(void);
// Fungsi hw_status_timer_init: menginisialisasi hw status timer init sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
void hw_status_timer_init(void);
// Fungsi hw_status_init: menginisialisasi hw status init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
bool hw_status_init(void);
// Parameter hz: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi hw_status_tone_start: memulai hw status tone start setelah prasyarat hardware, konfigurasi, dan state
// keselamatan terpenuhi.
void hw_status_tone_start(uint16_t hz);
// Parameter hz: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duration_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi hw_status_tone_start_for: memulai hw status tone start for setelah prasyarat hardware, konfigurasi,
// dan state keselamatan terpenuhi.
void hw_status_tone_start_for(uint16_t hz, uint32_t duration_ms);
// Fungsi hw_status_tone_stop: menghentikan hw status tone stop dengan menonaktifkan output atau state terkait
// secara aman.
void hw_status_tone_stop(void);
/* Replays the same non-blocking ~3.21 s TIM3 power-on melody. The command
 * layer only exposes this while both bridges are stopped. */
// Fungsi hw_status_startup_melody_replay: menjalankan operasi hw status startup melody replay sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void hw_status_startup_melody_replay(void);
// Fungsi hw_status_tone_is_running: menjalankan operasi hw status tone is running sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool hw_status_tone_is_running(void);
// Fungsi hw_status_tone_level: menjalankan operasi hw status tone level sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool hw_status_tone_level(void);
// Parameter on: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi hw_status_power_hold: menjalankan operasi hw status power hold sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void hw_status_power_hold(bool on);
// Fungsi hw_status_power_is_held: menjalankan operasi hw status power is held sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool hw_status_power_is_held(void);
// Parameter now_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi hw_status_service_10ms: menjalankan operasi hw status service 10ms sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void hw_status_service_10ms(uint32_t now_ms);
// Fungsi hw_status_tim3_irq_handler: menangani hw status tim3 irq handler pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void hw_status_tim3_irq_handler(void);
// Parameter noreturn: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi hw_status_early_fatal_loop: menjalankan operasi hw status early fatal loop sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void hw_status_early_fatal_loop(void) __attribute__((noreturn));
// Variabel g_vesc_buzzer_running: state global firmware yang dibagikan antarbagian modul.
extern volatile uint8_t g_vesc_buzzer_running;
// Variabel g_vesc_buzzer_hz: state global firmware yang dibagikan antarbagian modul.
extern volatile uint16_t g_vesc_buzzer_hz;
// Variabel g_vesc_buzzer_remaining: state global firmware yang dibagikan antarbagian modul.
extern volatile uint32_t g_vesc_buzzer_remaining;
// Variabel g_vesc_startup_melody_active: penanda bahwa state atau fitur sedang aktif.
extern volatile uint8_t g_vesc_startup_melody_active;
// Variabel g_vesc_startup_melody_index: indeks elemen yang sedang diproses.
extern volatile uint8_t g_vesc_startup_melody_index;
