#include "hwconf/hw.h"
#include "hwconf/hw_hoverboard.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "comm/commands.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

// Variabel s_tone_level: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_tone_level = false;
// Variabel g_vesc_buzzer_hz: state global firmware yang dibagikan antarbagian modul.
volatile uint16_t g_vesc_buzzer_hz = 0U;
// Variabel g_vesc_buzzer_remaining: state global firmware yang dibagikan antarbagian modul.
volatile uint32_t g_vesc_buzzer_remaining = 0U;
// Variabel g_vesc_startup_melody_active: penanda bahwa state atau fitur sedang aktif.
volatile uint8_t g_vesc_startup_melody_active = 0U;
// Variabel g_vesc_startup_melody_index: indeks elemen yang sedang diproses.
volatile uint8_t g_vesc_startup_melody_index = 0U;
// Variabel s_tone_running: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_tone_running = false;
/* VESC-correct finite-beep support. A tone may be infinite (stopped only by an
 * explicit hw_status_tone_stop) or self-terminate after a fixed number of
 * half-cycles. The self-terminating path is what the vesc_stm32f103rct6 status-thread refactor
 * accidentally dropped when it removed the old s_tone_toggle_remaining guard:
 * without it a tone that the status task fails to stop (delayed/blocked/stopped
 * thread) keeps TIM3 toggling the pin forever, i.e. a stuck-ON buzzer. */
// Variabel s_tone_infinite: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_tone_infinite = false;
// Variabel s_tone_toggle_remaining: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile uint32_t s_tone_toggle_remaining = 0U;
// Variabel s_melody_sequencer: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_melody_sequencer = false;
// Variabel s_melody_gap: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_melody_gap = false;
// Variabel s_melody_next_index: indeks elemen yang sedang diproses.
static volatile uint8_t s_melody_next_index = 0U;
// Variabel g_vesc_buzzer_running: state global firmware yang dibagikan antarbagian modul.
volatile uint8_t g_vesc_buzzer_running = 0U;
// Variabel s_power_held: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_power_held = true;
// Variabel s_status_started: status runtime untuk diagnostik atau keputusan kendali.
static bool s_status_started = false;

#define STATUS_CUE_CAL_START (1U << 0)
#define STATUS_CUE_CAL_DONE  (1U << 1)
#define STATUS_CUE_EEPROM    0x80U

// Variabel s_cue_pending: bit cue audio kalibrasi non-fault yang menunggu kondisi aman untuk dimainkan.
static volatile uint8_t s_cue_pending = 0U;
// Variabel s_eeprom_cue_pending: jumlah transaksi EEPROM/flash sukses yang belum memperoleh cue 5-beep.
// Counter dipakai, bukan satu bit, agar dua save berurutan tidak terkoalesensi menjadi satu notifikasi.
static volatile uint8_t s_eeprom_cue_pending = 0U;
// Variabel s_cue_active: jenis cue audio non-fault yang sedang dimainkan.
static uint8_t s_cue_active = 0U;
// Variabel s_cue_step: tahap state machine cue, termasuk lima tone dan empat jeda EEPROM.
static uint8_t s_cue_step = 0U;
// Variabel s_cue_deadline: deadline millisecond state machine cue tanpa blocking delay.
static uint32_t s_cue_deadline = 0U;

typedef struct {
    // Variabel hz: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t hz;
    // Variabel duration_ms: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t duration_ms;
} startup_note_t;

/* ~3 s VESC-inspired power-up chime. This is intentionally a buzzer melody,
 * not a blocking motor-beep sequence: the real VESC startup sound is normally
 * produced through the motor phases. The ascending E-major arpeggio gives the
 * same short, technical VESC-like character while keeping both bridges OFF.
 * Every tone/gap is advanced autonomously by TIM3, so it starts before the
 * FreeRTOS scheduler and never blocks UART or FOC startup. */
// Variabel s_startup_notes: state internal modul yang dipertahankan antar pemanggilan fungsi.
static const startup_note_t s_startup_notes[] = {
    {659U, 180U}, /* E5  */
    {
        0U, 55U
    }
    ,
    {831U, 180U}, /* G#5 */
    {
        0U, 55U
    }
    ,
    {988U, 210U}, /* B5  */
    {
        0U, 70U
    }
    ,
    {1319U, 260U}, /* E6  */
    {
        0U, 120U
    }
    ,
    {
        988U, 150U
    }
    ,
    {1175U, 150U}, /* D6  */
    {
        1319U, 180U
    }
    ,
    {
        0U, 80U
    }
    ,
    {1661U, 220U}, /* G#6 */
    {
        0U, 70U
    }
    ,
    {1976U, 260U}, /* B6  */
    {
        0U, 100U
    }
    ,
    {2637U, 520U}, /* E7 resolve */
    {
        0U, 350U
    }
};

/* TIM3 owns the complete startup melody so it starts before FreeRTOS and is
 * independent of timer_thread latency. Tone segments count half-cycles; silent
 * gaps run TIM3 at 1 kHz and count milliseconds. All helpers below are called
 * with interrupts already masked or from TIM3 IRQ itself. */
// Fungsi status_timer_stop_locked: menghentikan status timer stop locked dengan menonaktifkan output atau state
// terkait secara aman.
static void status_timer_stop_locked(void) {
    TIM3->DIER = 0U;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR = 0U;
    s_tone_running = false;
    s_tone_infinite = false;
    s_tone_toggle_remaining = 0U;
    s_melody_gap = false;
    g_vesc_buzzer_running = 0U;
    g_vesc_buzzer_hz = 0U;
    g_vesc_buzzer_remaining = 0U;
    s_tone_level = false;
    BUZZER_PORT->BRR = BUZZER_PIN;
}

// Parameter hz: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duration_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Parameter gap: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi status_program_segment_locked: menjalankan operasi status program segment locked sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static void status_program_segment_locked(uint16_t hz, uint32_t duration_ms, bool gap) {
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->DIER = 0U;
    TIM3->SR = 0U;
    TIM3->CNT = 0U;
    s_tone_level = false;
    BUZZER_PORT->BRR = BUZZER_PIN;
    s_tone_infinite = false;
    s_melody_gap = gap;

    if (gap) {
        TIM3->ARR = 999U; /* 1 MHz / 1000 = 1 ms update */
        s_tone_toggle_remaining = duration_ms ? duration_ms : 1U;
        g_vesc_buzzer_hz = 0U;
    }
    else {
        if (hz < 100U)
            hz = 100U;
        if (hz > 5000U)
            hz = 5000U;
        // Variabel arr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t arr = 500000UL / (uint32_t)hz;
        if (arr == 0U)
            arr = 1U;
        TIM3->ARR = arr - 1U;
        // Variabel toggles: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint64_t toggles = ((uint64_t)hz * 2ULL * (uint64_t)duration_ms + 999ULL) / 1000ULL;
        if (toggles == 0ULL)
            toggles = 1ULL;
        if (toggles > 0xFFFFFFFEULL)
            toggles = 0xFFFFFFFEULL;
        s_tone_toggle_remaining = (uint32_t)toggles;
        g_vesc_buzzer_hz = hz;
    }
    g_vesc_buzzer_remaining = s_tone_toggle_remaining;
    s_tone_running = true;
    g_vesc_buzzer_running = 1U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
}

// Fungsi status_melody_load_next_locked: memuat status melody load next locked dan memvalidasi integritas data
// sebelum digunakan oleh runtime.
static void status_melody_load_next_locked(void) {
    if (s_melody_next_index >= (uint8_t)(sizeof(s_startup_notes) / sizeof(s_startup_notes[0]))) {
        s_melody_sequencer = false;
        g_vesc_startup_melody_active = 0U;
        status_timer_stop_locked();
        return;
    }
    // Variabel note: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const startup_note_t note = s_startup_notes[s_melody_next_index++];
    g_vesc_startup_melody_index = s_melody_next_index;
    status_program_segment_locked(note.hz, note.duration_ms, note.hz == 0U);
}

// Fungsi hw_status_startup_melody_begin: menjalankan operasi hw status startup melody begin sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
static void hw_status_startup_melody_begin(void) {
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_melody_sequencer = true;
    s_melody_next_index = 0U;
    g_vesc_startup_melody_active = 1U;
    g_vesc_startup_melody_index = 0U;
    status_melody_load_next_locked();
    if (!primask)
        __enable_irq();
}

// Fungsi hw_status_startup_melody_replay: menjalankan operasi hw status startup melody replay sesuai tanggung
// jawab modul dengan input tervalidasi dan state yang konsisten.
void hw_status_startup_melody_replay(void) {
    hw_status_startup_melody_begin();
}


// Parameter mask: bit cue yang akan diantrikan secara atomik dari task konfigurasi atau kalibrasi.
// Fungsi status_queue_cue: menambahkan cue non-fault tanpa blocking dan tanpa mengubah state motor.
static void status_queue_cue(uint8_t mask) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_cue_pending |= mask;
    if (!primask) {
        __enable_irq();
    }
}

// Fungsi hw_status_notify_calibration_start: mengantrikan beep pendek saat kalibrasi mulai; power-on melody
// tetap dibiarkan selesai dan cue akan dimainkan saat jalur audio aman.
void hw_status_notify_calibration_start(void) {
    status_queue_cue(STATUS_CUE_CAL_START);
}

// Parameter ok: true bila kalibrasi selesai valid; false membatalkan cue normal karena fault yang menjelaskan
// kegagalan memiliki prioritas audio lebih tinggi.
// Fungsi hw_status_notify_calibration_done: mengantrikan beep keberhasilan kalibrasi secara non-blocking.
void hw_status_notify_calibration_done(bool ok) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_cue_pending &= (uint8_t)~STATUS_CUE_CAL_START;
    if (ok) {
        s_cue_pending |= STATUS_CUE_CAL_DONE;
    }
    else {
        s_cue_pending &= (uint8_t)~STATUS_CUE_CAL_DONE;
    }
    if (!primask) {
        __enable_irq();
    }
}

// Fungsi status_queue_eeprom_cue: menaikkan counter save secara atomik agar event tidak hilang bila
// transaksi konfigurasi dan status-thread terjadi bersamaan.
static void status_queue_eeprom_cue(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_eeprom_cue_pending < 255U) {
        s_eeprom_cue_pending++;
    }
    if (!primask) {
        __enable_irq();
    }
}

// Fungsi hw_status_notify_eeprom_saved: mengantrikan tepat lima beep pendek untuk SETIAP transaksi
// EEPROM/flash yang sukses. Cue ditunda bila motor sedang running atau detect sedang aktif agar indikator
// running tetap LED-only dan buzzer tidak mengganggu motor.
void hw_status_notify_eeprom_saved(void) {
    status_queue_eeprom_cue();
}

// Parameter now: waktu millisecond service status.
// Parameter calibrating: true saat offset-current calibration masih berjalan.
// Parameter detecting: true saat worker Hall/encoder/detect sedang menjalankan motor.
// Parameter running: true saat salah satu bridge PWM aktif di luar kalibrasi.
// Fungsi status_service_cue_10ms: menjalankan beep kalibrasi/save tanpa blocking dan tanpa bunyi saat running.
static void status_service_cue_10ms(uint32_t now, bool calibrating, bool detecting, bool running) {
    /* Running/detect harus senyap. Jika motor mulai ketika cue non-fault masih
       berbunyi, hentikan segera lalu antrekan ulang cue yang belum selesai agar
       dapat dimainkan setelah bridge kembali idle. */
    if (detecting || running) {
        if (s_cue_active != 0U) {
            if (s_cue_active == STATUS_CUE_CAL_START || s_cue_active == STATUS_CUE_CAL_DONE) {
                s_cue_pending |= s_cue_active;
            }
            else {
                /* EEPROM sequence yang terpotong diulang penuh setelah motor idle. */
                status_queue_eeprom_cue();
            }
            s_cue_active = 0U;
            s_cue_step = 0U;
            hw_status_tone_stop();
        }
        /* Power-on melody berjalan pada TIM3 independen. Jangan memotongnya
           hanya karena host memberi command lebih cepat dari 3.21 s; motor dan
           buzzer tidak berbagi resource power-stage. */
        return;
    }
    if (g_vesc_startup_melody_active) {
        return;
    }

    if (s_cue_active == 0U) {
        uint8_t next = 0U;
        if (calibrating && (s_cue_pending & STATUS_CUE_CAL_START) != 0U) {
            next = STATUS_CUE_CAL_START;
        }
        else if ((s_cue_pending & STATUS_CUE_CAL_DONE) != 0U) {
            next = STATUS_CUE_CAL_DONE;
        }
        else if (!calibrating && s_eeprom_cue_pending > 0U) {
            next = STATUS_CUE_EEPROM; /* local marker: EEPROM 5-beep, bukan bit s_cue_pending */
        }
        else if (!calibrating && (s_cue_pending & STATUS_CUE_CAL_START) != 0U) {
            /* Kalibrasi boot dapat selesai selama power-on melody; jangan memainkan cue start yang sudah basi. */
            s_cue_pending &= (uint8_t)~STATUS_CUE_CAL_START;
        }

        if (next == 0U) {
            return;
        }

        if (next == STATUS_CUE_EEPROM) {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (s_eeprom_cue_pending > 0U) {
                s_eeprom_cue_pending--;
            }
            if (!primask) {
                __enable_irq();
            }
        }
        else {
            s_cue_pending &= (uint8_t)~next;
        }
        s_cue_active = next;
        s_cue_step = 0U;
        if (next == STATUS_CUE_CAL_START) {
            hw_status_tone_start_for(1500U, 120U);
            s_cue_deadline = now + 120U;
        }
        else if (next == STATUS_CUE_CAL_DONE) {
            hw_status_tone_start_for(1900U, 160U);
            s_cue_deadline = now + 160U;
        }
        else {
            hw_status_tone_start_for(2500U, 80U);
            s_cue_deadline = now + 80U;
        }
        return;
    }

    if ((int32_t)(now - s_cue_deadline) < 0) {
        return;
    }

    if (s_cue_active != STATUS_CUE_EEPROM) {
        hw_status_tone_stop();
        s_cue_active = 0U;
        s_cue_step = 0U;
        return;
    }

    /* EEPROM cue = tepat 5 tone: T-G-T-G-T-G-T-G-T. step 0 dimulai
       setelah tone pertama selesai; step 8 mengakhiri tone kelima. */
    if (s_cue_step < 8U) {
        if ((s_cue_step & 1U) == 0U) {
            hw_status_tone_stop();
            s_cue_step++;
            s_cue_deadline = now + 80U;
        }
        else {
            hw_status_tone_start_for(2500U, 80U);
            s_cue_step++;
            s_cue_deadline = now + 80U;
        }
    }
    else {
        hw_status_tone_stop();
        s_cue_active = 0U;
        s_cue_step = 0U;
    }
}

/* Highest-priority fault present on either bridge owns both indicators. */
// Variabel s_fault_priority: status atau data gangguan untuk sistem proteksi.
static const motor_fault_t s_fault_priority[] = {
    MOTOR_FAULT_FLASH_CONFIG,
    MOTOR_FAULT_BREAK,
    MOTOR_FAULT_MCU_UNDER_VOLTAGE,
    MOTOR_FAULT_ABS_OVER_CURRENT,
    MOTOR_FAULT_ADC_DMA,
    MOTOR_FAULT_FOC_ISR_OVERRUN,
    MOTOR_FAULT_OVER_VOLTAGE,
    MOTOR_FAULT_UNDER_VOLTAGE,
    MOTOR_FAULT_CURRENT_OFFSET,
    MOTOR_FAULT_OVER_TEMP_BOARD,
    MOTOR_FAULT_OVER_TEMP_MOTOR,
    MOTOR_FAULT_OVERSPEED,
    MOTOR_FAULT_UNDERSPEED,
    MOTOR_FAULT_ABS_OVERSPEED,
    MOTOR_FAULT_ENCODER_SLIP,
    MOTOR_FAULT_HALL_INVALID,
    MOTOR_FAULT_SENSOR_DETECT,
    MOTOR_FAULT_SENSORLESS_OBSERVER
};

// Parameter argument: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi status_thread: menjalankan operasi status thread sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
void status_thread(void *argument);

// Fungsi highest_priority_fault: menangani highest priority fault dengan memprioritaskan pemadaman keluaran
// daya, pencatatan penyebab, dan pemulihan yang aman.
static motor_fault_t highest_priority_fault(void) {
    // Variabel left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const motor_fault_t left = g_motor_left.fault;
    // Variabel right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const motor_fault_t right = g_motor_right.fault;

    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t i = 0U;
            i < (uint32_t)(sizeof(s_fault_priority) / sizeof(s_fault_priority[0]));
            i++) {
        if (left == s_fault_priority[i] || right == s_fault_priority[i]) {
            return s_fault_priority[i];
        }
    }
    return MOTOR_FAULT_NONE;
}

// Fungsi hw_status_early_init: menginisialisasi hw status early init sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
void hw_status_early_init(void) {
    /* This executes immediately after HAL_Init, while the MCU is still on the
     * reset HSI clock. It therefore proves execution even if the later 64 MHz
     * PLL or motor subsystem fails. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Variabel g: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    GPIO_InitTypeDef g = {
        0
    }
    ;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &g);

    g.Pin = BUZZER_PIN | OFF_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    /* Proven hoverboard board behavior: PA5 HIGH holds the controller powered.
     * It is a power-latch/OFF control, not the TIM1/TIM8 MOE gate-enable. */
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
    s_power_held = true;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

// Fungsi hw_status_timer_init: menginisialisasi hw status timer init sehingga resource, konfigurasi awal, dan
// state modul siap digunakan dengan aman.
void hw_status_timer_init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1 = 0U;
    TIM3->DIER = 0U;
    TIM3->SR = 0U;
    /* At the required 64 MHz hoverboard clock, APB1=/2 but timer clock is
     * doubled back to 64 MHz. PSC=63 produces a 1 MHz timer counter. */
    TIM3->PSC = 63U;
    TIM3->ARR = 499U;
    TIM3->CNT = 0U;
    TIM3->EGR = TIM_EGR_UG;
    HAL_NVIC_SetPriority(TIM3_IRQn, 8U, 0U);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// Parameter hz: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi hw_status_tone_start: memulai hw status tone start setelah prasyarat hardware, konfigurasi, dan state
// keselamatan terpenuhi.
void hw_status_tone_start(uint16_t hz) {
    /* Infinite tone: only an explicit hw_status_tone_stop ends it. Retained for
     * VESC API parity; the fault/startup sequences below prefer the bounded
     * hw_status_tone_start_for() so the buzzer can never be left stuck ON. */
    hw_status_tone_start_for(hz, 0U);
}

// Parameter hz: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter duration_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks
// algoritma pada lingkup ini.
// Fungsi hw_status_tone_start_for: memulai hw status tone start for setelah prasyarat hardware, konfigurasi,
// dan state keselamatan terpenuhi.
void hw_status_tone_start_for(uint16_t hz, uint32_t duration_ms) {
    if (hz < 100U)
        hz = 100U;
    if (hz > 5000U)
        hz = 5000U;

    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* A user/fault tone pre-empts the startup melody deterministically. */
    s_melody_sequencer = false;
    g_vesc_startup_melody_active = 0U;
    s_melody_gap = false;

    if (duration_ms == 0U) {
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->DIER = 0U;
        // Variabel arr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint32_t arr = 500000UL / (uint32_t)hz;
        if (arr == 0U)
            arr = 1U;
        TIM3->ARR = arr - 1U;
        TIM3->CNT = 0U;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->SR = 0U;
        s_tone_infinite = true;
        s_tone_toggle_remaining = 0U;
        s_tone_level = false;
        s_tone_running = true;
        g_vesc_buzzer_running = 1U;
        g_vesc_buzzer_hz = hz;
        g_vesc_buzzer_remaining = 0U;
        BUZZER_PORT->BRR = BUZZER_PIN;
        TIM3->DIER = TIM_DIER_UIE;
        TIM3->CR1 |= TIM_CR1_CEN;
    }
    else {
        status_program_segment_locked(hz, duration_ms, false);
    }
    if (!primask)
        __enable_irq();
}

// Parameter on: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi hw_status_power_hold: menjalankan operasi hw status power hold sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void hw_status_power_hold(bool on) {
    s_power_held = on;
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Fungsi hw_status_power_is_held: menjalankan operasi hw status power is held sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool hw_status_power_is_held(void) {
    return s_power_held;
}

// Fungsi hw_status_tone_stop: menghentikan hw status tone stop dengan menonaktifkan output atau state terkait
// secara aman.
void hw_status_tone_stop(void) {
    // Variabel primask: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_melody_sequencer = false;
    g_vesc_startup_melody_active = 0U;
    status_timer_stop_locked();
    if (!primask)
        __enable_irq();
}

// Fungsi hw_status_tone_is_running: menjalankan operasi hw status tone is running sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool hw_status_tone_is_running(void) {
    return s_tone_running;
}

// Fungsi hw_status_tone_level: menjalankan operasi hw status tone level sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool hw_status_tone_level(void) {
    return s_tone_level;
}

// Fungsi hw_status_tim3_irq_handler: menangani hw status tim3 irq handler pada konteks interrupt dengan
// pekerjaan minimum agar timing FOC tetap deterministik.
void hw_status_tim3_irq_handler(void) {
    if ((TIM3->SR & TIM_SR_UIF) == 0U)
        return;
    TIM3->SR &= ~TIM_SR_UIF;
    if (!s_tone_running)
        return;

    if (s_tone_infinite) {
        s_tone_level = !s_tone_level;
        if (s_tone_level)
            BUZZER_PORT->BSRR = BUZZER_PIN;
        else BUZZER_PORT->BRR = BUZZER_PIN;
        return;
    }

    if (s_tone_toggle_remaining > 0U)
        s_tone_toggle_remaining--;
    g_vesc_buzzer_remaining = s_tone_toggle_remaining;
    if (!s_melody_gap) {
        s_tone_level = !s_tone_level;
        if (s_tone_level)
            BUZZER_PORT->BSRR = BUZZER_PIN;
        else BUZZER_PORT->BRR = BUZZER_PIN;
    }
    else {
        s_tone_level = false;
        BUZZER_PORT->BRR = BUZZER_PIN;
    }

    if (s_tone_toggle_remaining == 0U) {
        if (s_melody_sequencer) {
            status_melody_load_next_locked();
        }
        else {
            status_timer_stop_locked();
        }
    }
}

// Parameter now: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi hw_status_service_10ms: menjalankan operasi hw status service 10ms sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
void hw_status_service_10ms(uint32_t now) {
    /* Status/LED/buzzer is intentionally NOT a sixth task. Its non-blocking
     * state machine is serviced by VESC timer_thread every 10 ms. */
    // Variabel initialized: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static bool initialized = false;
    // Variabel fault_deadline: status atau data gangguan untuk sistem proteksi.
    static uint32_t fault_deadline;
    // Variabel led_state: state mesin keadaan yang menentukan tahap operasi.
    static uint8_t led_state; /* 0=burst / 1=gap */
    // Variabel led_pulses_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static uint8_t led_pulses_left;
    // Variabel led_output_on: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static bool led_output_on;
    // Variabel led_deadline: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static uint32_t led_deadline;
    // Variabel hb_led_on: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static bool hb_led_on;
    // Variabel hb_deadline: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static uint32_t hb_deadline;
    // Variabel led_mode: mode operasi yang menentukan jalur algoritma aktif.
    static uint8_t led_mode; /* 0=heartbeat, 1=cal, 2=detect, 3=run */
    // Variabel announced: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    static motor_fault_t announced;
    // Variabel fault_pulses_left: status atau data gangguan untuk sistem proteksi.
    static uint8_t fault_pulses_left;
    // Variabel fault_stage: status atau data gangguan untuk sistem proteksi.
    static uint8_t fault_stage;
    // Variabel fault_output_on: status atau data gangguan untuk sistem proteksi.
    static bool fault_output_on;

    if (!initialized) {
        fault_deadline = now;
        led_state = 0U;
        led_pulses_left = 0U;
        led_output_on = false;
        led_deadline = now;
        hb_led_on = false;
        hb_deadline = now + 500U;
        led_mode = 0xFFU; /* force first-mode initialization */
        motor_hw_led(false);
        announced = MOTOR_FAULT_NONE;
        fault_pulses_left = fault_stage = 0U;
        fault_output_on = false;
        initialized = true;
    }

    // Variabel fault: status atau data gangguan untuk sistem proteksi.
    const motor_fault_t fault = highest_priority_fault();

    /* Fault tetap langsung mematikan bridge melalui jalur motor fault. Audio
       fault-code ditunda sampai power-on melody selesai agar bunyi power-on
       selalu utuh dan dapat dipakai sebagai indikator MCU benar-benar boot. */
    if (fault != MOTOR_FAULT_NONE && g_vesc_startup_melody_active) {
        return;
    }

    if (fault != MOTOR_FAULT_NONE) {
        if (fault != announced) {
            announced = fault;
            const uint8_t code = (uint8_t)motor_fault_to_vesc(fault);
            fault_pulses_left = code != 0U ? code : 1U;
            fault_stage = 1U;
            fault_output_on = true;
            s_cue_pending = 0U;
            s_cue_active = 0U;
            s_cue_step = 0U;
            motor_hw_led(true);
            hw_status_tone_start_for(2200U, 100U);
            fault_deadline = now + 100U;
        }
        else if ((int32_t)(now - fault_deadline) >= 0) {
            if (fault_stage == 1U) {
                if (fault_output_on) {
                    motor_hw_led(false);
                    hw_status_tone_stop();
                    fault_output_on = false;
                    if (fault_pulses_left > 0U) {
                        fault_pulses_left--;
                    }
                    if (fault_pulses_left == 0U) {
                        fault_stage = 2U;
                        fault_deadline = now + 1000U;
                    }
                    else {
                        fault_deadline = now + 100U;
                    }
                }
                else {
                    fault_output_on = true;
                    motor_hw_led(true);
                    hw_status_tone_start_for(2200U, 100U);
                    fault_deadline = now + 100U;
                }
            }
            else {
                const uint8_t code = (uint8_t)motor_fault_to_vesc(fault);
                fault_pulses_left = code != 0U ? code : 1U;
                fault_stage = 1U;
                fault_output_on = true;
                motor_hw_led(true);
                hw_status_tone_start_for(2200U, 100U);
                fault_deadline = now + 100U;
            }
        }
    }
    else {
        if (announced != MOTOR_FAULT_NONE || fault_output_on) {
            hw_status_tone_stop();
            fault_output_on = false;
            /* Reset mesin mode-cue (burst/gap) biar LED gak nyangkut di
             * tengah pulsa saat keluar dari fault. */
            led_state = 0U;
            led_pulses_left = 0U;
            led_output_on = false;
            hb_led_on = false;
            hb_deadline = now + 500U;
            led_mode = 0xFFU;
            motor_hw_led(false);
        }
        announced = MOTOR_FAULT_NONE;
        fault_stage = 0U;

        /* LED status follows the proven SmartESC idea: a healthy controller
         * must always have an obvious periodic heartbeat. Additional burst modes
         * are retained for calibration/detection/running, but every transition
         * is explicitly re-initialized so the LED cannot inherit a stale pulse
         * counter/deadline from the previous mode.
         *
         *   calibrating -> 1 pulse burst (200 ms on/off), 1 s gap
         *   detecting   -> 2 pulse burst, 1 s gap
         *   running     -> 3 pulse burst, 1 s gap
         *   idle/ready  -> 500 ms toggle heartbeat (SmartESC-like normal blink)
         */
        // Variabel calibrating: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const bool calibrating = !foc_calibration_done();
        // Variabel detecting: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const bool detecting = g_motor_left.detect.busy ||
                g_motor_right.detect.busy;
        // Variabel running: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const bool running = g_motor_left.pwm_enabled ||
                g_motor_right.pwm_enabled;
        // Variabel mode: mode operasi yang menentukan jalur algoritma aktif.
        const uint8_t mode = calibrating ? 1U :
                             (detecting ? 2U :
                             (running ? 3U : 0U));
        // Variabel pulses: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const uint8_t pulses = mode;

        /* Audio non-fault dilayani terpisah dari LED. Running/detect menunda cue sehingga motor tidak
         * pernah menerima bunyi status biasa ketika bridge sedang aktif. */
        status_service_cue_10ms(now, calibrating, detecting, running && !calibrating && !detecting);

        if (mode != led_mode) {
            led_mode = mode;
            led_state = 0U;
            led_pulses_left = 0U;
            led_output_on = false;
            led_deadline = now;
            hb_led_on = false;
            hb_deadline = now + 500U;
            motor_hw_led(false);
        }

        if (pulses != 0U) {
            if (led_state == 0U) {
                if (led_pulses_left == 0U) {
                    led_pulses_left = pulses;
                    led_output_on = true;
                    motor_hw_led(true);
                    led_deadline = now + 200U;
                }
                else if ((int32_t)(now - led_deadline) >= 0) {
                    led_output_on = !led_output_on;
                    motor_hw_led(led_output_on);
                    if (!led_output_on) {
                        led_pulses_left--;
                        if (led_pulses_left == 0U) {
                            led_state = 1U;
                            led_deadline = now + 1000U;
                        }
                        else {
                            led_deadline = now + 200U;
                        }
                    }
                    else {
                        led_deadline = now + 200U;
                    }
                }
            }
            else {
                motor_hw_led(false);
                if ((int32_t)(now - led_deadline) >= 0) {
                    led_state = 0U;
                    led_pulses_left = 0U;
                    led_output_on = false;
                }
            }
        }
        else if ((int32_t)(now - hb_deadline) >= 0) {
            /* SmartESC normal path toggles its LED roughly every 0.5 s. Use an
             * absolute deadline rather than an 8-bit divider so delayed timer
             * service cannot permanently distort the blink cadence. */
            hb_led_on = !hb_led_on;
            motor_hw_led(hb_led_on);
            hb_deadline = now + 500U;
        }

    }

}

// Fungsi hw_status_init: menginisialisasi hw status init sehingga resource, konfigurasi awal, dan state modul
// siap digunakan dengan aman.
bool hw_status_init(void) {
    s_status_started = true;
    /* Start the complete 3.21 s power-on melody before FreeRTOS. TIM3 advances
     * notes/gaps autonomously, so UART/FOC startup proceeds concurrently and the
     * melody cannot disappear merely because timer_thread has not run yet. */
    hw_status_startup_melody_begin();
    return true;
}

// Parameter delay_ms: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma
// pada lingkup ini.
// Fungsi early_fatal_delay_with_comm: menjalankan operasi early fatal delay with comm sesuai tanggung jawab
// modul dengan input tervalidasi dan state yang konsisten.
static void early_fatal_delay_with_comm(uint32_t delay_ms) {
    // Variabel start: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < delay_ms) {
        /* If USART3/commands were already brought up, preserve VESC Tool
         * recovery access even though the motor subsystem is in early-fatal. */
        if (commands_is_initialized()) {
            (void)vesc_comm_poll_once();
        }
        HAL_Delay(1U);
    }
}

// Fungsi hw_status_early_fatal_loop: menjalankan operasi hw status early fatal loop sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void hw_status_early_fatal_loop(void) {
    hw_status_tone_stop();
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    for (;; ) {
        /* Pre-scheduler failure memakai LED saja. Buzzer otomatis dicadangkan
         * untuk power-on melody dan fault-code runtime agar tidak ada pola audio
         * yang ambigu. UART tetap serviceable jika sudah diinisialisasi. */
        // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
        for (uint8_t i = 0U; i < 4U; i++) {
            motor_hw_led(true);
            early_fatal_delay_with_comm(80U);
            motor_hw_led(false);
            early_fatal_delay_with_comm(80U);
        }
        early_fatal_delay_with_comm(800U);
    }
}
