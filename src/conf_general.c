#include "conf_general.h"
#include "confgenerator.h"
#include "motor/mc_interface.h"
#include "hwconf/hw.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CFG_REGION_ADDR      0x0803E000UL
#define CFG_FLASH_PAGE_SIZE  2048UL
#define CFG_PAGE_COUNT       4U
#define CFG_MAGIC            0x56314346UL /* VESC F103 transactional config */
/* v0x0012 was the B1..B5 record: exactly two VESC6 MCCONF images + APPCONF.
 * v0x0013 appends two 64-bit odometers as byte arrays, preserving the wire
 * images byte-for-byte and allowing a lossless migration from old records. */
#define CFG_VERSION_LEGACY   0x0012U
#define CFG_VERSION          0x0013U
#define CFG_PAYLOAD_V12_LEN  (2U*VESC6_MCCONF_WIRE_SIZE + VESC6_APPCONF_WIRE_SIZE)
#define CFG_PAYLOAD_LEN      (CFG_PAYLOAD_V12_LEN + 16U)

typedef struct {
    // Variabel mc_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE];
    // Variabel mc_right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE];
    // Variabel app: state atau konfigurasi aplikasi VESC.
    uint8_t app[VESC6_APPCONF_WIRE_SIZE];
} config_payload_v12_t;

typedef struct {
    // Variabel mc_left: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE];
    // Variabel mc_right: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE];
    // Variabel app: state atau konfigurasi aplikasi VESC.
    uint8_t app[VESC6_APPCONF_WIRE_SIZE];
    // Variabel odometer_left_be: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t odometer_left_be[8];
    // Variabel odometer_right_be: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint8_t odometer_right_be[8];
} config_payload_t;

typedef struct {
    // Variabel magic: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t magic;
    // Variabel version: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t version;
    // Variabel payload_len: panjang data yang sedang diproses atau dikirim.
    uint16_t payload_len;
    // Variabel sequence: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t sequence;
    // Variabel crc32: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t crc32;
} config_record_header_t;

typedef struct {
    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    config_record_header_t h;
    // Variabel payload: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    config_payload_t payload;
} config_record_t;

_Static_assert(sizeof(config_record_header_t) == 16U, "persistent header layout changed");
_Static_assert(sizeof(config_payload_v12_t) == CFG_PAYLOAD_V12_LEN, "legacy wire payload layout changed");
_Static_assert(sizeof(config_payload_t) == CFG_PAYLOAD_LEN, "wire+odometer payload layout changed");
_Static_assert(sizeof(config_record_t) < CFG_FLASH_PAGE_SIZE, "VESC config must fit one F1 flash page");
// Variabel s_valid: penanda validitas hasil pengukuran atau konfigurasi.
static bool s_valid = false;
// Variabel s_boot_status: status runtime untuk diagnostik atau keputusan kendali.
static conf_boot_status_t s_boot_status = CONF_BOOT_VIRGIN;
// Variabel s_save_count: pencacah kejadian atau sampel.
static uint32_t s_save_count = 0U;
// Variabel s_aux_store_pending: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_aux_store_pending = false;
// Variabel s_odo_last_persist: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint64_t s_odo_last_persist[2] = {
    0U, 0U
}
;
// Variabel s_integrity_fault: status atau data gangguan untuk sistem proteksi.
static volatile bool s_integrity_fault = false;
// Variabel s_integrity_checks: state internal modul yang dipertahankan antar pemanggilan fungsi.
// Variabel s_integrity_failures: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint32_t s_integrity_checks = 0U, s_integrity_failures = 0U;
// Variabel s_integrity_div: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_integrity_div = 0U;
// Variabel s_store_busy: state internal modul yang dipertahankan antar pemanggilan fungsi.
static volatile bool s_store_busy = false;
// Variabel s_aux_retry_div: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_aux_retry_div = 0U;
/* Large (~1.5 KiB) flash staging record is static, never placed on a small RTOS thread stack. */
// Variabel s_stage: state internal modul yang dipertahankan antar pemanggilan fungsi.
static config_record_t s_stage;
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter n: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
// Fungsi crc32_ieee: menjalankan operasi crc32 ieee sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static uint32_t crc32_ieee(const uint8_t *p, uint32_t n) {
    // Variabel c: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t c = 0xFFFFFFFFUL;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        // Variabel b: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        for (uint8_t b = 0; b < 8; b++) {
            // Variabel m: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            uint32_t m = (uint32_t)-(int32_t)(c&1U);
            c = (c>>1)^(0xEDB88320UL&m);
        }
    }
    return ~c;
}
// Parameter page: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi page_hdr: menjalankan operasi page hdr sesuai tanggung jawab modul dengan input tervalidasi dan state
// yang konsisten.
static const config_record_header_t *page_hdr(uint32_t page) {
    return (const config_record_header_t *)(CFG_REGION_ADDR+page*CFG_FLASH_PAGE_SIZE);
}
// Parameter page: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi page_payload: menjalankan operasi page payload sesuai tanggung jawab modul dengan input tervalidasi
// dan state yang konsisten.
static const uint8_t *page_payload(uint32_t page) {
    return (const uint8_t *)(CFG_REGION_ADDR+page*CFG_FLASH_PAGE_SIZE+sizeof(config_record_header_t));
}
// Parameter page: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi rec_valid_page: menjalankan operasi rec valid page sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool rec_valid_page(uint32_t page) {
    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const config_record_header_t*h = page_hdr(page);
    if (!h || h->magic != CFG_MAGIC)
        return false;
    // Variabel need: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint16_t need = (h->version == CFG_VERSION) ? (uint16_t)CFG_PAYLOAD_LEN :
                  (h->version == CFG_VERSION_LEGACY) ? (uint16_t)CFG_PAYLOAD_V12_LEN : 0U;
    return need != 0U && h->payload_len == need && crc32_ieee(page_payload(page), need) == h->crc32;
}
// Fungsi config_region_blank: menjalankan operasi config region blank sesuai tanggung jawab modul dengan input
// tervalidasi dan state yang konsisten.
static bool config_region_blank(void) {
    // Variabel w: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const volatile uint32_t *w = (const volatile uint32_t*)CFG_REGION_ADDR;
    // Variabel words: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint32_t words = (CFG_PAGE_COUNT*CFG_FLASH_PAGE_SIZE)/sizeof(uint32_t);
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (uint32_t i = 0U; i < words; i++) {
        if (w[i] != 0xFFFFFFFFUL)
            return false;
    }
    return true;
}
// Fungsi best_page: menjalankan operasi best page sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static int best_page(void) {
    // Variabel best: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int best = -1;
    // Variabel seq: nomor urut untuk konsistensi snapshot atau record.
    uint32_t seq = 0;
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (uint32_t p = 0; p < CFG_PAGE_COUNT; p++) {
        // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const config_record_header_t*h = page_hdr(p);
        if (rec_valid_page(p) && (best < 0 || h->sequence > seq)) {
            best = (int)p;
            seq = h->sequence;
        }
    }
    return best;
}

/* Serialisasi semua transaksi flash config. Blocking worker dan timer 100 Hz
 * sama-sama dapat meminta penyimpanan; tanpa guard ini keduanya bisa menimpa
 * s_stage dan mengakses erase/program flash secara bersamaan. Critical section
 * hanya melindungi flag, bukan operasi flash yang panjang. */
// Fungsi store_lock_try: menyimpan store lock try secara transaksional dengan pemeriksaan integritas sehingga
// konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
static bool store_lock_try(void) {
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = false;
    taskENTER_CRITICAL();
    if (!s_store_busy) {
        s_store_busy = true;
        ok = true;
    }
    taskEXIT_CRITICAL();
    return ok;
}
// Fungsi store_unlock: menyimpan store unlock secara transaksional dengan pemeriksaan integritas sehingga
// konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
static void store_unlock(void) {
    taskENTER_CRITICAL();
    s_store_busy = false;
    taskEXIT_CRITICAL();
}

// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi get_u64_be8: membaca get u64 be8 tanpa mengubah state kendali utama dan mengembalikan data yang
// konsisten.
static uint64_t get_u64_be8(const uint8_t b[8]) {
    // Variabel v: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t v = 0U;
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (unsigned i = 0; i < 8U; i++)
        v = (v<<8)|b[i];
    return v;
}
// Parameter b: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi put_u64_be8: menyusun put u64 be8 ke buffer/wire format dengan urutan field, skala, dan batas data
// yang konsisten.
static void put_u64_be8(uint8_t b[8], uint64_t v) {
    // Variabel i: indeks iterasi lokal untuk menelusuri elemen atau sampel secara deterministik.
    for (int i = 7; i >= 0; i--) {
        b[i] = (uint8_t)v;
        v >>= 8;
    }
}

// Fungsi conf_general_init: menginisialisasi conf general init sehingga resource, konfigurasi awal, dan state
// modul siap digunakan dengan aman.
bool conf_general_init(void) {
    return conf_general_load_apply();
}

// Fungsi conf_general_load_apply: memuat conf general load apply dan memvalidasi integritas data sebelum
// digunakan oleh runtime.
bool conf_general_load_apply(void) {
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int p = best_page();
    if (p < 0) {
        s_valid = false;
        s_boot_status = config_region_blank() ? CONF_BOOT_VIRGIN : CONF_BOOT_CORRUPT;
        s_integrity_fault = (s_boot_status == CONF_BOOT_CORRUPT);
        return false;
    }
    // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const config_record_header_t*h = page_hdr((uint32_t)p);
    // Variabel oldp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const config_payload_v12_t*oldp = (const config_payload_v12_t*)page_payload((uint32_t)p);
    if (!vesc_config_import_wire(oldp->mc_left, oldp->mc_right, oldp->app)) {
        s_valid = false;
        s_boot_status = CONF_BOOT_CORRUPT;
        s_integrity_fault = true;
        return false;
    }
    if (h->version == CFG_VERSION) {
        // Variabel cp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const config_payload_t*cp = (const config_payload_t*)page_payload((uint32_t)p);
        mc_interface_set_odometer_motor(MOTOR_LEFT, get_u64_be8(cp->odometer_left_be));
        mc_interface_set_odometer_motor(MOTOR_RIGHT, get_u64_be8(cp->odometer_right_be));
    }
    else{
        mc_interface_set_odometer_motor(MOTOR_LEFT, 0U);
        mc_interface_set_odometer_motor(MOTOR_RIGHT, 0U);
    }
    s_odo_last_persist[0] = mc_interface_get_odometer_motor(MOTOR_LEFT);
    s_odo_last_persist[1] = mc_interface_get_odometer_motor(MOTOR_RIGHT);
    s_save_count = h->sequence;
    s_valid = true;
    s_boot_status = CONF_BOOT_VALID;
    s_integrity_fault = false;
    return true;
}
// Parameter p: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi erase_page: menjalankan operasi erase page sesuai tanggung jawab modul dengan input tervalidasi dan
// state yang konsisten.
static HAL_StatusTypeDef erase_page(uint32_t p) {
    // Variabel e: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    FLASH_EraseInitTypeDef e = {
        0
    }
    ;
    // Variabel err: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t err = 0;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.PageAddress = CFG_REGION_ADDR+p*CFG_FLASH_PAGE_SIZE;
    e.NbPages = 1;
    return HAL_FLASHEx_Erase(&e, &err);
}
// Parameter a: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter v: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi phw: menjalankan operasi phw sesuai tanggung jawab modul dengan input tervalidasi dan state yang
// konsisten.
static HAL_StatusTypeDef phw(uint32_t a, uint16_t v) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, a, v);
}
// Parameter addr: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Parameter r: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi write_commit: menyusun atau mengirim write commit dengan pemeriksaan panjang buffer dan jalur
// transport yang aman.
static HAL_StatusTypeDef write_commit(uint32_t addr, const config_record_t*r) {
    // Variabel s: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    const uint8_t *s = (const uint8_t *)r;
    // Variabel o: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    for (uint32_t o = 4U; o < sizeof(*r); o += 2U) {
        // Variabel h: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        uint16_t h = s[o];
        if (o + 1U < sizeof(*r)) {
            h |= (uint16_t)s[o + 1U] << 8;
        }
        // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        HAL_StatusTypeDef st = phw(addr + o, h);
        if (st != HAL_OK) {
            return st;
        }
    }
    // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    HAL_StatusTypeDef st = phw(addr, (uint16_t)(r->h.magic & 0xFFFFU));
    if (st != HAL_OK) {
        return st;
    }
    return phw(addr + 2U, (uint16_t)(r->h.magic >> 16));
}

// Fungsi store_stage: menyimpan store stage secara transaksional dengan pemeriksaan integritas sehingga
// konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
static bool store_stage(void) {
    motor_stop(&g_motor_left);
    motor_stop(&g_motor_right);
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        vTaskDelay(pdMS_TO_TICKS(5U));
    s_stage.h.magic = CFG_MAGIC;
    s_stage.h.version = CFG_VERSION;
    s_stage.h.payload_len = CFG_PAYLOAD_LEN;
    s_stage.h.sequence = s_save_count+1U;
    put_u64_be8(s_stage.payload.odometer_left_be, mc_interface_get_odometer_motor(MOTOR_LEFT));
    put_u64_be8(s_stage.payload.odometer_right_be, mc_interface_get_odometer_motor(MOTOR_RIGHT));
    s_stage.h.crc32 = crc32_ieee((const uint8_t*)&s_stage.payload, sizeof(s_stage.payload));
    // Variabel cur: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int cur = best_page();
    // Variabel next: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint32_t next = (cur < 0) ? 0U : ((uint32_t)(cur+1)%CFG_PAGE_COUNT);
    /* STM32F1 flash erase/program stalls code fetch. The complete request is
       already owned by a blocking worker or this save is deferred until both
       motors are idle. Previous committed page remains valid until magic is
       programmed last. */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    DMA1->IFCR = DMA_IFCR_CGIF1;
    // Variabel st: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st == HAL_OK) {
        st = erase_page(next);
    }
    if (st == HAL_OK) {
        st = write_commit(CFG_REGION_ADDR + next * CFG_FLASH_PAGE_SIZE, &s_stage);
    }
    (void)HAL_FLASH_Lock();
    DMA1->IFCR = DMA_IFCR_CGIF1;
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    if (st == HAL_OK && rec_valid_page(next) && page_hdr(next)->sequence == s_stage.h.sequence) {
        s_valid = true;
        s_boot_status = CONF_BOOT_VALID;
        s_integrity_fault = false;
        s_save_count = s_stage.h.sequence;
        s_odo_last_persist[0] = mc_interface_get_odometer_motor(MOTOR_LEFT);
        s_odo_last_persist[1] = mc_interface_get_odometer_motor(MOTOR_RIGHT);
        return true;
    }

    /* Jika erase/program/verify record baru gagal, record committed lama masih
     * menjadi fallback. Jangan menandai FLASH_CONFIG corrupt selama masih ada
     * page lama yang valid; caller tetap menerima false agar write dianggap gagal. */
    s_integrity_failures++;
    // Variabel fallback: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int fallback = best_page();
    if (fallback >= 0) {
        s_valid = true;
        s_boot_status = CONF_BOOT_VALID;
        s_integrity_fault = false;
        s_save_count = page_hdr((uint32_t)fallback)->sequence;
        return false;
    }
    s_valid = false;
    s_boot_status = CONF_BOOT_CORRUPT;
    s_integrity_fault = true;
    return false;
}

// Fungsi stage_persistent_base: menjalankan operasi stage persistent base sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
static void stage_persistent_base(void) {
    /* Read the last committed config directly from memory-mapped flash. This
       keeps temporary runtime MCCONF separate without spending ~1.45 KiB of
       SRAM on a persistent shadow. If there is no record yet, compiled VESC6
       defaults are the persistent base. */
    // Variabel p: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    int p = best_page();
    if (p >= 0) {
        // Variabel cp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
        const config_payload_v12_t *cp = (const config_payload_v12_t*)page_payload((uint32_t)p);
        memcpy(s_stage.payload.mc_left, cp->mc_left, VESC6_MCCONF_WIRE_SIZE);
        memcpy(s_stage.payload.mc_right, cp->mc_right, VESC6_MCCONF_WIRE_SIZE);
        memcpy(s_stage.payload.app, cp->app, VESC6_APPCONF_WIRE_SIZE);
    }
    else{
        memcpy(s_stage.payload.mc_left, vesc_config_mc_wire(MOTOR_LEFT, true), VESC6_MCCONF_WIRE_SIZE);
        memcpy(s_stage.payload.mc_right, vesc_config_mc_wire(MOTOR_RIGHT, true), VESC6_MCCONF_WIRE_SIZE);
        memcpy(s_stage.payload.app, vesc_config_app_wire(true), VESC6_APPCONF_WIRE_SIZE);
    }
}

// Fungsi conf_general_store_all: menyimpan conf general store all secara transaksional dengan pemeriksaan
// integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
bool conf_general_store_all(void) {
    if (!store_lock_try())
        return false;
    memset(&s_stage, 0, sizeof(s_stage));
    vesc_config_export_wire(s_stage.payload.mc_left, s_stage.payload.mc_right, s_stage.payload.app);
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = store_stage();
    store_unlock();
    if (ok) {
        hw_status_notify_eeprom_saved();
    }
    return ok;
}

// Parameter id: identitas motor, controller, kanal, atau objek yang sedang diproses.
// Parameter wire: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi conf_general_store_mc_wire_persistent: menyimpan conf general store mc wire persistent secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_mc_wire_persistent(motor_id_t id, const uint8_t *wire) {
    if (!wire || (id != MOTOR_LEFT && id != MOTOR_RIGHT))
        return false;
    if (!store_lock_try())
        return false;
    memset(&s_stage, 0, sizeof(s_stage));
    stage_persistent_base();
    memcpy(id == MOTOR_RIGHT ? s_stage.payload.mc_right : s_stage.payload.mc_left, wire, VESC6_MCCONF_WIRE_SIZE);
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = store_stage();
    store_unlock();
    if (ok) {
        hw_status_notify_eeprom_saved();
    }
    return ok;
}

// Parameter wire: nilai kerja yang menyimpan state, parameter, atau hasil antara sesuai konteks algoritma pada
// lingkup ini.
// Fungsi conf_general_store_app_wire_persistent: menyimpan conf general store app wire persistent secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_app_wire_persistent(const uint8_t *wire) {
    if (!wire)
        return false;
    if (!store_lock_try())
        return false;
    memset(&s_stage, 0, sizeof(s_stage));
    stage_persistent_base();
    memcpy(s_stage.payload.app, wire, VESC6_APPCONF_WIRE_SIZE);
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = store_stage();
    store_unlock();
    if (ok) {
        hw_status_notify_eeprom_saved();
    }
    return ok;
}

// Fungsi conf_general_store_aux_only: menyimpan conf general store aux only secara transaksional dengan
// pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
static bool conf_general_store_aux_only(void) {
    if (!store_lock_try())
        return false;
    memset(&s_stage, 0, sizeof(s_stage));
    stage_persistent_base();
    // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    bool ok = store_stage();
    store_unlock();
    if (ok) {
        hw_status_notify_eeprom_saved();
    }
    return ok;
}
// Fungsi conf_general_is_valid: menjalankan operasi conf general is valid sesuai tanggung jawab modul dengan
// input tervalidasi dan state yang konsisten.
bool conf_general_is_valid(void) {
    return s_valid && !s_integrity_fault;
}
// Fungsi conf_general_boot_status: menjalankan operasi conf general boot status sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
conf_boot_status_t conf_general_boot_status(void) {
    return s_boot_status;
}
// Fungsi conf_general_get_save_count: membaca conf general get save count tanpa mengubah state kendali utama
// dan mengembalikan data yang konsisten.
uint32_t conf_general_get_save_count(void) {
    return s_save_count;
}
// Fungsi conf_general_integrity_ok: menjalankan operasi conf general integrity ok sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
bool conf_general_integrity_ok(void) {
    return !s_integrity_fault;
}
// Fungsi conf_general_get_integrity_checks: membaca conf general get integrity checks tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
uint32_t conf_general_get_integrity_checks(void) {
    return s_integrity_checks;
}
// Fungsi conf_general_get_integrity_failures: membaca conf general get integrity failures tanpa mengubah state
// kendali utama dan mengembalikan data yang konsisten.
uint32_t conf_general_get_integrity_failures(void) {
    return s_integrity_failures;
}


// Fungsi conf_general_request_aux_store: menyimpan conf general request aux store secara transaksional dengan
// pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi gagal.
void conf_general_request_aux_store(void) {
    s_aux_store_pending = true;
}
// Fungsi conf_general_service_100hz: menjalankan operasi conf general service 100hz sesuai tanggung jawab modul
// dengan input tervalidasi dan state yang konsisten.
void conf_general_service_100hz(void) {
    /* Periodic read-only scrub of the last committed transactional record.
       It costs no flash wear and runs at 1 Hz, outside the 16-kHz FOC path. */
    if (++s_integrity_div >= 100U) {
        s_integrity_div = 0U;
        if (s_valid && !s_integrity_fault && !g_motor_left.pwm_enabled && !g_motor_right.pwm_enabled &&
            !g_motor_left.command_active && !g_motor_right.command_active) {
            s_integrity_checks++;
            // Variabel bp: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            int bp = best_page();
            // Variabel ok: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
            bool ok = bp >= 0 && rec_valid_page((uint32_t)bp) &&
                      page_hdr((uint32_t)bp)->sequence == s_save_count;
            if (!ok) {
                s_integrity_fault = true;
                s_integrity_failures++;
                motor_hw_emergency_all_off();
                motor_raise_fault_from_task(&g_motor_left, MOTOR_FAULT_FLASH_CONFIG);
                motor_raise_fault_from_task(&g_motor_right, MOTOR_FAULT_FLASH_CONFIG);
            }
        }
    }
    /* Wear-aware automatic persistence: crossing 1 km since the last committed
       odometer snapshot schedules a save, but flash is never touched while a
       motor is driving. Explicit COMM_SET_ODOMETER also sets this flag. */
    // Variabel l: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    // Variabel r: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t l = mc_interface_get_odometer_motor(MOTOR_LEFT), r = mc_interface_get_odometer_motor(MOTOR_RIGHT);
    // Variabel dl: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t dl = (l >= s_odo_last_persist[0]) ? (l-s_odo_last_persist[0]) : (s_odo_last_persist[0]-l);
    // Variabel dr: nilai kerja yang menyimpan state atau hasil antara sesuai konteks algoritma pada lingkup ini.
    uint64_t dr = (r >= s_odo_last_persist[1]) ? (r-s_odo_last_persist[1]) : (s_odo_last_persist[1]-r);
    if (dl >= 1000U || dr >= 1000U)
        s_aux_store_pending = true;
    if (!s_aux_store_pending)
        return;
    if (g_motor_left.pwm_enabled || g_motor_right.pwm_enabled || g_motor_left.command_active || g_motor_right.command_active)
        return;
    /* Auto-save tidak boleh mencoba erase/program setiap 10 ms ketika flash
     * sedang dipakai task lain atau transaksi sebelumnya gagal. Retry maksimum
     * sekitar 1 Hz; explicit MCCONF/APPCONF write tetap mencoba langsung. */
    if (s_aux_retry_div > 0U) {
        s_aux_retry_div--;
        return;
    }
    if (conf_general_store_aux_only()) {
        s_aux_store_pending = false;
        s_aux_retry_div = 0U;
    }
    else{
        s_aux_retry_div = 100U;
    }
}

/* ==================== Canonical VESC configuration API ==================== */
// Variabel s_typed_mc_wire: state internal modul yang dipertahankan antar pemanggilan fungsi.
static uint8_t s_typed_mc_wire[VESC6_MCCONF_WIRE_SIZE];
// Variabel s_typed_app_wire: state atau konfigurasi aplikasi VESC.
static uint8_t s_typed_app_wire[VESC6_APPCONF_WIRE_SIZE];

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi conf_general_read_app_configuration: menjalankan operasi conf general read app configuration sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void conf_general_read_app_configuration(app_configuration *conf) {
    if (conf == NULL) {
        return;
    }
    vesc_config_init_defaults();
    if (!confgenerator_deserialize_appconf(vesc_config_app_wire(false), conf)) {
        confgenerator_set_defaults_appconf(conf);
    }
}

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Fungsi conf_general_store_app_configuration: menyimpan conf general store app configuration secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_app_configuration(app_configuration *conf) {
    if (conf == NULL) {
        return false;
    }
    if (confgenerator_serialize_appconf(s_typed_app_wire, conf) != (int32_t)VESC6_APPCONF_WIRE_SIZE) {
        return false;
    }
    return vesc_config_set_app_wire(s_typed_app_wire, VESC6_APPCONF_WIRE_SIZE, true);
}

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter is_motor_2: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi conf_general_read_mc_configuration: menjalankan operasi conf general read mc configuration sesuai
// tanggung jawab modul dengan input tervalidasi dan state yang konsisten.
void conf_general_read_mc_configuration(mc_configuration *conf, bool is_motor_2) {
    if (conf == NULL) {
        return;
    }
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    motor_id_t id = is_motor_2 ? MOTOR_RIGHT : MOTOR_LEFT;
    vesc_config_init_defaults();
    if (!confgenerator_deserialize_mcconf(vesc_config_mc_wire(id, false), conf)) {
        confgenerator_set_defaults_mcconf(conf);
    }
}

// Parameter conf: struktur konfigurasi yang dibaca, divalidasi, atau diterapkan.
// Parameter is_motor_2: state, parameter, atau identitas motor yang sedang diproses.
// Fungsi conf_general_store_mc_configuration: menyimpan conf general store mc configuration secara
// transaksional dengan pemeriksaan integritas sehingga konfigurasi lama tetap dapat dipulihkan bila operasi
// gagal.
bool conf_general_store_mc_configuration(mc_configuration *conf, bool is_motor_2) {
    if (conf == NULL) {
        return false;
    }
    // Variabel id: identitas motor, controller, kanal, atau objek yang sedang diproses.
    motor_id_t id = is_motor_2 ? MOTOR_RIGHT : MOTOR_LEFT;
    if (confgenerator_serialize_mcconf_motor(s_typed_mc_wire, conf, id) !=
        (int32_t)VESC6_MCCONF_WIRE_SIZE) {
        return false;
    }
    return vesc_config_set_mc_wire(id, s_typed_mc_wire, VESC6_MCCONF_WIRE_SIZE, true);
}
