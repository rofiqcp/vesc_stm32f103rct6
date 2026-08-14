#include "config_store.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_control.h"
#include "vesc_timeout.h"
#include "app_config.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <string.h>

#define CFG_FLASH_ADDR      0x0803F800UL /* last 2-KiB page on STM32F103RC */
#define CFG_FLASH_PAGE_SIZE 2048UL
#define CFG_MAGIC           0x56364346UL /* "V6CF" */
#define CFG_VERSION         0x0002U

typedef struct {
    uint8_t pole_pairs;
    uint8_t sensor_mode;
    uint8_t sensor_request_mode;
    uint8_t encoder_inverted;
    uint32_t encoder_cpr;
    uint16_t encoder_elec_offset_u16;
    uint16_t hall_offset_u16;
    uint8_t foc_hall_table[8];
    uint16_t hall_angle_u16[8];
    float current_kp;
    float current_ki;
    float speed_kp;
    float speed_ki;
    float speed_kd;
} motor_persist_t;

typedef struct {
    uint32_t timeout_ms;
    int32_t timeout_brake_mA;
    motor_persist_t motor[2];
} config_payload_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_len;
    uint32_t sequence;
    uint32_t crc32;
    config_payload_t payload;
} config_record_t;

#define CFG_SLOT_SIZE  ((uint32_t)((sizeof(config_record_t) + 3U) & ~3U))
#define CFG_SLOT_COUNT (CFG_FLASH_PAGE_SIZE / CFG_SLOT_SIZE)

_Static_assert(sizeof(config_record_t) < CFG_FLASH_PAGE_SIZE,
               "Persistent configuration must fit in one STM32F103RC flash page");
_Static_assert((CFG_SLOT_SIZE & 1U) == 0U,
               "Persistent slot size must be halfword aligned");
_Static_assert(CFG_SLOT_COUNT >= 2U,
               "Persistent page should provide multiple wear-level slots");

static bool s_valid = false;
static uint32_t s_save_count = 0U;

static uint32_t crc32_ieee(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static void fill_motor(motor_persist_t *d, const MotorRuntime *m) {
    memset(d, 0, sizeof(*d));
    d->pole_pairs = m->pole_pairs;
    d->sensor_mode = m->sensor_mode;
    d->sensor_request_mode = m->sensor_request_mode;
    d->encoder_inverted = m->encoder.inverted ? 1U : 0U;
    d->encoder_cpr = m->encoder.cpr;
    d->encoder_elec_offset_u16 = m->encoder.elec_offset_u16;
    d->hall_offset_u16 = m->hall_offset_u16;
    memcpy(d->foc_hall_table, m->foc_hall_table, sizeof(d->foc_hall_table));
    memcpy(d->hall_angle_u16, m->hall_angle_u16, sizeof(d->hall_angle_u16));
    d->current_kp = m->current_kp;
    d->current_ki = m->current_ki;
    d->speed_kp = m->speed_pid.kp;
    d->speed_ki = m->speed_pid.ki;
    d->speed_kd = m->speed_pid.kd;
}

static void apply_motor(MotorRuntime *m, const motor_persist_t *s) {
    if (s->pole_pairs >= 1U && s->pole_pairs <= SENSOR_DETECT_MAX_POLE_PAIRS) {
        m->pole_pairs = s->pole_pairs;
    }
    if (s->encoder_cpr > 0U && m->id == MOTOR_LEFT) m->encoder.cpr = s->encoder_cpr;
    m->encoder.inverted = s->encoder_inverted != 0U;
    m->encoder.elec_offset_u16 = s->encoder_elec_offset_u16;
    m->hall_offset_u16 = s->hall_offset_u16;
    memcpy(m->foc_hall_table, s->foc_hall_table, sizeof(m->foc_hall_table));
    memcpy(m->hall_angle_u16, s->hall_angle_u16, sizeof(m->hall_angle_u16));
    for (uint8_t k = 0U; k < 8U; k++) {
        m->hall_table[k] = (s->foc_hall_table[k] == 255U) ? -1 : (int8_t)((s->foc_hall_table[k] * 6U) / 200U);
    }
    if (s->current_kp > 0.0f) m->current_kp = s->current_kp;
    if (s->current_ki > 0.0f) m->current_ki = s->current_ki;
    /* Keep fast-loop fixed-point gains coherent with the restored float gains. */
    m->current_kp_q15 = (int32_t)((m->current_kp * FOC_CURRENT_Q_BASE_A /
                                   FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    m->current_ki_dt_q15 = (int32_t)((m->current_ki * FOC_DT_S *
                                      FOC_CURRENT_Q_BASE_A / FOC_VOLTAGE_Q_BASE_V) * 32768.0f);
    if (s->speed_kp >= 0.0f) m->speed_pid.kp = s->speed_kp;
    if (s->speed_ki >= 0.0f) m->speed_pid.ki = s->speed_ki;
    if (s->speed_kd >= 0.0f) m->speed_pid.kd = s->speed_kd;

    if (m->id == MOTOR_LEFT && m->encoder.cpr > 0U) {
        m->encoder.phase_per_count_q16 = (uint32_t)((((uint64_t)m->pole_pairs * 65536ULL) << 16) / m->encoder.cpr);
    }

    uint8_t mode = s->sensor_mode;
    if (mode != SENSOR_MODE_HALL && mode != SENSOR_MODE_ENCODER) mode = SENSOR_MODE_HALL;
    if (m->id == MOTOR_RIGHT && mode == SENSOR_MODE_ENCODER) mode = SENSOR_MODE_HALL;
    m->sensor_mode = mode;
    m->sensor_request_mode = (s->sensor_request_mode <= SENSOR_MODE_ENCODER) ? s->sensor_request_mode : mode;
    motor_hw_configure_sensor(m, mode);
    if (mode == SENSOR_MODE_HALL) motor_hall_edge_isr(m);
}

static const config_record_t *slot_record(uint32_t slot) {
    return (const config_record_t *)(CFG_FLASH_ADDR + slot * CFG_SLOT_SIZE);
}

static bool record_valid(const config_record_t *r) {
    if (r == NULL || r->magic != CFG_MAGIC || r->version != CFG_VERSION ||
        r->payload_len != sizeof(config_payload_t)) {
        return false;
    }
    return crc32_ieee((const uint8_t *)&r->payload, sizeof(r->payload)) == r->crc32;
}

static int32_t find_best_slot(void) {
    int32_t best = -1;
    uint32_t best_seq = 0U;
    for (uint32_t slot = 0U; slot < CFG_SLOT_COUNT; slot++) {
        const config_record_t *r = slot_record(slot);
        if (record_valid(r) && (best < 0 || r->sequence > best_seq)) {
            best = (int32_t)slot;
            best_seq = r->sequence;
        }
    }
    return best;
}

static bool slot_is_fully_erased(uint32_t slot) {
    const uint32_t *w = (const uint32_t *)(CFG_FLASH_ADDR + slot * CFG_SLOT_SIZE);
    for (uint32_t off = 0U; off < CFG_SLOT_SIZE; off += 4U) {
        if (w[off / 4U] != 0xFFFFFFFFUL) return false;
    }
    return true;
}

static int32_t find_erased_slot(void) {
    for (uint32_t slot = 0U; slot < CFG_SLOT_COUNT; slot++) {
        /* A power-failed uncommitted slot has magic==0xFFFFFFFF but may have
         * programmed payload bits. Never try to reuse such a slot. */
        if (slot_is_fully_erased(slot)) return (int32_t)slot;
    }
    return -1;
}

bool config_store_load_apply(void) {
    int32_t slot = find_best_slot();
    if (slot < 0) {
        s_valid = false;
        return false;
    }

    const config_record_t *r = slot_record((uint32_t)slot);
    apply_motor(&g_motor_left, &r->payload.motor[0]);
    apply_motor(&g_motor_right, &r->payload.motor[1]);
    vesc_timeout_configure(r->payload.timeout_ms,
                           (float)r->payload.timeout_brake_mA / 1000.0f);
    s_save_count = r->sequence;
    s_valid = true;
    return true;
}

static HAL_StatusTypeDef erase_config_page(void) {
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CFG_FLASH_ADDR;
    erase.NbPages = 1U;
    return HAL_FLASHEx_Erase(&erase, &page_error);
}

static HAL_StatusTypeDef program_halfword(uint32_t addr, uint16_t value) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, value);
}

static HAL_StatusTypeDef write_record_commit_last(uint32_t addr,
                                                   const config_record_t *r) {
    const uint8_t *src = (const uint8_t *)r;
    HAL_StatusTypeDef st = HAL_OK;

    /* Write everything except magic first. CRC/version/sequence/payload are
     * therefore present before the slot can ever be considered committed. */
    for (uint32_t off = 4U; off < sizeof(*r); off += 2U) {
        uint16_t hw = src[off];
        if ((off + 1U) < sizeof(*r)) hw |= (uint16_t)src[off + 1U] << 8;
        st = program_halfword(addr + off, hw);
        if (st != HAL_OK) return st;
    }

    /* Atomic-ish commit marker: magic is programmed last. A power loss before
     * these two halfwords leaves the slot invalid while the prior record stays. */
    st = program_halfword(addr + 0U, (uint16_t)(r->magic & 0xFFFFU));
    if (st != HAL_OK) return st;
    return program_halfword(addr + 2U, (uint16_t)(r->magic >> 16));
}

bool config_store_save_all(void) {
    /* Same safety principle as upstream conf_general: do not erase/program
     * flash while a motor is energised. */
    motor_stop(&g_motor_left);
    motor_stop(&g_motor_right);
    motor_hw_gate_global(false);
    if (osKernelGetState() == osKernelRunning) osDelay(5U);

    config_record_t r;
    memset(&r, 0, sizeof(r));
    r.magic = CFG_MAGIC;
    r.version = CFG_VERSION;
    r.payload_len = sizeof(config_payload_t);
    r.sequence = s_save_count + 1U;
    r.payload.timeout_ms = vesc_timeout_get_timeout_ms();
    r.payload.timeout_brake_mA = (int32_t)(vesc_timeout_get_brake_current() * 1000.0f);
    fill_motor(&r.payload.motor[0], &g_motor_left);
    fill_motor(&r.payload.motor[1], &g_motor_right);
    r.crc32 = crc32_ieee((const uint8_t *)&r.payload, sizeof(r.payload));

    /* F1 executes code from flash; prevent the fast ADC/FOC ISR from running
     * while flash is busy. Power stage is already hard-gated off above. */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    DMA1->IFCR = DMA_IFCR_CGIF1;

    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) {
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        return false;
    }

    int32_t slot = find_erased_slot();
    if (slot < 0) {
        st = erase_config_page();
        slot = 0;
    }

    if (st == HAL_OK) {
        uint32_t addr = CFG_FLASH_ADDR + (uint32_t)slot * CFG_SLOT_SIZE;
        st = write_record_commit_last(addr, &r);
    }

    (void)HAL_FLASH_Lock();
    DMA1->IFCR = DMA_IFCR_CGIF1;
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    if (st == HAL_OK) {
        const config_record_t *verify = slot_record((uint32_t)slot);
        if (record_valid(verify) && verify->sequence == r.sequence) {
            s_valid = true;
            s_save_count = r.sequence;
            return true;
        }
    }
    return false;
}

bool config_store_valid(void) { return s_valid; }
uint32_t config_store_save_count(void) { return s_save_count; }
