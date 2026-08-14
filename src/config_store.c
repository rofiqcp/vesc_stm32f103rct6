#include "config_store.h"
#include "vesc_config.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <string.h>

#define CFG_REGION_ADDR      0x0803E000UL
#define CFG_FLASH_PAGE_SIZE  2048UL
#define CFG_PAGE_COUNT       4U
#define CFG_MAGIC            0x56314346UL /* V11 config format */
#define CFG_VERSION          0x000BU
#define CFG_PAYLOAD_LEN      (2U*VESC6_MCCONF_WIRE_SIZE + VESC6_APPCONF_WIRE_SIZE)

typedef struct {
    uint8_t mc_left[VESC6_MCCONF_WIRE_SIZE];
    uint8_t mc_right[VESC6_MCCONF_WIRE_SIZE];
    uint8_t app[VESC6_APPCONF_WIRE_SIZE];
} config_payload_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t payload_len;
    uint32_t sequence;
    uint32_t crc32;
    config_payload_t payload;
} config_record_t;

_Static_assert(sizeof(config_payload_t)==CFG_PAYLOAD_LEN,"wire payload layout changed");
_Static_assert(sizeof(config_record_t)<CFG_FLASH_PAGE_SIZE,"VESC config must fit one F1 flash page");

static bool s_valid=false;
static uint32_t s_save_count=0U;
/* Large (~1.5 KiB) flash staging record is static, never placed on a small RTOS thread stack. */
static config_record_t s_stage;

static uint32_t crc32_ieee(const uint8_t *p,uint32_t n){
    uint32_t c=0xFFFFFFFFUL;for(uint32_t i=0;i<n;i++){c^=p[i];for(uint8_t b=0;b<8;b++){uint32_t m=(uint32_t)-(int32_t)(c&1U);c=(c>>1)^(0xEDB88320UL&m);}}return ~c;
}
static const config_record_t *page_rec(uint32_t page){return (const config_record_t *)(CFG_REGION_ADDR+page*CFG_FLASH_PAGE_SIZE);}
static bool rec_valid(const config_record_t *r){return r&&r->magic==CFG_MAGIC&&r->version==CFG_VERSION&&r->payload_len==CFG_PAYLOAD_LEN&&crc32_ieee((const uint8_t*)&r->payload,sizeof(r->payload))==r->crc32;}
static int best_page(void){int best=-1;uint32_t seq=0;for(uint32_t p=0;p<CFG_PAGE_COUNT;p++){const config_record_t*r=page_rec(p);if(rec_valid(r)&&(best<0||r->sequence>seq)){best=(int)p;seq=r->sequence;}}return best;}

bool config_store_load_apply(void){
    int p=best_page();if(p<0){s_valid=false;return false;}const config_record_t*r=page_rec((uint32_t)p);
    if(!vesc_config_import_wire(r->payload.mc_left,r->payload.mc_right,r->payload.app)){s_valid=false;return false;}
    s_save_count=r->sequence;s_valid=true;return true;
}
static HAL_StatusTypeDef erase_page(uint32_t p){FLASH_EraseInitTypeDef e={0};uint32_t err=0;e.TypeErase=FLASH_TYPEERASE_PAGES;e.PageAddress=CFG_REGION_ADDR+p*CFG_FLASH_PAGE_SIZE;e.NbPages=1;return HAL_FLASHEx_Erase(&e,&err);}
static HAL_StatusTypeDef phw(uint32_t a,uint16_t v){return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,a,v);}
static HAL_StatusTypeDef write_commit(uint32_t addr,const config_record_t*r){
    const uint8_t*s=(const uint8_t*)r;for(uint32_t o=4;o<sizeof(*r);o+=2){uint16_t h=s[o];if(o+1<sizeof(*r))h|=(uint16_t)s[o+1]<<8;HAL_StatusTypeDef st=phw(addr+o,h);if(st!=HAL_OK)return st;}
    HAL_StatusTypeDef st=phw(addr,(uint16_t)(r->magic&0xFFFFU));if(st!=HAL_OK)return st;return phw(addr+2,(uint16_t)(r->magic>>16));
}
bool config_store_save_all(void){
    motor_stop(&g_motor_left);motor_stop(&g_motor_right);motor_hw_gate_global(false);if(osKernelGetState()==osKernelRunning)osDelay(5);
    memset(&s_stage,0,sizeof(s_stage));s_stage.magic=CFG_MAGIC;s_stage.version=CFG_VERSION;s_stage.payload_len=CFG_PAYLOAD_LEN;s_stage.sequence=s_save_count+1U;
    vesc_config_export_wire(s_stage.payload.mc_left,s_stage.payload.mc_right,s_stage.payload.app);s_stage.crc32=crc32_ieee((const uint8_t*)&s_stage.payload,sizeof(s_stage.payload));
    int cur=best_page();uint32_t next=(cur<0)?0U:((uint32_t)(cur+1)%CFG_PAGE_COUNT);
    /* STM32F1 flash erase/program stalls code fetch. Current loop is stopped and
       its DMA IRQ is masked; UART RX DMA remains circular so received bytes are
       not lost and are drained after the short flash transaction. */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);DMA1->IFCR=DMA_IFCR_CGIF1;
    HAL_StatusTypeDef st=HAL_FLASH_Unlock();if(st==HAL_OK)st=erase_page(next);if(st==HAL_OK)st=write_commit(CFG_REGION_ADDR+next*CFG_FLASH_PAGE_SIZE,&s_stage);(void)HAL_FLASH_Lock();
    DMA1->IFCR=DMA_IFCR_CGIF1;HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    if(st==HAL_OK){const config_record_t*v=page_rec(next);if(rec_valid(v)&&v->sequence==s_stage.sequence){s_valid=true;s_save_count=s_stage.sequence;return true;}}return false;
}
bool config_store_valid(void){return s_valid;}uint32_t config_store_save_count(void){return s_save_count;}
