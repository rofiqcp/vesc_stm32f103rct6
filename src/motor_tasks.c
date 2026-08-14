#include "motor_tasks.h"
#include "motor_control.h"
#include "motor_hw.h"
#include "foc_control.h"
#include "sensor_detect.h"
#include "debug_sample.h"
#include "telemetry.h"
#include "vesc_comm.h"
#include "cmsis_os2.h"
#include "app_config.h"

/* CMSIS-RTOS2 equivalents of the VESC mc_interface/mcpwm_foc threads.
 * Names deliberately match the upstream semantic thread names requested. */
static osThreadId_t timer_thread_tp;
static osThreadId_t pid_thread_tp;
static osThreadId_t sample_send_tp;
static osThreadId_t fault_stop_tp;
static osThreadId_t stat_thread_tp;

static void timer_thread(void *arg);
static void pid_thread(void *arg);
static void sample_send_thread(void *arg);
static void fault_stop_thread(void *arg);
static void stat_thread(void *arg);

void motor_threads_fault_signal(motor_id_t id) {
    if (fault_stop_tp != NULL) (void)osThreadFlagsSet(fault_stop_tp,1UL<<(uint32_t)id);
}
void motor_threads_sample_signal(void) {
    if (sample_send_tp != NULL) (void)osThreadFlagsSet(sample_send_tp,1UL);
}

static void timer_thread(void *arg) {
    (void)arg;
    uint32_t next=osKernelGetTickCount(); uint32_t blink=0U;
    for(;;) {
        next+=1U; uint32_t now=osKernelGetTickCount();
        sensor_detect_update_1khz(&g_motor_left,now); sensor_detect_update_1khz(&g_motor_right,now);
        motor_slow_update_1khz(&g_motor_left,now); motor_slow_update_1khz(&g_motor_right,now);

        uint32_t pending=motor_take_pending_fault_mask();
        if(pending&(1UL<<MOTOR_LEFT)) motor_threads_fault_signal(MOTOR_LEFT);
        if(pending&(1UL<<MOTOR_RIGHT)) motor_threads_fault_signal(MOTOR_RIGHT);

        if(debug_sample_ready()) motor_threads_sample_signal();
        if(!g_motor_left.pwm_enabled&&!g_motor_right.pwm_enabled) motor_hw_gate_global(false);

        if(++blink>=250U){blink=0U;static bool led=false;led=!led;motor_hw_led(led);}
        osDelayUntil(next);
    }
}

static void pid_thread(void *arg) {
    (void)arg; uint32_t next=osKernelGetTickCount();
    for(;;){next+=1U;motor_pid_update_1khz(&g_motor_left);motor_pid_update_1khz(&g_motor_right);osDelayUntil(next);}
}

static void sample_send_thread(void *arg) {
    (void)arg;
    for(;;){
        uint32_t f=osThreadFlagsWait(1UL,osFlagsWaitAny,osWaitForever);if(f&osFlagsError)continue;
        if(debug_sample_ready()) { vesc_comm_send_sample_buffer(debug_sample_data(),debug_sample_count()); debug_sample_mark_sent(); }
    }
}

static void fault_stop_thread(void *arg) {
    (void)arg;
    const uint32_t mask=(1UL<<MOTOR_LEFT)|(1UL<<MOTOR_RIGHT);
    for(;;){
        uint32_t f=osThreadFlagsWait(mask,osFlagsWaitAny,osWaitForever);if(f&osFlagsError)continue;
        if(f&(1UL<<MOTOR_LEFT)) motor_hw_set_pwm_enabled(&g_motor_left,false);
        if(f&(1UL<<MOTOR_RIGHT)) motor_hw_set_pwm_enabled(&g_motor_right,false);
        motor_hw_buzzer(true);osDelay(40U);motor_hw_buzzer(false);
    }
}

static void stat_thread(void *arg) {
    (void)arg; uint32_t next=osKernelGetTickCount();
    for(;;){
        next+=STAT_PERIOD_MS;
        telemetry_update_100hz();
        vesc_comm_periodic_100hz();
        osDelayUntil(next);
    }
}

void motor_threads_init(void) {
    const osThreadAttr_t timer_attr={.name="timer_thread",.priority=osPriorityAboveNormal,.stack_size=512U};
    const osThreadAttr_t pid_attr={.name="pid_thread",.priority=osPriorityHigh,.stack_size=512U};
    const osThreadAttr_t sample_attr={.name="sample_send_thread",.priority=osPriorityBelowNormal,.stack_size=512U};
    const osThreadAttr_t fault_attr={.name="fault_stop_thread",.priority=osPriorityHigh,.stack_size=512U};
    const osThreadAttr_t stat_attr={.name="stat_thread",.priority=osPriorityNormal,.stack_size=512U};
    timer_thread_tp=osThreadNew(timer_thread,NULL,&timer_attr);
    pid_thread_tp=osThreadNew(pid_thread,NULL,&pid_attr);
    sample_send_tp=osThreadNew(sample_send_thread,NULL,&sample_attr);
    fault_stop_tp=osThreadNew(fault_stop_thread,NULL,&fault_attr);
    stat_thread_tp=osThreadNew(stat_thread,NULL,&stat_attr);
    (void)timer_thread_tp;(void)pid_thread_tp;(void)sample_send_tp;(void)fault_stop_tp;(void)stat_thread_tp;
}
