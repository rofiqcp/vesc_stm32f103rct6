#include "fault.h"
#include "motor_tasks.h"
void fault_signal(motor_id_t id){motor_threads_fault_signal(id);}
