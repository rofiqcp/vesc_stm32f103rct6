#pragma once
#include "encoder_datatype.h"
#include "encoder_cfg.h"

bool enc_abi_init(MotorRuntime *m, const encoder_cfg_ABI_t *cfg);
void enc_abi_deinit(MotorRuntime *m);
float enc_abi_read_deg(MotorRuntime *m);
void enc_abi_set_deg(MotorRuntime *m, float deg);
bool enc_abi_index_found(const MotorRuntime *m);
