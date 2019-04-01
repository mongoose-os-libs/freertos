/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#pragma once

#include <stdbool.h>

#include "FreeRTOS.h"

#include "mgos_init.h"

#ifdef __cplusplus
extern "C" {
#endif

void mgos_freertos_run_mgos_task(bool start_scheduler);

extern enum mgos_init_result mgos_freertos_pre_init(void);

#ifdef __cplusplus
}
#endif
