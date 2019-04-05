/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#pragma once

#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "mgos_init.h"

#ifdef __cplusplus
extern "C" {
#endif

void mgos_freertos_run_mgos_task(bool start_scheduler);

extern enum mgos_init_result mgos_freertos_pre_init(void);

void mgos_freertos_core_dump(void);

// This function extracts registers from the task's stack frame
// and populates GDB stack frame.
size_t mgos_freertos_extract_regs(StackType_t *sp, void *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif
