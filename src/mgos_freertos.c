/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos_freertos.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "common/cs_dbg.h"

#include "mgos_app.h"
#include "mgos_core_dump.h"
#include "mgos_debug_internal.h"
#include "mgos_hal.h"
#include "mgos_init_internal.h"
#include "mgos_mongoose_internal.h"
#ifdef MGOS_HAVE_OTA_COMMON
#include "mgos_ota.h"
#endif
#include "mgos_system.h"
#include "mgos_uart_internal.h"
#include "mgos_utils.h"

extern const char *build_version, *build_id;
extern const char *mg_build_version, *mg_build_id;

static QueueHandle_t s_main_queue;
static SemaphoreHandle_t s_mgos_mux;
#if MGOS_BG_TASK_PRIORITY > 0
static QueueHandle_t s_bg_queue;
#else
#define s_bg_queue s_main_queue
#endif

#if MGOS_BG_TASK_PRIORITY >= MGOS_TASK_PRIORITY
#error Background task's priority must be less than the main task's
#endif
#if MGOS_BG_TASK_PRIORITY >= configTIMER_TASK_PRIORITY
#error Background task's priority must be less than the timer task's
#endif

/* ESP32 has a slightly different FreeRTOS API */
#if CS_PLATFORM == CS_P_ESP32
#include <sdkconfig.h>

static portMUX_TYPE s_poll_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define ENTER_CRITICAL() portENTER_CRITICAL(&s_poll_spinlock)
#define EXIT_CRITICAL() portEXIT_CRITICAL(&s_poll_spinlock)
#define YIELD_FROM_ISR(should_yield)        \
  {                                         \
    if (should_yield) portYIELD_FROM_ISR(); \
  }
#else
#define ENTER_CRITICAL() portENTER_CRITICAL()
#define EXIT_CRITICAL() portEXIT_CRITICAL()
#define YIELD_FROM_ISR(should_yield) portYIELD_FROM_ISR(should_yield)
#endif

/* Ticks until next poll. */
static volatile TickType_t s_poll_timeout_ticks = 0;
/* Next poll's absolute time. */
static volatile TickType_t s_poll_deadline_ticks = 0;
static void mgos_mg_do_poll(void);

struct mgos_event {
  mgos_cb_t cb;
  void *arg;
};

static enum mgos_init_result mgos_init2(void) {
  enum mgos_init_result r;

  cs_log_set_level(MGOS_EARLY_DEBUG_LEVEL);

  setvbuf(stdout, NULL, _IOLBF, 256);
  setvbuf(stderr, NULL, _IOLBF, 256);
  fputs("\n\n", stderr);

  if (strcmp(MGOS_APP, "mongoose-os") != 0) {
    LOG(LL_INFO, ("%s %s (%s)", MGOS_APP, build_version, build_id));
  }
  LOG(LL_INFO, ("Mongoose OS %s (%s)", mg_build_version, mg_build_id));
  LOG(LL_INFO, ("CPU: %d MHz, FreeRTOS %d.%d.%d, heap: %u total, %u free",
                (int) (mgos_get_cpu_freq() / 1000000), tskKERNEL_VERSION_MAJOR,
                tskKERNEL_VERSION_MINOR, tskKERNEL_VERSION_BUILD,
                mgos_get_heap_size(), mgos_get_free_heap_size()));
#ifdef _NEWLIB_VERSION
  LOG(LL_INFO, ("Newlib %s", _NEWLIB_VERSION));
#endif

  r = mgos_freertos_pre_init();
  if (r != MGOS_INIT_OK) return r;

  r = mgos_init();

  return r;
}

void mgos_task(void *arg UNUSED_ARG) {
  struct mgos_event e;

  mgos_wdt_enable();
  mgos_wdt_set_timeout(MGOS_EARLY_WDT_TIMEOUT);

  enum mgos_init_result r = mgos_init2();
  bool success = (r == MGOS_INIT_OK);

  if (!success) {
    LOG(LL_ERROR, ("MGOS init failed: %d", r));
  }

#ifdef MGOS_HAVE_OTA_COMMON
  mgos_ota_boot_finish(success, mgos_ota_is_first_boot());
#endif

  if (!success) {
    /* Arbitrary delay to make potential reboot loop less tight. */
    mgos_usleep(500000);
    mgos_system_restart();
  }

  while (true) {
    if (xQueueReceive(s_main_queue, &e, s_poll_timeout_ticks)) {
      e.cb(e.arg);
      /* Check if a poll is due. */
      const int diff = s_poll_deadline_ticks - xTaskGetTickCount();
      if (diff > 0) {
        s_poll_timeout_ticks = diff;
        continue;
      }
    }
    mgos_mg_do_poll();
  }
}

void mgos_bg_task(void *arg UNUSED_ARG) {
  struct mgos_event e;
  while (true) {
    while (xQueueReceive(s_bg_queue, &e, portMAX_DELAY)) {
      e.cb(e.arg);
    }
  }
}

IRAM bool mgos_invoke_cb(mgos_cb_t cb, void *arg, uint32_t flags) {
  struct mgos_event e = {.cb = cb, .arg = arg};
  QueueHandle_t q =
      ((flags & MGOS_INVOKE_CB_F_BG_TASK) ? s_bg_queue : s_main_queue);
  if ((flags & MGOS_INVOKE_CB_F_FROM_ISR)) {
    BaseType_t should_yield = false;
    if (!xQueueSendToBackFromISR(q, &e, &should_yield)) {
      return false;
    }
    YIELD_FROM_ISR(should_yield);
    return true;
  } else {
    return xQueueSendToBack(q, &e, 10);
  }
}

static void mgos_mg_do_poll(void) {
  if (mongoose_poll(0) == 0) {
    /* Nothing is happening now, see when next timer is due. */
    int timeout_ms = 0;
    double min_timer = mg_mgr_min_timer(mgos_get_mgr());
    if (min_timer > 0) {
      /* Note: timeout_ms can get negative if a timer is past due. That's ok. */
      timeout_ms = (int) ((min_timer - mg_time()) * 1000.0);
      if (timeout_ms < 0) {
        timeout_ms = 0; /* Now */
      } else if (timeout_ms > MGOS_MONGOOSE_MAX_POLL_SLEEP_MS) {
        timeout_ms = MGOS_MONGOOSE_MAX_POLL_SLEEP_MS;
      }
    } else {
      timeout_ms = MGOS_MONGOOSE_MAX_POLL_SLEEP_MS;
    }
    s_poll_timeout_ticks = (timeout_ms / portTICK_PERIOD_MS);
    /* Wraparound? Some extra polls, should be fine. */
    s_poll_deadline_ticks = xTaskGetTickCount() + s_poll_timeout_ticks;
  } else {
    /* Things are happening, we need another poll ASAP. */
    s_poll_timeout_ticks = 0;
    s_poll_deadline_ticks = 0;
  }
}

static void mgos_mg_poll_cb(void *arg UNUSED_ARG) {
  /* Poll will be executed immediately after we return. */
  s_poll_deadline_ticks = 0;
}

IRAM void mongoose_schedule_poll(bool from_isr) {
  /* If queue is not empty, this will be sufficient. */
  s_poll_timeout_ticks = 0;
  s_poll_deadline_ticks = 0;
  /* If queue is empty, throw a callback on it. */
  bool is_empty;
  if (from_isr) {
    is_empty = (uxQueueMessagesWaitingFromISR(s_main_queue) == 0);
  } else {
    is_empty = (uxQueueMessagesWaiting(s_main_queue) == 0);
  }
  if (is_empty) {
    mgos_invoke_cb(mgos_mg_poll_cb, NULL, from_isr);
  }
}

void mg_lwip_mgr_schedule_poll(struct mg_mgr *mgr UNUSED_ARG) {
  mongoose_schedule_poll(false /* from_isr */);
}

#if configSUPPORT_STATIC_ALLOCATION
#if CS_PLATFORM != CS_P_ESP32
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif // CS_PLATFORM != CS_P_ESP32

void mgos_freertos_run_mgos_task(bool start_scheduler) {
#define STACK_SIZE (MGOS_TASK_STACK_SIZE_BYTES / sizeof(StackType_t))
  static StaticTask_t mgos_task_tcb;
  static StackType_t mgos_task_stack[STACK_SIZE];
  s_main_queue =
      xQueueCreate(MGOS_TASK_QUEUE_LENGTH, sizeof(struct mgos_event));
#if MGOS_BG_TASK_PRIORITY > 0
  static StaticTask_t mgos_bg_task_tcb;
  static StackType_t mgos_bg_task_stack[STACK_SIZE];
  s_bg_queue = xQueueCreate(MGOS_TASK_QUEUE_LENGTH, sizeof(struct mgos_event));
#endif

  mgos_uart_init();
  mgos_debug_init();
  mgos_debug_uart_init();

  mgos_app_preinit();

  mgos_cd_register_section_writer(mgos_freertos_core_dump);

  s_mgos_mux = xSemaphoreCreateRecursiveMutex();
#if CS_PLATFORM == CS_P_ESP32 && !defined(CONFIG_FREERTOS_UNICORE)
  // On ESP32 in SMP mode pin our tasks to core 1 (app cpu).
  // This is to avoid difficulties with interrupt allocation / deallocation:
  // https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/system/intr_alloc.html#multicore-issues
  xTaskCreateStaticPinnedToCore(
      mgos_task, "mgos", MGOS_TASK_STACK_SIZE_BYTES / MGOS_TASK_STACK_SIZE_UNIT,
      NULL, MGOS_TASK_PRIORITY, mgos_task_stack, &mgos_task_tcb, 1);
#if MGOS_BG_TASK_PRIORITY > 0
  xTaskCreateStaticPinnedToCore(
      mgos_bg_task, "mgos_bg",
      MGOS_TASK_STACK_SIZE_BYTES / MGOS_TASK_STACK_SIZE_UNIT, NULL,
      MGOS_BG_TASK_PRIORITY, mgos_bg_task_stack, &mgos_bg_task_tcb, 1);
#endif
#else
  xTaskCreateStatic(mgos_task, "mgos",
                    MGOS_TASK_STACK_SIZE_BYTES / MGOS_TASK_STACK_SIZE_UNIT,
                    NULL, MGOS_TASK_PRIORITY, mgos_task_stack, &mgos_task_tcb);
#if MGOS_BG_TASK_PRIORITY > 0
  xTaskCreateStatic(mgos_bg_task, "mgos_bg",
                    MGOS_TASK_STACK_SIZE_BYTES / MGOS_TASK_STACK_SIZE_UNIT,
                    NULL, MGOS_BG_TASK_PRIORITY, mgos_bg_task_stack,
                    &mgos_bg_task_tcb);
#endif
#endif
  if (start_scheduler) {
    vTaskStartScheduler();
    mgos_cd_puts("Scheduler failed to start!\n");
    mgos_dev_system_restart();
  }
}

#else

void mgos_freertos_run_mgos_task(bool start_scheduler) {
  mgos_app_preinit();

  s_main_queue =
      xQueueCreate(MGOS_TASK_QUEUE_LENGTH, sizeof(struct mgos_event));
  s_mgos_mux = xSemaphoreCreateRecursiveMutex();
  xTaskCreate(mgos_task, "mgos",
              MGOS_TASK_STACK_SIZE_BYTES / MGOS_TASK_STACK_SIZE_UNIT, NULL,
              MGOS_TASK_PRIORITY, NULL);
  if (start_scheduler) {
    vTaskStartScheduler();
    mgos_cd_puts("Scheduler failed to start!\n");
    mgos_dev_system_restart();
  }
}
#endif /* configSUPPORT_STATIC_ALLOCATION */

#ifndef MGOS_BOOT_BUILD
IRAM void mgos_ints_disable(void) {
  ENTER_CRITICAL();
}

IRAM void mgos_ints_enable(void) {
  EXIT_CRITICAL();
}

void mgos_lock(void) {
  xSemaphoreTakeRecursive(s_mgos_mux, portMAX_DELAY);
}

void mgos_unlock(void) {
  xSemaphoreGiveRecursive(s_mgos_mux);
}

IRAM struct mgos_rlock_type *mgos_rlock_create(void) {
  return (struct mgos_rlock_type *) xSemaphoreCreateRecursiveMutex();
}

IRAM void mgos_rlock(struct mgos_rlock_type *l) {
  if (l == NULL) return;
  xSemaphoreTakeRecursive((SemaphoreHandle_t) l, portMAX_DELAY);
}

IRAM void mgos_runlock(struct mgos_rlock_type *l) {
  if (l == NULL) return;
  xSemaphoreGiveRecursive((SemaphoreHandle_t) l);
}

IRAM void mgos_rlock_destroy(struct mgos_rlock_type *l) {
  if (l == NULL) return;
  vSemaphoreDelete((SemaphoreHandle_t) l);
}

#if CS_PLATFORM == CS_P_ESP32

IRAM int64_t mgos_uptime_micros(void) {
  return (int64_t) esp_timer_get_time();
}

#else /* All the ARM cores have SYSTICK */

#if defined(__arm__) || defined(__TI_COMPILER_VERSION__)
#define SYSTICK_VAL (*((volatile uint32_t *) 0xe000e018))
#define SYSTICK_RLD (*((volatile uint32_t *) 0xe000e014))
#else
#error Does not look like an ARM processor to me. Help!
#endif

#if CS_PLATFORM == CS_P_CC3200 || CS_PLATFORM == CS_P_CC3220
#define SystemCoreClockMHZ (SYS_CLK / 1000000)
#else
extern uint32_t SystemCoreClockMHZ;
#endif

IRAM int64_t mgos_uptime_micros(void) {
  static uint8_t num_overflows = 0;
  static uint32_t prev_tc = 0;
  uint32_t tc, frac;
  do {
    tc = xTaskGetTickCount();
    frac = (SYSTICK_RLD - SYSTICK_VAL);
    // If a tick happens in between, fraction calculation may be wrong.
  } while (xTaskGetTickCount() != tc);
  if (tc < prev_tc) num_overflows++;
  prev_tc = tc;
  return ((((int64_t) num_overflows) << 32) + tc) * portTICK_PERIOD_MS * 1000 +
         (int64_t)(frac / SystemCoreClockMHZ);
}

#endif /* CS_P_ESP32 */

IRAM void mgos_usleep(uint32_t usecs) {
  if (usecs < (1000000 / configTICK_RATE_HZ)) {
    (*mgos_nsleep100)(usecs * 10);
  } else {
    int64_t threshold = mgos_uptime_micros() + (int64_t) usecs;
    int ticks = usecs / (1000000 / configTICK_RATE_HZ);
    if (ticks > 0) vTaskDelay(ticks);
    while (mgos_uptime_micros() < threshold) {
    }
  }
}

IRAM void mgos_msleep(uint32_t msecs) {
  mgos_usleep(msecs * 1000);
}

#endif /* MGOS_BOOT_BUILD */

bool mgos_freertos_init(void) {
  return true;
}
