#ifndef APP_PX4_DEBUG_H_
#define APP_PX4_DEBUG_H_

#include "./app_main.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/sem.h>
#include <drivers/drv_hrt.h>

/**
 * 1. test px4 common: task, log, (sem, time)(directly from posix layer)
 * 2. test px4 px4_work_queue module (hrtimer+container)
 * 3. test px4 lib cdev, perf
 */
#define PX4_TEST_ITEM      (1)

void app_px4_debug_init();

#endif
