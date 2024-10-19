#ifndef _TTSCHEDULER_H_
#define _TTSCHEDULER_H_

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*! @brief Global macro for function of count task run time
 * 1 - enable, 0 - disable
 */
#define CONFIG_RUN_TIME_INFO (1)

/*! @brief Global macro for function to provide high resolution timer for count
 * task run time 1 - enable, 0 - disable
 */
#define USE_LONG_LONG_RUN_TIME_COUNTER (0)

/*! @brief percision of task timer(per second)
 */
#define CORE_TIMER_DIVIDE_TIMES_OF_SECOND (1000)

/*! @brief percision of task run time count timer(per second)
 */
#define INFO_TIMER_DIVIDE_TIMES_OF_SECOND (1000 * 100)

#define TTSCHEDULER_MAX_TASK_NAME_LEN (16)
#define TTSCHEDULER_MAX_TASKS (30)

#define LONG_LONG_RUN_TIMER (CONFIG_RUN_TIME_INFO && USE_LONG_LONG_RUN_TIME_COUNTER)
#if LONG_LONG_RUN_TIMER
    #include "timer/timer.h"
    #define PORT_GET_RUN_TIME_COUNTER_VALUE() ((uint32_t)(Timer::get_runningTime() * 100000))
#else
    #define PORT_GET_RUN_TIME_COUNTER_VALUE() (uint64_t)(0)
#endif

typedef void (*tttask_func_t)(void *);

/*! @brief init the co_scheduler(task array)
 *
 */
void ttscheduler_init(void);

/*! @brief co_scheduler kernel update function
 * it must be called by kernel timer update function to work
 */
void ttscheduler_update(void);

/*! @brief dispatch all task
 * it must be called by while(1) of main function
 */
void ttscheduler_dispatch_tasks(void);

/*! @brief delete task
 * @return uint8_t 0 - false, 1 - true
 */
uint8_t ttscheduler_delete_task(uint8_t index_task);

/*! @brief turn bytes to double value
 * @param name task name string
 * @param code task function entry
 * @param params task function parameter pointer
 * @param period task function running period, in units of
 * co_scheduler_update() call cycle
 * @param initial_delay task function delay period, it only work to the first
 * time
 * @return uint8_t index of task index
 */
uint8_t ttscheduler_add_task(const char *const name,
                              tttask_func_t code,
                              void *const params,
                              uint32_t period,
                              uint32_t initial_delay);

/*! @brief idle task, inner call
 */
void ttscheduler_idle(void *params);

#if CONFIG_RUN_TIME_INFO
/*! @brief get all task running state
 * @param write_buf output state string buffer
 */
void ttscheduler_get_runtime_stats(char *write_buf);

/*! @brief get task period, in units of task run time count timer percision
 * @param index_task task index
 */
uint32_t ttscheduler_get_period_time(uint8_t index_task);

/*! @brief get cpu load
 * @return uint32_t load percentage
 */
uint32_t ttscheduler_get_cpu_load(void);
#endif

#ifdef __cplusplus
}
#endif

#endif  // co_scheduler_H
