#include "ttscheduler.h"

#ifndef coTRUE
    #define coTRUE 1
#endif
#ifndef coFALSE
    #define coFALSE 0
#endif
#ifndef NULL
    #define NULL 0
#endif

typedef struct data
{
    char task_name[TTSCHEDULER_MAX_TASK_NAME_LEN];
    void (*p_task)(void *);
    void *parameters;
    uint32_t delay_step_cnt;
    uint32_t period_step_cnt;
#if LONG_LONG_RUN_TIMER
    uint64_t run_time_cnt;
#else
    uint32_t run_time_cnt;
#endif
    uint32_t period_time;
    uint32_t run_me;
} tt_task;

static tt_task idle_task;
static tt_task list_sch_tasks[TTSCHEDULER_MAX_TASKS];
static uint8_t list_sch_num = 0;

#if LONG_LONG_RUN_TIMER
static uint64_t total_run_time      = 0;
static uint64_t task_switch_in_time = 0;
#else
static uint32_t total_run_time = 0;
static uint32_t task_switch_in_time = 0;
#endif

void ttscheduler_idle(void *params) {}

#if CONFIG_RUN_TIME_INFO
static void ttscheduler_run_info(uint8_t index)
{
    total_run_time = PORT_GET_RUN_TIME_COUNTER_VALUE();

    if (total_run_time > task_switch_in_time)
    {
        list_sch_tasks[index].period_time = (total_run_time - task_switch_in_time);
        list_sch_tasks[index].run_time_cnt += list_sch_tasks[index].period_time;
    }
    task_switch_in_time = total_run_time;

    list_sch_tasks[index].run_me--;
}
static void ttscheduler_run_info_idle()
{
    total_run_time = PORT_GET_RUN_TIME_COUNTER_VALUE();

    if (total_run_time > task_switch_in_time)
    {
        idle_task.period_time = (total_run_time - task_switch_in_time);
        idle_task.run_time_cnt += idle_task.period_time;
    }
    task_switch_in_time = total_run_time;
}
static char *write_name_to_buff(char *pc_buffer, const char *pc_task_name)
{
    uint32_t x;

    strcpy(pc_buffer, pc_task_name);

    for (x = strlen(pc_buffer); x < (uint32_t)(TTSCHEDULER_MAX_TASK_NAME_LEN - 1); x++)
        pc_buffer[x] = ' ';

    pc_buffer[x] = 0x00;

    return &(pc_buffer[x]);
}
uint32_t ttscheduler_get_period_time(uint8_t index_task)
{
    return list_sch_tasks[index_task].period_time;
}
uint32_t ttscheduler_get_cpu_load()
{
    uint8_t i;
    uint32_t total_time;
    uint32_t res   = 0;
    total_run_time = PORT_GET_RUN_TIME_COUNTER_VALUE();

    total_time = total_run_time / 100;

    for (i = 0; i < TTSCHEDULER_MAX_TASKS; i++)
    {
        if (list_sch_tasks[i].p_task != NULL)
        {
            res += list_sch_tasks[i].run_time_cnt / total_time;
        }
    }

    return res;
}
#endif

void ttscheduler_init()
{
    uint8_t index, j;
    for (index = 0; index < TTSCHEDULER_MAX_TASKS; index++)
    {
        for (j = 0; j < TTSCHEDULER_MAX_TASK_NAME_LEN; j++)
            list_sch_tasks[index].task_name[j] = 0x00;
        list_sch_tasks[index].p_task          = NULL;
        list_sch_tasks[index].parameters      = NULL;
        list_sch_tasks[index].delay_step_cnt  = 0;
        list_sch_tasks[index].period_step_cnt = 0;
        list_sch_tasks[index].run_time_cnt    = 0;
        list_sch_tasks[index].period_time     = 0;
        list_sch_tasks[index].run_me          = coFALSE;
    }

    idle_task.task_name[0]    = 'i';
    idle_task.task_name[1]    = 'd';
    idle_task.task_name[2]    = 'l';
    idle_task.task_name[3]    = 'e';
    idle_task.task_name[4]    = '\0';
    idle_task.p_task          = ttscheduler_idle;
    idle_task.parameters      = NULL;
    idle_task.delay_step_cnt  = 0;
    idle_task.period_step_cnt = 0;
    idle_task.run_time_cnt    = 0;
    idle_task.period_time     = 0;
    idle_task.run_me          = coFALSE;
}

uint8_t ttscheduler_add_task(const char *const name,
                              tttask_func_t task_code,
                              void *const params,
                              uint32_t period,
                              uint32_t initial_delay)
{
    uint8_t list_task_index;
    uint8_t i;

    if (list_sch_num >= TTSCHEDULER_MAX_TASKS || task_code == NULL || name == NULL)
        return TTSCHEDULER_MAX_TASKS;

    for (list_task_index = 0; list_task_index < TTSCHEDULER_MAX_TASKS; list_task_index++)
        if (list_sch_tasks[list_task_index].p_task == NULL)
            break;

    for (i = 0; i < TTSCHEDULER_MAX_TASK_NAME_LEN; i++)
    {
        list_sch_tasks[list_task_index].task_name[i] = name[i];
        if (list_sch_tasks[list_task_index].task_name[i] == 0x00)
            break;
    }
    list_sch_tasks[list_task_index].task_name[TTSCHEDULER_MAX_TASK_NAME_LEN - 1]  = '\0';
    list_sch_tasks[list_task_index].p_task                                        = task_code;
    list_sch_tasks[list_task_index].parameters                                    = params;
    list_sch_tasks[list_task_index].period_step_cnt                               = period;
    list_sch_tasks[list_task_index].delay_step_cnt = period + initial_delay;
    list_sch_tasks[list_task_index].run_time_cnt   = 0;
    list_sch_tasks[list_task_index].period_time    = 0;
    list_sch_tasks[list_task_index].run_me         = coFALSE;

    list_sch_num++;

    return list_task_index;
}

uint8_t ttscheduler_delete_task(uint8_t index_task)
{
    uint8_t j;
    if (index_task >= TTSCHEDULER_MAX_TASKS)
        return coFALSE;

    for (j = 0; j < TTSCHEDULER_MAX_TASK_NAME_LEN; j++)
        list_sch_tasks[index_task].task_name[j] = 0x00;
    list_sch_tasks[index_task].p_task          = NULL;
    list_sch_tasks[index_task].parameters      = NULL;
    list_sch_tasks[index_task].period_step_cnt = 0;
    list_sch_tasks[index_task].delay_step_cnt  = 0;
    list_sch_tasks[index_task].run_time_cnt    = 0;
    list_sch_tasks[index_task].period_time     = 0;
    list_sch_tasks[index_task].run_me          = coFALSE;

    list_sch_num--;

    return coTRUE;
}

void ttscheduler_update()
{
    uint8_t i;

    for (i = 0; i < TTSCHEDULER_MAX_TASKS; i++)
    {
        if (list_sch_tasks[i].p_task != NULL)
        {
            if (list_sch_tasks[i].delay_step_cnt == 0)
            {
                list_sch_tasks[i].run_me++;
                if (list_sch_tasks[i].period_step_cnt != 0)
                {
                    list_sch_tasks[i].delay_step_cnt = list_sch_tasks[i].period_step_cnt - 1;
                }
            }
            else
            {
                list_sch_tasks[i].delay_step_cnt--;
            }
        }
    }
}

void ttscheduler_dispatch_tasks()
{
    uint8_t i;

    for (i = 0; i < TTSCHEDULER_MAX_TASKS; i++)
    {
        if (list_sch_tasks[i].run_me > 0)
        {
            (*list_sch_tasks[i].p_task)(list_sch_tasks[i].parameters);

#if CONFIG_RUN_TIME_INFO
            ttscheduler_run_info(i);
#endif

            if (list_sch_tasks[i].period_step_cnt == 0)
            {
                ttscheduler_delete_task(i);
            }
        }
    }

    (*idle_task.p_task)(idle_task.parameters);
#if CONFIG_RUN_TIME_INFO
    ttscheduler_run_info_idle();
#endif
}

#if CONFIG_RUN_TIME_INFO
/*
        debug code ===========

        char debug_string[300];
        co_scheduler_get_runtime_stats(debug_string);
        uint32_t cpu_load = co_scheduler_get_cpu_load();
        if (cpu_load < 1)
                format_output("cpu load : <1%%\r\n",cpu_load);
        else
                format_output("cpu load: %u%%\r\n",cpu_load);
        format_output("task_name\tcount\t\tload\r\n");
        format_output("%s",debug_string);
*/
void ttscheduler_get_runtime_stats(char *write_buf)
{
    uint8_t i;
    uint32_t stats_percent = 0;

    uint32_t total_time;

    total_run_time = PORT_GET_RUN_TIME_COUNTER_VALUE();

    total_time = total_run_time / 100;

    *write_buf = 0x00;

    if (total_time > 0)
    {
        for (i = 0; i < TTSCHEDULER_MAX_TASKS; i++)
        {
            if (list_sch_tasks[i].p_task != NULL)
            {
                stats_percent = list_sch_tasks[i].run_time_cnt / total_time;

                write_buf = write_name_to_buff(write_buf, list_sch_tasks[i].task_name);

                if (stats_percent > 0)
                {
#if LONG_LONG_RUN_TIMER
                    sprintf(write_buf, "\t%lld\t\t%u%%\r\n",
                            (uint64_t)list_sch_tasks[i].run_time_cnt, (uint32_t)stats_percent);
#else
                    sprintf(write_buf, "\t%u\t\t%u%%\r\n", (uint32_t)list_sch_tasks[i].run_time_cnt,
                            (uint32_t)stats_percent);
#endif
                }
                else
                {
#if LONG_LONG_RUN_TIMER
                    sprintf(write_buf, "\t%lld\t\t<1%%\r\n",
                            (uint64_t)list_sch_tasks[i].run_time_cnt);
#else
                    sprintf(write_buf, "\t%u\t\t<1%%\r\n",
                            (uint32_t)list_sch_tasks[i].run_time_cnt);
#endif
                }

                write_buf += strlen(write_buf);
            }
        }

        total_run_time = PORT_GET_RUN_TIME_COUNTER_VALUE();
        total_time     = total_run_time / 100;

        stats_percent = idle_task.run_time_cnt / total_time;

        write_buf = write_name_to_buff(write_buf, idle_task.task_name);
        sprintf(write_buf, "\t%u\t\t%u%%\r\n", (uint32_t)idle_task.run_time_cnt,
                (uint32_t)stats_percent);
        write_buf += strlen(write_buf);
    }
}
#endif

