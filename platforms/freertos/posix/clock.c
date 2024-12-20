#include <stddef.h>
#include <string.h>
#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/time.h"
#include "include/utils.h"

extern int snprintf(char *s, size_t n, const char *format, ...);

clock_t clock()
{
    return (clock_t)-1;
}

int clock_getcpuclockid(pid_t pid, clockid_t *clock_id)
{
    (void)pid;
    (void)clock_id;
    return EPERM;
}

int clock_getres(clockid_t clock_id, struct timespec *res)
{
    (void)clock_id;

    if (res != NULL) {
        res->tv_sec = 0;
        res->tv_nsec = NANOSECONDS_PER_TICK;
    }
    return 0;
}

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    TimeOut_t cur = { 0 };
    uint64_t tick_cnt = 0ULL;
    (void) clock_id;

    vTaskSetTimeOutState(&cur);
    tick_cnt = (uint64_t)(cur.xOverflowCount) << (sizeof(TickType_t) * 8);
    tick_cnt += cur.xTimeOnEntering;

    utils_nanoseconds_totimespec((int64_t)tick_cnt * NANOSECONDS_PER_TICK, tp);
    return 0;
}

int clock_nanosleep(clockid_t clock_id, int flags, const struct timespec *rqtp, struct timespec *rmtp)
{
    int ret = 0;
    TickType_t sleep_time = 0;
    struct timespec cur = { 0 };

    (void)clock_id;
    (void)rmtp;
    (void)flags;

    if (utils_validtimespec( rqtp ) == false) {
        ret = EINVAL;
    }

    if (ret == 0 && clock_gettime(CLOCK_REALTIME, &cur) != 0) {
        ret = EINVAL;
    }

    if (ret == 0) {
        if ((flags & TIMER_ABSTIME) == TIMER_ABSTIME) {
            /* Get current time */
            if (clock_gettime(CLOCK_REALTIME, &cur) != 0) {
                ret = EINVAL;
            }

            if (ret == 0 && utils_timespec_todeltaticks(rqtp, &cur, &sleep_time) == 0) {
                #if (INCLUDE_vTaskDelayUntil == 1)
                    TickType_t cur_ticks = xTaskGetTickCount();
                    vTaskDelayUntil(&cur_ticks, sleep_time);
                #else
                    vTaskDelay( sleep_time );
                #endif
            }
        } else {
            if (utils_timespec_toticks(rqtp, &sleep_time) == 0) {
                vTaskDelay(sleep_time);
            }
        }
    }
    return ret;
}

int clock_settime(clockid_t clock_id, const struct timespec *tp)
{
    (void)clock_id;
    (void)tp;
    // errno = EPERM;
    return -1;
}

int nanosleep(const struct timespec *rqtp, struct timespec *rmtp)
{
    int ret = 0;
    TickType_t sleep_time = 0;
    (void) rmtp;

    if (utils_validtimespec(rqtp) == false) {
        // errno = EINVAL;
        ret = -1;
    }

    if (ret == 0) {
        if (utils_timespec_toticks(rqtp, &sleep_time) == 0) {
            vTaskDelay( sleep_time );
        }
    }
    return ret;
}

