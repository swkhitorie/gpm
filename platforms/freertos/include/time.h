#ifndef POSIX_TIME_H_
#define POSIX_TIME_H_

#include "sys/types.h"
#include "signal.h"

#define MICROSECONDS_PER_SECOND    ( 1000000LL )
#define NANOSECONDS_PER_SECOND     ( 1000000000LL )
#define NANOSECONDS_PER_TICK       ( NANOSECONDS_PER_SECOND / configTICK_RATE_HZ )

#define CLOCK_REALTIME     0
#define CLOCK_MONOTONIC    1
#define CLOCKS_PER_SEC     ((clock_t)configTICK_RATE_HZ)
#define TIMER_ABSTIME      0x01

struct timespec
{
    time_t tv_sec;
    long tv_nsec;
};

struct itimerspec
{
    struct timespec it_interval;
    struct timespec it_value;
};

clock_t clock               (void);
int     clock_getcpuclockid (pid_t pid, clockid_t *clock_id);
int     clock_getres        (clockid_t clock_id, struct timespec *res);
int     clock_gettime       (clockid_t clock_id, struct timespec *tp);
int     clock_settime       (clockid_t clock_id, const struct timespec *tp);
int     clock_nanosleep     (clockid_t clock_id, int flags, const struct timespec *rqtp, struct timespec *rmtp);
int     nanosleep           (const struct timespec *rqtp, struct timespec *rmtp);
int     timer_create        (clockid_t clockid, struct sigevent *evp, timer_t *timerid);
int     timer_delete        (timer_t timerid);
int     timer_getoverrun    (timer_t timerid);
int     timer_gettime       (timer_t timerid, struct itimerspec *value);
int     timer_settime       (timer_t timerid, int flags, const struct itimerspec *value, struct itimerspec *ovalue);

#endif
