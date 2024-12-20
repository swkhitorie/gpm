#include <stddef.h>
#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/pthread.h"
#include "include/signal.h"
#include "include/time.h"
#include "include/utils.h"
#include "fr_timers.h"

#define TIMESPEC_IS_ZERO(obj)        (obj.tv_sec == 0 && obj.tv_nsec == 0)
#define TIMESPEC_IS_NOT_ZERO(obj)    (!(TIMESPEC_IS_ZERO(obj))) 

typedef struct timer_internal {
    StaticTimer_t buff;
    struct sigevent event;
    TickType_t period;
} timer_internal_t;

void prvTimerCallback(TimerHandle_t handle)
{
    timer_internal_t *p = (timer_internal_t *)pvTimerGetTimerID(handle);
    pthread_t notify_thread;

    configASSERT(p != NULL);
    configASSERT(p->event.sigev_notify != SIGEV_SIGNAL);

    if (p->period > 0) {
        xTimerChangePeriod(handle, p->period, 0);
    }

    if (p->event.sigev_notify == SIGEV_THREAD) {
        i f( p->event.sigev_notify_attributes == NULL ) {
            (*p->event.sigev_notify_function)(p->event.sigev_value);
        } else {
            (void)pthread_create(&notify_thread,
                                p->event.sigev_notify_attributes,
                                (void *(*)(void *))p->event.sigev_notify_function,
                                p->event.sigev_value.sival_ptr);
        }
    }
}

int timer_create(clockid_t clockid, struct sigevent *evp, timer_t *timerid)
{
    int ret = 0;
    timer_internal_t *p = NULL;

    (void)clockid;

    if ( evp == NULL || evp->sigev_notify == SIGEV_SIGNAL) {
        // errno = ENOTSUP;
        ret = -1;
    }

    if (ret == 0) {
        p = pvPortMalloc(sizeof(timer_internal_t));
        if (p == NULL) {
            // errno = EAGAIN;
            ret = -1;
        }
    }

    if (ret == 0) {
        p->event = *evp;
        p->period = 0;
        *timerid = (timer_t)xTimerCreateStatic( posixconfigTIMER_NAME,
                                                portMAX_DELAY,
                                                pdFALSE,
                                                (void *)p,
                                                prvTimerCallback,
                                                &p->buff );
    }
    return ret;
}

int timer_delete(timer_t timerid)
{
    TimerHandle_t handle = timerid;
    timer_internal_t *p = (timer_internal_t *)pvTimerGetTimerID(handle);

    configASSERT(p != NULL);
    (void)xTimerStop(handle, portMAX_DELAY);

    while (xTimerIsTimerActive(handle) == pdTRUE ) {
        vTaskDelay(1);
    }

    vPortFree(p);
    return 0;
}

int timer_getoverrun(timer_t timerid)
{
    (void)timerid;
    return 0;
}

int timer_settime(timer_t timerid, int flags, const struct itimerspec *value, struct itimerspec *ovalue)
{
    int ret = 0;
    TimerHandle_t handle = timerid;
    timer_internal_t *p = (timer_internal_t *)pvTimerGetTimerID(handle);
    TickType_t next_expiration = 0, expiration_period = 0;

    if (TIMESPEC_IS_NOT_ZERO(value->it_value)) {
        if(( utils_validtimespec(&value->it_interval) == false) ||
        (utils_validtimespec(&value->it_value) == false)) {
            // errno = EINVAL;
            ret = -1;
        }
    }

    if (ovalue != NULL) {
        (void)timer_gettime(timerid, ovalue);
    }

    if ( ret == 0 && xTimerIsTimerActive(handle)) {
        (void)xTimerStop(handle, portMAX_DELAY);
    }

    if (ret == 0 && TIMESPEC_IS_NOT_ZERO(value->it_value)) {
        if (TIMESPEC_IS_NOT_ZERO(value->it_interval)) {
            (void)utils_timespec_toticks(&value->it_interval, &expiration_period);
        }

        p->period = expiration_period;
        if (TIMESPEC_IS_NOT_ZERO(value->it_value)) {
            if ((flags & TIMER_ABSTIME) == TIMER_ABSTIME) {
                struct timespec cur = { 0 };

                if (clock_gettime(CLOCK_REALTIME, &cur) != 0) {
                    ret = EINVAL;
                } else {
                    ret = utils_timespec_todeltaticks(&value->it_value, &cur, &next_expiration);
                }

                if (ret != 0) {
                    next_expiration = 0;
                    if (ret == ETIMEDOUT) {
                        ret = 0;
                    }
                }
            } else {
                (void)utils_timespec_toticks(&value->it_value, &next_expiration);
            }
        }

        if (next_expiration == 0) {
            prvTimerCallback(handle);
        } else {
            (void)xTimerChangePeriod(handle, next_expiration, portMAX_DELAY);
            (void)xTimerStart(handle, next_expiration);
        }
    }
    return ret;
}

/*-----------------------------------------------------------*/

int timer_gettime(timer_t timerid, struct itimerspec *value)
{
    TimerHandle_t handle = timerid;
    timer_internal_t *p = (timer_internal_t *)pvTimerGetTimerID(handle);
    TickType_t next_expiration = xTimerGetExpiryTime(handle) - xTaskGetTickCount(),
                expiration_period = p->period;

    if (xTimerIsTimerActive(handle) != pdFALSE) {
        value->it_value.tv_sec = (time_t)(next_expiration / configTICK_RATE_HZ);
        value->it_value.tv_nsec = (long)((next_expiration % configTICK_RATE_HZ) * NANOSECONDS_PER_TICK);
    } else {
        value->it_value.tv_sec = 0;
        value->it_value.tv_nsec = 0;
    }

    if (expiration_period != portMAX_DELAY) {
        value->it_interval.tv_sec = (time_t)(expiration_period / configTICK_RATE_HZ);
        value->it_interval.tv_nsec = (long)((expiration_period % configTICK_RATE_HZ) * NANOSECONDS_PER_TICK);
    } else {
        value->it_interval.tv_sec = 0;
        value->it_interval.tv_nsec = 0;
    }
    return 0;
}

