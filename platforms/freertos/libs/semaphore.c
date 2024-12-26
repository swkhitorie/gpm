#include <stddef.h>
#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/semaphore.h"
#include "include/utils.h"
#include "../include/fr_atomic.h"

int sem_destroy(sem_t *sem)
{
    sem_internal_t *p = (sem_internal_t *)sem;

    vSemaphoreDelete((SemaphoreHandle_t)&p->sem);
    return 0;
}

int sem_getvalue(sem_t *sem, int *sval)
{
    sem_internal_t *p = (sem_internal_t *)sem;
    *sval = p->val;
    return 0;
}

int sem_init(sem_t *sem, int pshared, unsigned value)
{
    int ret = 0;
    sem_internal_t *p = (sem_internal_t *)sem;
    (void)pshared;

    if (value > SEM_VALUE_MAX) {
        // errno = EINVAL;
        ret = -1;
    }
    p->val = (int)value;
    if (ret == 0) {
        (void)xSemaphoreCreateCountingStatic(SEM_VALUE_MAX, 0, &p->sem);
    }

    return ret;
}

int sem_post(sem_t *sem)
{
    sem_internal_t *p = (sem_internal_t *)sem;

    int pre_val = Atomic_Increment_u32((uint32_t *)&p->val);
    if (pre_val < 0) {
        (void)xSemaphoreGive((SemaphoreHandle_t)&p->sem);
    }

    return 0;
}

int sem_timedwait(sem_t *sem, const struct timespec *abstime)
{
    int ret = 0;
    sem_internal_t *p = (sem_internal_t *)sem;
    TickType_t delay = portMAX_DELAY;

    if (abstime != NULL) {
        if (utils_validtimespec(abstime) == false) {
            delay = 0;
            ret = EINVAL;
        } else {
            struct timespec cur = {0};
            if (clock_gettime(CLOCK_REALTIME, &cur) != 0) {
                ret = EINVAL;
            } else {
                ret = utils_timespec_todeltaticks(abstime, &cur, &delay);
            }
            if (ret == ETIMEDOUT) {
                delay = 0;
            }
        }
    }

    int pre_val = Atomic_Decrement_u32((uint32_t *)&p->val);
    if (pre_val > 0) {
        ret = 0;
    } else {
        if (xSemaphoreTake((SemaphoreHandle_t) &p->sem, delay) != pdTRUE) {
            if( ret == 0 ) {
                // errno = ETIMEDOUT;
            } else {
                // errno = iStatus;
            }
            ret = -1;
        } else {
            ret = 0;
        }
    }
    return ret;
}

/*-----------------------------------------------------------*/

int sem_trywait(sem_t *sem)
{
    int ret = 0;

    struct timespec xTimeout = { 0 };

    ret = sem_timedwait( sem, &xTimeout );

    //if ((ret == -1 ) && errno == ETIMEDOUT) {
        // errno = EAGAIN;
    //}
    return ret;
}

int sem_wait(sem_t *sem)
{
    return sem_timedwait(sem, NULL);
}

