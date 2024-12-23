#include <stddef.h>
#include <string.h>

#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/pthread.h"
#include "include/utils.h"

static const pthread_mutexattr_internal_t default_mutex_attr =
{
    .type = PTHREAD_MUTEX_DEFAULT,
};

static void init_staticmutix(pthread_mutex_internal_t *pxMutex)
{
    if (pxMutex->initialized == pdFALSE) {
        taskENTER_CRITICAL();
        if (pxMutex->initialized == pdFALSE) {
            pxMutex->attr.type = PTHREAD_MUTEX_DEFAULT;
            #if PTHREAD_MUTEX_DEFAULT == PTHREAD_MUTEX_RECURSIVE
                (void)xSemaphoreCreateRecursiveMutexStatic(&pxMutex->mutex);
            #else
                (void)xSemaphoreCreateMutexStatic(&pxMutex->mutex);
            #endif
            pxMutex->initialized = pdTRUE;
        }
        taskEXIT_CRITICAL();
    }
}

int pthread_mutex_destroy(pthread_mutex_t *mutex)
{
    pthread_mutex_internal_t *p = (pthread_mutex_internal_t *)mutex;

    if (p->owner == NULL) {
        vSemaphoreDelete((SemaphoreHandle_t)&p->mutex);
    }
    return 0;
}

int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr)
{
    int ret = 0;
    pthread_mutex_internal_t *p = (pthread_mutex_internal_t *)mutex;

    if (p == NULL) {
        ret = ENOMEM;
    }

    if (ret == 0) {
        *p = FREERTOS_POSIX_MUTEX_INITIALIZER;

        if (attr == NULL) {
            p->attr = default_mutex_attr;
        } else {
            p->attr = *((pthread_mutexattr_internal_t *)attr);
        }

        if (p->attr.type == PTHREAD_MUTEX_RECURSIVE) {
            (void)xSemaphoreCreateRecursiveMutexStatic(&p->mutex);
        } else {
            (void)xSemaphoreCreateMutexStatic(&p->mutex);
        }

        if ((SemaphoreHandle_t)&p->mutex == NULL) {
            ret = EAGAIN;
            vPortFree(p);
        } else {
            p->initialized = pdTRUE;
        }
    }
    return ret;
}

int pthread_mutex_lock(pthread_mutex_t *mutex)
{
    return pthread_mutex_timedlock(mutex, NULL);
}

int pthread_mutex_timedlock(pthread_mutex_t *mutex, const struct timespec *abstime)
{
    int ret = 0;
    pthread_mutex_internal_t *p = (pthread_mutex_internal_t *)mutex;
    TickType_t delay = portMAX_DELAY;
    BaseType_t fr_mutex_take_status = pdFALSE;

    init_staticmutix(p);
    configASSERT(p->initialized == pdTRUE);

    if (abstime != NULL) {
        struct timespec cur = {0};
        if (clock_gettime(CLOCK_REALTIME, &cur)!= 0) {
            ret = EINVAL;
        } else {
            ret = utils_timespec_todeltaticks(abstime, &cur, &delay);
        }
        if (ret == ETIMEDOUT) {
            delay = 0;
            ret = 0;
        }
    }

    if ((ret == 0) && (p->attr.type == PTHREAD_MUTEX_ERRORCHECK) && 
        (p->owner == xTaskGetCurrentTaskHandle())) {
        ret = EDEADLK;
    }

    if (ret == 0) {
        if (p->attr.type == PTHREAD_MUTEX_RECURSIVE) {
            fr_mutex_take_status = xSemaphoreTakeRecursive((SemaphoreHandle_t)&p->mutex, delay);
        } else {
            fr_mutex_take_status = xSemaphoreTake((SemaphoreHandle_t)&p->mutex, delay);
        }

        if (fr_mutex_take_status == pdPASS) {
            pxMutex->xTaskOwner = xTaskGetCurrentTaskHandle();
        } else {
            ret = ETIMEDOUT;
        }
    }
    return ret;
}

int pthread_mutex_trylock(pthread_mutex_t *mutex)
{
    int ret = 0;
    struct timespec timeout =
    {
        .tv_sec  = 0,
        .tv_nsec = 0
    };

    ret = pthread_mutex_timedlock(mutex, &timeout);
    if (ret == ETIMEDOUT) {
        ret = EBUSY;
    }
    return ret;
}

int pthread_mutex_unlock(pthread_mutex_t *mutex)
{
    int ret = 0;
    pthread_mutex_internal_t *p = (pthread_mutex_internal_t *)mutex;

    init_staticmutix(p);

    if(((p->attr.type == PTHREAD_MUTEX_ERRORCHECK) ||
        (p->attr.type == PTHREAD_MUTEX_RECURSIVE)) &&
        (p->owner != xTaskGetCurrentTaskHandle())) {
        ret = EPERM;
    }

    if (ret == 0) {
        vTaskSuspendAll();
        if (p->attr.type == PTHREAD_MUTEX_RECURSIVE) {
            (void)xSemaphoreGiveRecursive((SemaphoreHandle_t)&p->mutex);
        } else {
            (void)xSemaphoreGive((SemaphoreHandle_t)&p->mutex);
        }
        p->owner = xSemaphoreGetMutexHolder((SemaphoreHandle_t)&p->mutex);
        (void)xTaskResumeAll();
    }
    return ret;
}

int pthread_mutexattr_destroy(pthread_mutexattr_t *attr)
{
    (void)attr;
    return 0;
}

int pthread_mutexattr_gettype(const pthread_mutexattr_t *attr, int *type)
{
    pthread_mutexattr_internal_t *p = (pthread_mutexattr_internal_t *)attr;
    *type = p->type;
    return 0;
}

int pthread_mutexattr_init(pthread_mutexattr_t *attr)
{
    *((pthread_mutexattr_internal_t *)attr) = default_mutex_attr;
    return 0;
}

int pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type)
{
    int ret = 0;
    pthread_mutexattr_internal_t *p = (pthread_mutexattr_internal_t *)attr;

    switch (type) {
        case PTHREAD_MUTEX_NORMAL:
        case PTHREAD_MUTEX_RECURSIVE:
        case PTHREAD_MUTEX_ERRORCHECK:
            p->type = type;
            break;
        default:
            ret = EINVAL;
            break;
    }
    return ret;
}
