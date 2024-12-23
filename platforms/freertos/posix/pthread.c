#include <stddef.h>
#include <string.h>
#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/pthread.h"

typedef struct pthread_attr_internal
{
    uint16_t usStackSize;
    uint16_t usSchedPriorityDetachState;
} pthread_attr_internal_t;

#define pthreadDETACH_STATE_MASK      0x8000
#define pthreadSCHED_PRIORITY_MASK    0x7FFF
#define pthreadDETACH_STATE_SHIFT     15
#define pthreadGET_SCHED_PRIORITY( var )    ( ( var ) & ( pthreadSCHED_PRIORITY_MASK ) )
#define pthreadIS_JOINABLE( var )           ( ( ( var ) & ( pthreadDETACH_STATE_MASK ) ) == pthreadDETACH_STATE_MASK )

/**
 * @brief Thread object.
 */
typedef struct pthread_internal
{
    pthread_attr_internal_t xAttr;        /**< Thread attributes. */
    void * ( *pvStartRoutine )( void * ); /**< Application thread function. */
    void * xTaskArg;                      /**< Arguments for application thread function. */
    TaskHandle_t xTaskHandle;             /**< FreeRTOS task handle. */
    StaticSemaphore_t xJoinBarrier;       /**< Synchronizes the two callers of pthread_join. */
    StaticSemaphore_t xJoinMutex;         /**< Ensures that only one other thread may join this thread. */
    void * xReturn;                       /**< Return value of pvStartRoutine. */
} pthread_internal_t;

static const pthread_attr_internal_t xDefaultThreadAttributes =
{
    .usStackSize                = PTHREAD_STACK_MIN,
    .usSchedPriorityDetachState = ( ( uint16_t ) tskIDLE_PRIORITY & pthreadSCHED_PRIORITY_MASK ) | ( PTHREAD_CREATE_JOINABLE << pthreadDETACH_STATE_SHIFT ),
};

static void prvExitThread( void )
{
    pthread_internal_t *p = (pthread_internal_t *)pthread_self();

    if (pthreadIS_JOINABLE(p->xAttr.usSchedPriorityDetachState)) {
        (void)xSemaphoreGive((SemaphoreHandle_t)&p->xJoinBarrier);
        vTaskSuspend(NULL);
    } else {
        vPortFree(p);
        vTaskDelete(NULL);
    }
}

static void prvRunThread( void * pxArg )
{
    pthread_internal_t *p = (pthread_internal_t *) pxArg;
    p->xReturn = p->pvStartRoutine((void *)p->xTaskArg);
    prvExitThread();
}

int pthread_attr_destroy(pthread_attr_t *attr)
{
    (void)attr;
    return 0;
}

int pthread_attr_getdetachstate(const pthread_attr_t *attr, int *detachstate)
{
    pthread_attr_internal_t *p = (pthread_attr_internal_t *)attr;

    if (pthreadIS_JOINABLE(p->usSchedPriorityDetachState)) {
        *detachstate = PTHREAD_CREATE_JOINABLE;
    } else {
        *detachstate = PTHREAD_CREATE_DETACHED;
    }
    return 0;
}

int pthread_attr_getschedparam(const pthread_attr_t *attr, struct sched_param *param)
{
    pthread_attr_internal_t *p = (pthread_attr_internal_t *)attr;
    param->sched_priority = (int)(pthreadGET_SCHED_PRIORITY(p->usSchedPriorityDetachState));
    return 0;
}

int pthread_attr_getstacksize(const pthread_attr_t *attr, size_t *stacksize)
{
    pthread_attr_internal_t *p = (pthread_attr_internal_t *)attr;
    *stacksize = (size_t)p->usStackSize;
    return 0;
}

int pthread_attr_init(pthread_attr_t *attr)
{
    *((pthread_attr_internal_t *)attr) = xDefaultThreadAttributes;
    return 0;
}

int pthread_attr_setdetachstate(pthread_attr_t *attr, int detachstate)
{
    int ret = 0;
    pthread_attr_internal_t *p = (pthread_attr_internal_t *)attr;

    if ((detachstate != PTHREAD_CREATE_DETACHED) && (detachstate != PTHREAD_CREATE_JOINABLE)) {
        ret = EINVAL;
    } else {
        p->usSchedPriorityDetachState &= ~pthreadDETACH_STATE_MASK;
        p->usSchedPriorityDetachState |= ((uint16_t)detachstate << pthreadDETACH_STATE_SHIFT);
    }
    return ret;
}

int pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param)
{
    int ret = 0;
    pthread_attr_internal_t *p = (pthread_attr_internal_t *)attr;

    if (param == NULL) {
        ret = EINVAL;
    }

    if ((ret == 0) &&
        ((param->sched_priority > sched_get_priority_max(SCHED_OTHER)) ||
        (param->sched_priority < 0 ))) {
        ret = ENOTSUP;
    }

    if (ret == 0) {
        p->usSchedPriorityDetachState &= ~pthreadSCHED_PRIORITY_MASK;
        p->usSchedPriorityDetachState |= ((uint16_t)param->sched_priority);
    }
    return ret;
}

int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy)
{
    (void)attr;
    (void)policy;
    return 0;
}

int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stacksize)
{
    int ret = 0;
    pthread_attr_internal_t *p = (pthread_attr_internal_t *)attr;

    if (stacksize < PTHREAD_STACK_MIN) {
        ret = EINVAL;
    } else {
        p->usStackSize = (uint16_t)stacksize;
    }
    return ret;
}

int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
            void *(*startroutine)(void *), void *arg)
{
    int ret = 0;
    pthread_internal_t *pxThread = NULL;
    struct sched_param xSchedParam = { .sched_priority = tskIDLE_PRIORITY };

    pxThread = (pthread_internal_t *)pvPortMalloc(sizeof(pthread_internal_t));

    if (pxThread == NULL) {
        ret = EAGAIN;
    }

    if (ret == 0) {
        if (attr == NULL) {
            pxThread->xAttr = xDefaultThreadAttributes;
        } else {
            pxThread->xAttr = *((pthread_attr_internal_t *)attr);
        }

        xSchedParam.sched_priority = (int)pthreadGET_SCHED_PRIORITY(pxThread->xAttr.usSchedPriorityDetachState);

        pxThread->xTaskArg = arg;
        pxThread->pvStartRoutine = startroutine;

        if (pthreadIS_JOINABLE(pxThread->xAttr.usSchedPriorityDetachState)) {
            (void)xSemaphoreCreateMutexStatic(&pxThread->xJoinMutex);
            (void)xSemaphoreCreateBinaryStatic(&pxThread->xJoinBarrier);
        }
    }

    if (ret == 0) {
        vTaskSuspendAll();
        if (xTaskCreate(prvRunThread,
                        posixconfigPTHREAD_TASK_NAME,
                        (uint16_t)(pxThread->xAttr.usStackSize / sizeof(StackType_t)),
                         (void *)pxThread,
                        xSchedParam.sched_priority,
                        &pxThread->xTaskHandle) != pdPASS) {
            /* Task creation failed, no memory. */
            vPortFree(pxThread);
            ret = EAGAIN;
        } else {
            vTaskSetApplicationTaskTag(pxThread->xTaskHandle, (TaskHookFunction_t) pxThread);
            *thread = (pthread_t)pxThread;
        }
        xTaskResumeAll();
    }
    return ret;
}

int pthread_getschedparam(pthread_t thread, int *policy, struct sched_param *param)
{
    int ret = 0;
    pthread_internal_t *p = (pthread_internal_t *)thread;

    *policy = SCHED_OTHER;
    param->sched_priority = (int)pthreadGET_SCHED_PRIORITY(p->xAttr.usSchedPriorityDetachState);
    return ret;
}

int pthread_equal(pthread_t t1, pthread_t t2)
{
    return t1 == t2;
}

void pthread_exit(void *value_ptr)
{
    pthread_internal_t *p = (pthread_internal_t *)pthread_self();
    p->xReturn = value_ptr;
    prvExitThread();
}

int pthread_join(pthread_t pthread, void **retval)
{
    int ret = 0;
    pthread_internal_t *p = (pthread_internal_t *)pthread;

    if (!pthreadIS_JOINABLE(p->xAttr.usSchedPriorityDetachState )) {
        ret = EDEADLK;
    }

    if (ret == 0) {
        if (xSemaphoreTake((SemaphoreHandle_t)&p->xJoinMutex, 0) != pdPASS) {
            ret = EDEADLK;
        }
    }

    if (ret == 0) {
        if (pthread_equal(pthread_self(), pthread) != 0) {
            ret = EDEADLK;
        }
    }

    if (ret == 0) {
        (void)xSemaphoreTake((SemaphoreHandle_t)&p->xJoinBarrier, portMAX_DELAY);

        vTaskSuspendAll();

        (void)xSemaphoreGive((SemaphoreHandle_t)&p->xJoinBarrier);
        vSemaphoreDelete((SemaphoreHandle_t)&p->xJoinBarrier);

        (void)xSemaphoreGive((SemaphoreHandle_t)&p->xJoinMutex);
        vSemaphoreDelete((SemaphoreHandle_t)&p->xJoinMutex);

        vTaskDelete( p->xTaskHandle );

        if (retval != NULL) {
            *retval = p->xReturn;
        }

        vPortFree(p);
        xTaskResumeAll();
    }
    return ret;
}

pthread_t pthread_self(void)
{
    return (pthread_t)xTaskGetApplicationTaskTag(NULL);
}

int pthread_setschedparam(pthread_t thread, int policy, const struct sched_param *param)
{
    int ret = 0;
    pthread_internal_t *p = (pthread_internal_t *)thread;

    (void)policy;
    ret = pthread_attr_setschedparam((pthread_attr_t *)&p->xAttr, param);

    if (ret == 0) {
        vTaskPrioritySet(p->xTaskHandle, param->sched_priority);
    }
    return ret;
}


int pthread_setname_np(pthread_t thread, const char *name) { return -1; }
int pthread_getname_np(pthread_t thread, char *name, int namelen) { return -1; }
int pthread_cancel(pthread_t thread) { return -1; }
int pthread_kill(pthread_t thread, int sig) { return -1; }
int pthread_attr_setinheritsched(pthread_attr_t *attr, int inheritsched) { return -1; }
int pthread_condattr_setclock(pthread_condattr_t *attr, clockid_t clock_id) { return -1; }
int pthread_condattr_init(pthread_condattr_t *attr)
{ 
    int ret = 0;
    if (!attr) {
        ret = EINVAL;
    } else {
      *attr = 0;
    }
    return ret;
}
