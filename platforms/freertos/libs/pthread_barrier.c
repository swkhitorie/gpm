
#include <stddef.h>

#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/pthread.h"
#include "../include/fr_atomic.h"

#if ( configUSE_16_BIT_TICKS == 1 )
    #define posixPTHREAD_BARRIER_MAX_COUNT    ( 8 )
#else
    #define posixPTHREAD_BARRIER_MAX_COUNT    ( 24 )
#endif

int pthread_barrier_destroy(pthread_barrier_t *barrier)
{
    pthread_barrier_internal_t *p = (pthread_barrier_internal_t *)barrier;

    (void)vEventGroupDelete((EventGroupHandle_t)&p->xBarrierEventGroup);
    (void)vSemaphoreDelete((SemaphoreHandle_t)&p->xThreadCountSemaphore);
    return 0;
}

int pthread_barrier_init(pthread_barrier_t *barrier, 
   const pthread_barrierattr_t *attr, unsigned count)
{
    int ret = 0;
    pthread_barrier_internal_t *p = (pthread_barrier_internal_t *)barrier;

    (void)attr;
    if (count == 0) {
        ret = EINVAL;
    }

    if (ret == 0) {
        if (count > posixPTHREAD_BARRIER_MAX_COUNT){
            /* No memory exists in the event group for more than
             * posixPTHREAD_BARRIER_MAX_COUNT threads. */
            ret = ENOMEM;
        }
    }

    if (ret == 0) {
        p->thread_cnt = 0;
        p->threshold = count;
        (void)xEventGroupCreateStatic(&p->barrier_eventgrp);
        (void)xSemaphoreCreateCountingStatic((UBaseType_t)count,
                (UBaseType_t)count, &p->thread_cnt_sem);
    }
    return ret;
}

int pthread_barrier_wait(pthread_barrier_t *barrier)
{
    int ret = 0;
    unsigned i = 0;
    pthread_barrier_internal_t *p = (pthread_barrier_internal_t *)barrier;
    unsigned thread_num = 0;

    (void)xSemaphoreTake((SemaphoreHandle_t)&p->thread_cnt_sem, portMAX_DELAY);

    thread_num = Atomic_Increment_u32((uint32_t *) &p->thread_cnt);

    (void)xEventGroupSync((EventGroupHandle_t)&p->barrier_eventgrp,
                1 << thread_num, (1 << p->threshold) - 1, portMAX_DELAY);

    if (thread_num == 0) {
        ret = PTHREAD_BARRIER_SERIAL_THREAD;
        p->thread_cnt = 0;
        for (i = 0; i < p->threshold; i++) {
            xSemaphoreGive((SemaphoreHandle_t)&p->thread_cnt_sem);
        }
    }
    return ret;
}
