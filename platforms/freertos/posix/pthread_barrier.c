
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

int pthread_barrier_destroy( pthread_barrier_t * barrier )
{
    pthread_barrier_internal_t *p = (pthread_barrier_internal_t *)barrier;

    (void)vEventGroupDelete((EventGroupHandle_t)&p->xBarrierEventGroup);
    (void)vSemaphoreDelete((SemaphoreHandle_t)&p->xThreadCountSemaphore);
    return 0;
}

/*-----------------------------------------------------------*/

int pthread_barrier_init( pthread_barrier_t * barrier,
                          const pthread_barrierattr_t * attr,
                          unsigned count )
{
    int iStatus = 0;
    pthread_barrier_internal_t * pxNewBarrier = ( pthread_barrier_internal_t * ) ( barrier );

    /* Silence warnings about unused parameters. */
    ( void ) attr;

    /* Ensure count is greater than 0. */
    if( count == 0 )
    {
        iStatus = EINVAL;
    }

    /* Ensure that count will fit in a FreeRTOS event group. */
    if( iStatus == 0 )
    {
        if( count > posixPTHREAD_BARRIER_MAX_COUNT )
        {
            /* No memory exists in the event group for more than
             * posixPTHREAD_BARRIER_MAX_COUNT threads. */
            iStatus = ENOMEM;
        }
    }

    if( iStatus == 0 )
    {
        /* Set the current thread count and threshold. */
        pxNewBarrier->uThreadCount = 0;
        pxNewBarrier->uThreshold = count;

        /* Create the FreeRTOS event group. This call will not fail when its
         * argument isn't NULL. */
        ( void ) xEventGroupCreateStatic( &pxNewBarrier->xBarrierEventGroup );

        /* Create the semaphore that prevents more than count threads from being
         * unblocked by a single successful pthread_barrier_wait. This semaphore
         * counts down from count and cannot decrement below 0. */
        ( void ) xSemaphoreCreateCountingStatic( ( UBaseType_t ) count, /* Max count. */
                                                 ( UBaseType_t ) count, /* Initial count. */
                                                 &pxNewBarrier->xThreadCountSemaphore );
    }

    return iStatus;
}

/*-----------------------------------------------------------*/

int pthread_barrier_wait( pthread_barrier_t * barrier )
{
    int iStatus = 0;
    unsigned i = 0; /* Loop iterator. */
    pthread_barrier_internal_t * pxBarrier = ( pthread_barrier_internal_t * ) ( barrier );
    unsigned uThreadNumber = 0;

    /* Decrement the number of threads waiting on this barrier. This will prevent more
     * than pxBarrier->uThreshold threads from being unblocked by a single successful
     * pthread_barrier_wait call.
     *
     * This call will never fail because it blocks forever.
     */
    ( void ) xSemaphoreTake( ( SemaphoreHandle_t ) &pxBarrier->xThreadCountSemaphore, portMAX_DELAY );

    uThreadNumber = Atomic_Increment_u32( ( uint32_t * ) &pxBarrier->uThreadCount );

    /* Set the bit in the event group representing this thread, then wait for the other
     * threads to set their bit. This call should wait forever until all threads have set
     * their bit, so the return value is ignored. */
    ( void ) xEventGroupSync( ( EventGroupHandle_t ) &pxBarrier->xBarrierEventGroup,
                              1 << uThreadNumber,                 /* Which bit in the event group to set. */
                              ( 1 << pxBarrier->uThreshold ) - 1, /* Wait for all threads to set their bits. */
                              portMAX_DELAY );

    /* The first thread to enter the barrier gets PTHREAD_BARRIER_SERIAL_THREAD as its
     * return value and resets xThreadCountSemaphore. */

    if( uThreadNumber == 0 )
    {
        iStatus = PTHREAD_BARRIER_SERIAL_THREAD;

        /* uThreadCount can be safely changed without locking xThreadCountMutex
         * because xThreadCountSemaphore is currently 0. */
        pxBarrier->uThreadCount = 0;

        /* Reset xThreadCountSemaphore. This allows more threads to enter the
         * barrier, starting a new cycle. */
        for( i = 0; i < pxBarrier->uThreshold; i++ )
        {
            xSemaphoreGive( ( SemaphoreHandle_t ) &pxBarrier->xThreadCountSemaphore );
        }
    }

    return iStatus;
}
