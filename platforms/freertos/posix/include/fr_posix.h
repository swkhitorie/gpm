#ifndef POSIX_FREERTOS_H_
#define POSIX_FREERTOS_H_

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "fr_event_groups.h"
#include "fr_task.h"
#include "semphr.h"

#include "doubly_linked_list.h"
#include "sys/types.h"

/*
    https://pubs.opengroup.org/onlinepubs/9699919799/functions/
*/

#define posixconfigPTHREAD_TASK_NAME    "pthread"
#define posixconfigTIMER_NAME           "timer"
#define posixconfigMQ_MAX_MESSAGES       10 /**< Maximum number of messages in an mq at one time. */
#define posixconfigMQ_MAX_SIZE           128 /**< Maximum size (in bytes) of each message. */
#define PTHREAD_STACK_MIN                configMINIMAL_STACK_SIZE * sizeof(StackType_t)
/**< Maximum number of bytes in a filename (not including terminating null). */
#define NAME_MAX                         64
/**< Maximum value of a sem_t. */
#define SEM_VALUE_MAX        0x7FFFU

typedef sem_internal_t             posix_semtype_t;
typedef struct
{
    StaticSemaphore_t sem;
    int val;
} sem_internal_t;

typedef struct pthread_mutexattr_internal
{
    int iType;
} pthread_mutexattr_internal_t;

typedef struct pthread_mutex_internal
{
    BaseType_t xIsInitialized;
    StaticSemaphore_t xMutex;
    TaskHandle_t xTaskOwner;
    pthread_mutexattr_internal_t xAttr;
} pthread_mutex_internal_t;

#define FREERTOS_POSIX_MUTEX_INITIALIZER \
    (((pthread_mutex_internal_t)         \
    {                                    \
        .xIsInitialized = pdFALSE,       \
        .xMutex = {{0}},                 \
        .xTaskOwner = NULL,              \
        .xAttr = {.iType = 0}            \
    }))

typedef struct pthread_cond_internal
{
    BaseType_t xIsInitialized;
    StaticSemaphore_t xCondWaitSemaphore;
    unsigned iWaitingThreads;
} pthread_cond_internal_t;

#define FREERTOS_POSIX_COND_INITIALIZER \
    (((pthread_cond_internal_t)         \
    {                                   \
        .xIsInitialized = pdFALSE,      \
        .xCondWaitSemaphore = {{0}},    \
        .iWaitingThreads = 0            \
    }))

typedef struct pthread_barrier_internal
{
    unsigned uThreadCount;
    unsigned uThreshold;
    StaticSemaphore_t xThreadCountSemaphore;
    StaticEventGroup_t xBarrierEventGroup;
} pthread_barrier_internal_t;

typedef pthread_mutex_internal_t   PthreadMutexType_t;
typedef pthread_cond_internal_t    PthreadCondType_t;

typedef struct pthread_mutexattr
{
    uint32_t ulpthreadMutexAttrStorage;
} PthreadMutexAttrType_t;
    
typedef struct pthread_attr
{
    uint32_t ulpthreadAttrStorage;
} PthreadAttrType_t;
    
typedef pthread_barrier_internal_t PthreadBarrierType_t;



#endif
