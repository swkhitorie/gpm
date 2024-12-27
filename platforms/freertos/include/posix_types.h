#ifndef POSIX_TYPE_H_
#define POSIX_TYPE_H_

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "semphr.h"
#include "doubly_linked_list.h"

typedef struct
{
    StaticSemaphore_t sem;
    int val;
} sem_internal_t;
typedef sem_internal_t             posix_semtype_t;

typedef struct pthread_mutexattr_internal
{
    int type;
} pthread_mutexattr_internal_t;

typedef struct pthread_mutex_internal
{
    BaseType_t initialized;
    StaticSemaphore_t mutex;
    TaskHandle_t owner;
    pthread_mutexattr_internal_t attr;
} pthread_mutex_internal_t;

typedef struct pthread_cond_internal
{
    BaseType_t initialized;
    StaticSemaphore_t condwait_sem;
    unsigned wait_threads;
} pthread_cond_internal_t;

typedef struct pthread_barrier_internal
{
    unsigned thread_cnt;
    unsigned threshold;
    StaticSemaphore_t thread_cnt_sem;
    StaticEventGroup_t barrier_eventgrp;
} pthread_barrier_internal_t;

typedef struct pthread_mutexattr
{
    uint32_t pthread_mutex_attrstorage;
} pthreadMutexAttrType_t;

typedef struct pthread_attr
{
    uint32_t pthread_attrstorage;
} pthreadAttrType_t;

typedef pthread_mutex_internal_t   pthreadMutexType_t;
typedef pthread_cond_internal_t    pthreadCondType_t;
typedef pthread_barrier_internal_t pthreadBarrierType_t;

#define FREERTOS_POSIX_MUTEX_INITIALIZER \
    (((pthread_mutex_internal_t)         \
    {                                    \
        .initialized = pdFALSE,          \
        .mutex = {{0}},                  \
        .owner = NULL,                   \
        .attr = {.iType = 0}             \
    }))

#define FREERTOS_POSIX_COND_INITIALIZER \
    (((pthread_cond_internal_t)         \
    {                                   \
        .initialized = pdFALSE,         \
        .condwait_sem = {{0}},          \
        .wait_threads = 0               \
    }))

#endif
