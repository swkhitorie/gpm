#ifndef POSIX_TYPES_H_
#define POSIX_TYPES_H_

#include <stdint.h>
#include "fr_posix_types.h"

typedef uint32_t                 clock_t;
typedef int                      clockid_t;
typedef int                      mode_t;
typedef int                      pid_t;
typedef int                      ssize_t;
typedef int64_t                  time_t;
typedef void                     *timer_t;
typedef unsigned long            useconds_t;
typedef long int                 off_t;

typedef void                     *pthread_t;
typedef void                     *pthread_barrierattr_t;
typedef void                     *pthread_condattr_t;
typedef PthreadAttrType_t        pthread_attr_t;
typedef PthreadBarrierType_t     pthread_barrier_t;
typedef PthreadCondType_t        pthread_cond_t;
typedef PthreadMutexType_t       pthread_mutex_t;
typedef PthreadMutexAttrType_t   pthread_mutexattr_t;

#endif
