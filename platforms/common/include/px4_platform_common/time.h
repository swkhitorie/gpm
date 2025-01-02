#pragma once

#include "unistd.h"
#include "sys/types.h"
#include "time.h"
#include "pthread.h"

/************************************************************
 * px4 time interface base on posix interface
 ***********************************************************/

#define px4_clock_gettime system_clock_gettime
#define px4_clock_settime system_clock_settime
#define px4_usleep system_usleep
#define px4_sleep system_sleep
#define px4_pthread_cond_timedwait system_pthread_cond_timedwait
