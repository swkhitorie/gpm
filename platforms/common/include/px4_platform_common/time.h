#pragma once

#include <unistd.h>
#include <sys/types.h>
#include <sys/cdefs.h>
#include <sys/config.h>
#include <time.h>
#include <pthread.h>

#if 0
__BEGIN_DECLS
__EXPORT int px4_clock_gettime(clockid_t clk_id, struct timespec *tp);
__END_DECLS
#else
#define px4_clock_gettime system_clock_gettime
#endif

#define px4_clock_settime system_clock_settime
#define px4_usleep system_usleep
#define px4_sleep system_sleep
#define px4_pthread_cond_timedwait system_pthread_cond_timedwait

#ifndef system_pthread_cond_timedwait
#define system_pthread_cond_timedwait pthread_cond_timedwait
#endif

