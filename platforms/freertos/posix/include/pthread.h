#ifndef POSIX_PTHREAD_H_
#define POSIX_PTHREAD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sched.h"
#include "time.h"
#include "fr_posix.h"

/**
 * @name pthread detach state.
 */
/**@{ */
#define PTHREAD_CREATE_DETACHED    0       /**< Detached. */
#define PTHREAD_CREATE_JOINABLE    1       /**< Joinable (default). */
/**@} */

/**
 * @name Returned to a single thread after a successful pthread_barrier_wait.
 *
 * @brief POSIX specifies that "The constant PTHREAD_BARRIER_SERIAL_THREAD is defined in
 * <pthread.h> and its value shall be distinct from any other value returned by pthread_barrier_wait()."
 * So it's defined as negative to distinguish it from the errnos, which are positive.
 */
#define PTHREAD_BARRIER_SERIAL_THREAD    (-2)

#ifndef PTHREAD_MUTEX_NORMAL
    #define PTHREAD_MUTEX_NORMAL        0                        /**< Non-robust, deadlock on relock, does not remember owner. */
#endif
#ifndef PTHREAD_MUTEX_ERRORCHECK
    #define PTHREAD_MUTEX_ERRORCHECK    1                        /**< Non-robust, error on relock,  remembers owner. */
#endif
#ifndef PTHREAD_MUTEX_RECURSIVE
    #define PTHREAD_MUTEX_RECURSIVE     2                        /**< Non-robust, recursive relock, remembers owner. */
#endif
#ifndef PTHREAD_MUTEX_DEFAULT
    #define PTHREAD_MUTEX_DEFAULT       PTHREAD_MUTEX_NORMAL     /**< PTHREAD_MUTEX_NORMAL (default). */
#endif

#define PTHREAD_COND_INITIALIZER    FREERTOS_POSIX_COND_INITIALIZER
#define PTHREAD_MUTEX_INITIALIZER   FREERTOS_POSIX_MUTEX_INITIALIZER

int   pthread_attr_destroy          (pthread_attr_t *attr);
int   pthread_attr_getdetachstate   (const pthread_attr_t *attr, int *detachstate);
int   pthread_attr_getschedparam    (const pthread_attr_t *attr, struct sched_param *param);
int   pthread_attr_getstacksize     (const pthread_attr_t *attr, size_t *stacksize);
int   pthread_attr_init             (pthread_attr_t *attr);
int   pthread_attr_setdetachstate   (pthread_attr_t *attr, int detachstate);
int   pthread_attr_setschedparam    (pthread_attr_t *attr, const struct sched_param *param);
int   pthread_attr_setschedpolicy   (pthread_attr_t *attr, int policy);
int   pthread_attr_setstacksize     (pthread_attr_t *attr, size_t stacksize);

int   pthread_barrier_destroy       (pthread_barrier_t *barrier);
int   pthread_barrier_init          (pthread_barrier_t *barrier, const pthread_barrierattr_t *attr, unsigned count);
int   pthread_barrier_wait          (pthread_barrier_t *barrier);

int   pthread_create                (pthread_t *thread, const pthread_attr_t *attr, void *(*startroutine)(void *), void *arg);

int   pthread_cond_broadcast        (pthread_cond_t *cond);
int   pthread_cond_destroy          (pthread_cond_t *cond);
int   pthread_cond_init             (pthread_cond_t *cond, const pthread_condattr_t *attr);
int   pthread_cond_signal           (pthread_cond_t *cond);
int   pthread_cond_timedwait        (pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *abstime);
int   pthread_cond_wait             (pthread_cond_t *cond, pthread_mutex_t *mutex);

pthread_t pthread_self              (void);
int   pthread_equal                 (pthread_t t1, pthread_t t2);
void  pthread_exit                  (void *value_ptr);
int   pthread_setschedparam         (pthread_t thread, int policy, const struct sched_param *param);
int   pthread_getschedparam         (pthread_t thread, int *policy, struct sched_param *param);
int   pthread_join                  (pthread_t thread, void **retval);

int   pthread_mutex_destroy         (pthread_mutex_t *mutex);
int   pthread_mutex_init            (pthread_mutex_t *mutex, const pthread_mutexattr_t *attr);
int   pthread_mutex_lock            (pthread_mutex_t *mutex);
int   pthread_mutex_timedlock       (pthread_mutex_t *mutex, const struct timespec *abstime);
int   pthread_mutex_trylock         (pthread_mutex_t *mutex);
int   pthread_mutex_unlock          (pthread_mutex_t *mutex);
int   pthread_mutexattr_destroy     (pthread_mutexattr_t *attr);
int   pthread_mutexattr_gettype     (const pthread_mutexattr_t *attr, int *type);
int   pthread_mutexattr_settype     (pthread_mutexattr_t *attr, int type);
int   pthread_mutexattr_init        (pthread_mutexattr_t *attr);

/* no use */
int   pthread_condattr_init         (pthread_condattr_t *attr);
int   pthread_condattr_setclock     (pthread_condattr_t *attr, clockid_t clock_id);
int   pthread_setname_np            (pthread_t thread, const char *name);
int   pthread_getname_np            (pthread_t thread, char *name, int namelen);
int   pthread_attr_setinheritsched  (pthread_attr_t *attr, int inheritsched);
int   pthread_cancel                (pthread_t thread);
int   pthread_kill                  (pthread_t thread, int sig);

#ifdef __cplusplus
}
#endif

#endif
