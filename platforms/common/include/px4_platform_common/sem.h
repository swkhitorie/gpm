#pragma once

#include <semaphore.h>

#if 0
	/* Values for protocol attribute */
	#define SEM_PRIO_NONE             0
	#define SEM_PRIO_INHERIT          1
	#define SEM_PRIO_PROTECT          2
	#define sem_setprotocol(s,p)
#endif

__BEGIN_DECLS

typedef struct {
	pthread_mutex_t lock;
	pthread_cond_t wait;
	int value;
} gpm_sem_t;

__EXPORT int		gpm_sem_init(gpm_sem_t *s, int pshared, unsigned value);
__EXPORT int		gpm_sem_setprotocol(gpm_sem_t *s, int protocol);
__EXPORT int		gpm_sem_wait(gpm_sem_t *s);
__EXPORT int		gpm_sem_trywait(gpm_sem_t *sem);
__EXPORT int		gpm_sem_timedwait(gpm_sem_t *sem, const struct timespec *abstime);
__EXPORT int		gpm_sem_post(gpm_sem_t *s);
__EXPORT int		gpm_sem_getvalue(gpm_sem_t *s, int *sval);
__EXPORT int		gpm_sem_destroy(gpm_sem_t *s);

__END_DECLS
