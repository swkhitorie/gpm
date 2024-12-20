#ifndef POSIX_SEMAPHORE_H_
#define POSIX_SEMAPHORE_H_

#include "time.h"

typedef PosixSemType_t sem_t;

int sem_destroy    (sem_t *sem);
int sem_getvalue   (sem_t *sem, int *sval);
int sem_init       (sem_t *sem, int pshared, unsigned value);
int sem_post       (sem_t * sem);
int sem_timedwait  (sem_t * sem, const struct timespec *abstime);
int sem_trywait    (sem_t *sem);
int sem_wait       (sem_t *sem);

#endif
