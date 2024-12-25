#ifndef POSIX_SCHED_H_
#define POSIX_SCHED_H_

#define SCHED_OTHER     0
#define SCHED_FIFO      1
#define SCHED_RR        2
#define SCHED_MIN       SCHED_OTHER
#define SCHED_MAX       SCHED_RR

struct sched_param
{
    int sched_priority;
};

int sched_get_priority_max (int policy);
int sched_get_priority_min (int policy);
int sched_yield            (void);

#endif
