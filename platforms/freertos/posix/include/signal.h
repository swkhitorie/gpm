#ifndef POSIX_SIGNAL_H_
#define POSIX_SIGNAL_H_

#define SIGEV_NONE      0
#define SIGEV_SIGNAL    1
#define SIGEV_THREAD    2

union sigval
{
    int sival_int;
    void *sival_ptr;
};

struct sigevent
{
    int sigev_notify;
    int sigev_signo;
    union sigval sigev_value;
    void (*sigev_notify_function)(union sigval);
    pthread_attr_t *sigev_notify_attributes;
};

#endif
