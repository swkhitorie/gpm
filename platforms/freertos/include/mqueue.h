#ifndef POSIX_MQUEUE_H_
#define POSIX_MQUEUE_H_

#include "time.h"

typedef void * mqd_t;

struct mq_attr
{
    long mq_flags;   /**< Message queue flags. */
    long mq_maxmsg;  /**< Maximum number of messages. */
    long mq_msgsize; /**< Maximum message size. */
    long mq_curmsgs; /**< Number of messages currently queued. */
};

int     mq_close        (mqd_t mqdes);
int     mq_getattr      (mqd_t mqdes, struct mq_attr *mqstat);
mqd_t   mq_open         (const char *name, int oflag, mode_t mode, struct mq_attr *attr);
ssize_t mq_receive      (mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned int *msg_prio);
int     mq_send         (mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio);
ssize_t mq_timedreceive (mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned *msg_prio, const struct timespec *abstime);
int     mq_timedsend    (mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio, const struct timespec *abstime);
int     mq_unlink       (const char *name);

#endif
