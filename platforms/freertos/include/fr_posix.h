#ifndef POSIX_FREERTOS_H_
#define POSIX_FREERTOS_H_

#include <stddef.h>
#include <stdint.h>
#include "sys/types.h"

/*
    https://pubs.opengroup.org/onlinepubs/9699919799/functions/
*/

#define posixconfigPTHREAD_TASK_NAME    "pthread"
#define posixconfigTIMER_NAME           "timer"
#define posixconfigMQ_MAX_MESSAGES       10 /**< Maximum number of messages in an mq at one time. */
#define posixconfigMQ_MAX_SIZE           128 /**< Maximum size (in bytes) of each message. */
#define PTHREAD_STACK_MIN                configMINIMAL_STACK_SIZE * sizeof(StackType_t)
/**< Maximum number of bytes in a filename (not including terminating null). */
#define NAME_MAX                         64
/**< Maximum value of a sem_t. */
#define SEM_VALUE_MAX        0x7FFFU

#endif
