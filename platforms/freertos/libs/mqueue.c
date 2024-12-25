#include <string.h>

#include "include/fr_posix.h"
#include "include/errno.h"
#include "include/fcntl.h"
#include "include/mqueue.h"
#include "include/utils.h"

typedef struct __queuedata
{
    char *p;
    size_t size;
} queue_element_t;

typedef struct QueueListElement
{
    Link_t link;
    QueueHandle_t queue;
    size_t open_descriptors;
    char * name;
    struct mq_attr attr;
    BaseType_t pending_unlink;
} queuelist_element_t;

static StaticSemaphore_t queue_listmutex = { { 0 }, .u = { 0 } };
static Link_t queue_listhead = { 0 };

static int cal_ticktimeout(long flag, const struct timespec *ptimeout, TickType_t *ptimeout_ticks)
{
    int ret = 0;

    if (flag & O_NONBLOCK) {
        *ptimeout_ticks = 0;
    } else {
        if (ptimeout == NULL) {
            *ptimeout_ticks = portMAX_DELAY;
        } else {
            struct timespec cur = {0};
            if (utils_validtimespec(ptimeout) == false) {
                ret = EINVAL;
            }
            if (ret == 0 && clock_gettime(CLOCK_REALTIME, &cur) != 0) {
                ret = EINVAL;
            }
            if ( ret == 0 && utils_timespec_todeltaticks(ptimeout, &cur, ptimeout_ticks) != 0) {
                ret = ETIMEDOUT;
            }
        }
    }
    return ret;
}

static BaseType_t create_new_messagequeue(queuelist_element_t **p,
    const struct mq_attr *pattr, const char *name, size_t len)
{
    BaseType_t ret = pdTRUE;

    *p = pvPortMalloc(sizeof(queuelist_element_t));
    if (*p == NULL) {
        ret = pdFALSE;
    }

    if (ret == pdTRUE) {
        (*p)->queue = xQueueCreate(pattr->mq_maxmsg, sizeof(queue_element_t));
        if ((*p)->queue == NULL) {
            vPortFree(*p);
            ret = pdFALSE;
        }
    }

    if (ret == pdTRUE) {
        (*p)->name = pvPortMalloc(len + 1);
        if ((*p)->name == NULL) {
            vQueueDelete((*p)->queue);
            vPortFree(*p);
            ret = pdFALSE;
        } else {
            (void)strncpy((*p)->name, name, len + 1);
        }
    }

    if (ret == pdTRUE) {
        (*p)->attr = *pattr;
        (*p)->open_descriptors = 1;
        (*p)->pending_unlink = pdFALSE;
        listADD(&queue_listhead, &(*p)->link);
    }
    return ret;
}

static void delete_messagequeue(const queuelist_element_t *p)
{
    queue_element_t element = {0};
    while (xQueueReceive(p->queue, (void *)&element, 0) == pdTRUE) {
        vPortFree(element.p);
    }

    vQueueDelete(p->queue);
    vPortFree((void *)p->name);
    vPortFree((void *)p);
}

static BaseType_t find_queue_inlist(queuelist_element_t **p, const char *name, mqd_t descriptor)
{
    Link_t * queue_listlink = NULL;
    queuelist_element_t *msg_queue = NULL;
    BaseType_t found = pdFALSE;

    /* Iterate through the list of queues. */
    listFOR_EACH(queue_listlink, &queue_listhead) {
        msg_queue = listCONTAINER(queue_listlink, queuelist_element_t, link);

        if ( name != NULL && strcmp(msg_queue->name, name) == 0) {
            found = pdTRUE;
            break;
        } else {
            if ((mqd_t)msg_queue == descriptor) {
                found = pdTRUE;
                break;
            }
        }
    }

    if (found == pdTRUE && p != NULL){
        *p = msg_queue;
    }
    return found;
}

static void init_queuelist( void )
{
    static BaseType_t queuelist_initialized = pdFALSE;

    if (queuelist_initialized == pdFALSE) {
        taskENTER_CRITICAL();
        if (queuelist_initialized == pdFALSE) {
            (void)xSemaphoreCreateMutexStatic(&queue_listmutex);
            listINIT_HEAD(&queue_listhead);
            queuelist_initialized = pdTRUE;
        }
        taskEXIT_CRITICAL();
    }
}

static BaseType_t validate_queuename(const char *name, size_t *len)
{
    BaseType_t ret = pdTRUE;
    size_t name_len = 0;

    /* All message queue names must start with '/'. */
    if (name[0] != '/') {
        ret = pdFALSE;
    } else {
        /* Get the length of pcName, excluding the first '/' and null-terminator. */
        name_len = utils_strlen(name, NAME_MAX + 2);
        if (name_len == NAME_MAX + 2) {
            ret = pdFALSE;
        } else {
            *len = name_len;
        }
    }
    return ret;
}

int mq_close(mqd_t mqdes)
{
    int ret = 0;
    queuelist_element_t *p = (queuelist_element_t *)mqdes;
    BaseType_t removed = pdFALSE;

    init_queuelist();

    (void)xSemaphoreTake((SemaphoreHandle_t)&queue_listmutex, portMAX_DELAY);

    /* Attempt to find the message queue based on the given descriptor. */
    if (find_queue_inlist(NULL, NULL, mqdes) == pdTRUE) {
        if (p->open_descriptors > 0){
            p->open_descriptors--;
        }

        if (p->open_descriptors == 0) {
            if (p->pending_unlink == pdTRUE) {
                listREMOVE(&p->link);
                removed = pdTRUE;
            } else {
                p->pending_unlink = pdTRUE;
            }
        }
    } else {
        // errno = EBADF;
        ret = -1;
    }

    (void)xSemaphoreGive((SemaphoreHandle_t)&queue_listmutex);

    if (removed == pdTRUE) {
        delete_messagequeue(p);
    }
    return ret;
}

int mq_getattr(mqd_t mqdes, struct mq_attr *mqstat)
{
    int iStatus = 0;
    queuelist_element_t *p = (queuelist_element_t *)mqdes;

    (void)xSemaphoreTake((SemaphoreHandle_t)&queue_listmutex, portMAX_DELAY);

    if (find_queue_inlist(NULL, NULL, mqdes) == pdTRUE) {
        p->attr.mq_curmsgs = (long)uxQueueMessagesWaiting(pxMessageQueue->xQueue);
        *mqstat = p->attr;
    } else {
        // errno = EBADF;
        iStatus = -1;
    }

    (void)xSemaphoreGive((SemaphoreHandle_t)&queue_listmutex);
    return iStatus;
}

/*-----------------------------------------------------------*/

mqd_t mq_open(const char *name, int oflag, mode_t mode, struct mq_attr *attr)
{
    mqd_t msg_queue = NULL;
    size_t name_len = 0;

    /* Default mq_attr. */
    struct mq_attr queue_creation_attr =
    {
        .mq_flags   = 0,
        .mq_maxmsg  = posixconfigMQ_MAX_MESSAGES,
        .mq_msgsize = posixconfigMQ_MAX_SIZE,
        .mq_curmsgs = 0
    };

    (void)mode;
    init_queuelist();
    if (validate_queuename(name, &name_len) == pdFALSE) {
        // errno = EINVAL;
        msg_queue = (mqd_t)-1;
    }

    if (msg_queue == NULL) {
        if ((oflag & O_CREAT) && (attr != NULL) && ((attr->mq_maxmsg <= 0) || (attr->mq_msgsize <= 0))) {
            /* Invalid mq_attr.mq_maxmsg or mq_attr.mq_msgsize. */
            // errno = EINVAL;
            msg_queue = (mqd_t)-1;
        }
    }

    if (msg_queue == NULL) {
        (void)xSemaphoreTake((SemaphoreHandle_t)&queue_listmutex, portMAX_DELAY);
        if (find_queue_inlist((queuelist_element_t **)&msg_queue, name, (mqd_t)NULL) == pdTRUE) {
            if ((oflag & O_EXCL) && (oflag & O_CREAT)) {
                // errno = EEXIST;
                msg_queue = (mqd_t)-1;
            } else {
                if(((queuelist_element_t *)msg_queue )->pending_unlink == pdTRUE) {
                    /* Queue pending deletion. Don't allow it to be re-opened. */
                    // errno = EINVAL;
                    msg_queue = ( mqd_t ) -1;
                } else {
                    /* Increase count of open file descriptors for queue. */
                    ((queuelist_element_t *) msg_queue)->open_descriptors++;
                }
            }
        } else {
            /* Only create the new queue if O_CREAT was specified. */
            if (oflag & O_CREAT) {
                /* Copy attributes if provided. */
                if (attr != NULL) {
                    queue_creation_attr = *attr;
                }
                queue_creation_attr.mq_flags = (long)oflag;
                if (create_new_messagequeue((queuelist_element_t **)&msg_queue,
                    &queue_creation_attr, name, name_len) == pdFALSE ) {
                    // errno = ENOSPC;
                    msg_queue = (mqd_t)-1;
                }
            } else {
                // errno = ENOENT;
                msg_queue = (mqd_t)-1;
            }
        }
        (void)xSemaphoreGive((SemaphoreHandle_t)&queue_listmutex);
    }
    return msg_queue;
}

ssize_t mq_receive(mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned int *msg_prio)
{
    return mq_timedreceive(mqdes, msg_ptr, msg_len, msg_prio, NULL);
}

int mq_send(mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned msg_prio)
{
    return mq_timedsend(mqdes, msg_ptr, msg_len, msg_prio, NULL);
}

ssize_t mq_timedreceive(mqd_t mqdes, char *msg_ptr, size_t msg_len, unsigned *msg_prio, const struct timespec *abstime)
{
    ssize_t ret = 0;
    int cal_timeout_return = 0;
    TickType_t timeout_ticks = 0;
    queuelist_element_t *p = (queuelist_element_t *)mqdes;
    queue_element_t rcv = { 0 };

    (void)msg_prio;
    (void)xSemaphoreTake((SemaphoreHandle_t)&queue_listmutex, portMAX_DELAY);

    if (find_queue_inlist(NULL, NULL, mqdes) == pdFALSE) {
        // errno = EBADF;
        ret = -1;
    }

    if (ret == 0) {
        if (msg_len < (size_t)p->queue.mq_msgsize) {
            /* msg_len too small. */
            // errno = EMSGSIZE;
            ret = -1;
        }
    }

    if (ret == 0) {
        cal_timeout_return = cal_ticktimeout(p->attr.mq_flags, abstime, &timeout_ticks);
        if( cal_timeout_return != 0 ) {
            // errno = cal_timeout_return;
            ret = -1;
        }
    }

    (void)xSemaphoreGive((SemaphoreHandle_t)&queue_listmutex);

    if (ret == 0) {
        if (xQueueReceive(p->queue, &rcv, timeout_ticks) == pdFALSE) {
            /* If queue receive fails, set the appropriate errno. */
            if( p->attr.mq_flags & O_NONBLOCK ) {
                /* Set errno to EAGAIN for nonblocking mq. */
                // errno = EAGAIN;
            } else {
                /* Otherwise, set errno to ETIMEDOUT. */
                // errno = ETIMEDOUT;
            }
            ret = -1;
        }
    }

    if (ret == 0) {
        ret = (ssize_t)rcv.size;
        (void)memcpy(msg_ptr, rcv.p, rcv.size);
        vPortFree(rcv.p);
    }

    return ret;
}

int mq_timedsend( mqd_t mqdes, const char *msg_ptr, size_t msg_len, unsigned int msg_prio, const struct timespec *abstime)
{
    int ret = 0, cal_timeout_return = 0;
    TickType_t timeout_ticks = 0;
    queuelist_element_t *p = ( queuelist_element_t * ) mqdes;
    queue_element_t send = { 0 };

    (void)msg_prio;
    (void)xSemaphoreTake((SemaphoreHandle_t)&queue_listmutex, portMAX_DELAY);

    if (find_queue_inlist(NULL, NULL, mqdes) == pdFALSE) {
        // errno = EBADF;
        ret = -1;
    }

    if (ret == 0) {
        if (msg_len > (size_t)p->attr.mq_msgsize) {
            /* msg_len too large. */
            // errno = EMSGSIZE;
            ret = -1;
        }
    }

    if (ret == 0) {
        cal_timeout_return = cal_ticktimeout( p->attr.mq_flags, abstime, &timeout_ticks);
        if (cal_timeout_return != 0) {
            // errno = cal_timeout_return;
            ret = -1;
        }
    }

    (void)xSemaphoreGive((SemaphoreHandle_t)&queue_listmutex);

    if (ret == 0) {
        send.size = msg_len;
        send.p = pvPortMalloc(msg_len);
        if (send.p == NULL) {
            // errno = EMSGSIZE;
            ret = -1;
        } else {
            (void)memcpy(send.p, msg_ptr, msg_len);
        }
    }

    if (ret == 0) {
        if (xQueueSend(p->queue, &send, timeout_ticks ) == pdFALSE ) {
            if (p->attr.mq_flags & O_NONBLOCK ) {
                /* Set errno to EAGAIN for nonblocking mq. */
                // errno = EAGAIN;
            } else {
                /* Otherwise, set errno to ETIMEDOUT. */
                // errno = ETIMEDOUT;
            }
            vPortFree(send.p);
            ret = -1;
        }
    }
    return ret;
}

int mq_unlink(const char *name)
{
    int ret = 0;
    size_t name_size = 0;
    BaseType_t queue_removed = pdFALSE;
    queuelist_element_t *p = NULL;

    init_queuelist();

    if (validate_queuename(name, &name_size) == pdFALSE) {
        // errno = EINVAL;
        ret = -1;
    }

    if (ret == 0) {
        (void)xSemaphoreTake((SemaphoreHandle_t)&queue_listmutex, portMAX_DELAY);
        if (find_queue_inlist(&p, name, (mqd_t)NULL) == pdTRUE) {
            if (p->open_descriptors == 0) {
                listREMOVE(&p->link);
                queue_removed = pdTRUE;
            } else {
                p->pending_unlink = pdTRUE;
            }
        } else {
            // errno = ENOENT;
            ret = -1;
        }
        (void)xSemaphoreGive((SemaphoreHandle_t)&queue_listmutex);
    }

    if( queue_removed == pdTRUE ) {
        delete_messagequeue(p);
    }
    return ret;
}
