#pragma once

#include <stdint.h>
#include <queue.h>
// #include <px4_platform_types.h>

__BEGIN_DECLS

#define HPWORK 0
#define LPWORK 1
#define NWORKERS 2

struct wqueue_s {
	pid_t             pid; /* The task ID of the worker thread */
	struct dq_queue_s q;   /* The queue of pending work */
};

extern struct wqueue_s g_work[NWORKERS];

/* Defines the work callback */

typedef void (*worker_t)(void *arg);

struct work_s {
	struct dq_entry_s dq;  /* Implements a doubly linked list */
	worker_t  worker;      /* Work callback */
	void *arg;             /* Callback argument */
	uint64_t  qtime;       /* Time work queued */
	uint32_t  delay;       /* Delay until work performed */
};

void work_queues_init(void);


int work_queue(int qid, struct work_s *work, worker_t worker, void *arg, uint32_t delay);


int work_cancel(int qid, struct work_s *work);

uint32_t clock_systimer(void);

int work_hpthread(int argc, char *argv[]);
int work_lpthread(int argc, char *argv[]);

__END_DECLS

