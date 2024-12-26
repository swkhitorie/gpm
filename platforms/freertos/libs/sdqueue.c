
#include <stddef.h>
#include "include/queue.h"

void dq_addlast(dq_entry_t *node, dq_queue_t *queue)
{
	node->flink = NULL;
	node->blink = queue->tail;

	if (!queue->head) {
		queue->head = node;
		queue->tail = node;

	} else {
		queue->tail->flink = node;
		queue->tail        = node;
	}
}

void dq_rem(dq_entry_t *node, dq_queue_t *queue)
{
	dq_entry_t *prev = node->blink;
	dq_entry_t *next = node->flink;

	if (!prev) {
		queue->head = next;

	} else {
		prev->flink = next;
	}

	if (!next) {
		queue->tail = prev;

	} else {
		next->blink = prev;
	}

	node->flink = NULL;
	node->blink = NULL;
}

dq_entry_t *dq_remfirst(dq_queue_t *queue)
{
	dq_entry_t *ret = queue->head;

	if (ret) {
		dq_entry_t *next = ret->flink;

		if (!next) {
			queue->head = NULL;
			queue->tail = NULL;

		} else {
			queue->head = next;
			next->blink = NULL;
		}

		ret->flink = NULL;
		ret->blink = NULL;
	}

	return ret;
}

void sq_addfirst(sq_entry_t *node, sq_queue_t *queue)
{
	node->flink = queue->head;

	if (!queue->head) {
		queue->tail = node;
	}

	queue->head = node;
}


void sq_addafter(sq_entry_t *prev, sq_entry_t *node, sq_queue_t *queue)
{
	if (!queue->head || prev == queue->tail) {
		sq_addlast(node, queue);

	} else {
		node->flink = prev->flink;
		prev->flink = node;
	}
}

void sq_addlast(sq_entry_t *node, sq_queue_t *queue)
{
	node->flink = NULL;

	if (!queue->head) {
		queue->head = node;
		queue->tail = node;

	} else {
		queue->tail->flink = node;
		queue->tail        = node;
	}
}

void sq_rem(sq_entry_t *node, sq_queue_t *queue)
{
	if (queue->head && node) {
		if (node == queue->head) {
			queue->head = node->flink;

			if (node == queue->tail) {
				queue->tail = NULL;
			}

		} else {
			sq_entry_t *prev;

			for (prev = (sq_entry_t *)queue->head;
                prev && prev->flink != node;
                prev = prev->flink) {}

			if (prev) {
				sq_remafter(prev, queue);
			}
		}
	}
}

sq_entry_t *sq_remafter(sq_entry_t *node, sq_queue_t *queue)
{
	sq_entry_t *ret = node->flink;

	if (queue->head && ret) {
		if (queue->tail == ret) {
			queue->tail = node;
			node->flink = NULL;

		} else {
			node->flink = ret->flink;
		}

		ret->flink = NULL;
	}

	return ret;
}

sq_entry_t *sq_remfirst(sq_queue_t *queue)
{
	sq_entry_t *ret = queue->head;

	if (ret) {
		queue->head = ret->flink;

		if (!queue->head) {
			queue->tail = NULL;
		}

		ret->flink = NULL;
	}

	return ret;
}

