#ifndef POSIX_QUEUE_H_
#define POSIX_QUEUE_H_

#include <sys/types.h>

#ifdef __cplusplus
#include <cstddef>
#else
#include <stddef.h>
#endif

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

#define sq_init(q) do { (q)->head = NULL; (q)->tail = NULL; } while (0)
#define dq_init(q) do { (q)->head = NULL; (q)->tail = NULL; } while (0)

#define sq_next(p) ((p)->flink)
#define dq_next(p) ((p)->flink)
#define dq_prev(p) ((p)->blink)

#define sq_empty(q) ((q)->head == NULL)
#define dq_empty(q) ((q)->head == NULL)

#define sq_peek(q)  ((q)->head)
#define dq_peek(q)  ((q)->head)

// Required for Linux
#define FAR

/************************************************************************
 * Global Type Declarations
 ************************************************************************/

struct sq_entry_s {
	FAR struct sq_entry_s *flink;
};
typedef struct sq_entry_s sq_entry_t;

struct dq_entry_s {
	FAR struct dq_entry_s *flink;
	FAR struct dq_entry_s *blink;
};
typedef struct dq_entry_s dq_entry_t;

struct sq_queue_s {
	FAR sq_entry_t *head;
	FAR sq_entry_t *tail;
};
typedef struct sq_queue_s  sq_queue_t;

struct dq_queue_s {
	FAR dq_entry_t *head;
	FAR dq_entry_t *tail;
};
typedef struct dq_queue_s dq_queue_t;

/************************************************************************
 * Global Function Prototypes
 ************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN void  sq_addfirst(FAR sq_entry_t *node, sq_queue_t *queue);
EXTERN void  dq_addfirst(FAR dq_entry_t *node, dq_queue_t *queue);
EXTERN void  sq_addlast(FAR sq_entry_t *node, sq_queue_t *queue);
EXTERN void  dq_addlast(FAR dq_entry_t *node, dq_queue_t *queue);
EXTERN void  sq_addafter(FAR sq_entry_t *prev, FAR sq_entry_t *node,
			 sq_queue_t *queue);
EXTERN void  dq_addafter(FAR dq_entry_t *prev, FAR dq_entry_t *node,
			 dq_queue_t *queue);
EXTERN void  dq_addbefore(FAR dq_entry_t *next, FAR dq_entry_t *node,
			  dq_queue_t *queue);

EXTERN FAR sq_entry_t *sq_remafter(FAR sq_entry_t *node, sq_queue_t *queue);
EXTERN void            sq_rem(FAR sq_entry_t *node, sq_queue_t *queue);
EXTERN void            dq_rem(FAR dq_entry_t *node, dq_queue_t *queue);
EXTERN FAR sq_entry_t *sq_remlast(sq_queue_t *queue);
EXTERN FAR dq_entry_t *dq_remlast(dq_queue_t *queue);
EXTERN FAR sq_entry_t *sq_remfirst(sq_queue_t *queue);
EXTERN FAR dq_entry_t *dq_remfirst(dq_queue_t *queue);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_QUEUE_H_ */

