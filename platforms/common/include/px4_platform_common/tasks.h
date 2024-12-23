#pragma once

#include <stdbool.h>
#include <pthread.h>
#include <sched.h>

typedef int px4_task_t;
typedef struct {
	int argc;
	char **argv;
} px4_task_args_t;

// PX4 work queue starting high priority
#define PX4_WQ_HP_BASE (SCHED_PRIORITY_MAX - 15)

// Fast drivers - they need to run as quickly as possible to minimize control
// latency.
#define SCHED_PRIORITY_FAST_DRIVER		(SCHED_PRIORITY_MAX - 0)

// Actuator outputs should run as soon as the rate controller publishes
// the actuator controls topic
#define SCHED_PRIORITY_ACTUATOR_OUTPUTS		(PX4_WQ_HP_BASE - 3)

// Attitude controllers typically are in a blocking wait on driver data
// they should be the first to run on an update, using the current sensor
// data and the *previous* attitude reference from the position controller
// which typically runs at a slower rate
#define SCHED_PRIORITY_ATTITUDE_CONTROL		(PX4_WQ_HP_BASE - 4)

// Estimators should run after the attitude controller but before anything
// else in the system. They wait on sensor data which is either coming
// from the sensor hub or from a driver. Keeping this class at a higher
// priority ensures that the estimator runs first if it can, but will
// wait for the sensor hub if its data is coming from it.
#define SCHED_PRIORITY_ESTIMATOR		(PX4_WQ_HP_BASE - 5)

// Position controllers typically are in a blocking wait on estimator data
// so when new sensor data is available they will run last. Keeping them
// on a high priority ensures that they are the first process to be run
// when the estimator updates.
#define SCHED_PRIORITY_POSITION_CONTROL		(PX4_WQ_HP_BASE - 7)

// The log capture (which stores log data into RAM) should run faster
// than other components, but should not run before the control pipeline
#define SCHED_PRIORITY_LOG_CAPTURE		(PX4_WQ_HP_BASE - 10)

// Slow drivers should run at a rate where they do not impact the overall
// system execution
#define SCHED_PRIORITY_SLOW_DRIVER		(PX4_WQ_HP_BASE - 35)

// The navigation system needs to execute regularly but has no realtime needs
#define SCHED_PRIORITY_NAVIGATION		(SCHED_PRIORITY_DEFAULT + 5)
//      SCHED_PRIORITY_DEFAULT
#define SCHED_PRIORITY_LOG_WRITER		(SCHED_PRIORITY_DEFAULT - 10)
#define SCHED_PRIORITY_PARAMS			(SCHED_PRIORITY_DEFAULT - 15)
//      SCHED_PRIORITY_IDLE

typedef int (*px4_main_t)(int argc, char *argv[]);

__BEGIN_DECLS

__EXPORT int px4_task_delete(px4_task_t pid);
__EXPORT int px4_task_kill(px4_task_t pid, int sig);
__EXPORT void px4_task_exit(int ret);
__EXPORT void px4_show_tasks(void);
__EXPORT bool px4_task_is_running(const char *taskname);
__EXPORT const char *px4_get_taskname(void);
__EXPORT px4_task_t px4_task_spawn_cmd(const char *name,
                        int scheduler,
                        int priority,
                        int stack_size,
                        px4_main_t entry,
				        char *const argv[]);

__END_DECLS
