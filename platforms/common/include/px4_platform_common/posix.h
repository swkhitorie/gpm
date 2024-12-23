#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include "sem.h"

#define  PX4_F_RDONLY 1
#define  PX4_F_WRONLY 2
#define	 PX4_STACK_OVERHEAD	(1024 * 11)

__BEGIN_DECLS

typedef short gpm_pollevent_t;

typedef struct {
	/* This part of the struct is POSIX-like */
	int		fd;       /* The descriptor being polled */
	gpm_pollevent_t 	events;   /* The input event flags */
	gpm_pollevent_t 	revents;  /* The output event flags */

	/* Required for PX4 compatibility */
	gpm_sem_t   *sem;  	/* Pointer to semaphore used to post output event */
	void   *priv;     	/* For use by drivers */
} gpm_pollfd_struct_t;

#ifndef POLLIN
#define POLLIN       (0x01)
#endif

__EXPORT int 		px4_open(const char *path, int flags, ...);
__EXPORT int 		px4_close(int fd);
__EXPORT ssize_t	px4_read(int fd, void *buffer, size_t buflen);
__EXPORT ssize_t	px4_write(int fd, const void *buffer, size_t buflen);
__EXPORT int		px4_ioctl(int fd, int cmd, unsigned long arg);
__EXPORT int		px4_poll(gpm_pollfd_struct_t *fds, unsigned int nfds, int timeout);
__EXPORT int		px4_access(const char *pathname, int mode);
__EXPORT px4_task_t	px4_getpid(void);

__END_DECLS

// The stack size is intended for 32-bit architectures; therefore
// we often run out of stack space when pointers are larger than 4 bytes.
// Double the stack size on posix when we're on a 64-bit architecture.
// Most full-scale OS use 1-4K of memory from the stack themselves
#define PX4_STACK_ADJUSTED(_s) (_s * (__SIZEOF_POINTER__ >> 2) + PX4_STACK_OVERHEAD)

__BEGIN_DECLS

__EXPORT void		px4_show_files(void);

__END_DECLS
