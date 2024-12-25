#ifndef POSIX_UNISTD_H_
#define POSIX_UNISTD_H_

#include "sys/types.h"

#define	F_OK	0

unsigned sleep     (unsigned seconds);
int      usleep    (useconds_t usec);

#endif
