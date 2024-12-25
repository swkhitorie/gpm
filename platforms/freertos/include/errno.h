#ifndef POSIX_ERRNO_H_
#define POSIX_ERRNO_H_

extern int frerrno;
#define errno frerrno

#undef EPERM
#undef ENOENT
#undef ESRCH
#undef ENOFILE
#undef EINTR
#undef EIO
#undef ENXIO
#undef E2BIG
#undef ENOEXEC
#undef EBADF
#undef ECHILD 
#undef EAGAIN
#undef ENOMEM
#undef EFAULT
#undef EEXIST
#undef EBUSY
#undef EINVAL
#undef ENOTTY
#undef ENOSPC
#undef ERANGE
#undef ENAMETOOLONG
#undef EDEADLK
#undef EOVERFLOW
#undef ENOSYS
#undef EMSGSIZE
#undef ENOTSUP
#undef ETIMEDOUT

#define EPERM           1   /**< Operation not permitted. */
#define ENOENT          2   /**< No such file or directory. */
#define ESRCH           3
#define ENOFILE     ENOENT
#define EINTR           4
#define EIO             5
#define ENXIO           6
#define E2BIG           7
#define ENOEXEC         8
#define EBADF           9   /**< Bad file descriptor. */
#define ECHILD          10
#define EAGAIN          11  /**< Resource unavailable, try again. */
#define ENOMEM          12  /**< Not enough space. */
#define EFAULT          14
#define EBUSY           16  /**< Device or resource busy. */
#define EEXIST          17  /**< File exists. */
#define ENODEV          19
#define EINVAL          22  /**< Invalid argument. */
#define ENFILE          23
#define ENOTTY          25
#define ENOSPC          28  /**< No space left on device. */
#define ERANGE          34  /**< Result too large. */
#define ENAMETOOLONG    36  /**< File name too long. */
#define EDEADLK         45  /**< Resource deadlock would occur. */
#define EOVERFLOW       75  /**< Value too large to be stored in data type. */
#define ENOSYS          88  /**< Function not supported. */
#define EMSGSIZE        90  /**< Message too long. */
#define ENOTSUP         95  /**< Operation not supported. */
#define ETIMEDOUT       116 /**< Connection timed out. */

#endif
