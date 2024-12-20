/*
 * Amazon FreeRTOS POSIX V1.1.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file errno.h
 * @brief System error numbers.
 *
 * http://pubs.opengroup.org/onlinepubs/9699919799/basedefs/errno.h.html
 *
 * The values defined in this file may not be compatible with the strerror
 * function provided by this system.
 */

#ifndef _FREERTOS_POSIX_ERRNO_H_
#define _FREERTOS_POSIX_ERRNO_H_

/* Undefine all errnos to avoid redefinition errors with system errnos. */
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

/**
 * @name Definition of POSIX errnos.
 */
/**@{ */
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
/**@} */

/**
#define EBADF 9
#define EAGAIN 11
#define ENOMEM 12
#define EACCES 13
#define EFAULT 14
#define EBUSY 16
#define EEXIST 17
#define EXDEV 18
#define ENODEV 19
#define ENOTDIR 20
#define EISDIR 21
#define ENFILE 23
#define EMFILE 24
#define ENOTTY 25
#define EFBIG 27
#define ENOSPC 28
#define ESPIPE 29
#define EROFS 30
#define EMLINK 31
#define EPIPE 32
#define EDOM 33
#define EDEADLK 36
#define ENAMETOOLONG 38
#define ENOLCK 39
#define ENOSYS 40
#define ENOTEMPTY 41
 */


/**
 * @name System Variable
 *
 * @brief Define FreeRTOS+POSIX errno, if enabled.
 * Set configUSE_POSIX_ERRNO to enable, and clear to disable. See FreeRTOS.h.
 *
 * @{
 */
#ifndef configUSE_POSIX_ERRNO
    #define configUSE_POSIX_ERRNO 1
#endif
#if ( configUSE_POSIX_ERRNO == 1 )
    extern int FreeRTOS_errno;
    #define errno FreeRTOS_errno
#endif
/**@} */

#endif /* ifndef _FREERTOS_POSIX_ERRNO_H_ */
