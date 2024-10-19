#pragma once

#include "platform_defines.h"
#include "mavlink_vehicle.h"

#define ecl_absolute_time hrt_absolute_time
#define ecl_elapsed_time hrt_elapsed_time

#define ISFINITE(x) PX4_ISFINITE(x)

#define ecl_absolute_time() (0)
#define ecl_elapsed_time(t) (*t * 0UL) // TODO: add simple time functions

using ecl_abstime = uint64_t;

#define ECL_INFO(...) mavlink_log_info(MAVLINK_COMM_1, __VA_ARGS__)
#define ECL_WARN(...) mavlink_log_warning(MAVLINK_COMM_1, __VA_ARGS__)
#define ECL_ERR(...) mavlink_log_error(MAVLINK_COMM_1, __VA_ARGS__)
#define ECL_INFO_TIMESTAMPED(...) //PX4_INFO
#define ECL_WARN_TIMESTAMPED(...) //PX4_WARN
#define ECL_ERR_TIMESTAMPED(...) //PX4_ERR