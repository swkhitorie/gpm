#pragma once

#if defined(__PX4_NUTTX)

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include "micro_hal.h"
#include <board_config.h>

#elif defined (__PX4_POSIX)

#include "micro_hal.h"
//#include <board_config.h>

#endif
