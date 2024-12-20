#include "include/fr_posix.h"
#include "include/sched.h"

int sched_get_priority_max(int policy)
{
    (void)policy;

    return configMAX_PRIORITIES - 1;
}

int sched_get_priority_min(int policy)
{
    (void)policy;

    return tskIDLE_PRIORITY;
}

int sched_yield()
{
    taskYIELD();
    return 0;
}
