#include "include/fr_posix.h"
#include "include/unistd.h"

unsigned sleep( unsigned seconds )
{
    vTaskDelay(pdMS_TO_TICKS(seconds * 1000));
    return 0;
}

int usleep( useconds_t usec )
{
    vTaskDelay(pdMS_TO_TICKS(usec / 1000 + (usec % 1000 != 0)));
    return 0;
}
