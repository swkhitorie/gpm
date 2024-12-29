#include "unistd.h"

/* FreeRTOS interface include */
#include <FreeRTOS.h>
#include <task.h>

unsigned sleep(unsigned seconds)
{
    vTaskDelay(pdMS_TO_TICKS(seconds * 1000));
    return 0;
}
