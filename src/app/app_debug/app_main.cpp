#include "./app_main.h"
#include <stdio.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

void debug_led_toggle()
{
#if   BOARD_SELECT == BOARD_FMUV2
    board_led_toggle();
#elif BOARD_SELECT == BOARD_FANKEH7
    board_blue_led_toggle();
#elif BOARD_SELECT == BOARD_FMUV6C
    board_blue_led_toggle();
    board_red_led_toggle();
#endif
}

void fr_heart(void *p)
{
    char newname[] = "h2_d1";
    pcTaskSetName(xTaskGetCurrentTaskHandle(), &newname[0]);
    for (;;) {
        float a1 = HAL_GetTick()/1e3f;
        float a2 = hrt_absolute_time()/1e6f;
        fprintf(stdout, "[heart] %s, kernel %.6f %.6f %.6f\r\n", 
            pcTaskGetName(xTaskGetCurrentTaskHandle()), 
            a1, a2, a2-a1);
        debug_led_toggle();
        vTaskDelay(500);
    }
}

void debug()
{
    uint32_t tickm = HAL_GetTick();
    if (HAL_GetTick() - tickm >= 100) {
        tickm = HAL_GetTick();
        fprintf(stdout, "[heart] stdio %s, kernel %d\r\n", 
            pcTaskGetName(xTaskGetCurrentTaskHandle()), HAL_GetTick());
        debug_led_toggle();
    }
}

int main(void)
{
    board_init();

    hrt_init();

    xTaskCreate(fr_heart, "ht_debug", 1024, NULL, 3, NULL);
    vTaskStartScheduler();
    for (;;);
}
