#include "app_main.h"
#include <stdio.h>

void heart(void *p)
{
    char newname[] = "h2_d1";
    pcTaskSetName(xTaskGetCurrentTaskHandle(), &newname[0]);
    for (;;) {
        fprintf(stdout, "[heart] stdio %s\r\n", pcTaskGetName(xTaskGetCurrentTaskHandle()));
        board_blue_led_toggle();
        vTaskDelay(500);
    }
}

void fr_start(void *p);

int main(void)
{
    board_init();
    board_usb_init();

    xTaskCreate(heart, "ht_debug", 1024, NULL, 3, NULL);
    vTaskStartScheduler();
    for (;;);
}

void fr_start(void *p)
{

}
