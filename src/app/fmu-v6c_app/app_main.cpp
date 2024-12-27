#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"

void heart(void *p)
{
    for (;;) {
        cdc_acm_print(0, "[heart] %d\r\n", HAL_GetTick());
        board_blue_led_toggle();
        board_red_led_toggle();
        vTaskDelay(500);
    }
}

int main(void)
{
    board_init();
    board_usb_init();

    xTaskCreate(heart, "heart", 1024, NULL, 3, NULL);
    vTaskStartScheduler();
    for (;;);
}

