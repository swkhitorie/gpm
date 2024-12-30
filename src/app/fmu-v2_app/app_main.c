#include "app_main.h"

int main(void)
{
    board_init();
    board_usb_init();

    uint32_t tick = HAL_GetTick();

    while (1) {
        if (HAL_GetTick() - tick > 500) {
            tick = HAL_GetTick();
            cdc_acm_print(0, "[heart] v2\r\n");
            board_led_toggle();
        }
    }
}
