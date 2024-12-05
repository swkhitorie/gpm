#include "app_main.h"
#include "FreeRTOS.h"
#include "fr_task.h"

TaskHandle_t hdebug;
void debug(void *p)
{
    for (;;) {
        cdc_acm_print(0, "hello world %d \r\n", HAL_GetTick());
        board_blue_led_toggle();
        vTaskDelay(500);
    }
}

void debug2()
{
    uint32_t tick = HAL_GetTick();
    for (;;) {
        if (HAL_GetTick() - tick >= 500) {
            tick = HAL_GetTick();
            cdc_acm_print(0, "hello world %d \r\n", HAL_GetTick());
            board_blue_led_toggle();
        }
    }
}

int main(void)
{
	board_app_vector_init();
    board_system_config();
    board_system_rcc_config();
    
	board_io_array_init();
	board_app_init();

    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);

    BaseType_t res = xTaskCreate(debug, "debug", 256, NULL, 2, &hdebug);
    vTaskStartScheduler();
    for (;;);
    debug2();
}

