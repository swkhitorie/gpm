#include "app_main.h"
#include "FreeRTOS.h"
#include "fr_task.h"
#include "rtklib.h"

TaskHandle_t hdebug;
void debug(void *p)
{
    struct tm *time_now;
    for (;;) {
        time_now = board_rtc_timeget();
        cdc_acm_print(0, "[core] fankeh7 running %d, %d-%d-%d|%d-%d-%d\r\n",
            HAL_GetTick(), 
            time_now->tm_year + 1900, time_now->tm_mon + 1, time_now->tm_mday,
            time_now->tm_hour, time_now->tm_min, time_now->tm_sec);
        board_blue_led_toggle();
        vTaskDelay(500);
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
}

