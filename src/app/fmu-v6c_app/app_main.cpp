#include "app_main.h"
#include "FreeRTOS.h"
#include "fr_task.h"

TaskHandle_t start_handle;
TaskHandle_t debug_handle;

void task_debug(void *pv) 
{
    static portTickType xLastWakeTime;  
    const portTickType xFrequency = pdMS_TO_TICKS(5);
    xLastWakeTime = xTaskGetTickCount();
	while(1) {
        cdc_acm_print(0, "hello world\r\n");
        board_blue_led_toggle();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}   

void task_start(void *pv)
{
	//BaseType_t xReturn;
    //taskENTER_CRITICAL();
//cdc_acm_print(0, "enter start \r\n");
	//xReturn = xTaskCreate(task_debug, "debug_task", 512, NULL, 2, &debug_handle);
    //cdc_acm_print(0, "create : %d\r\n", xReturn);
    //vTaskDelete(start_handle);
    //taskEXIT_CRITICAL();
}

int main(void)
{
	board_app_vector_init();
    board_system_config();
    board_system_rcc_config();

	board_io_array_init();
	board_app_init();

    
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);

	xTaskCreate(task_start, "start", 128, NULL, 1, &start_handle);		
	vTaskStartScheduler();
    while(1) {}

    // uint32_t tick = HAL_GetTick();
    // while (1) {
    //     if (HAL_GetTick() - tick >= 500) {
    //         tick = HAL_GetTick();
    //         cdc_acm_print(0, "hello world\r\n");
    //         board_blue_led_toggle();
    //     }
    // }
}
