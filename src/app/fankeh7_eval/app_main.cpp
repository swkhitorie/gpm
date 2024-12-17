#include "app_main.h"
#include "FreeRTOS.h"
#include "fr_task.h"
#include <cubeh7/include/lld_uart.h>

lld_uart_t com1;

void heart(void *p)
{
    for (;;) {
        cdc_acm_print(0, "[heart] %d %d \r\n", devbuf_size(&com1.rxbuf));
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

    lld_uart_init(&com1, 1, 460800, 1, 1, true, true, UART_PARITY_NONE, 
        UART_WORDLENGTH_8B, UART_STOPBITS_1);

    BaseType_t res;
    xTaskCreate(heart, "heart", 1024, NULL, 3, NULL);
    vTaskStartScheduler();
    for (;;);
}

