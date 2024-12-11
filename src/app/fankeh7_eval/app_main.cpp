#include "app_main.h"
#include "FreeRTOS.h"
#include "fr_task.h"
#include <cubeh7/include/lld_uart.h>
#include "rtklib.h"

int ires;
lld_uart_t com1;
uint8_t rcvbuf[1024];

rtcm_t msgf;
int rtcm_res;

char tmpaa[] = "get frame \r\n";

void debug(void *p)
{
    for (;;) {
        int rsize = devbuf_size(&com1.rxbuf);
        if (rsize > 0) {
            devbuf_read(&com1.rxbuf, (uint8_t *)&rcvbuf[0], rsize);
            for (int i = 0; i < rsize; i++) {
                rtcm_res = input_rtcm3(&msgf, rcvbuf[i]);
                if (rtcm_res != 0) {
                }
            }
        }
    }
}

void heart(void *p)
{
    for (;;) {
        cdc_acm_print(0, "[heart] %d %d \r\n", devbuf_size(&com1.rxbuf), ires);
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

    ires = init_rtcm(&msgf);
    lld_uart_init(&com1, 1, 460800, 1, 1, true, true, UART_PARITY_NONE, 
        UART_WORDLENGTH_8B, UART_STOPBITS_1);

    BaseType_t res;
    xTaskCreate(debug, "debug", 256, NULL, 2, NULL);
    xTaskCreate(heart, "heart", 1024, NULL, 3, NULL);

    vTaskStartScheduler();
    for (;;);
}

