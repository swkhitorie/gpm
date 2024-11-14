#include "app_main.h"

int main(void)
{
	board_app_vector_init();
	board_io_array_init();
	board_app_init();

    HAL_Init();

    board_system_rcc_config();

    HAL_Delay(1000);
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);
    HAL_Delay(800);

    uint32_t tick = HAL_GetTick();

    while (1) {
	  if (HAL_GetTick() - tick > 500) {
		  tick = HAL_GetTick();
		  cdc_acm_data_send_with_dtr_test(0);
		  board_led_toggle();
	  }
    }
}