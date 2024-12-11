#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#ifdef __cplusplus
    extern "C" {
#endif
/**
 * board fankeh7 file, rely on STM32CubeH7 v1.11.2
 *
 *  
 */
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#define APP_LOAD_ADDRESS     (0x08000000)

#define GPIO_nLED_BLUE_PORT   (GPIOH)
#define GPIO_nLED_BLUE_PIN    (GPIO_PIN_7)

#define BOARD_BLUE_LED(on_true)            HAL_GPIO_WritePin(GPIO_nLED_BLUE_PORT, \
                                                   GPIO_nLED_BLUE_PIN, !(on_true))

void board_system_rcc_config();

void board_system_config();

void board_app_vector_init();

void board_io_array_init();

void board_app_init();

void board_blue_led_toggle();

void board_debug_delay();

void board_reboot();
void board_rtc_init();
struct tm *board_rtc_timeget();
void board_rtc_timeset(struct tm *val);
uint64_t board_kernel_timeget(uint8_t way);

#ifdef __cplusplus
}
#endif

#endif
