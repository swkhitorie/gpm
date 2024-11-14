#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#ifdef __cplusplus
    extern "C" {
#endif
/**
 * board pixhawk v2 file, rely on STM32CubeH4 V1.8.2
 */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define APP_LOAD_ADDRESS     (0x08004000)

#define GPIO_nLED_PORT   (GPIOE)
#define GPIO_nLED_PIN    (GPIO_PIN_12)

#define BOARD_LED(on_true)            HAL_GPIO_WritePin(GPIO_nLED_BLUE_PORT, \
                                     GPIO_nLED_BLUE_PIN, !(on_true))

void board_app_vector_init();

void board_io_array_init();

void board_app_init();

void board_led_toggle();

void board_system_rcc_config();

#ifdef __cplusplus
}
#endif

#endif
