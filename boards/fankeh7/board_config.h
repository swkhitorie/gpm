#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#ifdef __cplusplus
    extern "C" {
#endif
/** 
 * Fanke STM32H743IIT6 Evaluation Board Config Header File
 * HSE 25MHZ
 */
#include <stm32h7xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

#define APP_LOAD_ADDRESS      (0x08000000)

#define HSE_VALUE             ((uint32_t)25000000)
#define LSE_VALUE             ((uint32_t)32768)
#define __FPU_PRESENT         1
#define __FPU_USED            1

#define GPIO_nLED_BLUE_PORT   (GPIOH)
#define GPIO_nLED_BLUE_PIN    (GPIO_PIN_7)

#define BOARD_BLUE_LED(on_true)            HAL_GPIO_WritePin(GPIO_nLED_BLUE_PORT, \
                                                    GPIO_nLED_BLUE_PIN, !(on_true))
void board_irqreset();

void board_reboot();

void board_init();

void board_usb_init();

void board_blue_led_toggle();

#ifdef __cplusplus
}
#endif

#endif
