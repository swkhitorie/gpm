#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#ifdef __cplusplus
    extern "C" {
#endif
/**
 * board pixhawk v6c file, rely on STM32CubeH7 v1.11.2
 *
 *  bug 1: v6c can not enter SysTick_Handler() irq
 *        -> fixed: call __set_BASEPRI(0); of set it priority higher, default 15
 *  bug 2: v6c can not enter OTG_FS_IRQHandler() irq
 *        change h7 usb's clock source HSI48 -> PLL3Q 
 *        edit VID and PID to default 0xFFFF, why????
 */
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#define APP_LOAD_ADDRESS     (0x08020000)

#define GPIO_nLED_RED_PORT   (GPIOD)
#define GPIO_nLED_RED_PIN    (GPIO_PIN_10)

#define GPIO_nLED_BLUE_PORT   (GPIOD)
#define GPIO_nLED_BLUE_PIN    (GPIO_PIN_11)

#define GPIO_nPOWER_IN_A_PORT   (GPIOA)
#define GPIO_nPOWER_IN_A_PIN    (GPIO_PIN_15)

#define GPIO_nPOWER_IN_B_PORT   (GPIOB)
#define GPIO_nPOWER_IN_B_PIN    (GPIO_PIN_12)

#define GPIO_nPOWER_IN_C_PORT   (GPIOE)
#define GPIO_nPOWER_IN_C_PIN    (GPIO_PIN_15)

#define GPIO_VDD_5V_PERIPH_nEN_PORT  (GPIOE)
#define GPIO_VDD_5V_PERIPH_nEN_PIN  (GPIO_PIN_2)

#define GPIO_VDD_5V_PERIPH_nOC_PORT  (GPIOE)
#define GPIO_VDD_5V_PERIPH_nOC_PIN  (GPIO_PIN_3)

#define GPIO_VDD_5V_HIPOWER_nEN_PORT  (GPIOC)
#define GPIO_VDD_5V_HIPOWER_nEN_PIN  (GPIO_PIN_10)

#define GPIO_VDD_5V_HIPOWER_nOC_PORT  (GPIOC)
#define GPIO_VDD_5V_HIPOWER_nOC_PIN  (GPIO_PIN_11)

#define GPIO_VDD_3V3_SENSORS_EN_PORT  (GPIOB)
#define GPIO_VDD_3V3_SENSORS_EN_PIN  (GPIO_PIN_2)

#define GPIO_OTGFS_VBUS_PORT           (GPIOA)
#define GPIO_OTGFS_VBUS_PIN           (GPIO_PIN_9)

#define VDD_5V_PERIPH_EN(on_true)          HAL_GPIO_WritePin(GPIO_VDD_5V_PERIPH_nOC_PORT, \
                                                   GPIO_VDD_5V_PERIPH_nEN_PIN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         HAL_GPIO_WritePin(GPIO_VDD_5V_HIPOWER_nEN_PORT, \
                                                   GPIO_VDD_5V_HIPOWER_nEN_PIN, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)       HAL_GPIO_WritePin(GPIO_VDD_3V3_SENSORS_EN_PORT, \
                                                   GPIO_VDD_3V3_SENSORS_EN_PIN, (on_true))

#define BOARD_ADC_USB_CONNECTED           (HAL_GPIO_ReadPin(GPIO_OTGFS_VBUS_PORT, \
                                                   GPIO_OTGFS_VBUS_PIN))

#define BOARD_ADC_USB_VALID               (!HAL_GPIO_ReadPin(GPIO_nPOWER_IN_C_PORT, \
                                                   GPIO_nPOWER_IN_C_PIN))

#define BOARD_ADC_BRICK1_VALID  (!HAL_GPIO_ReadPin(GPIO_nPOWER_IN_A_PORT, GPIO_nPOWER_IN_A_PIN))
#define BOARD_ADC_BRICK2_VALID  (!HAL_GPIO_ReadPin(GPIO_nPOWER_IN_B_PORT, GPIO_nPOWER_IN_B_PIN))

#define BOARD_ADC_PERIPH_5V_OC  (!HAL_GPIO_ReadPin(GPIO_VDD_5V_PERIPH_nOC_PORT, \
                                                   GPIO_VDD_5V_PERIPH_nOC_PIN))
#define BOARD_ADC_HIPOWER_5V_OC (!HAL_GPIO_ReadPin(GPIO_VDD_5V_HIPOWER_nOC_PORT, \
                                                   GPIO_VDD_5V_HIPOWER_nOC_PIN))
#define BOARD_BLUE_LED(on_true)            HAL_GPIO_WritePin(GPIO_nLED_BLUE_PORT, \
                                                   GPIO_nLED_BLUE_PIN, !(on_true))
#define BOARD_RED_LED(on_true)            HAL_GPIO_WritePin(GPIO_nLED_RED_PORT, \
                                                   GPIO_nLED_RED_PIN, !(on_true))

void board_system_rcc_config();

void board_system_config();

void board_app_vector_init();

void board_io_array_init();

void board_app_init();

void board_red_led_toggle();

void board_blue_led_toggle();

void board_debug_delay();

#ifdef __cplusplus
}
#endif

#endif
