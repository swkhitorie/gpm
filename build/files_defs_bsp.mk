
PROJ_CDEFS += USE_HAL_DRIVER

PROJ_CINCDIRS += boards/bsp/stm32/libs/cmsis

ifeq (${CHIP_SOFT}, H7_HAL)

#
# CubeH7 include and sources
#

PROJ_CINCDIRS += boards/bsp/stm32/libs/stm32h7xx_hal/device
PROJ_CINCDIRS += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver

CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_cortex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_pwr.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_pwr_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_rcc.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_rcc_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_gpio.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_i2c.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_i2c_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_iwdg.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_flash.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_flash_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_uart.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_uart_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_usart.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_usart_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_dma.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_dma_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_spi.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_spi_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_exti.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_rtc.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_rtc_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_tim.c
CSOURCES += boards/bsp/stm32/libs/stm32h7xx_hal/hal_driver/stm32h7xx_hal_tim_ex.c

else ifeq (${CHIP_SOFT}, F4_HAL)

#
# CubeF4 include and sources
#
PROJ_CINCDIRS += boards/bsp/stm32/libs/stm32f4xx_hal/device
PROJ_CINCDIRS += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver

CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_cortex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_dma.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_dma_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_dma2d.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_exti.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_flash.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_flash_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_gpio.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_i2c.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_i2c_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_iwdg.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_pwr.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_pwr_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_rcc.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_rcc_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_rtc.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_rtc_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_spi.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_tim.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_tim_ex.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_uart.c
CSOURCES += boards/bsp/stm32/libs/stm32f4xx_hal/hal_driver/stm32f4xx_hal_usart.c


endif


