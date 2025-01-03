
PROJ_CDEFS += USE_HAL_DRIVER

ifeq (${CHIP_SOFT}, cubeh7)

#
# CubeH7 include and sources
#

PROJ_CINCDIRS += boards/edrvlib/pkg/cubeh7/core
PROJ_CINCDIRS += boards/edrvlib/pkg/cubeh7/dev
PROJ_CINCDIRS += boards/edrvlib/pkg/cubeh7/drivers

CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_cortex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_pwr.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_pwr_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_rcc.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_rcc_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_gpio.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_i2c.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_i2c_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_iwdg.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_flash.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_flash_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_uart.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_uart_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_usart.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_usart_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_dma.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_dma_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_spi.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_spi_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_exti.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_rtc.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_rtc_ex.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_tim.c
CSOURCES += boards/edrvlib/pkg/cubeh7/drivers/stm32h7xx_hal_tim_ex.c

else ifeq (${CHIP_SOFT}, cubef4)

#
# CubeF4 include and sources
#
PROJ_CINCDIRS += boards/edrvlib/pkg/cubef4/core
PROJ_CINCDIRS += boards/edrvlib/pkg/cubef4/dev
PROJ_CINCDIRS += boards/edrvlib/pkg/cubef4/drivers

CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_cortex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_dma.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_dma_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_dma2d.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_exti.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_flash.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_flash_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_gpio.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_i2c.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_i2c_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_iwdg.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_pwr.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_pwr_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_rcc.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_rcc_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_rtc.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_rtc_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_spi.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_tim.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_tim_ex.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_uart.c
CSOURCES += boards/edrvlib/pkg/cubef4/drivers/stm32f4xx_hal_usart.c


endif


