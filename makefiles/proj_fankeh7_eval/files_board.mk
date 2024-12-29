
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
CSOURCES += boards/fankeh7/board_irq.c
CSOURCES += boards/fankeh7/board_rcc_init.c

CSOURCES += boards/fankeh7/board_hrt.c
CSOURCES += boards/fankeh7/board_usb.c
CSOURCES += boards/fankeh7/board_init.c

ASMSOURCES += boards/fankeh7/fankeh7_startup_gcc.s
SCF_FILE   += boards/fankeh7/fankeh7_lnk_gcc
