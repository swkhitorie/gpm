################################################################################
#
# Standard set of files for the project
#
################################################################################

ASMSOURCES+=boards/fankeh7/fankeh7_startup_arm.s
SCF_FILE+=boards/fankeh7/fankeh7_lnk_arm
CSOURCES+=boards/fankeh7/src/system_stm32h7xx.c
CSOURCES+=boards/fankeh7/src/stm32h7xx_it.c
CSOURCES+=boards/fankeh7/src/stm32h7xx_hal_msp.c

CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_cortex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_dma.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_dma_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_dma2d.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_exti.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_fdcan.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_flash.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_flash_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_gpio.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_i2c.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_i2c_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_iwdg.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_pwr.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_pwr_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_rcc.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_rcc_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_rtc.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_rtc_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_spi.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_spi_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_tim.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_tim_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_uart.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_uart_ex.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_usart.c
CSOURCES+=boards/fankeh7/src/drivers/stm32h7xx_hal_usart_ex.c

CSOURCES+=platforms/component/lld_svc/cubeh7/lld_board_kernel.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_kernel.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_interrupt.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_iwdg.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_flash.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_gpio.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_exirq.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_spi.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_uart.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_i2c.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_can.c
CSOURCES+=platforms/component/lld_svc/cubeh7/lld_timer.c

CSOURCES+=platforms/common/ttscheduler.c
CSOURCES+=platforms/common/devfifobuffer.c
CSOURCES+=platforms/common/llddebug.c

CPPSOURCES+=src/app/fankeh7_eval/app_main.cpp
