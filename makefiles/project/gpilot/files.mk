################################################################################
#
# Standard set of files for the project
#
################################################################################

CSOURCES+=boards/st/cubeh7/v1.11.2/user/system_stm32h7xx.c
CSOURCES+=boards/st/cubeh7/v1.11.2/user/stm32h7xx_it.c
CSOURCES+=boards/st/cubeh7/v1.11.2/user/stm32h7xx_hal_msp.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_cortex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_dma.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_dma_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_dma2d.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_exti.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_fdcan.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_flash.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_flash_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_gpio.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_i2c.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_i2c_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_iwdg.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_pwr.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_pwr_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_rcc.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_rcc_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_rtc.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_rtc_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_spi.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_spi_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_tim.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_tim_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_uart.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_uart_ex.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_usart.c
CSOURCES+=boards/st/cubeh7/v1.11.2/drivers/stm32h7xx_hal_usart_ex.c

CSOURCES+=platforms/lld_svc/cubeh7/lld_kernel.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_interrupt.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_iwdg.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_flash.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_gpio.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_exirq.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_spi.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_uart.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_i2c.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_can.c
CSOURCES+=platforms/lld_svc/cubeh7/lld_timer.c

CSOURCES+=platforms/ttscheduler.c
CSOURCES+=platforms/devfifobuffer.c
CSOURCES+=platforms/llddebug.c

CSOURCES+=src/app/gpilot/app_main.c
#CSOURCES+=src/app/gpilot/syscall.c

#ASMSOURCES+=boards/st/cubeh7/v1.11.2/startup/arm/startup_stm32h743xx.s
ASMSOURCES+=boards/st/cubeh7/v1.11.2/startup/gcc/startup_stm32h743xx.s

#SCF_FILE+=build/scripts/linker/arm/ld_gpilot
SCF_FILE+=build/scripts/linker/gcc/ld_gpilot
