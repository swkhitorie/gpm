################################################################################
# application configuration defaults section
################################################################################

PROJ_CDEFS:=\
  USE_HAL_DRIVER                         \
  STM32H743xx                            \
  USE_COMMON_UTILS

PROJ_CINCDIRS:=\
  boards/fmu-v6c/src/core               \
  boards/fmu-v6c/src/drivers            \
  boards/fmu-v6c/src/drivers/Legacy     \
  boards/fmu-v6c/src/                   \
  boards/msdk/cubeh7                    \
  platforms/common/include              \
  platforms/opmd/cherryusb/class/cdc         \
  platforms/opmd/cherryusb/common            \
  platforms/opmd/cherryusb/core              \
  platforms/opmd/cherryusb/port/dwc2         \
  platforms/opmd/cherryusb/demo              \
  src/app/fmu-v6c_app

# ld script and startup arm file
ASMSOURCES+=boards/fmu-v6c/fmu_startup.s
SCF_FILE+=boards/fmu-v6c/fmu_lnk_script

# STM32CubeH7 HAL driver
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_cortex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma2d.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_exti.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_fdcan.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_flash.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_flash_ex.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_gpio.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_i2c.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_i2c_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_iwdg.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_pwr.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_pwr_ex.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_rcc.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_rcc_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_rtc.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_rtc_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_spi.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_spi_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_tim.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_tim_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_uart.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_uart_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_usart.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_usart_ex.c

# STM32CubeH7 HAL driver enter file, config rcc and usb rcc and fmuv6c setting
CSOURCES+=boards/fmu-v6c/src/system_stm32h7xx.c
CSOURCES+=boards/fmu-v6c/src/stm32h7xx_it.c
CSOURCES+=boards/fmu-v6c/src/board_init.c
CSOURCES+=boards/fmu-v6c/src/board_usb.c

# Low Level Driver depend on STM32CubeH7 HAL
# CSOURCES+=boards/msdk/cubeh7/lld_board_kernel.c
# CSOURCES+=boards/msdk/cubeh7/lld_kernel.c
# CSOURCES+=boards/msdk/cubeh7/lld_interrupt.c
# CSOURCES+=boards/msdk/cubeh7/lld_iwdg.c
# CSOURCES+=boards/msdk/cubeh7/lld_flash.c
# CSOURCES+=boards/msdk/cubeh7/lld_gpio.c
# CSOURCES+=boards/msdk/cubeh7/lld_exirq.c
# CSOURCES+=boards/msdk/cubeh7/lld_spi.c
# CSOURCES+=boards/msdk/cubeh7/lld_uart.c
# CSOURCES+=boards/msdk/cubeh7/lld_i2c.c
# CSOURCES+=boards/msdk/cubeh7/lld_can.c
# CSOURCES+=boards/msdk/cubeh7/lld_timer.c
# CSOURCES+=platforms/common/common_utils.c

# CherryUSB
CSOURCES+=platforms/opmd/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES+=platforms/opmd/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES+=platforms/opmd/cherryusb/core/usbd_core.c
CSOURCES+=platforms/opmd/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES+=platforms/opmd/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES+=platforms/opmd/cherryusb/demo/cdc_acm_template.c

CPPSOURCES+=src/app/fmu-v6c_app/app_main.cpp
CSOURCES+=src/app/fmu-v6c_app/syscalls.c

