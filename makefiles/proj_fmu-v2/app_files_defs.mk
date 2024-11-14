################################################################################
# application configuration defaults section
################################################################################

PROJ_CDEFS:=\
  USE_HAL_DRIVER                   \
  STM32F427xx                      \
  USE_COMMON_UTILS

PROJ_CINCDIRS:=\
  boards/fmu-v2/src/core                \
  boards/fmu-v2/src/drivers             \
  boards/fmu-v2/src/drivers/Legacy      \
  boards/fmu-v2/src/                    \
  platforms/common/include              \
  platforms/cherryusb/class/cdc         \
  platforms/cherryusb/common            \
  platforms/cherryusb/core              \
  platforms/cherryusb/port/dwc2         \
  platforms/cherryusb/demo              \
  src/app/fmu-v2_app

# ld script and startup arm file
ASMSOURCES+=boards/fmu-v2/fmu_startup.s
SCF_FILE+=boards/fmu-v2/fmu_lnk_script

# STM32CubeH7 HAL driver
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal.c
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_cortex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_dma.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_dma_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_dma2d.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_exti.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_fdcan.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_flash.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_flash_ex.c
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_gpio.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_i2c.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_i2c_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_iwdg.c
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_pwr.c
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_pwr_ex.c
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_rcc.c
CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_rcc_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_rtc.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_rtc_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_spi.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_spi_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_tim.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_tim_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_uart.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_uart_ex.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_usart.c
# CSOURCES+=boards/fmu-v2/src/drivers/stm32f4xx_hal_usart_ex.c

# STM32CubeH7 HAL driver enter file, config rcc and usb rcc and fmuv2 setting
CSOURCES+=boards/fmu-v2/src/system_stm32f4xx.c
CSOURCES+=boards/fmu-v2/src/stm32f4xx_it.c
CSOURCES+=boards/fmu-v2/src/board_init.c
CSOURCES+=boards/fmu-v2/src/board_usb.c

# Low Level Driver depend on STM32CubeF4 HAL
# CSOURCES+=platforms/lld_svc/cubeh7/lld_board_kernel.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_kernel.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_interrupt.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_iwdg.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_flash.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_gpio.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_exirq.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_spi.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_uart.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_i2c.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_can.c
# CSOURCES+=platforms/lld_svc/cubeh7/lld_timer.c
# CSOURCES+=platforms/common/common_utils.c

# CherryUSB
CSOURCES+=platforms/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES+=platforms/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES+=platforms/cherryusb/core/usbd_core.c
CSOURCES+=platforms/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES+=platforms/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES+=platforms/cherryusb/demo/cdc_acm_template.c

CSOURCES+=src/app/fmu-v2_app/app_main.c
CSOURCES+=src/app/fmu-v2_app/syscalls.c

