################################################################################
# application configuration defaults section
################################################################################

PROJ_CDEFS:=\
  USE_HAL_DRIVER                         \
  STM32H743xx                            \
  USE_COMMON_UTILS                       \
  __PX4_POSIX

PROJ_CINCDIRS:=\
  boards/fmu-v6c/src/core                                 \
  boards/fmu-v6c/src/drivers                              \
  boards/fmu-v6c/src/drivers/Legacy                       \
  boards/fmu-v6c/src/                                     \
  boards/ahl                                              \
  boards/ahl/cubeh7                                       \
  platforms/common/include                                \
  platforms/opmd/cherryusb/class/cdc                      \
  platforms/opmd/cherryusb/common                         \
  platforms/opmd/cherryusb/core                           \
  platforms/opmd/cherryusb/port/dwc2                      \
  platforms/opmd/cherryusb/demo                           \
  platforms/rtos/fr/include                               \
  platforms/rtos/fr/portable/GCC/ARM_CM7/r0p1             \
  platforms/rtos/posix/fr/include                         \
  platforms/rtos/posix/fr/include/portable                \
  platforms/rtos/posix/fr/include/sys                     \
  src/                                                    \
  src/drivers                                             \
  src/include                                             \
  src/modules/                                            \
  src/modules/uORB/                                       \
  src/lib                                                 \
  src/app/fmu-v6c_app

# ld script and startup arm file
ASMSOURCES+=boards/fmu-v6c/fmu_startup.s
SCF_FILE+=boards/fmu-v6c/fmu_lnk_script

# STM32CubeH7 HAL driver
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_cortex.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_pwr.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_pwr_ex.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_rcc.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_rcc_ex.c
CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_gpio.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma2d.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_exti.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_fdcan.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_flash.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_flash_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_i2c.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_i2c_ex.c
# CSOURCES+=boards/fmu-v6c/src/drivers/stm32h7xx_hal_iwdg.c
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
CSOURCES+=platforms/common/work_queue/sq_queue.c

CSOURCES+=platforms/rtos/fr/fr_user.c
CSOURCES+=platforms/rtos/fr/fr_croutine.c
CSOURCES+=platforms/rtos/fr/fr_event_groups.c
CSOURCES+=platforms/rtos/fr/fr_list.c
CSOURCES+=platforms/rtos/fr/fr_queue.c
CSOURCES+=platforms/rtos/fr/fr_stream_buffer.c
CSOURCES+=platforms/rtos/fr/fr_tasks.c
CSOURCES+=platforms/rtos/fr/fr_timers.c
CSOURCES+=platforms/rtos/fr/portable/MemMang/heap_4.c
CSOURCES+=platforms/rtos/fr/portable/GCC/ARM_CM7/r0p1/port.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_clock.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_mqueue.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_pthread_barrier.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_pthread_cond.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_pthread_mutex.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_pthread.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_sched.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_semaphore.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_timer.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_unistd.c
CSOURCES+=platforms/rtos/posix/fr/fr_posix_utils.c

# CherryUSB
CSOURCES+=platforms/opmd/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES+=platforms/opmd/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES+=platforms/opmd/cherryusb/core/usbd_core.c
CSOURCES+=platforms/opmd/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES+=platforms/opmd/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES+=platforms/opmd/cherryusb/demo/cdc_acm_template.c

CPPSOURCES+=src/app/fmu-v6c_app/app_main.cpp
#CSOURCES+=src/app/fmu-v6c_app/syscalls.c

CPPSOURCES+=src/modules/uORB/Subscription.cpp
CPPSOURCES+=src/modules/uORB/uORB.cpp
CPPSOURCES+=src/modules/uORB/uORBDeviceMaster.cpp
CPPSOURCES+=src/modules/uORB/uORBDeviceNode.cpp
CPPSOURCES+=src/modules/uORB/uORBMain.cpp
CPPSOURCES+=src/modules/uORB/uORBManager.cpp
CPPSOURCES+=src/modules/uORB/uORBUtils.cpp


