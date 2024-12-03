################################################################################
# application configuration defaults section
################################################################################

PROJ_CDEFS:=\
  USE_HAL_DRIVER                         \
  STM32H743xx                            \
  __PX4_POSIX

PROJ_CINCDIRS:=\
  boards/fmu-v6c/src/core                                 \
  boards/fmu-v6c/src/drivers                              \
  boards/fmu-v6c/src/drivers/Legacy                       \
  boards/fmu-v6c/src/                                     \
  boards/ahl                                              \
  boards/ahl/cubeh7                                       \
  platforms/freertos/include                               \
  platforms/freertos/portable/GCC/ARM_CM7/r0p1             \
  platforms/freertos/posix/include                         \
  platforms/freertos/posix/include/portable                \
  platforms/freertos/posix/include/sys                     \
  platforms/opmd/cherryusb/class/cdc                      \
  platforms/opmd/cherryusb/common                         \
  platforms/opmd/cherryusb/core                           \
  platforms/opmd/cherryusb/port/dwc2                      \
  platforms/opmd/cherryusb/demo                           \
  platforms/common/include                                \
  src/                                                    \
  src/drivers                                             \
  src/include                                             \
  src/modules/                                            \
  src/modules/uORB/                                       \
  src/lib                                                 \
  src/app/fmu-v6c_app

# ld script and startup arm file
ASMSOURCES += boards/fmu-v6c/fmu_startup.s
SCF_FILE   += boards/fmu-v6c/fmu_lnk_script
# STM32CubeH7 HAL driver, and enter file, config rcc and usb rcc and fmuv6c setting
CSOURCES += boards/fmu-v6c/src/board_init.c
CSOURCES += boards/fmu-v6c/src/board_usb.c
CSOURCES += boards/fmu-v6c/src/system_stm32h7xx.c
CSOURCES += boards/fmu-v6c/src/stm32h7xx_it.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_cortex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_pwr.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_pwr_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_rcc.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_rcc_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_gpio.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_i2c.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_i2c_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_iwdg.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_flash.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_flash_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_uart.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_uart_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_usart.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_usart_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_dma_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_spi.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_spi_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_exti.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_rtc.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_rtc_ex.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_tim.c
CSOURCES += boards/fmu-v6c/src/drivers/stm32h7xx_hal_tim_ex.c


# CSOURCES+=platforms/freertos/fr_user.c
# CSOURCES+=platforms/freertos/fr_croutine.c
# CSOURCES+=platforms/freertos/fr_event_groups.c
# CSOURCES+=platforms/freertos/fr_list.c
# CSOURCES+=platforms/freertos/fr_queue.c
# CSOURCES+=platforms/freertos/fr_stream_buffer.c
# CSOURCES+=platforms/freertos/fr_tasks.c
# CSOURCES+=platforms/freertos/fr_timers.c
# CSOURCES+=platforms/freertos/portable/MemMang/heap_4.c
# CSOURCES+=platforms/freertos/portable/GCC/ARM_CM7/r0p1/port.c
# CSOURCES+=platforms/freertos/posix/fr_posix_clock.c
# CSOURCES+=platforms/freertos/posix/fr_posix_mqueue.c
# CSOURCES+=platforms/freertos/posix/fr_posix_pthread_barrier.c
# CSOURCES+=platforms/freertos/posix/fr_posix_pthread_cond.c
# CSOURCES+=platforms/freertos/posix/fr_posix_pthread_mutex.c
# CSOURCES+=platforms/freertos/posix/fr_posix_pthread.c
# CSOURCES+=platforms/freertos/posix/fr_posix_sched.c
# CSOURCES+=platforms/freertos/posix/fr_posix_semaphore.c
# CSOURCES+=platforms/freertos/posix/fr_posix_timer.c
# CSOURCES+=platforms/freertos/posix/fr_posix_unistd.c
# CSOURCES+=platforms/freertos/posix/fr_posix_utils.c

# CSOURCES+=platforms/common/work_queue/sq_queue.c
# CSOURCES+=platforms/common/work_queue/dq_addlast.c
# CSOURCES+=platforms/common/work_queue/dq_rem.c
# CSOURCES+=platforms/common/work_queue/dq_remfirst.c
# CSOURCES+=platforms/common/work_queue/hrt_queue.c
# CSOURCES+=platforms/common/work_queue/hrt_thread.c
# CSOURCES+=platforms/common/work_queue/hrt_work_cancel.c
# CSOURCES+=platforms/common/work_queue/sq_addafter.c
# CSOURCES+=platforms/common/work_queue/sq_addlast.c
# CSOURCES+=platforms/common/work_queue/sq_remfirst.c
# CSOURCES+=platforms/common/work_queue/work_cancel.c
# CSOURCES+=platforms/common/work_queue/work_lock.c
# CSOURCES+=platforms/common/work_queue/work_thread.c

# CPPSOURCES+=platforms/common/drv_hrt.cpp
# CPPSOURCES+=platforms/common/module.cpp
# CPPSOURCES+=platforms/common/px4_sem.cpp
# CPPSOURCES+=platforms/common/px4_log.cpp
# CPPSOURCES+=platforms/common/tasks.cpp
# CPPSOURCES+=src/lib/cdev/CDev.cpp
# CPPSOURCES+=src/lib/cdev/posix/cdev_platform.cpp
# CPPSOURCES+=src/modules/uORB/Subscription.cpp
# CPPSOURCES+=src/modules/uORB/uORB.cpp
# CPPSOURCES+=src/modules/uORB/uORBDeviceMaster.cpp
# CPPSOURCES+=src/modules/uORB/uORBDeviceNode.cpp
# CPPSOURCES+=src/modules/uORB/uORBMain.cpp
# CPPSOURCES+=src/modules/uORB/uORBManager.cpp
# CPPSOURCES+=src/modules/uORB/uORBUtils.cpp

# CherryUSB
CSOURCES+=platforms/opmd/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES+=platforms/opmd/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES+=platforms/opmd/cherryusb/core/usbd_core.c
CSOURCES+=platforms/opmd/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES+=platforms/opmd/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES+=platforms/opmd/cherryusb/demo/cdc_acm_template.c

CPPSOURCES+=src/app/fmu-v6c_app/app_main.cpp
#CSOURCES+=src/app/fmu-v6c_app/syscalls.c



