################################################################################
# application configuration defaults section
################################################################################
  # STM32F40_41xxx                         
  # STM32F407xx                
PROJ_CDEFS:=\
  STM32F40_41xxx                      \
  USE_STDPERIPH_DRIVER                \
  __FPU_PRESENT                       \
  __CC_ARM

PROJ_CINCDIRS:=\
  boards/quadpilot/src/core           \
  boards/quadpilot/src/driver         \
  boards/quadpilot/src                \
  platforms/common                    \
  platforms/common/include            \
  platforms/component/lld_svc/stdf4   \
  src/app/quadpilot

CSOURCES+=boards/quadpilot/src/driver/misc.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_adc.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_dac.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_dma.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_exti.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_flash.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_gpio.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_i2c.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_iwdg.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_pwr.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_rcc.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_rtc.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_spi.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_syscfg.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_tim.c
CSOURCES+=boards/quadpilot/src/driver/stm32f4xx_usart.c
CSOURCES+=boards/quadpilot/src/system_stm32f4xx.c
CSOURCES+=boards/quadpilot/src/stm32f4xx_it.c

CSOURCES+=platforms/component/lld_svc/stdf4/lld_kernel.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_interrupt.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_gpio.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_flash.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_uart.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_adc.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_dac.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_exit.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_spi.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_i2c.c
CSOURCES+=platforms/component/lld_svc/stdf4/lld_timer.c
CSOURCES+=platforms/common/ttscheduler.c
CSOURCES+=platforms/common/common_utils.c

CPPSOURCES+=src/app/quadpilot_eval/test_cpp.cpp
CPPSOURCES+=src/app/quadpilot_eval/app_main.cpp

ASMSOURCES+=boards/quadpilot/quadpilot_startup_arm.s
SCF_FILE+=boards/quadpilot/quadpilot_lnk_arm
