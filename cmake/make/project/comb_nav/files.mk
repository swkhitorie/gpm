################################################################################
#
# Standard set of files for the project
#
################################################################################

#
# stdf4 lib only support arm compiler v5
#

CSOURCES+=boards/st/stdf4/v1.9.0/user/system_stm32f4xx.c
CSOURCES+=boards/st/stdf4/v1.9.0/user/stm32f4xx_it.c

CSOURCES+=boards/st/stdf4/v1.9.0/driver/misc.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_adc.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_dac.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_dma.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_exti.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_flash.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_gpio.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_i2c.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_iwdg.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_pwr.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_rcc.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_rtc.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_spi.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_syscfg.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_tim.c
CSOURCES+=boards/st/stdf4/v1.9.0/driver/stm32f4xx_usart.c

CSOURCES+=platforms/lld_svc/stdf4/lld_kernel.c
CSOURCES+=platforms/lld_svc/stdf4/lld_interrupt.c
CSOURCES+=platforms/lld_svc/stdf4/lld_gpio.c
CSOURCES+=platforms/lld_svc/stdf4/lld_flash.c
CSOURCES+=platforms/lld_svc/stdf4/lld_uart.c
CSOURCES+=platforms/lld_svc/stdf4/lld_adc.c
CSOURCES+=platforms/lld_svc/stdf4/lld_dac.c
CSOURCES+=platforms/lld_svc/stdf4/lld_exit.c
CSOURCES+=platforms/lld_svc/stdf4/lld_spi.c
CSOURCES+=platforms/lld_svc/stdf4/lld_i2c.c
CSOURCES+=platforms/lld_svc/stdf4/lld_timer.c
CSOURCES+=platforms/ttscheduler.c
CSOURCES+=platforms/devfifobuffer.c
CSOURCES+=platforms/llddebug.c

#CSOURCES+=src/app/comb_nav/syscall.c
CPPSOURCES+=src/app/comb_nav/test_cpp.cpp
CPPSOURCES+=src/app/comb_nav/app_main.cpp


#ASMSOURCES+=boards/st/stdf4/v1.9.0/startup/arm/startup_stm32f40_41xxx.s
#ASMSOURCES+=boards/st/stdf4/v1.9.0/startup/gcc_ride7/startup_stm32f40_41xxx.s
ASMSOURCES+=boards/st/stdf4/v1.9.0/startup/arm/startup_stm32f427x.s


#SCF_FILE+=build/scripts/linker/gcc/ld_comb_nav
SCF_FILE+=build/scripts/linker/arm/ld_comb_nav
