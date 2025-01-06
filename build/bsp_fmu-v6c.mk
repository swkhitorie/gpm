
MOD_ARCH  :=  m7

CHIP_SOFT := H7_HAL

# header macros and sources list
PROJ_CDEFS := STM32H743xx

PROJ_CINCDIRS += boards/bsp/stm32/stm32h743_fmuv6c

CSOURCES += boards/bsp/stm32/stm32h743_fmuv6c/board_irq.c
CSOURCES += boards/bsp/stm32/stm32h743_fmuv6c/board_rcc_init.c
CSOURCES += boards/bsp/stm32/stm32h743_fmuv6c/board_usb.c
CSOURCES += boards/bsp/stm32/stm32h743_fmuv6c/board_init.c

ASMSOURCES += boards/bsp/stm32/stm32h743_fmuv6c/fmu_startup.s
SCF_FILE   += boards/bsp/stm32/stm32h743_fmuv6c/fmu_lnk_script

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif
