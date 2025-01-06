
MOD_ARCH  :=  m4

CHIP_SOFT := F4_HAL

# header macros and sources list
PROJ_CDEFS := STM32F427xx

PROJ_CINCDIRS += boards/bsp/stm32/stm32f427_fmuv2/

CSOURCES += boards/bsp/stm32/stm32f427_fmuv2/board_irq.c
CSOURCES += boards/bsp/stm32/stm32f427_fmuv2/board_rcc_init.c
CSOURCES += boards/bsp/stm32/stm32f427_fmuv2/board_usb.c
CSOURCES += boards/bsp/stm32/stm32f427_fmuv2/board_init.c

ASMSOURCES += boards/bsp/stm32/stm32f427_fmuv2/fmu_startup.s
SCF_FILE   += boards/bsp/stm32/stm32f427_fmuv2/fmu_lnk_script

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif
