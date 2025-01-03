
MOD_ARCH  :=  m4

CHIP_SOFT := cubef4

# header macros and sources list
PROJ_CDEFS := STM32F427xx

PROJ_CINCDIRS += boards/fmu-v2

CSOURCES += boards/fmu-v2/board_irq.c
CSOURCES += boards/fmu-v2/board_rcc_init.c
CSOURCES += boards/fmu-v2/board_usb.c
CSOURCES += boards/fmu-v2/board_init.c

ASMSOURCES += boards/fmu-v2/fmu_startup.s
SCF_FILE   += boards/fmu-v2/fmu_lnk_script

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif
