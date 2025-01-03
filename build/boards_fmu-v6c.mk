
MOD_ARCH  :=  m7

CHIP_SOFT := cubeh7

# header macros and sources list
PROJ_CDEFS := STM32H743xx

PROJ_CINCDIRS += boards/fmu-v6c

CSOURCES += boards/fmu-v6c/board_irq.c
CSOURCES += boards/fmu-v6c/board_rcc_init.c
CSOURCES += boards/fmu-v6c/board_usb.c
CSOURCES += boards/fmu-v6c/board_init.c

ASMSOURCES += boards/fmu-v6c/fmu_startup.s
SCF_FILE   += boards/fmu-v6c/fmu_lnk_script

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif
