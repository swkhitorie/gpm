
MOD_ARCH  :=  m7

CHIP_SOFT := cubeh7

# header macros and sources list
PROJ_CDEFS := STM32H743xx

PROJ_CINCDIRS += boards/fankeh7

CSOURCES += boards/fankeh7/board_irq.c
CSOURCES += boards/fankeh7/board_rcc_init.c
CSOURCES += boards/fankeh7/board_usb.c
CSOURCES += boards/fankeh7/board_init.c

ASMSOURCES += boards/fankeh7/fankeh7_startup_gcc.s
SCF_FILE   += boards/fankeh7/fankeh7_lnk_gcc

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif

