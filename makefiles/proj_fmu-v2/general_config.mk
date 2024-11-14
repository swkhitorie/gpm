################################################################################
#
# application configuration file
#
################################################################################
PROJ_NAME         :=  fmu-v2

#
# Toolchain used in this project
#
PROJ_TC           :=  gae

MOD_ARCH          :=  m4

PROJ_OPENOCD_DEBUG := stlink

PROJ_OPENOCD_CHIP := stm32h7x

PROJ_OPENOCD_LOAD_ADDR := 0x08000000

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif

TARGET_POSTBUILD      := ${TARGET_DEST_FILENAME_BIN}

