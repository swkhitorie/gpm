
#
# application configuration file
#
PROJ_NAME  :=  fankeh7_eval

#
# Toolchain used in this project
#
PROJ_TC  :=  gae

TARGET_POSTBUILD := ${TARGET_DEST_FILENAME_BIN}

#####################################
# bsp configuration
#####################################

BSP_ROOT           := boards/bsp
BSP_ABSROOTDIR     := ${SDK_ROOTDIR}/${BSP_ROOT}

CONFIG_PX4_HRTIMER := y
include ${BSP_ABSROOTDIR}/stm32/stm32h743_eval/make.mk

PROJ_CDEFS        += ${BSP_CDEFS}

CSOURCES          += ${BSP_CSRCS}

PROJ_CINCDIRS     += ${BSP_CINCDIRS}

ASMSOURCES        := ${BSP_ASMSOURCES}

SCF_FILE          := ${BSP_LNK_FILE}

PROJ_ENTRY_POINT  := ${BSP_BOARD_ENTRY_POINT}

PROJ_OPENOCD_LOAD_ADDR := 0x08000000

MOD_ARCH  =  m7

#####################################
# module configuration
#####################################

include ${SDK_ROOTDIR}/src/app/module_config.mk

#####################################
# app configuration
#####################################

PROJ_CDEFS += BSP_COM_PRINTF
PROJ_CDEFS += BSP_MODULE_FR
#PROJ_CDEFS += BSP_MODULE_USB_CHERRY

PROJ_CINCDIRS += src/app

CPPSOURCES += src/app/app_debug/app_main.cpp
CPPSOURCES += src/app/app_debug/app_posix_debug.cpp
CPPSOURCES += src/app/app_debug/app_px4_debug.cpp
