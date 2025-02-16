
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
# platform os configuration
#####################################
FR_ROOTDIR := ${SDK_ROOTDIR}/platforms/frtos
CONFIG_FR_ARCH=m7
CONFIG_FR_TOOLCHAIN=gcc
CONFIG_FR_MEM_METHOD=4
CONFIG_FR_POSIX=y
CONFIG_FR_USB=y
CONFIG_FR_PX4_SUPPORT=y
CONFIG_FR_APPS_CLI=y
CONFIG_FR_FATFS=y
include ${FR_ROOTDIR}/Make.mk

CSOURCES          += ${addprefix platforms/frtos/,${FR_CSOURCES}}

CPPSOURCES        += ${addprefix platforms/frtos/,${FR_CPPSOURCES}}

PROJ_CINCDIRS     += ${addprefix platforms/frtos/,${FR_CINCDIRS}}

#####################################
# px4 Layer configuration
#####################################
PROJ_CDEFS += __PX4_FR

PROJ_CINCDIRS += platforms/common/include
PROJ_CINCDIRS += platforms/common/include/px4_platform_common
PROJ_CINCDIRS += platforms/common/include/px4_platform_common/px4_work_queue
PROJ_CINCDIRS += src/lib/cdev
PROJ_CINCDIRS += src/lib/cdev/posix
PROJ_CINCDIRS += src/lib/perf
PROJ_CINCDIRS += src/include
PROJ_CINCDIRS += src/lib/systemlib
PROJ_CINCDIRS += src/drivers
PROJ_CINCDIRS += src
PROJ_CINCDIRS += src/lib
PROJ_CINCDIRS += src/modules

CPPSOURCES += platforms/common/px4_work_queue/ScheduledWorkItem.cpp
CPPSOURCES += platforms/common/px4_work_queue/WorkItem.cpp
CPPSOURCES += platforms/common/px4_work_queue/WorkItemSingleShot.cpp
CPPSOURCES += platforms/common/px4_work_queue/WorkQueue.cpp
CPPSOURCES += platforms/common/px4_work_queue/WorkQueueManager.cpp
CPPSOURCES += platforms/common/module.cpp
CPPSOURCES += platforms/common/px4_log.cpp

CPPSOURCES += src/lib/cdev/posix/cdev_platform.cpp
CPPSOURCES += src/lib/cdev/CDev.cpp
CPPSOURCES += src/lib/perf/perf_counter.cpp

CPPSOURCES += src/modules/uORB/Subscription.cpp
CPPSOURCES += src/modules/uORB/uORB.cpp
CPPSOURCES += src/modules/uORB/uORBDeviceMaster.cpp
CPPSOURCES += src/modules/uORB/uORBDeviceNode.cpp
CPPSOURCES += src/modules/uORB/uORBMain.cpp
CPPSOURCES += src/modules/uORB/uORBManager.cpp
CPPSOURCES += src/modules/uORB/uORBUtils.cpp

CPPSOURCES += src/modules/uORB/topics_sources/log_message.cpp

#####################################
# app configuration
#####################################
PROJ_CDEFS += BSP_COM_PRINTF
PROJ_CDEFS += BSP_MODULE_FR
# PROJ_CDEFS += BSP_MODULE_USB_CHERRY
# PROJ_CDEFS += CRUSB_STD_INOUT_ENABLE

PROJ_CINCDIRS += src/app

CPPSOURCES += src/app/app_debug/app_main.cpp
CPPSOURCES += src/app/app_debug/app_posix_debug.cpp
CPPSOURCES += src/app/app_debug/app_px4_debug.cpp
CSOURCES += src/app/app_debug/app_fatfs_debug.c
