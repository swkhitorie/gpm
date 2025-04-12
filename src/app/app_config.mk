
PROJ_NAME  :=  test

PROJ_TC    :=  gae

TARGET_POSTBUILD := ${TARGET_DEST_FILENAME_BIN}

# bsp configuration
include ${SDK_ROOTDIR}/bsp/make.mk

# os and library configuration
CONFIG_FR_ARCH=${MOD_ARCH}
CONFIG_FR_TOOLCHAIN=gcc
CONFIG_FR_MEM_METHOD=4
CONFIG_FR_LIB_CPP=y
CONFIG_FR_LIB_PX4_SUPPORT=y
CONFIG_FR_LIB_POSIX=y
CONFIG_FR_FAT_FATFS=n
CONFIG_CRUSB=n
CONFIG_USE_DRV_HRT_INTERNAL=n
CONFIG_FR_LIB_UORB=n

include ${SDK_ROOTDIR}/sched/make.mk
include ${SDK_ROOTDIR}/mm/make.mk
include ${SDK_ROOTDIR}/libs/make.mk
include ${SDK_ROOTDIR}/include/make.mk
include ${SDK_ROOTDIR}/usb/make.mk
CSOURCES      += ${FR_CSOURCES}
CPPSOURCES    += ${FR_CPPSOURCES}
PROJ_CINCDIRS += ${FR_CINCDIRS}

#####################################
# app configuration
#####################################
PROJ_CDEFS += CONFIG_BOARD_COM_STDINOUT
PROJ_CDEFS += CONFIG_BOARD_FREERTOS_ENABLE
# PROJ_CDEFS += CONFIG_BOARD_CRUSB_CDC_ACM_ENABLE
# PROJ_CDEFS += CONFIG_BOARD_CRUSB_CDC_ACM_STDINOUT
PROJ_CDEFS += CONFIG_FR_MALLOC_FAILED_HANDLE
PROJ_CDEFS += CONFIG_FR_IDLE_TIMER_TASKCREATE_HANDLE

PUSER_CINCDIRS += src/app
PUSER_CPPSOURCES += src/app/app_debug/app_main.cpp

CONFIG_LINK_PRINTF_FLOAT:=y
CONFIG_LINK_SCANF_FLOAT:=n
CONFIG_COMPILE_OPTIMIZE:=O1

#####################################
# px4 Layer configuration
#####################################
PROJ_CDEFS += __PX4_FR

PUSER_CINCDIRS += platforms/common/include
PUSER_CINCDIRS += platforms/common/include/px4_platform_common
PUSER_CINCDIRS += platforms/common/include/px4_platform_common/px4_work_queue
PUSER_CINCDIRS += src/lib/cdev
PUSER_CINCDIRS += src/lib/cdev/posix
PUSER_CINCDIRS += src/lib/perf
PUSER_CINCDIRS += src/include
PUSER_CINCDIRS += src/lib/systemlib
PUSER_CINCDIRS += src/drivers
PUSER_CINCDIRS += src
PUSER_CINCDIRS += src/lib
PUSER_CINCDIRS += src/modules

PUSER_CPPSOURCES += platforms/common/px4_work_queue/ScheduledWorkItem.cpp
PUSER_CPPSOURCES += platforms/common/px4_work_queue/WorkItem.cpp
PUSER_CPPSOURCES += platforms/common/px4_work_queue/WorkItemSingleShot.cpp
PUSER_CPPSOURCES += platforms/common/px4_work_queue/WorkQueue.cpp
PUSER_CPPSOURCES += platforms/common/px4_work_queue/WorkQueueManager.cpp
PUSER_CPPSOURCES += platforms/common/module.cpp
PUSER_CPPSOURCES += platforms/common/px4_log.cpp

PUSER_CPPSOURCES += src/lib/cdev/posix/cdev_platform.cpp
PUSER_CPPSOURCES += src/lib/cdev/CDev.cpp
PUSER_CPPSOURCES += src/lib/perf/perf_counter.cpp

PUSER_CPPSOURCES += src/modules/uORB/Subscription.cpp
PUSER_CPPSOURCES += src/modules/uORB/uORB.cpp
PUSER_CPPSOURCES += src/modules/uORB/uORBDeviceMaster.cpp
PUSER_CPPSOURCES += src/modules/uORB/uORBDeviceNode.cpp
PUSER_CPPSOURCES += src/modules/uORB/uORBMain.cpp
PUSER_CPPSOURCES += src/modules/uORB/uORBManager.cpp
PUSER_CPPSOURCES += src/modules/uORB/uORBUtils.cpp

PUSER_CPPSOURCES += src/modules/uORB/topics_sources/log_message.cpp
