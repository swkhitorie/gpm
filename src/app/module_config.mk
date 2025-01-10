
#####################################
# module cherry usb
#####################################
PROJ_CINCDIRS += platforms/component/cherryusb/common
PROJ_CINCDIRS += platforms/component/cherryusb/core
PROJ_CINCDIRS += platforms/component/cherryusb/class/cdc
PROJ_CINCDIRS += platforms/component/cherryusb/port/dwc2
PROJ_CINCDIRS += platforms/component/cherryusb/demo

CSOURCES += platforms/component/cherryusb/core/usbd_core.c
CSOURCES += platforms/component/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES += platforms/component/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES += platforms/component/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES += platforms/component/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES += platforms/component/cherryusb/demo/cdc_acm_template.c

#####################################
# freertos
#####################################
PROJ_CINCDIRS += platforms/freertos/include/fr
PROJ_CINCDIRS += platforms/freertos/include

ifeq (${MOD_ARCH},m7)
PROJ_CINCDIRS += platforms/freertos/arch/cortex_m7/gcc
CSOURCES += platforms/freertos/arch/cortex_m7/gcc/port.c
else ifeq (${MOD_ARCH},m4)
PROJ_CINCDIRS += platforms/freertos/arch/cortex_m4/gcc
CSOURCES += platforms/freertos/arch/cortex_m4/gcc/port.c
endif

CSOURCES += platforms/freertos/mm/heap_4.c
CSOURCES += platforms/freertos/sched/event_groups.c
CSOURCES += platforms/freertos/sched/list.c
CSOURCES += platforms/freertos/sched/queue.c
CSOURCES += platforms/freertos/sched/stream_buffer.c
CSOURCES += platforms/freertos/sched/tasks.c
CSOURCES += platforms/freertos/sched/timers.c
CSOURCES += platforms/freertos/sched/user.c

CSOURCES += platforms/freertos/libs/utils.c
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/mqueue/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/pthread/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/queue/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/semaphore/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/time/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/timer/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/unistd/*c))

#####################################
# PX4 Layer
#####################################
PROJ_CDEFS += __PX4_FR

#
# px4 include
#
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

CSOURCES += platforms/freertos/px4/stm/hrt.c
CPPSOURCES += platforms/freertos/px4/ptasks.cpp

#
# px4 sources
#
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
