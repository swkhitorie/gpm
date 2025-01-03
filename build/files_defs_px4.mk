

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
