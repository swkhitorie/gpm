
#
# freertos include
#
PROJ_CINCDIRS += platforms/freertos/include/fr
PROJ_CINCDIRS += platforms/freertos/include

ifeq (${MOD_ARCH},m7)
PROJ_CINCDIRS += platforms/freertos/arch/cortex_m7/gcc
CSOURCES += platforms/freertos/arch/cortex_m7/gcc/port.c
else ifeq (${MOD_ARCH},m4)
PROJ_CINCDIRS += platforms/freertos/arch/cortex_m4/gcc
CSOURCES += platforms/freertos/arch/cortex_m4/gcc/port.c
endif

#
# freertos core source file
#
CSOURCES += platforms/freertos/mm/heap_4.c
CSOURCES += platforms/freertos/sched/event_groups.c
CSOURCES += platforms/freertos/sched/list.c
CSOURCES += platforms/freertos/sched/queue.c
CSOURCES += platforms/freertos/sched/stream_buffer.c
CSOURCES += platforms/freertos/sched/tasks.c
CSOURCES += platforms/freertos/sched/timers.c
CSOURCES += platforms/freertos/sched/user.c

#
# freertos posix source file
#
CSOURCES += platforms/freertos/libs/utils.c
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/mqueue/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/pthread/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/queue/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/semaphore/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/time/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/timer/*c))
CSOURCES += $(subst ${SDK_ROOTDIR}/,,$(wildcard ${SDK_ROOTDIR}/platforms/freertos/libs/unistd/*c))
