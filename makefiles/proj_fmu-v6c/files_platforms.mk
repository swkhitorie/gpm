
CSOURCES += platforms/component/cherryusb/class/cdc/usbd_cdc_acm.c
CSOURCES += platforms/component/cherryusb/class/cdc/usbd_cdc_ecm.c
CSOURCES += platforms/component/cherryusb/core/usbd_core.c
CSOURCES += platforms/component/cherryusb/port/dwc2/usb_dc_dwc2.c
CSOURCES += platforms/component/cherryusb/port/dwc2/usb_glue_st.c
CSOURCES += platforms/component/cherryusb/demo/cdc_acm_template.c

CSOURCES += platforms/freertos/arch/cortex_m7/gcc/port.c
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
