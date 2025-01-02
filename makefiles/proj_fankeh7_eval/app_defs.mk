
PROJ_CDEFS := USE_HAL_DRIVER STM32H743xx __PX4_FR

# change platforms/freertos/include and platforms/freertos/include/fr seq??

PROJ_CINCDIRS := \
    boards/edrvlib/pkg/cubeh7/core                          \
    boards/edrvlib/pkg/cubeh7/dev                           \
    boards/edrvlib/pkg/cubeh7/drivers                       \
	boards/fankeh7                                          \
    platforms/freertos/arch/cortex_m7/gcc                   \
    platforms/freertos/include                              \
    platforms/freertos/include/fr                           \
    platforms/component/cherryusb/class/cdc                 \
    platforms/component/cherryusb/common                    \
    platforms/component/cherryusb/core                      \
    platforms/component/cherryusb/port/dwc2                 \
    platforms/component/cherryusb/demo                      \
    src/app/fankeh7_eval

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