
PROJ_CDEFS := USE_HAL_DRIVER STM32H743xx

PROJ_CINCDIRS := \
    boards/edrvlib/pkg/cubeh7/core                          \
    boards/edrvlib/pkg/cubeh7/dev                           \
    boards/edrvlib/pkg/cubeh7/drivers                       \
	boards/fmu-v6c                                          \
    platforms/freertos/arch/cortex_m7/gcc                   \
    platforms/freertos/include                              \
    platforms/freertos/include/fr                           \
    platforms/component/cherryusb/class/cdc                 \
    platforms/component/cherryusb/common                    \
    platforms/component/cherryusb/core                      \
    platforms/component/cherryusb/port/dwc2                 \
    platforms/component/cherryusb/demo                      \
    src/app/fmu-v6c_app
