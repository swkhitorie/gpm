
PROJ_CDEFS := USE_HAL_DRIVER STM32H743xx

# change platforms/freertos/include and platforms/freertos/include/fr seq??

PROJ_CINCDIRS := \
    boards/edrvlib/pkg/cubeh7/core                          \
    boards/edrvlib/pkg/cubeh7/dev                           \
    boards/edrvlib/pkg/cubeh7/drivers                       \
	boards/fankeh7                                          \
    platforms/freertos/arch/cortex_m7/gcc                   \
    platforms/freertos/include                              \
    platforms/freertos/include/fr                           \
    platforms/opmd/cherryusb/class/cdc                      \
    platforms/opmd/cherryusb/common                         \
    platforms/opmd/cherryusb/core                           \
    platforms/opmd/cherryusb/port/dwc2                      \
    platforms/opmd/cherryusb/demo                           \
    src/app/fankeh7_eval
