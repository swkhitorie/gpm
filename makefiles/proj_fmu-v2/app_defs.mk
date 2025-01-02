
PROJ_CDEFS:= USE_HAL_DRIVER STM32F427xx

PROJ_CINCDIRS:=\
    boards/edrvlib/pkg/cubef4/core                          \
    boards/edrvlib/pkg/cubef4/dev                           \
    boards/edrvlib/pkg/cubef4/drivers                       \
	boards/fmu-v2                                           \
    platforms/component/cherryusb/class/cdc                 \
    platforms/component/cherryusb/common                    \
    platforms/component/cherryusb/core                      \
    platforms/component/cherryusb/port/dwc2                 \
    platforms/component/cherryusb/demo                      \
    src/app/fmu-v2_app
