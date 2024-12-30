
PROJ_CDEFS:= USE_HAL_DRIVER STM32F427xx

PROJ_CINCDIRS:=\
    boards/edrvlib/pkg/cubef4/core                          \
    boards/edrvlib/pkg/cubef4/dev                           \
    boards/edrvlib/pkg/cubef4/drivers                       \
	boards/fmu-v2                                           \
    platforms/opmd/cherryusb/class/cdc                      \
    platforms/opmd/cherryusb/common                         \
    platforms/opmd/cherryusb/core                           \
    platforms/opmd/cherryusb/port/dwc2                      \
    platforms/opmd/cherryusb/demo                           \
    src/app/fmu-v2_app
