################################################################################
# application configuration defaults section
################################################################################

PROJ_CDEFS:=\
  USE_HAL_DRIVER                         \
  STM32H743xx                            \
  ARM_MATH_LOOPUNROLL                    \
  ARM_MATH_CM7                           \
  ARM_MATH_MATRIX_CHECK                  \
  ARM_MATH_ROUNDING                      \
  #__clang__      

PROJ_CINCDIRS:=\
  boards/st/cubeh7/v1.11.2/core       \
  boards/st/cubeh7/v1.11.2/user       \
  boards/st/cubeh7/v1.11.2/drivers    \
  boards/st/cubeh7/v1.11.2/drivers/Legacy  \
  platforms                           \
  platforms/lld_svc                   \
  platforms/lld_svc/cubeh7            \
  src/app/gpilot-v3                   

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif

TARGET_POSTBUILD      := ${TARGET_DEST_FILENAME_BIN}

