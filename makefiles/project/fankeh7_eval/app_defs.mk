################################################################################
# application configuration defaults section
################################################################################

PROJ_CDEFS:=\
  USE_HAL_DRIVER                         \
  STM32H743xx                            \
  __clang__      

PROJ_CINCDIRS:=\
  boards/fankeh7/src/core       \
  boards/fankeh7/src/drivers       \
  boards/fankeh7/src/drivers/Legacy    \
  boards/fankeh7/src/  \
  platforms/common                          \
  platforms/component/lld_svc/cubeh7            \
  src/app/fankeh7_eval                 

#
# Code execution entry point
#
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler
endif

TARGET_POSTBUILD      := ${TARGET_DEST_FILENAME_BIN}

