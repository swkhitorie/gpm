################################################################################
# application configuration defaults section
################################################################################
  # STM32F40_41xxx                         
  # STM32F407xx                
PROJ_CDEFS:=\
  STM32F427_437xx                        \
  USE_STDPERIPH_DRIVER                   \
  ARM_MATH_CM4                           \
  ARM_MATH_MATRIX_CHECK                  \
  ARM_MATH_ROUNDING                      \
  __FPU_USED                             \
  __FPU_PRESENT                          \
  __CC_ARM

PROJ_CINCDIRS:=\
  boards/st/stdf4/v1.9.0/user       \
  boards/st/stdf4/v1.9.0/driver     \
  boards/st/stdf4/v1.9.0/core       \
  platforms                         \
  platforms/lld_svc                 \
  platforms/lld_svc/stdf4           \
  src/app/comb_nav

#
# Code execution entry point
# isr_vector
# Reset_Handler
# 
ifeq (${PROJ_ENTRY_POINT},)
PROJ_ENTRY_POINT      :=  Reset_Handler 
endif

TARGET_POSTBUILD      := ${TARGET_DEST_FILENAME_BIN}

