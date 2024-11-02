################################################################################
#
# application configuration file
#
################################################################################
PROJ_NAME         :=  fankeh7_eval

#
# Toolchain used in this project
#
PROJ_TC           :=  armclang

MOD_ARCH          :=  m7

PROJ_OPENOCD_DEBUG := stlink

PROJ_OPENOCD_CHIP := stm32h7x

PROJ_OPENOCD_LOAD_ADDR := 0x08000000
