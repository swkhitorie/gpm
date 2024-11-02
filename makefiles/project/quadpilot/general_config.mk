################################################################################
#
# application configuration file
#
################################################################################
PROJ_NAME         :=  quadpilot

#
# Toolchain used in this project
#
PROJ_TC           :=  armcc

MOD_ARCH          :=  m4

PROJ_OPENOCD_DEBUG := stlink

PROJ_OPENOCD_CHIP := stm32f4x

PROJ_OPENOCD_LOAD_ADDR := 0x08000000
