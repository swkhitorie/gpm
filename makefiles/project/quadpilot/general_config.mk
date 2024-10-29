################################################################################
#
# application configuration file
#
################################################################################
PROJ_NAME         :=  quadpilot

#
# Type of project
#
PROJ_MOD_APP      :=  binimg

#
# Optional settings.
# Some settings are defined by default in specific makefiles. If you want to
# apply specific settings, use this section to uncomment and define them.
# Check documentation for explanation of each setting and allowed values.
#
PROJ_OPT_FEATURES     :=  
PROJ_FAST_ENABLE      :=  

#
# Operating system used in this project
#
PROJ_MOD_OS       :=  FreeRTOS

#
# Toolchain used in this project
#
PROJ_TC           :=  armcc
MOD_ARCH          :=  m4

PROJ_OPENOCD_DEBUG := stlink

PROJ_OPENOCD_CHIP := stm32f4x

PROJ_OPENOCD_LOAD_ADDR := 0x08000000