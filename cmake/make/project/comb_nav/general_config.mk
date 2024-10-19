################################################################################
#
# application configuration file
#
################################################################################
PROJ_NAME         :=  comb_nav

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
PROJ_TC           :=  arm

MOD_ARCH          :=  m4

#
# Toolchain variant used for this project
# It can be:
# - V5     arm compiler v5. armcc
# - V6     arm compiler v6. armclang
# std periph lib stm32f4 only support arm compiler v5
#
PROJ_COMPILER     :=  V5

PROJ_OPENOCD_DEBUG := stlink

PROJ_OPENOCD_CHIP := stm32f4x

PROJ_OPENOCD_LOAD_ADDR := 0x08000000
