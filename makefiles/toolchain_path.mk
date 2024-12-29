
################################################################################
# 1: pc in home, 2: pc in job
################################################################################
COMPILER_MACHINE = 1

ifeq (${COMPILER_MACHINE}, 2)

TC_PATH_INST_ARMCC := C:\Keil_v5\ARM\ARMCC

TC_PATH_INST_ARMCLANG := C:\Keil_v5\ARM\ARMCLANG

TC_PATH_INST_GCC:= D:\envir\gcc-arm-none-eabi

TC_OPENOCD_PATH := D:\envir\openocd\xpack-openocd-0.11.0-1

endif # COMPILER_MACHINE - 2


ifeq (${COMPILER_MACHINE}, 1)

TC_PATH_INST_ARMCC := D:\__dev_envir\__env\mdk5\CORE\ARM\ARMCC

TC_PATH_INST_ARMCLANG := D:\__dev_envir\__env\mdk5\CORE\ARM\ARMCLANG

TC_PATH_INST_GCC:= D:\__dev_envir\__env\arm_gcc\gcc-arm-none-eabi

TC_OPENOCD_PATH := D:\__dev_envir\__env\openocd\xpack-openocd-0.11.0-1

endif # COMPILER_MACHINE - 1
