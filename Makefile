
ifneq ($(OS), Linux)
TC_PATH_INST_GCC := $(shell where arm-none-eabi-gcc)/../../
TC_PATH_INST_ARMCC := $(shell where armcc)/../../
TC_PATH_INST_ARMCLANG := $(shell where armclang)/../../
TC_OPENOCD_PATH := $(shell where openocd)/../../
endif

# Path to root dir
SDK_DIR_MK = $(dir $(realpath $(lastword ${MAKEFILE_LIST})))
SDK_DIR_CUR = $(CURRENT_DIR)
GPM_ROOTDIR := ${SDK_DIR_MK}
SDK_ROOTDIR := ${GPM_ROOTDIR}/platforms/gpthread
PUSER_ROOTDIR := ${GPM_ROOTDIR}
# Absolute path to makefiles
MAKEFILES_ROOTDIR := ${SDK_ROOTDIR}/make

CONFIG_FILE := ${GPM_ROOTDIR}/src/app/app_config.mk
include ${CONFIG_FILE}

# including common build makefile
include $(MAKEFILES_ROOTDIR)/make/build.mk
all: prebuild build postbuild
