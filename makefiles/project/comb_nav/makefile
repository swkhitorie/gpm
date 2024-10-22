
# Path to root dir
SDK_CUR_DIR = $(dir $(realpath $(lastword ${MAKEFILE_LIST})))
ifeq ($(wildcard $(SDK_CUR_DIR)/build/makefile/build.mk),)
    SDK_CUR_DIR = $(CURRENT_DIR)
endif
SDK_ROOTDIR:=$(realpath $(SDK_CUR_DIR)../../../../)

# Absolute path to makefiles
MAKEFILES_ROOTDIR := ${SDK_ROOTDIR}/build/makefile

CompileDate        := $(shell echo %date:~0,4%%date:~5,2%%date:~8,2%)
CompileHour0       := $(shell echo %time:~0,2%)
CompileMinSec      := $(shell echo %time:~3,2%%time:~6,2%)
zero               := 0
empty              :=
space              := $(empty) $(empty)
CompileHour        := $(subst $(space),$(zero),$(CompileHour0))
CompileDateTime    := $(CompileDate)_$(CompileHour)$(CompileMinSec)

PROJ_CFG           := general_config

#
# including defines related to configuration
#
include ${PROJ_CFG}.mk

#
# including sources file list
#
include app_defs.mk

#
# including sources file list
#
include files.mk

#
# including common build makefile
#
include $(MAKEFILES_ROOTDIR)/common/build.mk

all: prebuild build postbuild #program
