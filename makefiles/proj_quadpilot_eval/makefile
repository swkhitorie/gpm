

# Path to root dir
SDK_CUR_DIR = $(dir $(realpath $(lastword ${MAKEFILE_LIST})))
ifeq ($(wildcard $(SDK_CUR_DIR)/build/makefile/build.mk),)
    SDK_CUR_DIR = $(CURRENT_DIR)
endif
SDK_ROOTDIR:=$(realpath $(SDK_CUR_DIR)../../)

# Absolute path to makefiles
MAKEFILES_ROOTDIR := ${SDK_ROOTDIR}/makefiles

COMPILE_DATE        := $(shell echo %date:~0,4%%date:~5,2%%date:~8,2%)
COMPILE_HOUR0       := $(shell echo %time:~0,2%)
COMPILE_MINSEC      := $(shell echo %time:~3,2%%time:~6,2%)
ZERO                := 0
EMPTY               :=
SPACE               := $(EMPTY) $(EMPTY)
COMPILE_HOUR        := $(subst $(SPACE),$(ZERO),$(COMPILE_HOUR0))
COMPILE_TIME        := $(COMPILE_DATE)_$(COMPILE_HOUR)$(COMPILE_MINSEC)

PROJ_CFG           := general_config

#
# including defines related to configuration
#
include ${PROJ_CFG}.mk

#
# including sources file list
#
include app_files_defs.mk

#
# including common build makefile
#
include $(MAKEFILES_ROOTDIR)/common/build.mk

all: prebuild build postbuild program
