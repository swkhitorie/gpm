
#
# application configuration file
#
PROJ_NAME  :=  fankeh7_eval

#
# Toolchain used in this project
#
PROJ_TC  :=  gae

TARGET_POSTBUILD := ${TARGET_DEST_FILENAME_BIN}

include ${MAKEFILES_ROOTDIR}/bsp_fankeh7.mk

include ${MAKEFILES_ROOTDIR}/files_defs_bsp.mk

include ${MAKEFILES_ROOTDIR}/files_defs_freertos.mk

include ${MAKEFILES_ROOTDIR}/files_defs_cherryusb.mk

include ${MAKEFILES_ROOTDIR}/files_defs_px4.mk

PROJ_CINCDIRS += src/app
#CPPSOURCES += src/app/app_debug/app_main.cpp
CPPSOURCES += src/app/app_debug_posix/app_main.cpp
