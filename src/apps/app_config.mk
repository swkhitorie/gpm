
ifeq (${APP_SUBPATH},)
ifeq (${MAKE_TARGET_CLEANS},n)
$(error Invalid empty APP_SUBPATH)
endif
endif

include ${GPM_ROOTDIR}/src/apps/${APP_SUBPATH}/config.mk
