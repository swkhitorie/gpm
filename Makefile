
SUBDIRS_APP_QUADPILOT := makefiles/proj_quadpilot_eval
SUBDIRS_APP_FANKEH7 := makefiles/proj_fankeh7_eval

.PHONY: app_quadpilot_eval app_fankeh7_eval

app_quadpilot_eval:
	${MAKE} -C ${SUBDIRS_APP_QUADPILOT} all -j4

app_fankeh7_eval:
	${MAKE} -C ${SUBDIRS_APP_FANKEH7} all -j4


