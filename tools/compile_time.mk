
COMPILE_DATE        := $(shell echo %date:~0,4%%date:~5,2%%date:~8,2%)

COMPILE_HOUR0       := $(shell echo %time:~0,2%)

COMPILE_MINSEC      := $(shell echo %time:~3,2%%time:~6,2%)

ZERO                := 0

EMPTY               :=

SPACE               := $(EMPTY) $(EMPTY)

COMPILE_HOUR        := $(subst $(SPACE),$(ZERO),$(COMPILE_HOUR0))

COMPILE_TIME        := $(COMPILE_DATE)_$(COMPILE_HOUR)$(COMPILE_MINSEC)
