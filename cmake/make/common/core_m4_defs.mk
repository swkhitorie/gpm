################################################################################
#
# Cortex-M4 configuration file
#
################################################################################

ifeq (${TC_NAME},arm)

ifeq ($(findstring V5,${PROJ_COMPILER}),V5)
CORE_ASMOPTS:=\
  --cpu=Cortex-M4.fp.sp             \
  --apcs=interwork

CORE_COPTS:=\
  --cpu=Cortex-M4.fp.sp             \
  --apcs=interwork
endif 

ifeq ($(findstring V6,${PROJ_COMPILER}),V6)
CORE_ASMOPTS:=\
  --cpu=Cortex-M4.fp.sp             \
  -g

CORE_COPTS:=\
  -mcpu=cortex-m4                   \
  -mfpu=fpv4-sp-d16                 \
  -mfloat-abi=hard
endif 
endif # TC_NAME

ifeq (${TC_NAME},gae)
CORE_ASMOPTS:=\
  -mcpu=cortex-m4                  \
  -mthumb                          \
  -mthumb-interwork
  
CORE_COPTS:=\
  -mcpu=cortex-m4                  \
  -mfpu=fpv4-sp-d16                \
  -mfloat-abi=hard                 \
  -mthumb                          \
  -mthumb-interwork                \
  -mapcs-frame                     \
  -mapcs-stack-check

CORE_LIBOPTS:=\
  -mcpu=cortex-m4                  \
  -mfpu=fpv4-sp-d16                \
  -mfloat-abi=hard                 \
  -mthumb                          \
  -mthumb-interwork
  
endif # TC_GCC_NAME
