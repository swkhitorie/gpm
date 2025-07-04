#!/usr/bin/env bash

EXEDIR=`pwd`
BASEDIR=$(dirname $0)
SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"

FMUBIN=$1
FMUTYPE=$2
SERIAL_PORTS=$3

PY_SCRIPT=${SCRIPTDIR}/../platforms/gpthread/px4/tools/px_upload.sh

${PY_SCRIPT} ${FMUBIN} ${FMUTYPE} ${SERIAL_PORTS}

# busybox.exe bash .\tools\px_uploader.sh .\bin\fmuv2_bsp_gae_.bin fmuv2 COM35
