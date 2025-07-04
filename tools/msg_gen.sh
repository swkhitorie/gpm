#!/usr/bin/env bash

EXEDIR=`pwd`
BASEDIR=$(dirname $0)
SCRIPTDIR="$(cd "$(dirname "$0")" && pwd)"

UORB_MSG_PATH=./msg/
UORB_GENERATE_PATH=./build/

PY_SCRIPT=${SCRIPTDIR}/../platforms/gpthread/px4/tools/uorb_gen/msg_gen.sh

${PY_SCRIPT} ${UORB_MSG_PATH} ${UORB_GENERATE_PATH}

# busybox.exe bash .\tools\msg_gen.sh
