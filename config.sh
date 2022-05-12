#!/usr/bin/env bash

SCRIPT=`realpath ${BASH_SOURCE}`
UTILS_SH_PATH=`dirname $SCRIPT`
echo "Abs path to script is: ${UTILS_SH_PATH}"

# Add important paths to python path
# Current project
proj_path="${UTILS_SH_PATH}"

# pyARTE
artepy_path="${UTILS_SH_PATH}/pyARTE/:${UTILS_SH_PATH}/pyARTE/artelib:${UTILS_SH_PATH}/pyARTE/robots"

# Coppelia
coppelia_path="${UTILS_SH_PATH}/../CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/remoteApiBindings/python/python/"

export PYTHONPATH="${PYTHONPATH}:${proj_path}:${artepy_path}:${coppelia_path}"

echo "New python path is: ${PYTHONPATH}"