#!/bin/bash
SANDBOX_DIR="$(dirname "$(readlink -f ${BASH_SOURCE[0]})")"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$SANDBOX_DIR

