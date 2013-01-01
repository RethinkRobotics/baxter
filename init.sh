#!/bin/bash
. /opt/ros/electric/setup.sh
DIR="$( cd "$( dirname "$0" )" && pwd )"
export ROS_PACKAGE_PATH=${DIR}:${ROS_PACKAGE_PATH}
[[ -n ${1} ]] && export ROS_MASTER_URI=http://${1}:11311
