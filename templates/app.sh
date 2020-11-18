#!/usr/bin/env bash


# Copyright (c) 2014,2020 ADLINK Technology Inc.
#
# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors: Gabriele Baldoni, ADLINK Technology Inc. - ROS2 Plugin

ROS_APP_PATH=$1
ROS_APP_NAME=$2
ROS_ENTRY_POINT=$3

shift
shift
shift

ROSDISTRO="{{ ros_distro }}"
ROS2="/opt/ros/{{ ros_distro }}/setup.bash"
RMW_IMPLEMENTATION="{{ rmw }}"

trap cleanup 1 2 3 6

cleanup(){
	echo "=== Kill ROS2 Application ==="
	# screen -S {{ id }} -X quit
	# echo "=== Bye ==="
	exit 0
}


export $RWM_IMPLEMENTATION
cd $ROS_APP_PATH
source $ROS2


echo "--- App Path = $ROS_APP_PATH ---"
echo "--- App Name = $ROS_APP_NAME ---"
echo "--- Entry Point = $ROS_ENTRY_POINT ---"
echo "--- ROS2 Distribution = $ROSDISTRO ---"
echo "--- RMW Implementation = $RMW_IMPLEMENTATION ---"


echo "=== Run ROS2 Application ==="

source install/setup.bash

ros2 run $ROS_APP_NAME $ROS_ENTRY_POINT $@
ros2 launch $ROS_APP_NAME $ROS_ENTRY_POINT $@
