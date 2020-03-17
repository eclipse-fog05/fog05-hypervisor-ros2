#!/bin/bash


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



id=$(cat /etc/machine-id)
first=$(echo $id | cut -c -8)
second=$(echo $id | cut -c 9-12)
third=$(echo $id | cut -c 13-16)
fourth=$(echo $id | cut -c 17-20)
fifth=$(echo $id | cut -c 21-)
echo $first-$second-$third-$fourth-$fifth
