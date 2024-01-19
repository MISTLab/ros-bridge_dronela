#
# Copyright (c) 2018-2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This represents a vehdroneicle control message sent to CARLA simulator

std_msgs/Header header

# The CARLA drone control data

# -1. <= vz <= 1.
float32 vz

# -1. <= vleft <= 1.
float32 vleft

# -1. <= vforward <= 1.
float32 vforward

# yawrate -1 or 1
float32 yawrate
