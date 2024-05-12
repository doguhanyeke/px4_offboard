#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Kartik Anand Pant"
__contact__ = "kpant14@gmail.com"

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    px4_ns = LaunchConfiguration('px4_ns')

    px4_ns_arg = DeclareLaunchArgument(
        'px4_ns',
        default_value='px4_1'
    )
   
    offboard_node_1 = Node(
        package='px4_offboard',
        executable='offboard_control_swarm',
        name='offboard_control_swarm',
        parameters=[
            {'px4_ns': 'px4_1'},
        ]   
    )

    offboard_node_2 = Node(
        package='px4_offboard',
        executable='offboard_control_swarm',
        name='offboard_control_swarm',
        parameters=[
            {'px4_ns': 'px4_2'},
        ]   
    )
    
    offboard_node_3 = Node(
        package='px4_offboard',
        executable='offboard_control_swarm',
        name='offboard_control_swarm',
        parameters=[
            {'px4_ns': 'px4_3'},
        ]   
    )

    return LaunchDescription([
        offboard_node_1,
        offboard_node_2,
        offboard_node_3
    ])
