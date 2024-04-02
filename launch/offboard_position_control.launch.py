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

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
           package='px4_offboard',
           executable='visualizer',
           name='visualizer'
        ),
        Node(
            package='px4_offboard',
            executable='offboard_control',
            output ='screen'
        ),
        Node(
           package='px4_offboard',
           executable='visualizer_camera_network',
           name='visualizer_camera_network'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"],
        #     output="screen"
        # ),
        

        # Node(
        #     package='ros_gz_bridge', 
        #     executable='parameter_bridge',
        #     output='screen',
        #     arguments= [  
        #     '/model/x500_1/pose@'
        #     'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        #     remappings=[
        #         ('/model/x500_1/pose', '/tf'),
        #     ]
        # ),
        
        Node(
            package='ros_gz_bridge', 
            executable='parameter_bridge',
            output='screen',
            arguments= [  
            '/model/camera_0/pose@'
            'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            # remappings=[
            #     ('/model/camera_0/pose', '/tf'),
            # ]
        ),
        IncludeLaunchDescription(XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("px4_gps"),
                "launch/foxglove_bridge.launch",))
        ),
        #ExecuteProcess(cmd=["foxglove-studio"]),
    ])
