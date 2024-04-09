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

from re import M
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleLocalPosition
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from geometry_msgs.msg import  TransformStamped
from tf2_ros import TransformBroadcaster

class CameraNetworkVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer_camera_network")
        # Configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/px4_1/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/track_points', 10)
        self.points = np.zeros((1000,3))

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.vehicle_local_position[0] = msg.y
        self.vehicle_local_position[1] = msg.x
        self.vehicle_local_position[2] = -msg.z


    def cmdloop_callback(self):
        cam0 = [-17.0, 10.0, 10.0, 0.0, 0.75, 0.0] 
        cam1 = [25.0, 94.0, 30.0, 0.0, 0.0, -0.5]
        cam2 = [75.0, 134.0, 32.0, 0.0, 0.0, -0.5]
        cam3 = [137.0, 161.0, 22.0, 0.0, 0.0, -0.6]
        cam4 = [24.0, 24.0, 30.0, 0.0, 0.5, 3.1] 
        cam5 = [56.0, 35.0, 30.0, 0.0, 0.0, 2.75]
        cam6 = [102.0, 72.0, 31.0, 0.0, 0.0, 2.75]
        cam7 = [183.0, 126.0, 30.0, 0.0, 0.0, 2.35]

        cam_list = np.array([cam0, cam1, cam2, cam3, cam4, cam5, cam6, cam7], dtype=float)
        for i in range(1000):
            self.points[i,0] = self.vehicle_local_position[0] + 5*np.random.randn()
            self.points[i,1] = self.vehicle_local_position[1] + 5*np.random.randn()
            self.points[i,2] = self.vehicle_local_position[2] + 5*np.random.randn()

        self.pcd = point_cloud(self.points, 'map')
        self.pcd_publisher.publish(self.pcd)
        for i in range(cam_list.shape[0]):
            t = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'Camera {i+1}'
            t.transform.translation.x = cam_list[i,0]
            t.transform.translation.y = cam_list[i,1]
            t.transform.translation.z = cam_list[i,2]
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            # Send the transformation
            self.tf_broadcaster.sendTransform(t)
    
    
def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    Code source:
        https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()
    fields = [sensor_msgs.PointField(
        name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )
       
def main(args=None):
    rclpy.init(args=args)
    cam_visualizer = CameraNetworkVisualizer()
    rclpy.spin(cam_visualizer)
    cam_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()