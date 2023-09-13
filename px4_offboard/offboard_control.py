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

import rclpy
import numpy as np
import os
import navpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Multicontainer initialization
        #PX4_NS = os.getenv("PX4_MICRODDS_NS")
        PX4_NS = "px4_1"
        fmu = f"{PX4_NS}/fmu"
        print(fmu)
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_sub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f"{fmu}/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_pub)
        
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            f"{fmu}/in/offboard_control_mode",
            qos_profile_pub)
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            f"{fmu}/in/trajectory_setpoint",
            qos_profile_pub)
        
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            f"{fmu}/in/vehicle_command",
            qos_profile_pub)
        
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{fmu}/out/vehicle_local_position",
            self.local_position_callback,
            qos_profile_sub)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.counter = 0
        # Setting the reference lla as the origin of the AbuDhabi Map 
        
        # Map Origin
        #self.lla_ref = np.array([24.483136, 54.367537, 0]) # latlonele -> (deg,deg,m)
        
        # Interesting trajectory origin
        self.lla_ref = np.array([24.484043629238872, 54.36068616768677, 0]) # latlonele -> (deg,deg,m)
        self.waypoint_idx = 0
        self.waypoints_lla = np.array([
            [24.484326113268185, 54.360644616972564, 10],
           [24.48476311664666, 54.3614948536716, 20],
           [24.485097533474377, 54.36197496905472, 20],
           [24.485400216562002, 54.3625570084458, 25], 
           [24.48585179883862, 54.36321951405934, 25], 
           [24.486198417650844, 54.363726451568475, 25], 
           [24.486564563238797, 54.36423338904003, 20], 
           [24.486894093361375, 54.364729597702144, 20], 
           [24.486664642851466, 54.36508096711639, 20],
           [24.486396136401133, 54.365263357350244, 25],
           [24.486066604972933, 54.36541087887424, 10],
           [24.485610141502686, 54.36572201510017,0],
        ])
        self.local_pos_ned = np.array([0,0,0])
        # Initializing the next_pos_ned with the first waypoint 
        self.next_pos_ned = navpy.lla2ned(self.waypoints_lla[0,0], self.waypoints_lla[0,1],
                    self.waypoints_lla[0,2],self.lla_ref[0], self.lla_ref[1], self.lla_ref[2],
                    latlon_unit='deg', alt_unit='m', model='wgs84')          
   
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 0  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
    
    def local_position_callback(self,msg):
        self.local_pos_ned      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)
        #self.local_vel_ned      =   np.array([msg.vx,msg.vy,msg.vz],dtype=np.float64)

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        self.counter += 1   # disable for an experiment

        if self.counter <= 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # If current waypoint is reached, move to next waypoint
            if (np.linalg.norm(self.local_pos_ned - self.next_pos_ned) < 1):
                self.waypoint_idx += 1
            self.next_pos_ned = navpy.lla2ned(self.waypoints_lla[self.waypoint_idx,0], self.waypoints_lla[self.waypoint_idx,1],
                    self.waypoints_lla[self.waypoint_idx,2],self.lla_ref[0], self.lla_ref[1], self.lla_ref[2],
                    latlon_unit='deg', alt_unit='m', model='wgs84')
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.next_pos_ned[0]
            trajectory_msg.position[1] = self.next_pos_ned[1]
            trajectory_msg.position[2] = self.next_pos_ned[2]
            self.publisher_trajectory.publish(trajectory_msg)
            

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
