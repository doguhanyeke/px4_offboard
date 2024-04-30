#!/usr/bin/env python

__author__ = "Kartik Anand Pant"
__contact__ = "kpant14@gmail.com"

import rclpy
import navpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand
from geometry_msgs.msg import PoseStamped

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[1]#ned to enu
    pose_msg.pose.position.y = position[0]
    pose_msg.pose.position.z = -position[2]
    return pose_msg

class OffboardSwarmMission(Node):

    def __init__(self):
        super().__init__("px4_offboard_mission")
        # set publisher and subscriber quality of service profile
        qos_profile_pub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )
        qos_profile_sub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )
        self.declare_parameter('px4_ns', 'px4_1')
        self.ns = self.get_parameter('px4_ns').get_parameter_value().string_value

        # define subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.ns}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{self.ns}/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile_sub)
        
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            f'{self.ns}/fmu/orca2/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile_sub)
        
        # define publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'{self.ns}/fmu/in/offboard_control_mode', 
            qos_profile_pub)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            f'{self.ns}/fmu/in/trajectory_setpoint', 
            qos_profile_pub)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            f'{self.ns}/fmu/in/vehicle_command', 
            qos_profile_pub)                                        # disable for an experiment

        # parameters for callback
        self.timer_period   =   0.04  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        self.arm_counter = 0
        # variables for subscribers
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.local_pos_ned_     =   None
        self.ocra2_setpoint = np.array([0,0,0])
        
       
    # subscriber callback
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state

    def local_position_callback(self,msg):
        self.local_pos_ned_      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)

    def trajectory_setpoint_callback(self,msg):
        self.ocra2_setpoint      =   np.array([msg.position[0],msg.position[1],msg.position[2]],dtype=np.float64)
    

    def publish_vehicle_command(self,command,param1=0.0,param2=0.0):            # disable for an experiment
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
        self.vehicle_command_publisher.publish(msg)

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        if self.nav_state != VehicleStatus.ARMING_STATE_ARMED and self.arm_counter < 10:
            self.arm_counter += 1
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            trajectory_msg = TrajectorySetpoint()
            self.get_logger().info("Offboard")  
            trajectory_msg.position[0]  = self.ocra2_setpoint[0]
            trajectory_msg.position[1]  = self.ocra2_setpoint[1]
            trajectory_msg.position[2]  = -5.0
            self.publisher_trajectory.publish(trajectory_msg)


def main():
    rclpy.init(args=None)
    offboard_swarm_mission = OffboardSwarmMission()
    rclpy.spin(offboard_swarm_mission)
    offboard_swarm_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()