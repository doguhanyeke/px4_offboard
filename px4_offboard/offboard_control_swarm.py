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
from std_msgs.msg import Bool
from functools import partial

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

          
        self.vehicle_hover_publisher = self.create_publisher(
            Bool, 
            f'{self.ns}/fmu/out/hover_status', 
            qos_profile_pub) 
            
            
        self.all_vehicle_hover_publisher = self.create_publisher(
            Bool, 
            f'{self.ns}/fmu/out/all_hover', 
            qos_profile_pub)                                            # disable for an experiment

        # parameters for callback
        self.timer_period   =   0.04  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        self.arm_counter = 0
        # variables for subscribers
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.local_pos_ned_     =   None
        self.ocra2_setpoint = np.array([0,0,0, 0,0,0])

        self.hover = False
        self.all_hover = False
        self.N_drone = 3
        self.hover_status = [False]*(self.N_drone)
        self.hover_subscribers    =   [{'hover_sub':None} for _ in range(self.N_drone)]

        for i in range(self.N_drone):

            ns             =   f'/px4_{i+1}'
            self.hover_subscribers[i]['hover_sub']     =   self.create_subscription(
                Bool,
                f'{ns}/fmu/out/hover_status',
                partial(self.vehicle_hover_callback,id=i),                             # instead of lambda function lambda msg: self.vehicle_status_callback(msg,id=i), use partial function
                qos_profile_sub)
        
       
    # subscriber callback
    def vehicle_hover_callback(self, msg, id):
        # TODO: handle NED->ENU transformation
        self.hover_status[id] = msg.data

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state

    def local_position_callback(self,msg):
        self.local_pos_ned_      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)

    def trajectory_setpoint_callback(self,msg):
        self.ocra2_setpoint      =   np.array([msg.position[0],msg.position[1],msg.position[2],msg.velocity[0],msg.velocity[1],msg.velocity[2]],dtype=np.float64)
    

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
        #offboard_msg.position=False
        offboard_msg.velocity=True
        #offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        if self.nav_state != VehicleStatus.ARMING_STATE_ARMED and self.arm_counter < 50:
            self.arm_counter += 1
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            hover_msg = Bool()
            all_hover_msg = Bool()
            hover_msg.data = False
            all_hover_msg.data = False 
            if not self.all_hover: 
                if not self.hover:
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.position[0]  = 0.0
                    trajectory_msg.position[1]  = 0.0
                    trajectory_msg.position[2]  = -1.5
                    #self.get_logger().info("Offboard")  
                    self.publisher_trajectory.publish(trajectory_msg)
                    # Reached the hover goal
                    if self.local_pos_ned_[2] < -1.45:
                        self.hover = True    
                else:
                    hover_msg.data = True
                
                # Check all vehicle hover status 
                self.all_hover =all(self.hover_status) 
                #self.get_logger().info(f'{self.all_hover}, {self.hover_status[0]}, {self.hover_status[1]}, {self.hover_status[2]}')
            else:
                hover_msg.data = True
                all_hover_msg.data = True

                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.position[0]  = np.nan
                trajectory_msg.position[1]  = np.nan
                trajectory_msg.position[2]  = np.nan
                trajectory_msg.velocity[0]  = self.ocra2_setpoint[3]
                trajectory_msg.velocity[1]  = self.ocra2_setpoint[4]
                trajectory_msg.velocity[2]  = 0#5*self.ocra2_setpoint[5]
                #self.get_logger().info("Offboard" + str(self.ocra2_setpoint[3]) + str(self.ocra2_setpoint[4]))  
                self.publisher_trajectory.publish(trajectory_msg)
            
            self.vehicle_hover_publisher.publish(hover_msg)
            self.all_vehicle_hover_publisher.publish(all_hover_msg)

def main():
    rclpy.init(args=None)
    offboard_swarm_mission = OffboardSwarmMission()
    rclpy.spin(offboard_swarm_mission)
    offboard_swarm_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()