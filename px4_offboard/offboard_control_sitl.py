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

class OffboardMission(Node):

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
        
        # Gazebo Model Origin 
        self.lla_ref = np.array([24.484043629238872, 54.36068616768677, 0]) # latlonele -> (deg,deg,m)
        self.waypoint_idx = 0
        self.waypoints_lla = np.array([
           [24.484326113268185, 54.360644616972564, 30],
           [24.48476311664666, 54.3614948536716, 30],
           [24.485097533474377, 54.36197496905472, 30],
           [24.485400216562002, 54.3625570084458, 30], 
           [24.48585179883862, 54.36321951405934, 30], 
           [24.486198417650844, 54.363726451568475, 30], 
           [24.486564563238797, 54.36423338904003, 0], 
        ])
        self.wpt_set_ = navpy.lla2ned(self.waypoints_lla[:,0], self.waypoints_lla[:,1],
                    self.waypoints_lla[:,2],self.lla_ref[0], self.lla_ref[1], self.lla_ref[2],
                    latlon_unit='deg', alt_unit='m', model='wgs84')

        self.arm_counter = 0
        self.p_gain = 10
        self.wpt_idx_ = np.int8(0)
        self.nav_wpt_reach_rad_ =   np.float32(10)     # waypoint reach condition radius
        # variables for subscribers
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.local_pos_ned_     =   None
        self.prev_wpt_ = np.array([0,0,0])
        self.next_wpt_ = self.wpt_set_[self.wpt_idx_]
       
    # subscriber callback
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state

    def local_position_callback(self,msg):
        self.local_pos_ned_      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)

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
            norm = np.linalg.norm(self.next_wpt_ - self.prev_wpt_)
            trajectory_msg = TrajectorySetpoint()
            # Move in the unit vector direction to the next way point with the given velocity. 
            # Clipping to make sure that it remains within the bounds until the drone is reached.
            x_min = np.min([self.prev_wpt_[0], self.next_wpt_[0]])
            x_max = np.max([self.prev_wpt_[0], self.next_wpt_[0]])
            x_pos = np.clip(self.local_pos_ned_[0] \
                            + self.p_gain * (self.next_wpt_[0] - self.prev_wpt_[0])/norm, \
                                x_min, x_max)
            
            y_min = np.min([self.prev_wpt_[1], self.next_wpt_[1]])
            y_max = np.max([self.prev_wpt_[1], self.next_wpt_[1]])
            y_pos = np.clip(self.local_pos_ned_[1] \
                            + self.p_gain * (self.next_wpt_[1] - self.prev_wpt_[1])/norm, \
                                y_min, y_max)
            
            z_min = np.min([-self.prev_wpt_[2], -self.next_wpt_[2]])
            z_max = np.max([-self.prev_wpt_[2], -self.next_wpt_[2]])
            z_pos = np.clip(self.local_pos_ned_[2] \
                            + self.p_gain * (self.next_wpt_[2] - self.prev_wpt_[2])/norm, \
                                -z_max, -z_min)

            trajectory_msg.position[0]  = x_pos
            trajectory_msg.position[1]  = y_pos
            trajectory_msg.position[2]  = z_pos
           
            trajectory_msg.yaw   =   np.float64(np.pi/2 - np.pi/4)
            # transition
            dist_xyz    =   np.sqrt(np.power(self.next_wpt_[0]-self.local_pos_ned_[0],2)+ \
                                    np.power(self.next_wpt_[1]-self.local_pos_ned_[1],2)+ \
                                    np.power(self.next_wpt_[2]-self.local_pos_ned_[2],2))
            
            if (dist_xyz <= self.nav_wpt_reach_rad_):
                if (self.wpt_idx_ == self.wpt_set_.shape[0] - 1):
                    self.get_logger().info("Offboard mission finished!!")
                else:  
                    self.get_logger().info("Offboard Waypoint: " + str(self.wpt_idx_))  
                    self.wpt_idx_ = self.wpt_idx_ + 1
                    self.prev_wpt_ = self.next_wpt_
                    self.next_wpt_ = self.wpt_set_[self.wpt_idx_]
            self.publisher_trajectory.publish(trajectory_msg)


def main():
    rclpy.init(args=None)
    offboard_mission = OffboardMission()
    rclpy.spin(offboard_mission)
    offboard_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()