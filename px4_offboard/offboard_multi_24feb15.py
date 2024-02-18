#!/usr/bin/env python

__author__ = "Yifan Guo, Minhyun Cho"
__contact__ = "@purdue.edu"

import argparse

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import navpy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand

from geometry_msgs.msg import PointStamped
from std_msgs.msg import UInt8, Bool

from functools import partial

class OffboardMission(Node):

    def __init__(self,n_drone,waypoints,ref):

        super().__init__("px4_offboard_mission")
        self.n_drone = n_drone
        self.all_publishers = [{'publisher_offboard_mode':None, 'publisher_trajectory':None, 'publisher_vehicle_command':None} for _ in range(n_drone)]
        self.all_subscribers = [{'status_sub':None, 'local_pos_sub':None} for _ in range(self.n_drone)]

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

        # self.local_pos_sub1 = self.create_subscription(
        #     VehicleLocalPosition,
        #     f'/px4_1/fmu/out/vehicle_local_position',
        #     lambda msg: self.local_position_callback(msg,id=0),
        #     qos_profile_sub)
        
        # self.local_pos_sub2 = self.create_subscription(
        #     VehicleLocalPosition,
        #     f'/px4_1/fmu/out/vehicle_local_position',
        #     lambda msg: self.local_position_callback(msg,id=1),
        #     qos_profile_sub)
        
        # self.local_pos_sub3 = self.create_subscription(
        #     VehicleLocalPosition,
        #     f'/px4_1/fmu/out/vehicle_local_position',
        #     lambda msg: self.local_position_callback(msg,id=2),
        #     qos_profile_sub)

        for i in range(self.n_drone):
            # define subscribers
            # print(1)
            # status_sub_temp = "self.status_sub_"+str(i+1)+" = self.create_subscription(VehicleStatus,f\'/px4_"+str(i+1)+"/fmu/out/vehicle_status\',lambda msg: self.vehicle_status_callback(msg, id="+str(i)+"),qos_profile_sub)"
            # exec(status_sub_temp)

            # exec(status_sub_temp, globals())
            # return_workaround = loc['return_me']
            # print(return_workaround)



            # status_sub = self.create_subscription(
            #     VehicleStatus,
            #     f'/px4_{i+1}/fmu/out/vehicle_status',
            #     lambda msg: self.vehicle_status_callback(msg, id=i),
            #     qos_profile_sub)
            self.all_subscribers[i]['status_sub'] = self.create_subscription(
                VehicleStatus,
                f'/px4_{i+1}/fmu/out/vehicle_status',
                partial(self.vehicle_status_callback,id=i),
                qos_profile_sub)


            # local_pos_sub = self.create_subscription(
            #     VehicleLocalPosition,
            #     f'/px4_{i+1}/fmu/out/vehicle_local_position',
            #     lambda msg: self.local_position_callback(msg,id=i),
            #     qos_profile_sub)

            self.all_subscribers[i]['local_pos_sub'] = self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{i+1}/fmu/out/vehicle_local_position',
                partial(self.local_position_callback,id=i),
                qos_profile_sub)

            # locals()["self.local_pos_sub"+str(i+1)] = self.create_subscription(
            #     VehicleLocalPosition,
            #     f'/px4_{i+1}/fmu/out/vehicle_local_position',
            #     partial(self.local_position_callback,id=i),
            #     qos_profile_sub)
            
            
            # self.all_subscribers[i]['local_pos_sub'] = self.create_subscription(
            #     VehicleLocalPosition,
            #     f'/px4_{i+1}/fmu/out/vehicle_local_position',
            #     lambda msg: self.local_position_callback(msg,id=i),
            #     qos_profile_sub)
            
            # define publishers
            publisher_offboard_mode = self.create_publisher(
                OffboardControlMode, 
                f'/px4_{i+1}/fmu/in/offboard_control_mode', 
                qos_profile_pub)
            self.all_publishers[i]['publisher_offboard_mode'] = publisher_offboard_mode

            publisher_trajectory = self.create_publisher(
                TrajectorySetpoint, 
                f'/px4_{i+1}/fmu/in/trajectory_setpoint', 
                qos_profile_pub)
            self.all_publishers[i]['publisher_trajectory'] = publisher_trajectory
            
            publisher_vehicle_command = self.create_publisher(
                VehicleCommand, 
                f'/px4_{i+1}/fmu/in/vehicle_command', 
                qos_profile_pub)                                        # disable for an experiment
            self.all_publishers[i]['publisher_vehicle_command'] = publisher_vehicle_command

        if 'self.local_pos_sub1' in locals():
            print('confirm1')

        if 'self.local_pos_sub2' in locals():
            print('confirm2')    

        if 'self.local_pos_sub3' in locals():
            print('confirm3')   

        # parameters for callback
        self.timer_period   =   0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        
         # Set up the origin and waypoints; Convert global coordiante to local ones.
        self.lla_ref = ref
        self.waypoint_idx = [0 for _ in range(self.n_drone)]
        self.waypoints_lla = waypoints
        wpt_ref = self.next_pos_ned = navpy.lla2ned(self.waypoints_lla[:,0], self.waypoints_lla[:,1],
                    self.waypoints_lla[:,2],self.lla_ref[0], self.lla_ref[1], self.lla_ref[2],
                    latlon_unit='deg', alt_unit='m', model='wgs84')

        self.wpt_set_list = []
        for i in range(self.n_drone):
            if i == 0:
                self.wpt_set_list.append(wpt_ref)
            elif i%2 == 0:
                wpt = wpt_ref.copy()
                wpt[:, 0]+=0.1*(i+1)/2
                self.wpt_set_list.append(wpt)
            else:
                wpt = wpt_ref.copy()
                wpt[:, 0]-=0.1*(i+1)/2
                self.wpt_set_list.append(wpt)
        
        self.arm_counter_list = [0 for i in range(self.n_drone)]

        self.velocity = 10
        self.wpt_idx_list = np.array([np.int8(0) for _ in range(self.n_drone)])
        self.nav_wpt_reach_rad_ =   np.float32(10)     # waypoint reach condition radius
        # variables for subscribers
        self.nav_state_list = [VehicleStatus.NAVIGATION_STATE_MAX for _ in range(self.n_drone)]
        self.local_pos_ned_list     =   [None for _ in range(self.n_drone)]
        self.prev_wpt_list = np.array([np.array([0,0,0]) for _ in range(self.n_drone)])
        self.next_wpt_list = []
        for i in range(self.n_drone):
            self.next_wpt_list.append(self.wpt_set_list[i][self.wpt_idx_list[i]])
        # self.curr_t = 0
        # self.prev_t = 0
        # self.next_t = np.linalg.norm(self.next_wpt_ -self.prev_wpt_)/self.velocity
        # print("next_time:" + str(self.next_t))

    # subscriber callback
    def vehicle_status_callback(self, msg, id):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state_list[id] = msg.nav_state
    
    def local_position_callback(self,msg,id):
        self.local_pos_ned_list[id]      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)


    def publish_vehicle_command(self,command, id, param1=0.0,param2=0.0):            # disable for an experiment
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
        self.all_publishers[id]['publisher_vehicle_command'].publish(msg)

    def cmdloop_callback(self):
        # print('drone 1 pos')
        # print(self.local_pos_ned_list[0])
        # print('drone 2 pos')
        # print(self.local_pos_ned_list[1])
        # print('drone 3 pos')
        # print(self.local_pos_ned_list[2])





        #Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        for i in range(self.n_drone):
            self.all_publishers[i]['publisher_offboard_mode'].publish(offboard_msg)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, i, 1., 6.)
            print(self.nav_state_list)
            if self.nav_state_list[i] != VehicleStatus.ARMING_STATE_ARMED or self.arm_counter_list[i] < 10:
                self.arm_counter_list[i] += 1
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, i, 1.0)

            if self.nav_state_list[i] == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                print(i)
                norm = np.linalg.norm(self.next_wpt_list[i] - self.prev_wpt_list[i])
                trajectory_msg = TrajectorySetpoint()
                # Move in the unit vector direction to the next way point with the given velocity. 
                # Clipping to make sure that it remains within the bounds until the drone is reached.
                x_pos = np.clip(self.local_pos_ned_list[i][0] \
                                + self.velocity * (self.next_wpt_list[i][0] - self.prev_wpt_list[i][0])/norm, \
                                    self.prev_wpt_list[i][0], self.next_wpt_list[i][0])
                y_pos = np.clip(self.local_pos_ned_list[i][1] \
                                + self.velocity * (self.next_wpt_list[i][1] - self.prev_wpt_list[i][1])/norm, \
                                    self.prev_wpt_list[i][1], self.next_wpt_list[i][1])
                z_pos = np.clip(self.local_pos_ned_list[i][2] \
                                + self.velocity * (self.next_wpt_list[i][2] - self.prev_wpt_list[i][2])/norm, \
                                    self.prev_wpt_list[i][2], self.next_wpt_list[i][2])

                trajectory_msg.position[0]  = x_pos
                trajectory_msg.position[1]  = y_pos
                trajectory_msg.position[2]  = z_pos
                
                trajectory_msg.yaw   =   np.float64(np.pi/2 - np.pi/4)
                # transition
                dist_xyz    =   np.sqrt(np.power(self.next_wpt_list[i][0]-self.local_pos_ned_list[i][0],2)+ \
                                        np.power(self.next_wpt_list[i][1]-self.local_pos_ned_list[i][1],2)+ \
                                        np.power(self.next_wpt_list[i][2]-self.local_pos_ned_list[i][2],2))
                
                if (dist_xyz <= self.nav_wpt_reach_rad_):
                    if (self.wpt_idx_list[i] == self.wpt_set_list[i].shape[0] - 1):
                        print("Offboard mission finished")
                    else:    
                        print("Waypoint: " + str(self.wpt_idx_list[i]))
                        self.wpt_idx_list[i] = self.wpt_idx_list[i] + 1
                        self.prev_wpt_list[i] = self.next_wpt_list[i]
                        self.next_wpt_list[i] = self.wpt_set_list[i][self.wpt_idx_list[i]]

                self.all_publishers[i]['publisher_trajectory'].publish(trajectory_msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', type=int)
    args = parser.parse_args()
    n_drone = args.n

    rclpy.init(args=None)
    ref = np.array([24.484043629238872, 54.36068616768677, 0]) # latlonele -> (deg,deg,m)
    waypoints = np.array([
            [24.484326113268185, 54.360644616972564, 30],
            [24.48476311664666, 54.3614948536716, 30],
            [24.485097533474377, 54.36197496905472, 30],
            [24.485400216562002, 54.3625570084458, 30], 
            [24.48585179883862, 54.36321951405934, 30], 
            [24.486198417650844, 54.363726451568475, 30], 
            [24.486564563238797, 54.36423338904003, 0], 
        ])
    offboard_mission = OffboardMission(n_drone, waypoints, ref)

    rclpy.spin(offboard_mission)

    offboard_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()