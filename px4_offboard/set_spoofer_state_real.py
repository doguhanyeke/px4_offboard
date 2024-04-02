#!/usr/bin/env python3
import numpy as np
from numpy.core.arrayprint import printoptions
from numpy.linalg import pinv

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from px4_msgs.msg import VehicleOdometry
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from std_msgs.msg import Bool


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SpooferTraj(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name="spoofer") -> None:
        self.target_name = target_name
        super().__init__('spoofer_gz_stream_real')
        ## Configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.ns = ""
        
        self.mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/drone162/pose',
            self.pose_cb,
            10
        )
        self.spoofing_flag_pub = self.create_publisher(Bool, "/spoofing_flag", 10)
        self.client = self.create_client(SetEntityPose, "/world/AbuDhabi/set_pose")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not available, waiting...')


        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.count = 0 
        self.mode = 0 
        self.attack_count = 0

    
    def pose_cb(self, msg: PoseStamped):
        self.vehicle_local_position[0] = msg.pose.position.x 
        self.vehicle_local_position[1] = msg.pose.position.y
        self.vehicle_local_position[2] = msg.pose.position.z

    def vector2PoseMsg(self,position, attitude):
        pose_msg = Pose()
        pose_msg.orientation.w = attitude[0]
        pose_msg.orientation.x = attitude[1]
        pose_msg.orientation.y = attitude[2]
        pose_msg.orientation.z = attitude[3]
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        return pose_msg
    
    def cmdloop_callback(self):
        if self.mode ==0:
            self.count-=1
            if (self.count>5000):
                self.mode=1
        else:
            self.count+=1
            if (self.count<0):
                self.mode=0 
         
        spoofer_position = [self.vehicle_local_position[0]+ self.count*0.001 , 
                            self.vehicle_local_position[1], 10.0]             
        vehicle_pose_msg = self.vector2PoseMsg(spoofer_position, self.vehicle_attitude)
        self.request = SetEntityPose.Request()
        entity = Entity()
        entity.name = 'spoofer'
        self.request.entity = entity
        self.request.pose = vehicle_pose_msg
        future = self.client.call_async(self.request)
        if future.done():
            response = future.result()
        
        # Adding delay for not causing sudden jump in residuals. 
        self.attack_count += 1
        if self.attack_count > 100:    
            bool_msg = Bool()
            bool_msg.data = True
            self.spoofing_flag_pub.publish(bool_msg)
        
def main(args=None):
    rclpy.init(args=args)
    
    # executor = rclpy.get_global_executor()
    executor = MultiThreadedExecutor()
    traj_1 = SpooferTraj()
    executor.add_node(traj_1)
    executor.spin()

    try:
        rclpy.shutdown()
    except Exception():
        pass

if __name__ == '__main__':
    main()
