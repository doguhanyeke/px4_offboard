#!/usr/bin/env python3
import numpy as np
from numpy.core.arrayprint import printoptions
from numpy.linalg import pinv

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from rclpy.clock import Clock


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MocapGzStream(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name="drone162") -> None:
        self.target_name = target_name
        super().__init__("mocap_gz_stream")
        ## Configure subscritpions    

        self.client = self.create_client(SetEntityPose, "/world/AbuDhabi/set_pose")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not available, waiting...')
        
        self.mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/drone162/pose',
            self.pose_cb,
            10
        )
        print(f"setting state for {self.target_name}")
        self.entity = Entity()
        self.entity.name = self.target_name
        self.request = SetEntityPose.Request()
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.drone_pose=None
    
    def pose_cb(self, msg: PoseStamped):
            self.drone_pose = msg
        

    def cmdloop_callback(self):
        if self.drone_pose is not None:
            if self.drone_pose.pose is not None:
                self.request.entity = self.entity
                self.request.pose = self.drone_pose.pose
                future = self.client.call_async(self.request)
                if future.done():
                    response = future.result()

def main(args=None):
    rclpy.init(args=args)
    
    # executor = rclpy.get_global_executor()
    executor = MultiThreadedExecutor()
    traj_1 = MocapGzStream()
    executor.add_node(traj_1)
    executor.spin()

    try:
        rclpy.shutdown()
    except Exception():
        pass

if __name__ == '__main__':
    main()
