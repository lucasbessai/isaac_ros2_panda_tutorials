#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from panda_interfaces.srv import PandaIK


JOINT_LIMITS = [
    (-2.8973, 2.8973),  # joint1
    (-1.7628, 1.7628),  # joint2
    (-2.8973, 2.8973),  # joint3
    (-3.0718, -0.0698), # joint4
    (-2.8973, 2.8973),  # joint5
    (-0.0175,  3.7525), # joint6
    (-2.8973, 2.8973),  # joint7
]

class TaskCommander(Node):
    def __init__(self):
        super().__init__('panda_task_commander')
        
        # Client for the IK service
        self.ik_client = self.create_client(PandaIK, 'panda_ik')
        
        # Publishes joint targets to the controller
        self.joint_pub = self.create_publisher(JointState, '/joint_target', 10)
        
        # Subscribes to task space targets from outside
        # (you or another node publishes a Pose here)
        self.pose_sub = self.create_subscription(
            Pose, '/target_pose', self.pose_callback, 10)
        
        # Subscribe to current joint state to use as IK seed
        self.current_joint_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Wait for IK service to be available
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
    

    def joint_state_callback(self, msg):
        self.current_joint_positions = list(msg.position)

    def pose_callback(self, pose_msg):
        # Build service request
        request = PandaIK.Request()
        request.target_pose = pose_msg
        request.q_initial = self.current_joint_positions if self.current_joint_positions is not None else []
        
        # Call IK service asynchronously
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_response_callback)

    def ik_response_callback(self, future):
        response = future.result()
        if response.success:
            self.joint_pub.publish(response.joint_target)
            self.get_logger().info('Published new joint target')
        else:
            self.get_logger().warn(f'IK failed: {response.message}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()