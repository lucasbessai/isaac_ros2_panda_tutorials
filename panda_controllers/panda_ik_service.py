#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ikpy.chain import Chain
import numpy as np

# You will need to define this custom service type in panda_interfaces
from panda_interfaces.srv import PandaIK

URDF_PATH = "/home/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"  # Replace with actual Franka URDF path


JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6",
    "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"
]

class IKService(Node):
    def __init__(self):
        super().__init__('panda_ik_service')
        
        # Load chain once at startup — expensive, do not do per request
        self.chain = Chain.from_urdf_file(
            URDF_PATH,
            base_elements=["panda_link0"],
            last_link_vector=[0, 0, 0.1],  # adjust for panda_hand offset
            active_links_mask=[False, True, True, True,
                               True, True, True, True, False, False]
        )
        
        self.srv = self.create_service(
            PandaIK, 'panda_ik', self.ik_callback)
        self.get_logger().info('IK service ready')

    def ik_callback(self, request, response):
        pos = request.target_pose.position
        ori = request.target_pose.orientation
        
        target_position = [pos.x, pos.y, pos.z]
        
        from scipy.spatial.transform import Rotation
        # Convert quaternion to 3x3 rotation matrix — ikpy expects this format
        rotation = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        target_orientation = rotation.as_matrix()
        
        if len(request.q_initial) == 9:
            initial = [0.0] + list(request.q_initial) + [0.0]
        else:
            initial = [0.0] * len(self.chain.links)
        
        try:
            joint_angles = self.chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
                orientation_mode="all",  # constrain all 3 orientation axes
                initial_position=initial
            )
            active_angles = joint_angles[1:8]
            
            from sensor_msgs.msg import JointState
            response.joint_target.name = JOINT_NAMES[:7]
            response.joint_target.position = active_angles.tolist()
            response.success = True
            response.message = "IK solved"
            
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IKService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()