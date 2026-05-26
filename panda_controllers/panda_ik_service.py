#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation

# You will need to define this custom service type in panda_interfaces
from panda_interfaces.srv import PandaIK

URDF_PATH = "/home/fpiadmin/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"

JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6",
    "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"
]

JOINT_LIMITS = [
    (-2.8973, 2.8973),  # joint1
    (-1.7628, 1.7628),  # joint2
    (-2.8973, 2.8973),  # joint3
    (-3.0718, -0.0698), # joint4
    (-2.8973, 2.8973),  # joint5
    (-0.0175,  3.7525), # joint6
    (-2.8973, 2.8973),  # joint7
]

class IKService(Node):
    def __init__(self):
        super().__init__('panda_ik_service')
        
        # Load chain once at startup — expensive, do not do per request
        try:
            self.chain = Chain.from_urdf_file(
                URDF_PATH,
                base_elements=["panda_link0"],
                active_links_mask=[False, True, True, True, True, True, True, True, False]
            )
            self.get_logger().info(f'Chain loaded with {len(self.chain.links)} links')
            self.get_logger().info(f'Link names: {[l.name for l in self.chain.links]}')
        except Exception as e:
            print(f"FAILED TO LOAD CHAIN: {e}")
            raise
        
        self.srv = self.create_service(
            PandaIK, 'panda_ik', self.ik_callback)
        self.get_logger().info('IK service ready')

    def check_limits(self, angles):
        for i, (angle, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS)):
            if not (lo <= angle <= hi):
                return False, f"joint{i+1}={angle:.3f} out of range [{lo}, {hi}]"
        return True, "ok"

    def ik_callback(self, request, response):
        pos = request.target_pose.position
        ori = request.target_pose.orientation
        
        target_position = [pos.x, pos.y, pos.z]
        
        # Convert quaternion to 3x3 rotation matrix — ikpy expects this format
        rotation = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        target_orientation = rotation.as_matrix()
        
        if len(request.q_initial) >= 7:
            # Use only the first 7 values (arm joints), wrap with base and tip zeros
            initial = [0.0] + list(request.q_initial[:7]) + [0.0]
        else:
            initial = [0.0] * len(self.chain.links)
        
        try:
            joint_angles = self.chain.inverse_kinematics(
                target_position=target_position,
                target_orientation=target_orientation,
                orientation_mode="all",  # constrain all 3 orientation axes
                initial_position=initial
            )
            active_angles = joint_angles[1:8] # panda_joint1 through panda_joint7
            angles_str = ", ".join(f"{a:.4f}" for a in active_angles)

            if self.check_limits(active_angles)[0] == False:
                response.success = False
                response.message = f"IK solution out of joint limits: [{angles_str}]"
                return response
            
            from sensor_msgs.msg import JointState
            response.joint_target.name = JOINT_NAMES[:7]
            response.joint_target.position = active_angles.tolist()
            response.success = True
            response.message = f"IK solved: [{angles_str}]"
            
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IKService()
        rclpy.spin(node)
    except Exception as e:
        print(f"Node failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()