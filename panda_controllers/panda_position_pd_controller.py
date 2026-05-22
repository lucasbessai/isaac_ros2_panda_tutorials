#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')
        
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        self.pub = self.create_publisher(
            JointState, '/joint_command', 10)
        
        # PD gains per joint
        # self.kp = np.array([400, 400, 400, 400, 400, 400, 400, 400, 400])
        # self.kd = np.array([80, 80, 80, 80, 80, 80, 80, 80, 80])
        # self.kp = np.array([100, 100, 100, 100, 50, 50, 50, 10, 10])
        # self.kd = np.array([10, 10, 10, 10, 5, 5, 5, 1, 1])

        self.kp = np.array([200, 200, 150, 100, 75, 50, 25, 0, 0])
        # self.kp = np.array([150, 150, 120, 100, 75, 50, 25, 0, 0])
        # self.kp = np.array([100, 100, 88, 75, 50, 40, 20, 0, 0])
        self.kd = np.array([20, 20, 15, 15, 10, 8, 5, 0, 0])
        # self.kd = np.array([10, 10, 8, 8, 5, 4, 2, 0, 0])
        # self.kd = np.array([5, 5, 4, 4, 2, 1, 0.5, 0, 0])
        self.ki = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        # Target position for 7 joints + 2 gripper fingers
        # arm straight up position, gripper open
        # self.target_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.target_position = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, 0.04, 0.04])
        self.current_position = None
        self.current_velocity = None
        self.joint_names = None
        # self.joint_names = [
        #     "panda_joint1",
        #     "panda_joint2",
        #     "panda_joint3",
        #     "panda_joint4",
        #     "panda_joint5",
        #     "panda_joint6",
        #     "panda_joint7",
        #     "gripper_finger_joint1",
        #     "gripper_finger_joint2"
        # ]

    def state_callback(self, msg):
        self.joint_names = msg.name
        self.current_position = np.array(msg.position)
        self.current_velocity = np.array(msg.velocity)
        self.compute_and_publish()

    def compute_and_publish(self):
        if self.current_position is None:
            return
        
        # PD control law
        error = self.target_position - self.current_position
        cmd = self.kp * error - self.kd * self.current_velocity
        
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        # For position control, publish to the position field and leave velocity/effort empty. 
        # For torque control, publish to the effort field and leave position/velocity empty.
        out.effort = cmd.tolist()
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    controller = PDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

