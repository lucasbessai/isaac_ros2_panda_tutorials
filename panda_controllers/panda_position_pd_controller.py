#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

# position limits for Franka Panda Arm

JOINT_LIMITS = {
    "panda_joint1":  (-2.8973, 2.8973),
    "panda_joint2":  (-1.7628, 1.7628),
    "panda_joint3":  (-2.8973, 2.8973),
    "panda_joint4":  (-3.0718, -0.0698),
    "panda_joint5":  (-2.8973, 2.8973),
    "panda_joint6":  (-0.0175, 3.7525),
    "panda_joint7":  (-2.8973, 2.8973),
    "gripper_finger_joint1": (0.0, 0.04),
    "gripper_finger_joint2": (0.0, 0.04)
}

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

        # self.kp = np.array([200, 200, 150, 100, 75, 50, 25, 0, 0])
        self.kp = np.array([150, 170, 120, 120, 75, 75, 25, 150, 150])
        # self.kp = np.array([100, 100, 88, 75, 50, 40, 20, 10, 10])

        # self.kd = np.array([20, 20, 15, 15, 10, 8, 5, 1, 1])
        self.kd = np.array([10, 10, 8, 8, 5, 4, 2, 0.5, 0.5])
        # self.kd = np.array([5, 5, 4, 4, 2, 1, 0.5, 0, 0])

        self.ki = np.array([1.0, 1.0, 0.75, 0.5, 0.375, 0.25, 0.125, 0.0, 0.0])
        # self.ki = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        
        # Target position for 7 joints + 2 gripper fingers
        # arm straight up position, gripper closed
        # self.target_position = np.array([0.0, 0.0, 0.0, -0.0698, 0.0, 0.0, 0.0, 0.0, 0.0])
        # rest potion, gripper open
        self.target_position = np.array([0.0, -1.16, 0.0, -2.3, 0.0, 1.6, 1.1, 0.4, 0.4])
        # self.target_position = np.array([0.0, -1.76, 0.0, -2.8, 0.0, 0.0, 1.1, 0.0, 0.0])
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

        # Initialize integral state
        self.error_integral = np.zeros(9, dtype=float)

        #Initialize time for integral calculation
        self.last_time = None

        #Anti-windup limits for integral term
        self.integral_limit = np.array([0.5, 2.0, 0.5, 2.0, 0.4, 1.0, 0.3, 0.0, 0.0], dtype=float)

        #Effort saturation limits (max torque/force per joint)
        self.effort_limit = np.array([87, 87, 87, 87, 12, 12, 12, 200, 200], dtype=float)


    def state_callback(self, msg):
        self.joint_names = msg.name
        self.current_position = np.array(msg.position)
        self.current_velocity = np.array(msg.velocity)
        self.compute_and_publish()

    def compute_and_publish(self):
        if self.current_position is None:
            return
        
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        if self.last_time is None:
            self.last_time = now_sec
            return

        dt = now_sec - self.last_time
        self.last_time = now_sec

        # Protect against bad dt values
        if dt <= 0.0 or dt > 0.1:
            return

        error = self.target_position - self.current_position

        # Numerical integration of error
        self.error_integral += error * dt

        # Anti-windup: clamp integral state
        self.error_integral = np.clip(
            self.error_integral,
            -self.integral_limit,
            self.integral_limit
        )

        # PID effort command
        cmd = (
            self.kp * error
            - self.kd * self.current_velocity
            + self.ki * self.error_integral
        )

        # Saturate final effort command
        cmd = np.clip(cmd, -self.effort_limit, self.effort_limit)        

        # PID control law
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

