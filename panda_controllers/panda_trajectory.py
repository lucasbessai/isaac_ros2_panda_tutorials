#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class PandaTrajectory(Node):
    def __init__(self):
        super().__init__('panda_trajectory')
        
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        self.pub = self.create_publisher(
            JointState, '/joint_command', 10)
        
        # Trajectory parameters
        self.start_time = None
        self.duration = 5.0  # seconds
        self.start_position = None
        self.target_position = np.array([0.0, -1.16, 0.0, -2.3, 0.0, 1.6, 1.1])  # rest position

    def state_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
            self.start_position = np.array(msg.position[:7])  # first 7 joints
        
        t = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
        if t > self.duration:
            t = self.duration
        
        # Linear interpolation between start and target
        alpha = t / self.duration
        cmd_position = (1 - alpha) * self.start_position + alpha * self.target_position
        
        cmd_msg = JointState()
        cmd_msg.name = msg.name[:7]
        cmd_msg.position = cmd_position.tolist()
        
        self.pub.publish(cmd_msg)

class TrapezoidalTrajectory:
    def __init__(self, q_start, q_goal, v_max, a_max):
        self.q_start = np.array(q_start)
        self.q_goal = np.array(q_goal)
        
        # Compute per-joint profiles, then scale to the
        # slowest joint so all joints arrive simultaneously
        delta = np.abs(q_goal - q_start)
        
        # Time to accelerate to v_max
        t_acc = v_max / a_max
        # Distance covered during acceleration
        d_acc = 0.5 * a_max * t_acc**2
        
        # For each joint, compute total time
        # (handling case where delta is too small to reach v_max)
        self.t_total = np.where(
            delta > 2 * d_acc,
            t_acc + (delta - 2 * d_acc) / v_max + t_acc,
            2 * np.sqrt(delta / a_max)
        )
        self.t_sync = np.max(self.t_total)  # sync to slowest joint
        
    def sample(self, t):
        # Returns joint positions at time t along the trajectory
        # ...interpolation logic here
        pass