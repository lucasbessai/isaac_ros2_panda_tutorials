import threading

import rclpy
from sensor_msgs.msg import JointState
import numpy as np

rclpy.init()
node = rclpy.create_node('position_velocity_publisher')
pub = node.create_publisher(JointState, 'joint_command', 10)

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

joint_state = JointState()
joint_state.name = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7"
]

# Joint in Isaac Sim must be configured for the specific control variable (ie. position, velocity, effort/torque) 
# Property -> Physics -> Joint State -> Angular 
# position control: stiffness > 0, damping > 0 
# velocity control: stiffness = 0, damping > 0
# Torque/force control: stiffness=0, damping=0, use effort field
joint_state.position = [np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2]  # target positions
# joint_state.velocity = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]  # max speed toward target

# Tutorial framework for controlling position and velocit of different joints seperately. This is not for the Franka Panda Arm.
# 1. seperate messages
# joint_state_position = JointState()
# joint_state_velocity = JointState()
# joint_state_position.name = ["joint1", "joint2","joint3"]
# joint_state_velocity.name = ["wheel_left_joint", "wheel_right_joint"]
# joint_state_position.position = [0.2,0.2,0.2]
# joint_state_velocity.velocity = [20.0, -20.0]

# 2. combined into a single message. Use ‘nan’ for joints that are not being controlled by that control mode.
# joint_state = JointState()
# joint_state.name = ["joint1", "joint2","joint3", "wheel_left_joint", "wheel_right_joint"]
# joint_state.position = [0.2,0.2,0.2, float('nan'), float('nan')]
# joint_state.velocity = [float('nan'), float('nan'), float('nan'), 20.0, -20.0]

rate = node.create_rate(10)
try:
    while rclpy.ok():
        # pub.publish(joint_state_position)
        pub.publish(joint_state)
        rate.sleep()
except KeyboardInterrupt:
    pass
rclpy.shutdown()
thread.join()