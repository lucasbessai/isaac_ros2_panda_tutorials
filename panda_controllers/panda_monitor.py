#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6",
    "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"
]
N_JOINTS = len(JOINT_NAMES)
HISTORY  = 200  # number of timesteps to show in the rolling window

class PandaMonitor(Node):
    def __init__(self):
        super().__init__('panda_monitor')

        self.sub_state = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)
        self.sub_cmd = self.create_subscription(
            JointState, '/joint_command', self.cmd_callback, 10)

        # Rolling buffers — one deque per joint
        self.t_state       = deque(maxlen=HISTORY)
        self.positions     = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self.velocities    = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self.cmd_positions = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self.efforts       = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]
        self.errors        = [deque(maxlen=HISTORY) for _ in range(N_JOINTS)]

        self.latest_cmd_position = np.zeros(N_JOINTS)
        self.latest_cmd_effort   = np.zeros(N_JOINTS)
        self.t0 = None
        self.lock = threading.Lock()

    def state_callback(self, msg):
        if self.t0 is None:
            self.t0 = self.get_clock().now().nanoseconds
        t = (self.get_clock().now().nanoseconds - self.t0) * 1e-9

        pos = np.array(msg.position) if msg.position else np.zeros(N_JOINTS)
        vel = np.array(msg.velocity) if msg.velocity else np.zeros(N_JOINTS)

        # Map by joint name in case order differs
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        with self.lock:
            self.t_state.append(t)
            for j, name in enumerate(JOINT_NAMES):
                i = name_to_idx.get(name)
                if i is not None:
                    self.positions[j].append(pos[i])
                    self.velocities[j].append(vel[i])
                    self.errors[j].append(self.latest_cmd_position[j] - pos[i])
                    self.cmd_positions[j].append(self.latest_cmd_position[j])
                    self.efforts[j].append(self.latest_cmd_effort[j])

    def cmd_callback(self, msg):
        pos = np.array(msg.position) if msg.position else np.zeros(N_JOINTS)
        eff = np.array(msg.effort)   if msg.effort   else np.zeros(N_JOINTS)
        with self.lock:
            self.latest_cmd_position = pos if len(pos) == N_JOINTS else np.zeros(N_JOINTS)
            self.latest_cmd_effort   = eff if len(eff) == N_JOINTS else np.zeros(N_JOINTS)


def build_figure(monitor):
    """Build a 3-row figure: position+command, error, effort."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Panda Joint Monitor", fontsize=13)
    axes[0].set_ylabel("Position (rad)")
    axes[1].set_ylabel("Error (rad)")
    axes[2].set_ylabel("Effort (Nm)  /  Cmd position (rad)")
    axes[2].set_xlabel("Time (s)")

    # One line per joint, colour-coded consistently
    colors = plt.cm.tab10(np.linspace(0, 1, N_JOINTS))
    pos_lines, cmd_lines, err_lines, eff_lines = [], [], [], []

    for j in range(N_JOINTS):
        c = colors[j]
        label = JOINT_NAMES[j].replace("panda_", "")
        p, = axes[0].plot([], [], color=c, lw=1.2, label=label)
        cm, = axes[0].plot([], [], color=c, lw=1.0, ls='--', alpha=0.5)
        e, = axes[1].plot([], [], color=c, lw=1.2)
        ef, = axes[2].plot([], [], color=c, lw=1.2)
        pos_lines.append(p)
        cmd_lines.append(cm)
        err_lines.append(e)
        eff_lines.append(ef)

    axes[0].legend(loc='upper right', fontsize=7, ncol=3)
    axes[0].axhline(0, color='gray', lw=0.5, ls=':')
    axes[1].axhline(0, color='gray', lw=0.5, ls=':')
    axes[2].axhline(0, color='gray', lw=0.5, ls=':')

    def update(_frame):
        with monitor.lock:
            t = list(monitor.t_state)
            if len(t) < 2:
                return pos_lines + cmd_lines + err_lines + eff_lines
            for j in range(N_JOINTS):
                pos_lines[j].set_data(t, list(monitor.positions[j]))
                cmd_lines[j].set_data(t, list(monitor.cmd_positions[j]))
                err_lines[j].set_data(t, list(monitor.errors[j]))
                eff_lines[j].set_data(t, list(monitor.efforts[j]))
            for ax in axes:
                ax.relim()
                ax.autoscale_view()
        return pos_lines + cmd_lines + err_lines + eff_lines

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=False, cache_frame_data=False)
    return fig, ani


def main(args=None):
    rclpy.init(args=args)
    monitor = PandaMonitor()

    # Spin ROS in a background thread so matplotlib can own the main thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    ros_thread.start()

    fig, ani = build_figure(monitor)
    plt.tight_layout()
    plt.show()  # blocks here — closing the window exits cleanly

    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()