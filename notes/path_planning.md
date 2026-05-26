**Trajectory Planning 1/2, MECH572, Inna Sharf, McGill University, 20***
-
- Path Planning vs Trajectory Planning
  - A path is a purely geometric object. No relation to time.
    - Often defined parametrically?
  - A trajectory is defines motion along a path as a function of time.
    - (x(t), y(t), z(t)) defines the position of the end effector as a function of time.
    - a trajectory planner with generate **p(t)**, **v(t)**, and **a(t)** for the end effector to move from the initial state to the final state.
- With a point to point trajectory, we want to define $q(t)$, $\dot{q}(t)$, and $\ddot{q}(t)$
  - given the initial position and constraints on the velocity and acceleration, coefficients of a polynomial trajectory can be derived.
  - contraints may be: $\dot{q_i}(t)=0$, and $\dot{q_f}(t)=0$ and similar for acceleration constraints. 
- Trapezoidal velocity profiles are often desirable since joints are often characterized by limits on their velocities and accelerations (torques). The trapezoidal path can guarantee operation within these limits. 
  - providing these limiting values and solving some constraining equations will output the position, velocity, and acceleration profile for a given joint. 

**Robotics: Modelling, Planning and Control**
-

- The actual name robot arose from its use by the playwright Karel Capek in the play ˇ Rossum’s Universal Robots (1920). 

- Inverse Kinematics 

  - Given an end effector position, solve for the joint angles that realise this position. 

    - The solution is often not unique: for a two link planar arm the same position can be reached with elbow-up or elbow-down. In this case the inverse kinematics result outputs a set of possible solution.  

    - For path planning, you need to decide which branch of paths to take. You do not want to start going down a branch that leads to a singularity before reaching the desired final state.  

*Singular points*  

- Singularities represent configurations at which mobility of the structure is reduced, i.e., it is not possible to impose an arbitrary motion to the end-effector.  

- When the structure is at a singularity, infinite solutions to the inverse kinematics problem may exist.  

- In the neighbourhood of a singularity, small velocities in the operational space may cause large velocities in the joint space. 

- a continuous path in task space does not always mean a nice continuous path in joint space. 

  - For path planning with obstacle avoidance: find a path that is collision free, and avoids singular points. 

*Tragectory Planning*
- In principle, it can be conceived that the inputs to a trajectory planning algorithm are the path description, the path constraints, and the constraints imposed by manipulator dynamics, whereas the outputs are the end-effector trajectories in terms of a time sequence of the values attained by position, velocity and acceleration. 

- the trajectory planning algorithm generates a time sequence of variables that describe end-effector position and orientation over time in respect of the imposed constraints. Since the control action on the manipulator is carried out in the joint space, a suitable inverse kinematics algorithm is to be used to reconstruct the time sequence of joint variables corresponding to the above sequence in the operational space. 

*Motion through a sequence of points* 

- It should not be forgotten that the corresponding joint variables have to be computed from the operational space poses. A dense path requires significant computations 

- Creating a degree N-1 order polynomial to describe q(t) for a path through N path points is undesirable for several reasons.  

- instead use a series of cubic polynomials to  

 

*Why are the path planning problem and the trajectory planning problem separated in the literature?*

- Simple solution: discretized path  à feed to PID controller at a fixed rate à End effector moves along waypoints to desired final state. 

  - There is a difference between waypoint-following with feedback control (setpoint regulation) and trajectory planning.  

    - Low level control strategy 

    - With a series of setpints, the controller does not know the desired velocity, or acceleration and the constraints on these parameters. 

  - A trajectory is implicitly created through this method but you do not get control over speed, dynamic feasibility, coordination of joints, etc.  

- The separation (of path planning and trajectory planning) is useful because many planners first solve a geometric problem: “Find any collision-free route from start to goal.” Then they solve a timing/dynamics problem: “Move along that route smoothly and safely within actuator limits.” 

- Just using the position error could lead to breaking the robot with unintended jerks, large velocities, etc. Instead: 
  1. if you want to move between two points you make a trajectory that smooths the velocity, acceleration, maybe jerk that isn’t too computationally heavy.  

  2. Today this is often done with packages like “moveit.py”. These mechanics problems were largely solved in the 90s.  

  3. If you want to move between two states and in the process avoid hitting things along the way, you need to define other path points between initial and final state then generate a trajectory that goes through all the path points that also smooths velocity, acceleration, jerk.  

  4. The trajectory is a sequence of positions, velocities and accelerations. These ae converted to joint space thereafter for control of the robot.  

  5. From the joint space realization of the trajectory (ie. q, q_dot, q_ddot of each joint) you then use closed loop feedback control at each joint (with an encoder) to realize the trajectory. The closed loop feedback at each joint ensures that you actually realize the prescribed trajectory. 

  6. A more complex control method would involve MIMO control of all the joint states together. This will result in better tracking since each joint’s is coupled with the error of the other joints. A MIMO control matrix effectively communicates error at the joint level to all the joints resulting in a more holistic realization of the goal. (Depending on the use case of the robot, this may not be required.)

Motion Planning with MATLAB, https://www.mathworks.com/campaigns/offers/motion-planning-with-matlab.html
- 
*overview*
- path planning alongside perception (vison) and control systems comprise the three central building blocks for autonomous navigation.
- **Sampling-based search algorithms**, which create a searchable tree by randomly sampling new nodes or robot configurations in a state space. Sampling-based algorithms can be suitable for high-dimensional search spaces such as those used to find a valid set of configurations for a robot arm to pick up an object. Generating dynamically feasible paths for various practical applications make sampling-based planning popular, even though it does not provide a complete solution.

*Path Planning with A\* and RRT | Autonomous Navigation, Part 4*

- A* is a search base algorithm that searches viable points to travel to next. This is an improvement from just searching all possible paths. It does this by computing the shortest distance to a node from the start and the straight line distance from each explored node to the end. These metrics are used as costs.
  - compulationally scales exponentially with grid dimension
- RRT* (randomly exploring random trees) randomly explores the grid and makes branches back to the starting position. The nearest node to each new randomly selected node is computed and path between them is made. For a new random node the path to the nearest node is compared to paths through the neighbourhood of points and shortest path back to the start is taken. 

*Trajectory Planning for Robot Manipulators*
- A trajectory decribes how to follow a path as a function of time.
  - A path is often described by a series of position waypoints that are interpolated between depending on the trajectory style chosen.
- Trajectories can be developed in task space or joints space
  - Task space will more accurately align with the task (end effector position)
    - requires more computation
    - results in more joint accuater effort
  - Joint space will result in smoother joint movements
    - more effort to design path
    - not as accurate task following
- Waypoint orientations can also be defined. Interpolating between angles is a bit more tricky than positions because they are continuously wrapping or non-unique (Euler angles, gimble-lock)
  - Quaternions are an unambiguous way of representing orientations. SLERP (spherical linear interpolation) is used in Matlab. It finds the shortest  path between orientations alongs a sphere.  
- Trapezoidal paths can be restrictive since

**ROS2 Joint Control: Extension Python Scripting, https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html**
-
- "ros2_publisher.py" creates a simpler publisher node that publishes a "JointState" object to the `/joint_command` topic using a timer and timer_callback framework. A "JointState" instance has an instance method for name, position, and other properties. 
  - Specifically, the program computes a sinusoidal position for each joint every 50ms centered around `default_joints` with $\pm 0.5$ radian amplitude, publishes that as a JointState message with explicit joint names, and `spin()` keeps the timer callback running indefinitely.
- the default position defined in the code can be returned to by running this command in the terminal:
```bash
ros2 topic pub -r 20 /joint_command sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'],
  position: [0.0, -1.16, 0.0, -2.3, 0.0, 1.6, 1.1, 0.4, 0.4],
  velocity: [],
  effort: []
}"
```
*"-r 20" controls the frequency at which the message is published. 20 corresponds to 20Hz. Just sending a message a single time will not get the arm to move to the right place.
- There is an script in the online tutorial that recreates the action graph node and connects when run in the script editor window. It simply imports `import omni.graph.core as og` and defines all the nodes and connections between them through code rather than a GUI.
- "ROS2 Subscribe Joint State" node in the action graph subscribes to a topic you define that is getting a "JointState" type of object. In this case the topic is "/joint_command"
- The motion is driven by PhysX's internal joint drive, not a trajectory planner. Each joint in the Franka prim has a drive API with stiffness and damping properties (Property -> Physics -> Joint State -> Angular). A PD law is applied to the joints:
  - `torque = stiffness × (target_position − current_position) − damping × current_velocity`
  - ROS2 Subscribe Joint State recieves a message -> unpacks position/velocity/effort array and other information (names, etc.) -> passes information to Articulation Controller node -> node calls physics API -> Simulation responds
- An external python file can be used to control the simulated arm through ros2. Making a file executable (chmod +x) is only needed if you want to run it as ./position_velocity_publisher.py directly, which also requires a shebang line at the top:

`#!/usr/bin/env python3`

- it can also be run without being an executable with python3 path/to/script.py. Specifcally:

```bash
python3 ~/IsaacRos2_tutorials/test_ws/src/panda_controllers/scripts/position_velocity_publisher.py
```

**Toy Problem: motion planning for Franka Panada arm in Issac Sim using ros2**
- 
- The .tolist() instance method in Python converts array-like objects—most commonly from the NumPy or Pandas libraries—into native Python lists. It is widely used when you need to serialize data, pass it to standard Python functions, or convert multidimensional arrays into nested lists
  - in ros2, mostly used when you need to manipulate arrays then publish them to a topic as a python list. 

Build workspace:
```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

cd ~/IsaacRos2_tutorials/test_ws/src

ros2 pkg create panda_motion_planning --build-type ament_python --dependencies rclpy

cd ~/IsaacRos2_tutorials/test_ws/src/panda_controllers/panda_controllers
chmod +x panda_position_pd_controller.py

cd ~/IsaacRos2_tutorials/test_ws
colcon build --symlink-install

```
in setup.py

```python
entry_points={
    'console_scripts': [
        'position_pd_controller = panda_controllers.panda_position_pd_controller:main',
    ],
},
```
to run the node program
```bash
source ~/IsaacRos2_tutorials/test_ws/install/setup.bash
ros2 run panda_controllers position_pd_controller
```
after killing controller node run in terminal to zero joint efforts (arm goes limp):
```bash
ros2 topic pub -r 5 /joint_command sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'],
  position: [],
  velocity: [],
  effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"
```


script to quickly change joint stiffnesses and damping in Isaac Sim using the script editor window (Window -> Script Editor). stiffness = 0.0 and damping = 0.05 work well: 
```python
import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

# Set these values as desired
stiffness = 400.0
damping = 80.0

# Walk every prim in the stage and find joints
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.DriveAPI):
        # Franka has both linear and angular drives
        for drive_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive:
                drive.GetStiffnessAttr().Set(stiffness)
                drive.GetDampingAttr().Set(damping)
                print(f"Set {prim.GetPath()} [{drive_type}] stiffness={stiffness} damping={damping}")
```
Parameters can be changed individually by specifing each joint path:

```python
import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()

joint_settings = {
    "/World/panda/panda_joint1": (100.0, 20.0),
    "/World/panda/panda_joint2": (100.0, 20.0),
    "/World/panda/panda_joint7": (20.0, 5.0),
    # etc.
}

for path, (stiff, damp) in joint_settings.items():
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if drive:
        drive.GetStiffnessAttr().Set(stiff)
        drive.GetDampingAttr().Set(damp)
```
- "pinocchio" library can be used for static gravity compensation. 
  - gravity compensation is a feedforward term added to the control law. Based on the model parameters and the current position you can calculate the torque required to counter the force of gravity 

- "ikpy" is also a standard library used for inverse kinematics from a .URDF file
  - `ikpy.Chain.from_urdf_file()` extracts the kinematic chain from the urdf file and creates a class of object that has an inverse kinematics method. Several parameters are required to define the chain proberly


Node files using packages that conflict with the ros2 python framework must be run inside a dedicated .venv. They cannot be run with `ros2 run /path/to`. 

```bash
source /opt/ros/humble/setup.bash
source ~/IsaacRos2_tutorials/test_ws/install/setup.bash
source ~/IsaacRos2_tutorials/test_ws/monitor_venv/bin/activate
python3 ~/IsaacRos2_tutorials/test_ws/src/panda_controllers/panda_controllers/
```

```
test_ws/src/
├── panda_controllers/        # ament_python — nodes only
│   └── panda_controllers/
│       ├── panda_position_pd_controller.py
│       ├── panda_ik_service.py
│       └── panda_task_commander.py
└── panda_interfaces/         # ament_cmake — message/service definitions only
    └── srv/
        └── PandaIK.srv
```

```bash
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{
  position: {x: 0.4, y: 0.0, z: 0.5},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}"
```