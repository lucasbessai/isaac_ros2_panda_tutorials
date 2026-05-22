**Isaac Sim Documentation, Nvidia**
- 
- Isaac Sim can be installed on a workstation or through a Docker container. 
  - Workstation is suitable for tutorials, DIY dev projects, testing, etc.. System constraints can really slow down Isaac Sim on a workstation.
    - You do not necessarily have to meet the minimum requirements to install and run isaac sim?
    - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html

  - Container installation is suitable for running isaac sim on a remote server. Would be used if developing an application? Can be useful for running different distributions of ros2 that may not be compatable with certain OS. 
    - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html
- Isaac Sim and ros handle the data transfer from Isaac and ros running on different versions of Python3?
  - Isaac sim uses Python 3.11
  - Ubuntu 22.04, ros2 humble uses Python 3.10
- Open isaac sim with ros2 bridge enabled in terminal:
  - There will be a terminal for ros2 functionalities and a terminal running Isaac sim. Do not source the ros2 install in the same terminal that is running isaac sim since they use different python versions (remove from ~/.bashrc).
  - ros2 terminal:
  ```
  source /opt/ros/humble/setup.bash
  source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash

  #optional (source arg complete and other projects)
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  source ~/ros2_ws/install/setup.bash
  ```
  - isaac sim terminal:
  ```
  export isaac_sim_package_path=$HOME/isaacsim
  export ROS_DISTRO=humble
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
  ~/isaacsim/isaac-sim.sh
  ```

**Driving TurtleBot using ROS 2 Messages, https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html**
- *ROS2 Context Node*: ROS2 uses DDS for its middleware communication. DDS uses Domain ID to allow for different logical networks operate independently even though they share a physical network. ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot. ROS2 context node creates a context with a given Domain ID. It is set to 0 by default. If Use Domain ID Env Var is checked, it will import the ROS_DOMAIN_ID from the environment in which you launched the current instance of Isaac Sim.
  - only necessary when you want explicit control of the domain ID or environment. 
    - automatically created without the node in the UI. The default is linked to "ROS_DOMAIN_ID" in the terminal Isaac Sim is opened in. 
- *Articulation Controller Node*: This node is assigned to a target robot, then takes in the names or the indices of the joints that needs to be moved, and moves them by the commands given in Position Commands, Velocity Commands, or Effort Commands.
  - this node effectively translates the inputs coming from a ros2 subscriber or client into usable information of isaac sim. Connects ros2 information to an isaac sim primitive (the "target")
  - When Isaac Sim's physics engine simulates a robot, it needs to know which prim is the root of the kinematic chain — the anchor point from which all joints and links are computed. The Articulation Root API is the property you apply to a prim to designate it as that anchor. The Articulation Controller OmniGraph node then needs to be pointed at that same root prim so it knows which robot to send joint commands to.
  - Protery -> Physics -> Articulation Root



**Isaac Sim in under half an hour, Articulated Robotics, https://www.youtube.com/watch?v=SjVqOqEXXrY**
- 
- Isaac sim is a robotics simulator. It can create very realistic camera data. 
  - popular ways of Isaacsim is integrated into a developement workflow:
    1. interaction with ros
    2. reinforcement learning with isaaclab

-  