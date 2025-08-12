# UR_Robotiq_ROS2
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS 2 package for Universal Robot e-Series mounted with Robotiq Adaptive Grippers

## Features
- [ur_robotiq_description](https://github.com/tonydle/UR_Robotiq_ROS2/tree/main/ur_robotiq_description): Combined URDF into a single robot description
- [ur_robotiq_control](https://github.com/tonydle/UR_Robotiq_ROS2/tree/main/ur_robotiq_control): Combined launch file and ROS 2 controllers
- [ur_robotiq_moveit_config](https://github.com/tonydle/UR_Robotiq_ROS2/tree/main/ur_robotiq_moveit_config): MoveIt! configuration package for the combined robot and controllers

## Dependencies (included in the installation steps below)
- [Universal Robot ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) - humble branch
- [Robotiq ROS2 Driver](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/humble) - humble branch

## Installation

1. Navigate to your ROS2 workspace and **clone the repository** into the `src` directory:
   ```sh
   git clone https://github.com/tonydle/UR_Robotiq_ROS2.git src/ur_robotiq
   ```
2. Install git dependencies using `vcs`:
   ```sh
   vcs import src --input src/ur_robotiq/required.repos --recursive
   ```
3. Let rosdep install ROS 2 dependencies:
   ```sh
   rosdep install -y --from-paths src --ignore-src
   ```
4. Build using colcon with symlink install:
   ```sh
   colcon build --symlink-install
   ```
5. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## Hardware Setup
Follow the [Installation Guide](https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_Instruction_Manual_Gen_HTML_20190524.zip/2F-85_2F-140_Instruction_Manual_Gen_HTML/Content/3.%20Installation.htm) to connect the Robotiq gripper to your PC using the RS485 to USB converter.

More details TBA.

## Usage
### View the URDF
   ```sh
   ros2 launch ur_robotiq_description view_robot.launch.py ur_type:=ur10e
   ```

### Start robot
   ```sh
   ros2 launch ur_robotiq_control start_robot.launch.py ur_type:=ur10e robot_ip:=<robot_ip> com_port:=<robotiq_com_port>
   ```
Other arguments:
- `initial_joint_controller` (default: `scaled_joint_trajectory_controller`): Change to `forward_position_controller` to use with MoveIt Servo
- `use_fake_hardware` (default: `false`): Use mock hardware interface for testing
- `launch_rviz` (default: `true`): Launch RViz with the robot model
- `tf_prefix` (default: `""`): Prefix for all TF frames

### Start MoveIt!
   ```sh
   ros2 launch ur_robotiq_moveit_config moveit.launch.py ur_type:=ur10e
   ```

### Get the full joint states
   ```sh
   ros2 topic echo /joint_states
   ```
### Control the gripper with `gripper_knuckle_joint_position_controller`(JointGroupPositionController)
   ```sh
   ros2 topic pub --once /gripper_knuckle_joint_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.4]}"
   ```
   Data range is from 0.0 (open) to 0.8 (closed) in radians.

## Author
[Tony Le](https://github.com/tonydle)

## License
This software is released under the MIT License, see [LICENSE](./LICENSE).