# Bartending Cobot
A 6dof robot manipulator + end-effector (e.g. 7dof) controlled using ROS2 Jazzy on Ubuntu 24.04. The robot will be simulated using Gazebo Harmonic. 

## Table of Contents
- [Bartending Cobot](#bartending-cobot)
  - [Table of Contents](#table-of-contents)
  - [Project Description](#project-description)
  - [Install](#install)
    - [Dependencies](#dependencies)
    - [Build](#build)
  - [Usage](#usage)

## Project Description

A handoff to the cobot will be initiated via button and hardcoded location placement or intent inference with HRC techniques, etc. Then the handoff bartending cobot will take the cocktail shaker and the arm will make a motion that will shake the contents within the cocktail shaker. Finally, the bartending cobot will place the cocktail shaker back in a predetermined location or using kind of advanced algorithm to negotiate handoff back to the human.

## Install

### Dependencies
The following pre-requisites are necessary to run program for the bartending cobot:
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
  - Ensure `colcon` is installed and set up.
  - Ensure rviz2 is installed: `sudo apt install ros-jazzy-rviz2`
  - Install xacro if not already installed: `sudo apt install ros-jazzy-xacro`
  - Ensure rosdep is install `sudo apt-get install python3-rosdep`
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/)
  - Ensure Gazebo is properly installed and configured to work with ROS 2 Jazzy. [Dependency install guide](https://gazebosim.org/docs/harmonic/ros_installation/): `sudo apt-get install ros-${ROS_DISTRO}-ros-gz`
- [ros2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
  - Ensure ros2_control-gazebo communication is set up. [ros2_control installation guide](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

### Build

The repo can be build with the following cmds assuming the dependencies [above](#dependencies) are met. 

```bash
cd ./bartending_cobot
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink
```

## Usage

Make sure the dependencies are installed and ensure that the **hardcoded path in the [URDF](src/bartending_cobot_description/urdf/bartending_cobot.xacro.urdf) for the gazebo plugin tag is the correct path on your machine (this bug will be fixed shorted)** then run the following:

Terminal 1
```bash
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
ros2 launch bartending_cobot_bringup r6bot.launch.py
```

Terminal 2
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```