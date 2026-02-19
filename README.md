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
  - [Dev Notes](#dev-notes)
    - [Dev Dependencies](#dev-dependencies)
    - [Github Actions](#github-actions)
    - [How to use Git for this project:](#how-to-use-git-for-this-project)

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



## Dev Notes

### Dev Dependencies
Install all dependencies under the [dependencies section](#dependencies). 

### Github Actions
This repo implements various github actions to ensure a smooth CI/CD process and enable faster prototyping. The actions implemented are described below:
- [pylint.yaml](.github/workflows/pylint.yml) workflow: This action is designed to ensure that code is the highest quality. It will find all the python files in the repo and then produce a lint.txt as an artifact which can be downloaded from Github.
- [colcon_build.yaml](.github/workflows/colcon_build.yml) workflow: This action will create a ros2_jazzy dev env, install any dependencies, and then build the workspace. This action ensures that the committed code can be build on another machine. Currently, the tests are skipped since the codebase is fairly new. This will be removed once the codebase develops. More details available [here](https://github.com/ros-tooling/action-ros-ci).

### How to use Git for this project:
The project uses Git for version control. In order to ensure a clean, organized commit history, please follow these guidelines:
- Make sure `pre-commit` command is installed.
    - VS Code may return an error when using the IDE for committing changes. Try using the CLI `git commit -m "<message>"` if these errors cause issues.
- Create a new branch for each feature or bug fix you work on. Use descriptive names for your branches.
- Write clear and concise commit messages that describe the changes you made.
- Before pushing your changes, ensure that your code is properly formatted and passes all tests.
- Regularly pull changes from the main branch to keep your branch up to date.
- Ensure that minor changes are squashed into larger commits to keep the commit history clean.
    - Use interactive rebase (`git rebase -i`) to squash commits before merging ([guide](https://thoughtbot.com/blog/git-interactive-rebase-squash-amend-rewriting-history)).
