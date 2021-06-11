# RRBOT RL

This repository contains the implementation for integrating Reinforcement learning with 2R robot arm manipulator. The application is moving the cuboid using the robot arm to the desired position. The implementation is tested in ROS Noetic and the algorithm used for training the robot is Soft Actor Critic (SAC).

## Installation

 - First, start by cloning the repository into a ROS workspace. For example `robot_ws/src`

```
git clone <repo id>
```

 - Then build the package when you are in workspace `robot_ws/`

```
catkin_make
```

 - To install the dependencies, run

```
rosdep install --from-paths src --ignore-src -r -y
```

 - Create an `virualenv env -p=3.8` or `virtualenv env -p=2.7` based on the ROS system.

 - To install the required packages for SAC agent, move to the repository folder and run,

```
pip install -r requirements.txt
```

## Usage

 - To start using the package, check whether the workspace is sourced bu,

```
source <workspace>/devel/setup.bash
```

 - To start the environment setup for the reinforcement learning,

```
roslaunch rrbot_rl env_setup.launch
```

 - To start the training process, in directory (from workspace) `src/rrbot_rl/rrbot_rl/scripts`

```
python sac
```

## Test

To test the ros controller, when the workspace is sourced, run in a new terminal

```
rosrun rrbot_rl move_box
```
