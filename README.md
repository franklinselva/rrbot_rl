# RRBOT RL

This repository contains the implementation for integrating Reinforcement learning with 2R robot arm. The application is moving the cuboid using the robot arm to the desired position. The implementation is tested in ROS Noetic and the algorithm used for training the robot is Soft Actor Critic (SAC).

## Installation

First, start by cloning the repository into a ROS workspace. For example `robot_ws/src`

```
git clone <repo id>
```

Then build the package when you are in workspace `robot_ws/`

```
catkin_make
```

## Usage

To start the environment setup for the reinforcement learning,

```
roslaunch rrbot_rl env_setup.launch
```

To test the ros controller, run in a new terminal

```
rosrun rrbot_rl move_box
```

To test the reset mechanism, in ` cd src/rrbot_rl/rrbot_rl/scripts` &&

```
python3 respawnGoal.py
```

## Todo

- Build the environment mode
- Implement SAC algorithm
