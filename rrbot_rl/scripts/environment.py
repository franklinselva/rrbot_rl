#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
from random import uniform

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from respawnGoal import Respawn

q1Value, q2Value = Float64(), Float64()


def q1jointCallback(data):
    """Callback function for subscribing to the joint data of the robot

    Args:
        data (std_msgs/Float64): Incoming message from ROS server

    Returns:
        joint data for Robot joint 1 i.e. q1
    """
    q1Value = data.data


def q2jointCallback(data):
    """Callback function for subscribing to the joint data of the robot

    Args:
        data (std_msgs/Float64): Incoming message from ROS server

    Returns:
        joint data for Robot joint 1 i.e. q1
    """
    q2Value = data.data


class Env():
    def __init__(self, action_dim=2, max_Q1_velocity=1., max_Q2_velocity=1.):
        # rospy.init_node("env_handler")

        # Subscribers
        self.q1_sub = rospy.Subscriber(
            '/rrbot/joint1_position_controller/command', Float64, q1jointCallback)
        self.q2_sub = rospy.Subscriber(
            '/rrbot/joint2_position_controller/command', Float64, q2jointCallback)

        # Publishers
        self.q1_pub = rospy.Publisher(
            '/rrbot/joint1_position_controller/command', Float64, queue_size=10)
        self.q2_pub = rospy.Publisher(
            '/rrbot/joint2_position_controller/command', Float64, queue_size=10)

        # Gazebo services
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy(
            'gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.respawner = Respawn()
        self.action_dim = action_dim

        self.EE_position = Pose()
        self.threshold = 0.1
        self.prev_goal_distance = 0.
        self.setHome = [1.0, 2.4]
        self.success = False
        self.box_thrown = False
        # Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """Performs envionrment shutdown on Keyboard Interrupt or ROS shutdown
        """

        rospy.loginfo("Stopping Robot at home position")
        self.respawner.robot_set_start()
        rospy.sleep(1)

    def getGoalDistace(self):
        """Calculates the euclidean distance between the cardboard box's current position and goal position

        Returns:
            goal_distance: Returns the goal distance in float
        """
        x, y, z = self.respawner.getPosition()
        goal_distance = round(
            math.sqrt((self.goal_x - x) ** 2 + (self.goal_y - y) ** 2 + (self.goal_z - z) ** 2))

        self.past_distance = goal_distance

        return goal_distance

    def getState(self):
        """Gets the current state of the cardboard box and robot joint values in the current iteration.

        Returns:
            state: Numpy array of the current state
        """

        response = self.respawner.getModelState()

        state = [response.position.x, response.position.y,
                 response.position.z, q1Value.data, q2Value.data]

        return np.asarray(state, dtype=np.float32)

    def setReward(self, state, done):
        """Generates the reward function for the current state

        Args:
            state (np.array): The current state from the action performed
            done (bool): Whether the action is performed or not

        Returns:
            reward (float): The generated reward value for the action performed
        """

        reward = 0.
        cube = Pose()
        goal_distance = self.getGoalDistace()
        cube = self.respawner.getModelState()

        if goal_distance >= self.threshold and goal_distance > self.prev_goal_distance:
            reward -= 2
            self.success = False
            self.done = False

        if goal_distance < self.prev_goal_distance:
            reward += 5
            self.success = False
            self.done = False

        if goal_distance <= self.threshold:
            reward += 10
            self.success = True
            self.done = True

        if cube.position.z <= 0.5:
            reward -= 50
            self.success = False
            self.box_thrown = True
        else:
            self.box_thrown = False

        return reward

    def step(self, action):
        """Performs the environment step function for the algorithm

        Args:
            action (np.array): The joint values for the robot configurations

        Returns:
            state (np.array): Present state after the action is performed
            reward (float): Current reward value for the action performed
            done (bool): Confirms whether the step is performed or not
        """

        self.update_robot(action)

        # Check whether the model is thrown
        cube = self.respawner.getModelState()

        if cube.position.z <= 0.5:
            self.box_thrown = True
            self.success = False

        state = self.getState()
        reward = self.setReward(state, self.done)

        return state, reward, self.done

    def update_robot(self, action):
        """Update the robot position using the ROS controller. Publihses the joint values.

        Args:
            action (np.array): The joint values for the RRBOT
        """

        self.q1_pub.publish(action[0])
        self.q2_pub.publish(action[1])

        self.done = True

    def reset(self):
        """Performs the simulation reset of the gazebo environemnt

        Returns:
            state(np.array): Returns the current state when performed reset
        """
        self.respawner.softRespawnModel()
        time.sleep(1)

        self.goal_x, self.goal_y, self.goal_z = self.get_goal_position()

        self.goal_distance = self.getGoalDistace()
        state = self.getState()

        return state

    def get_goal_position(self):
        """Generates the goal position for the cardboard box. Only the x coordinate should be varied the reset should stay constrained

        Returns:
            goal_i (float): goal coordinate of i coordinate system (3 Dimensional in this case)
        """

        init_pose = self.respawner.init_pose
        goal_x = uniform(init_pose.position.x - 0.5,
                         init_pose.position.x + 0.5)
        goal_y = init_pose.position.y
        goal_z = init_pose.position.z

        return goal_x, goal_y, goal_z


if __name__ == "__main__":
    env = Env()
