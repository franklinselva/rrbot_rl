#!/usr/bin/env python3

import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rospy
from std_msgs.msg import Float32


class Plotter():
    def __init__(self):
        rospy.init_node("result_plotter")
        rospy.Rate(50)

        self.reward = []
        self.ep = []
        self.count = 0

        self.load_data = rospy.get_param("load_data", False)

        self.reward_sub = rospy.Subscriber(
            "result", Float32, self.sub_callback)

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(1, 1, 1)

        if self.load_data:
            self.ep, self.data = self.load_data()
            self.size_ep = len(self.ep)
        self.plot()

    def sub_callback(self, data):
        self.reward.append(data.data)
        self.count += 1
        self.ep.append(self.count)

    def load_data(self):
        try:
            with open("graph.txt") as f:
                x, y = pickle.load(f)
        except:
            x, y = [], []
        return x, y

    def save_data(self, data):
        with open("graph.txt", "wb") as f:
            pickle.dump(data, f)

    def animate(self, i):
        self.ax.clear()
        print(self.reward)
        print(self.count)
        self.ax.plot(self.ep, self.reward)

    def plot(self):
        plt.title("Reward vs Episodes - SAC Agent")
        plt.xlabel("Episodes")
        plt.ylabel("Rewards")

        while not rospy.is_shutdown():
            # Start animation
            self.ani = animation.FuncAnimation(
                self.figure, self.animate, interval=1000)

            plt.show()

        if rospy.is_shutdown():
            self.save_data([self.ep, self.reward])


if __name__ == "__main__":
    plot = Plotter()
    plot.plot()
