#!/usr/bin/env python3

import pickle
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rospy
from std_msgs.msg import Float32


class Plotter():
    def __init__(self):
        rospy.init_node("result_plotter")
        rospy.Rate(50)

        self.reward = []
        self.time = []
        self.ep = 0

        self.load_data = rospy.get_param("load_data", False)

        reward_sub = rospy.Subscriber("result", Float32, self.sub_callback)

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(1, 1, 1)

        if self.load_data:
            self.ep, self.data = self.load_data()
            self.size_ep = len(self.ep)
        self.plot()

    def sub_callback(self, data):
        self.reward.append(data.data)
        self.ep += 1

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

    def animate(self, i, time, reward):
        self.time.append(dt.datetime.now().strftime('%H:%M:%S.%f'))

        self.ax.clear()
        self.ax.plot(time, reward)

    def plot(self):
        plt.title("Reward vs Time - SAC Agent")
        plt.xlabel("Time")
        plt.ylabel("Rewards per Episode")

        while not rospy.is_shutdown():
            # Start animation
            self.ani = animation.FuncAnimation(
                self.figure, self.animate, fargs=(self.time, self.reward), interval=1000)

            plt.show()

        if rospy.is_shutdown():
            self.save_data([self.ep, self.data])


if __name__ == "__main__":
    plot = Plotter()
    plot.plot()
