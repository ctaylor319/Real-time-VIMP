
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from motion_planning_msgs.msg import VisualizedPath

import time
import math
import random
    
class VisualizeResults(Node):
    def __init__(self):
        super().__init__('visualize_results')

        qosProfile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            durability = QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        self.visPathSubscriber = self.create_subscription(
            VisualizedPath,
            '/vis_path',
            self.visualizeCallback,
            qosProfile
        )
        self.visPathSubscriber

        # Kinematic variables
        self.a = np.array([0, -.1104, -.096, 0, 0, 0])
        self.alpha = np.array([np.pi/2, 0, 0, np.pi/2, np.pi/2, -np.pi/2])
        self.d = np.array([.13156, 0, 0, .06062, .07318, -.0456])
        self.theta_bias = np.array([np.pi/2, -np.pi/2, 0, -np.pi/2, np.pi/2, 0])
        self.test_theta = np.array([0,0,0,0,0,0])

    def visualizeCallback(self, msg):
        iter_path = np.array(msg.waypoints).reshape((msg.num_iter, \
                                                                msg.num_waypoints_per_path, msg.waypoint_size))
        iter_path = np.delete(iter_path, np.arange(msg.waypoint_size/2, msg.waypoint_size, dtype=int), 2) # Remove velocities

        self.displayPathEvolution(iter_path)
        self.displayFinalPath(iter_path, msg.env_render)

    def displayPathEvolution(self, path):
        ax = plt.figure().add_subplot(projection='3d')
        for i in range(path.shape[0]):
            x = np.array([]); y = np.array([]); z = np.array([])
            for j in range(path.shape[1]):
                T = self.forwardKinematics(path[i, j, :])
                x = np.append(x, T[0, 3])
                y = np.append(y, T[1, 3])
                z = np.append(z, T[2, 3])
            alpha = (1 + 4*(i/path.shape[0]))/5
            ax.plot3D(x, y, z, color='b', alpha=alpha)
        plt.show()
    
    def displayFinalPath(self, path, env):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        path = path[-1, :, :]
        for i in range(path.shape[0]):
            x_arm = np.array([]); y_arm = np.array([]); z_arm = np.array([])
            for j in range(0, path.shape[1]+1):
                T = self.forwardKinematics(path[i, :j])
                x_arm = np.append(x_arm, T[0, 3])
                y_arm = np.append(y_arm, T[1, 3])
                z_arm = np.append(z_arm, T[2, 3])
            alpha = (1 + 4*(i/path.shape[0]))/5
            ax.plot(x_arm, y_arm, z_arm, color='b', alpha=alpha)
        x_env, y_env, z_env = zip(*[(point.x, point.y, point.z) for point in env])

        xlims = [-0.6, 0.6]; ylims = [0, 0.6]; zlims = [-0.1, 0.7]
        x_env = np.array(x_env); y_env = np.array(y_env); z_env = np.array(z_env)
        box_bounds = (x_env>=xlims[0]) & (x_env<=xlims[1]) & (y_env>=ylims[0]) & (y_env<=ylims[1]) & (z_env>=zlims[0]) & (z_env<=zlims[1])
        x_env = x_env[box_bounds]
        y_env = y_env[box_bounds]
        z_env = z_env[box_bounds]

        ax.scatter(x_env, y_env, z_env, color='k', alpha=0.5, antialiased=True)
        ax.set_box_aspect([2.0, 1.0, 1.0])
        ax.set_xlim3d(-0.5,0.5)
        ax.set_ylim3d(0,0.5)
        ax.set_zlim3d(0,0.5)
        plt.show()   

    def forwardKinematics(self, joint_vals):
        T = np.eye(4)
        for i in range(joint_vals.shape[0]):
            T = T@self.T_i(i, joint_vals[i]+self.theta_bias[i])
        return T

    def T_i(self, i, theta):
        T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(self.alpha[i]),  np.sin(theta)*np.sin(self.alpha[i]), self.a[i]*np.cos(theta)], \
                      [np.sin(theta),  np.cos(theta)*np.cos(self.alpha[i]), -np.cos(theta)*np.sin(self.alpha[i]), self.a[i]*np.sin(theta)], \
                      [            0,                np.sin(self.alpha[i]),                np.cos(self.alpha[i]),               self.d[i]], \
                      [            0,                                    0,                                    0,                       1]])
        return T


def main():
    rclpy.init()
    vis_results = VisualizeResults()

    while rclpy.ok():
        rclpy.spin_once(vis_results)

    goalPublish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
