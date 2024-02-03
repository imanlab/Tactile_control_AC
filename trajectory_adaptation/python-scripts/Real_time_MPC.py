##! /usr/bin/env python3

import math
import random
import rospy
import time
import pandas as pd
import numpy as np
import message_filters
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Point
import matplotlib.pyplot as plt
import scipy

#Optimizer
from scipy.optimize import Bounds
from scipy.spatial import distance
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint



## Pytorch
import torch
import torch.onnx
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

#models
#from ACTP.ACTP_model import ACTP


class RobotController():
    def __init__(self):
        self.stop = 0
        self.time_step = 0
        self.prev_time_step = 0
        self.tau = 0
        self.E = 0
        self.x0 = 0.6436084411618019  #1500
        self.y0 = -0.0510171173408605 #500
        self.z0 = 0.8173126349141415 #unknown
        self.x_f = 0.5496180752549058#1400
        self.y_f = -0.21016977052503594#300
        self.z_f = 0.8173126349141415 #same as initial
        self.num_int_points = 10
        self.trajectory_history = []
        self.cost_history = []
        
        self.target_position = np.array([self.x_f, self.y_f, self.z_f])
        #self.d = 2
        #self.d = np.linalg.norm(self.target_position - self.initial_position) / (self.num_int_points+1)
        self.points = []
        self.ee_coordinates = []
        self.optimal_trajectory_history = []
        self.init_sub()
        self.control_loop()
        #self.plot_trajectory()

        
    def init_sub(self):
        rospy.init_node('optimizer', anonymous=True, disable_signals=True)
        self.optimal_traj_pub = rospy.Publisher('/opt_traj', Point, queue_size=11)

    def callback(self, ee_state):
        ee_state = self.move_group.get_current_pose().pose
        self.ee_coordinates.append(ee_state)

    def cost_callback(self, theta_values):    #usefull for tracking the cost value during the optimization
        points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(points)
        self.trajectory_history.append(points.copy())
        self.cost_history.append(cost)
    
    def gen_opt_traj(self):
        initial_theta = np.zeros(self.num_int_points + 1)
        bounds = None
        result = minimize(self.obj_func, initial_theta, bounds=bounds, callback=self.cost_callback, method='BFGS')
        optimal_theta = result.x
        return optimal_theta

    def obj_func(self, theta_values):
        points = self.circular_to_cartesian(theta_values)
        cost = self.calculate_cost(points)
        return cost
       
    def calculate_cost(self, points):
        return np.sum(np.linalg.norm(points - self.target_position, axis=1)**2)

    def circular_to_cartesian(self,theta_values): 
        x = np.zeros(self.num_int_points+2)
        y = np.zeros(self.num_int_points+2)
        z = np.zeros(self.num_int_points+2)
        for i in range(self.num_int_points+2):
            if i == 0:
                x[i] = self.initial_position[0]
                y[i] = self.initial_position[1]
                z[i] = self.initial_position[2]
            
            else:
                x[i] = x[i-1] + self.d * np.cos(theta_values[i-1])
                y[i] = y[i-1] + self.d * np.sin(theta_values[i-1])
                z[i] = z[i-1]
           
        return np.column_stack((x, y, z))

    def pub_goal_pose(self):
        goal_pose = Point()
        goal_pose.x = self.optimal_trajectory[1,0]
        goal_pose.y = self.optimal_trajectory[1,1]
        goal_pose.z = self.optimal_trajectory[1,2] 
        self.optimal_traj_pub.publish(goal_pose)

    def control_loop(self):

        start_position = np.array([self.x0, self.y0, self.z0])
        self.d = np.linalg.norm(self.target_position - start_position) / (10*(self.num_int_points+1))
        for k in range(1000000):
            if k == 0:
                self.initial_position = start_position    
            self.opt_theta = self.gen_opt_traj()
            self.optimal_trajectory = self.circular_to_cartesian(self.opt_theta)
            self.initial_position = self.optimal_trajectory[1]
            print(self.initial_position)
            self.pub_goal_pose()

        

    def plot_trajectory(self):
        # Plot the trajectory at each step
        #for i, trajectory in enumerate(self.trajectory_history):
        #   plt.plot(trajectory[:, 0], trajectory[:, 1], label=f'Step {i}')

        # Plot the final optimized trajectory
        plt.scatter(self.optimized_trajectory[:, 0], self.optimized_trajectory[:, 1], label='Optimized', color='red', marker='x')

        # Plot the target position
        plt.scatter(self.target_position[0], self.target_position[1], label='Target', color='green', marker='o')
        plt.scatter(self.initial_position[0], self.initial_position[1], label='Start', color='pink', marker='o')
    
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Optimization Steps for Trajectory with Fixed Distance')
        plt.show()

if __name__ == '__main__':
    io = RobotController()

    