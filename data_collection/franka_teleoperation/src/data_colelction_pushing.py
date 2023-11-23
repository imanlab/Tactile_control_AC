#!/usr/bin/env python3

import rospy
import cv2
import sys
import numpy as np
import pickle as pl
import message_filters
import tf.transformations
from PIL import Image as pil_im
import time
import datetime
import os
import pandas as pd

import moveit_commander
import moveit_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError
from franka_msgs.msg import FrankaState
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Brings in the SimpleActionClient
import actionlib                                                # top -> fing   arm -> front

import dynamic_reconfigure.client

from controller_manager_msgs.srv import SwitchController

class DataCollection:
    def __init__(self):
        rospy.init_node("data_collector_node")
        self.bridge = CvBridge()

        self.datasave_folder = "/home/gabriele/Dataset/"

        self.init_fing_cam = False
        self.init_front_cam = False 
        self.init_side_cam = False
        self.at_intital_position = False
        self.controller_switched = False
        self.follower_joint_states_arr = False
        self.follower_task_states_arr = False
        self.stop = False

        rate = rospy.Rate(10)   # 30hz
        time_for_trajectory = 2.0
        self.prev_i, self.i = 0, 1

        self.main()

    def franka_subscribers(self):
        fing_cam_sub = message_filters.Subscriber("/fing_camera/color/image_raw", Image)
        front_cam_sub = message_filters.Subscriber("/front_camera/color/image_raw", Image)
        side_cam_sub = message_filters.Subscriber("/side_camera/color/image_raw", Image)
        joint_state_sub = message_filters.Subscriber("/joint_states", JointState)
        # task_space_sub = message_filters.Subscriber("/panda_follower/panda_follower_state_controller/franka_states", FrankaState)

        ts = message_filters.ApproximateTimeSynchronizer([front_cam_sub, side_cam_sub, fing_cam_sub, joint_state_sub], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)

        # self.test_sub = rospy.Subscriber("/fing_camera/color/image_raw", Image, self.test_callback)

    # def test_callback(self, fing_cam):
    #     if self.i != self.prev_i:
    #         self.prev_i = self.i        
    #         self.camera_finger.append(fing_cam)

    def callback(self, front_cam, side_cam, fing_cam, joint_sub):
        if self.i != self.prev_i:
            self.prev_i = self.i
            self.robot_states.append(joint_sub)
            self.camera_finger.append(fing_cam)
            self.camera_front.append(front_cam)
            self.camera_side.append(side_cam)
            # ee_state = self.move_group.get_current_pose().pose            

    def format_data_for_saving(self):
        self.robot_states_formated = []
        

        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index]
            # ee_state = self.robot_states[data_sample_index][1]
            # self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
            #                                    [ee_state.position.x, ee_state.position.y, ee_state.position.z,
            #                                     ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort))
    
    
    def main(self):
        # self.robot_states = []
        # self.camera_side = []
        # self.camera_front = []
        # self.camera_finger = []
        rate = rospy.Rate(10)   # 30hz
        time_for_trajectory = 2.0
        self.prev_i, self.i = 0, 1
        
        for trial in range(100):

            a = input("start_collecting data - enter to continue:")
            if a == "n" or a == "N":
                break
            
            self.robot_states = []
            self.camera_side = []
            self.camera_front = []
            self.camera_finger = []

            self.franka_subscribers()


            print("starting trial")
            t0 = time.time()
            while not rospy.is_shutdown() and time.time() - t0 < time_for_trajectory:
                print(time.time() - t0)
                self.i += 1
                rate.sleep()

            print("robot_states", np.array(self.robot_states))
            print("camera_side", np.array(self.camera_side).shape)
            print("camera_front", np.array(self.camera_front).shape)
            print("camera_finger", np.array(self.camera_finger).shape)

            # save the data:
            folder = str(self.datasave_folder + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
            mydir = os.mkdir(folder)

            self.format_data_for_saving()
            T0 = pd.DataFrame(self.robot_states_formated)

            robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_joint8", "position_panda_joint9",
            "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_joint8", "velocity_panda_joint9",
            "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_joint8", "effort_panda_joint9"]

            T0.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
            np.save(folder + '/camera_side.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.camera_side]))
            np.save(folder + '/camera_front.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.camera_front]))
            np.save(folder + '/camera_finger.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.camera_finger]))


            # print(self.robot_states)
            # print(self.camera_side)
            # print(self.camera_front)
            # print(self.camera_finger)

data = DataCollection()