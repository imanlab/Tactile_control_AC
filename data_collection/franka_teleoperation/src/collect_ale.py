#!/usr/bin/env python3
# Updated: ALESSANDRO COSTA 12/23

import tf
import os
import sys
import copy
import time
import rospy
import termios
import datetime
import actionlib
import numpy as np
import pandas as pd
import message_filters
import moveit_msgs.msg
import moveit_commander
import PIL.Image as PILImage        #pyhton image editing tool, we use this here to resize the image

from cv_bridge import CvBridge
from pynput.keyboard import Key, Listener
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Image
# from franka_gripper.msg import MoveAction, HomingAction, MoveActionGoal, GraspAction, GraspActionGoal


class RobotReader(object):
    def __init__(self, save_path):
        super(RobotReader, self).__init__()
        rospy.init_node('data_collection_client', anonymous=True, disable_signals=False)
        self.settings = termios.tcgetattr(sys.stdin)

        self.robot_states = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_end_effector_link("panda_hand")
        self.listenertf = tf.TransformListener()
        self.setup_planner()
        self.listener = tf.TransformListener()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.JOINT_BASE = 0
        self.JOINT_WRIST = 6
        self.ee_to_finger = 0.13
        self.replan = False
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.save_path = save_path
        self.bridge = CvBridge()
        self.rate_hz = 30
        rate_ros = rospy.Rate(self.rate_hz)

            # Data collection Pose (0.529, -0.034, 0,197)
        print(self.current_pose())
        print(self.current_state())

        while input("press enter to start saving data, or type ctrl c then n to not: ") != "n":    
            self.stop= False
            self.robot_states   = []
            self.haptic_finger_data  = []

            self.listener = Listener(on_press=self.start_collection)
            
            self.listener.start()
            print(self.stop)
            
            self.robot_sub = message_filters.Subscriber('/joint_states', JointState)
            self.haptic_finger_sub = message_filters.Subscriber('/fing_camera/color/image_raw', Image)
            
            subscribers = [self.robot_sub, self.haptic_finger_sub]

            self.start_time = datetime.datetime.now()
            print(self.start_time)

            self.prev_i = 0
            self.i = 1
            self.index__ = 0
            self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=50, slop=0.5, allow_headerless=True)
            self.ts.registerCallback(self.read_robot_data)
            
            while not rospy.is_shutdown() and self.stop is False:
                if self.i == 1:
                    t0 = time.time()
                    self.go_home()
                    self.go_to_pose(0.529, -0.014, 0.187)
                    self.move_down(0.008)
                    self.go_to_pose(0.529, -0.014, 0.187)
                self.i += 1
                rate_ros.sleep()
            t1 = time.time()
            self.stop = False
            self.stop_time = datetime.datetime.now()
            print(self.stop_time)

            self.rate = (len(self.robot_states)) / (t1-t0)

            print("\n Stopped the data collection \n now saving the stored data")
            self.listener.stop()
            self.save_data()

    def setup_planner(self):
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_end_effector_link("panda_hand")
        self.group.set_max_velocity_scaling_factor(0.005)
        self.group.set_max_acceleration_scaling_factor(0.005)
        self.group.allow_replanning(True)
        self.group.set_num_planning_attempts(10)
        self.group.set_goal_position_tolerance(0.0005)
        self.group.set_goal_orientation_tolerance(0.001)
        self.group.set_planning_time(5)
        self.group.set_planner_id("RRTConnectkConfigDefault")
        rospy.sleep(2)
        print("Ready to go")

    def read_robot_data(self, robot_joint_data, haptic_finger_data_cb):
        if self.stop == False:
            self.prev_i = self.i
            self.index__ +=1
            ee_state = self.group.get_current_pose().pose
            self.robot_states.append([robot_joint_data, ee_state])
            # print(ee_state)
            
            image_message = haptic_finger_data_cb
            haptic_finger_img = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')      #haptic_finger_img is a cv2 image
            haptic_finger_img = PILImage.fromarray(haptic_finger_img).resize((256, 256), PILImage.ANTIALIAS)     #RESIZE THE VECTOR
            haptic_finger_img_new = np.array(haptic_finger_img)                                                     #RESIZED
            self.haptic_finger_data.append(haptic_finger_img_new)               #add the previous arry to the haptic_finger_data (which I think is initialiez to [] at the beginning)
            # print(self.index__)

    def current_pose(self):
        wpose = self.group.get_current_pose().pose          #panda moveit function
        return wpose
    
    def current_state(self):
        joint_state = self.group.get_current_joint_values()
        return joint_state
    
    def go_home(self):
        p = PoseStamped()
        p.header.frame_id = '/panda_link0'

        p.pose.position.x = 0.43
        p.pose.position.y = 0.00
        p.pose.position.z = 0.43

        p.pose.orientation.x = 1
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0

        target = self.group.set_pose_target(p)

        self.group.go(target)

    def go_to_pose(self, x, y, z):
        p = PoseStamped()
        p.header.frame_id = '/panda_link0'

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z

        p.pose.orientation.x = 1
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0

        target = self.group.set_pose_target(p)

        self.group.go(target)

    def move_down(self, Z):
        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.z -= Z
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group.execute(plan, wait=True)
    
    def move_up(self):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        wpose.position.z -= -0.03
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group.execute(plan, wait=True)

    def move_y(self):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        wpose.position.y -= 0.03
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)  #
        self.group.execute(plan, wait=True)

    def start_collection(self, key):
        print("here")
        if key == Key.esc:
            self.stop = True
            self.listener.stop()
            self.robot_sub.unregister()
            self.haptic_finger_sub.unregister()

    def format_data_for_saving(self):
        self.robot_states_formated = []

        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                                [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                                 ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])

        print("Formating data DONE")

    def save_data(self):

        print("saving data")
        self.format_data_for_saving()

        print("robot_states_formated; ", np.asarray(self.robot_states_formated).shape)
        print("rate: ", self.rate)

        T1 = pd.DataFrame(self.robot_states_formated)
       

        self.haptic_finger_data = np.array(self.haptic_finger_data)

        folder = self.save_path+str('/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]


        T1.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
        np.save(folder + '/haptic_finger.npy', self.haptic_finger_data)

        # save = input("save meta file? n to not")
        # if save != "n":
        #     meta_data = ['prob_type', 'slow/fast', 'notes/comments']
        #     meta_data_ans = []
        #     for info in meta_data:
        #         value = input(str("please enter the " + info))
        #         meta_data_ans.append(value)
        #     meta_data.extend(('frequency_hz', 'start_time', 'stop_time'))
        #     meta_data_ans.extend((str(self.rate), str(self.start_time), str(self.stop_time)))
        #     meta_data_ans = np.array([meta_data_ans])
        #     T5 = pd.DataFrame(meta_data_ans)
        #     T5.to_csv(folder + '/meta_data.csv', header=meta_data, index=False)

        # print("saving data DONE")
    
def main():
    save_path = "/home/alessandro/Dataset/preliminary_tries"
    
    robot = RobotReader(save_path=save_path)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        main()
    except Exception as e:
        print(e)
    else:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)