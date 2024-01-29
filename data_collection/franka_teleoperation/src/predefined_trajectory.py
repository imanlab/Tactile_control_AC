#!/usr/bin/env python3

import os
import sys
import cv2
import time
import math
import rospy
import random
import datetime
import message_filters
import moveit_msgs.msg
import moveit_commander
import matplotlib.pyplot

import numpy as np
import pandas as pd

from matplotlib.pyplot import imsave
from math import pi
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool, Int16MultiArray
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, MotionPlanRequest, Constraints, PositionConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list


class Trajectory():
    def __init__(self):
        super(Trajectory, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('FrankaRobotWorkshop', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.move_group.set_end_effector_link("panda_link8")
        self.group_names = self.robot.get_group_names()
        self.bridge = CvBridge()
        self.move_group.set_planning_pipeline_id("pilz_industrial_motion_planner") 
        # self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
        print(self.move_group.get_interface_description().name)    # Print the planner being used.

        # scaling down velocity
        
        self.move_group.set_max_velocity_scaling_factor(0.5)      
        self.move_group.set_max_acceleration_scaling_factor(0.5)

        self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        self.joint_home = [0.18984723979820606, -0.977749801850428, -0.22761550468348588, -2.526835711730154, -0.20211957115956533, 3.1466847225824988, 0.7832720796780586]
        # self.joint_via_point = [0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144]
        self.joint_via_point = [-0.2827285690477829, -0.10776317482340839, 0.25531004340188546, -1.720866153466074, 0.49796132276751903, 3.2227085454915225, 0.20333962609574804]
        # self.joint_push = [0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144]
        # self.joint_push = [0.04308905999495077, 0.1585832643771773, -0.14940384170854504, -1.3571466245316623, 0.02975108286944235, 3.0149748432071917, 0.7491484249470901]
        self.joint_push = [0.04446678416070445, 0.1751057482563725, -0.16586388862760443, -1.3552765184408364, 0.0260995250113227, 3.0163658870067884, 0.7461208454668522]
        
        
        self.resets           = 3
        self.pushes_per_reset = 3


        self.robot_sub       = message_filters.Subscriber('/joint_states', JointState)
        self.centroid_sub    = message_filters.Subscriber('/centroid', Point)
        self.ts              = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.centroid_sub] , queue_size=1, slop=0.1, allow_headerless=True)


    def pushing_actions(self):
        start_position, start_ori = self.get_robot_task_state()
            
        total_pushes = 0
        failure_cases = 0
        for j in range(self.resets):
            for i in range(self.pushes_per_reset):
                print("  ")
                print("resets: {},   push: {},  total_pushes: {},   failure cases: {}".format(j, i, total_pushes, failure_cases))
                # self.go_via_point()
                if(j==0 and i==0):
                    self.go_home()
                self.go_push()

                pilz_pose = MotionPlanRequest()
                pilz_pose.planner_id = "CIRC"
                pilz_pose.group_name = "panda_arm"
                pilz_pose.max_velocity_scaling_factor =  0.2
                print("max vel:",pilz_pose.max_velocity_scaling_factor )
                pilz_pose.max_acceleration_scaling_factor =  0.02
                pilz_pose.start_state.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
                pilz_pose.start_state.joint_state.position = self.move_group.get_current_joint_values()
                # pilz_pose.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # pilz_pose.start_state.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                pilz_pose.start_state.joint_state.header.stamp = rospy.Time.now()
        
                pose = self.move_group.get_current_pose()
                print("----------------------  ")
                print("x: {},   y: {},  z: {}".format(round(pose.pose.position.x, 3),round(pose.pose.position.y, 3),round(pose.pose.position.z, 3)))
                print(" --------------------- ")

                pose.pose.position.z += 0.0  #0.05  
                pose.pose.position.y -= 0.2  #0.12  
                pose.pose.position.x -= 0.0  #0.05
                pose.pose.orientation.x = 0.6847884219250332
                pose.pose.orientation.y = -0.018653069577975762
                pose.pose.orientation.z = 0.7265456014775064     
                pose.pose.orientation.w = 0.05337011491865343 
                print("----------------------  ")
                print("x: {},   y: {},  z: {}".format(round(pose.pose.position.x, 3),round(pose.pose.position.y, 3),round(pose.pose.position.z, 3)))
                print(" --------------------- ")
                constraint = Constraints()
                position_constraints_pose = PositionConstraint()

                position_constraints_pose.header = pose.header

                position_constraints_pose.constraint_region.primitives = SolidPrimitive()
                position_constraints_pose.constraint_region.primitives.type = 1
                position_constraints_pose.constraint_region.primitives.dimensions = [0.1, 0.1, 0.1]
                position_constraints_pose.constraint_region.primitive_poses = Pose()
                position_constraints_pose.constraint_region.primitive_poses = pose
                constraint.position_constraints = [position_constraints_pose]
                pilz_pose.goal_constraints = constraint

                target = self.move_group.set_pose_target(pose)
                trajectory = self.move_group.plan(target)
                if trajectory[0] == False:
                    print("False")
                    self.go_home()
                    failure_cases += 1
                    break
                #joint_values = self.move_group.get_current_joint_values()
                time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))
                self.move_group.go(target, wait=False)
                self.data_saver(time_for_trajectory)
                #self.move_group.stop()
                #self.move_group.clear_pose_targets()

            total_pushes += 1
            
            self.go_via_point()
            
        self.go_push()

    def data_saver(self, time_for_trajectory):
        rate                = rospy.Rate(30)
        self.robot_states   = []
        self.camera_finger  = []
        self.prev_i, self.i = 0, 1

        self.ts.registerCallback(self.read_robot_data)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < time_for_trajectory:
            print(time_for_trajectory, "    ----     ", time.time() - t0, end="\r")
            self.i += 1
            rate.sleep()
        t1 = time.time()
        self.rate = (len(self.robot_states)) / (t1-t0)
        self.save_data()

    def format_data_for_saving(self):
        self.robot_states_formated = []
        
        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                            [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                            ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])# + flattened_jac)
        

    def save_data():
        folder = str(self.datasave_folder + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        self.format_data_for_saving()
        T0 = pd.DataFrame(self.robot_states_formated)
        T1 = pd.DataFrame(self.centroid)
        print(T0)
        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"] 
        
        T0.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
        T1.to_csv(folder + '/centroid.csv', header = ["x" ,"y"], index = False)


    def read_robot_data(self, robot_joint_data):#, fing_cam):
            if self.i != self.prev_i:
                self.prev_i = self.i
                ee_state = self.move_group.get_current_pose().pose
                self.robot_states.append([robot_joint_data, ee_state])
                # self.camera_finger.append(fing_cam)

    def get_robot_task_state(self):
        robot_ee_pose = self.move_group.get_current_pose().pose
        return [robot_ee_pose.position.x, robot_ee_pose.position.y, robot_ee_pose.position.z], [robot_ee_pose.orientation.x, robot_ee_pose.orientation.y, robot_ee_pose.orientation.z, robot_ee_pose.orientation.w]
    
    def create_pose(self, start_position, orientation):
        pose = PoseStamped()
        pose.header.frame_id = '/panda_link0'
        pose.pose.position.x = start_position[0]
        pose.pose.position.y = start_position[1]
        pose.pose.position.z = start_position[2]

        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        return pose


    def go_home(self):
        self.move_group.set_planner_id("PTP")         # Set to be the straight line planner
        self.move_group.go(self.joint_home, wait=True)
        
    def go_push(self):
        self.move_group.set_planner_id("PTP")
        # self.move_group.go(self.target_pose, wait=True)
        self.move_group.go(self.joint_push, wait=True)

    def go_via_point(self):
        self.move_group.set_planner_id("PTP")   #was LIN
        self.move_group.go(self.joint_via_point, wait=True)

if __name__ == '__main__':
    robot = Trajectory()
    robot.pushing_actions()