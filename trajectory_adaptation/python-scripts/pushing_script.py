#!/usr/bin/env python

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

class RobotPusher():
    def __init__(self):
        super(RobotPusher, self).__init__(sys.argv)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pushing_action', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.move_group.set_end_effector_link("panda_link8")
        self.group_names = self.robot.get_group_names()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        
        self.move_group.set_max_velocity_scaling_factor(0.50)  # scaling down velocity
        self.move_group.set_max_acceleration_scaling_factor(0.5)  # scaling down velocity
        self.move_group.allow_replanning(True)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_goal_position_tolerance(0.0005)
        self.move_group.set_goal_orientation_tolerance(0.001)
        self.move_group.set_planning_time(5)

        self.move_group.set_planner_pipeline_id("pilz_industrial_motion_planner")

        self.goal_pose = rospy.Subscriber("/opt_traj", Point, self.receive_goal_postion)
        self.last_pose_pub = rospy.Publisher('/last_pose', Point, queue_size=11)

        self.robot_sub       = message_filters.Subscriber('/joint_states', JointState, self.read_robot_data)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.goal_pose] , queue_size=1, slop=0.1, allow_headerless=True)

        self.pushing_orientation = [-0.9238638957839016, 0.3827149349905697, -0.0020559535525728366, 0.0007440814108405214]
        self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        self.joint_home = [0.18984723979820606, -0.977749801850428, -0.22761550468348588, -2.526835711730154, -0.20211957115956533, 3.1466847225824988, 0.7832720796780586]

        #other initialization
        self.robot_states = []

    def pushing_actions(self):
        pilz_pose = MotionPlanRequest()
        pilz_pose.planner_id = "CIRC"
        pilz_pose.group_name = "panda_arm"
        pilz_pose.max_velocity_scaling_factor = 0.2
        pilz_pose.max_acceleration_scaling_factor = 0.05
        pilz_pose.start_state.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        pilz_pose.start_state.joint_state.position = self.move_group.get_current_joint_values()
        pilz_pose.start_state.joint_state.header.stamp = rospy.Time.now()

        pose = self.move_group.get_current_pose()
        pose.pose.position.x -= self.goal_pose[0]  # RIGHT ONE DX !!!!
        pose.pose.position.y -= self.goal_pose[1] # RIGHT ONE DX !!!!
        pose.pose.position.z += self.goal_pose[2]  # RIGHT ONE DX !!!!
        pose.pose.orientation.x = 0.6847884219250332
        pose.pose.orientation.y = -0.018653069577975762
        pose.pose.orientation.z = 0.7265456014775064
        pose.pose.orientation.w = 0.05337011491865343 

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
        #TARGET is always equal to None, don't know why
        trajectory = self.move_group.plan(target)

        time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))
        self.move_group.go(target, wait=False)

        #rate = rospy.Rate(30)

        last_pose = Point()
        last_pose.data = self.robot_states[8:10]
        self.last_pose_pub.publish(last_pose)



    def receive_goal_postion(self,goal_pose):
        rospy.loginfo("Goal position received")
        

    def read_robot_data(self, robot_joint_data):#, fing_cam):
            ee_state = self.move_group.get_current_pose().pose.point #CHECK HERE
            self.robot_states.append([robot_joint_data, ee_state])
            # self.camera_finger.append(fing_cam)


    def go_home(self):
        self.move_group.set_planner_id("PTP")         # Set to be the straight line planner
        self.move_group.go(self.joint_home, wait=True)


if __name__ == '__main__':
    robot = RobotPusher()
    robot.pushing_actions()


