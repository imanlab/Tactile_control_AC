#!/usr/bin/env python3

import os
import sys
import cv2
import copy
import time
import math
import rospy
import random
import datetime
import message_filters
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg

import numpy as np
import pandas as pd

from math import pi
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool, Int16MultiArray
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState, Image
from moveit_msgs.msg import CollisionObject, DisplayTrajectory, MotionPlanRequest, Constraints, PositionConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list

class FrankaRobot(object):
   def __init__(self):
     super(FrankaRobot, self).__init__()
     moveit_commander.roscpp_initialize(sys.argv)
     rospy.init_node('FrankaRobotWorkshop', anonymous= True)
     self.robot = moveit_commander.RobotCommander()
     self.scene = moveit_commander.PlanningSceneInterface()
     self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
     self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size =20)
     self.planning_frame = self.move_group.get_planning_frame()
     self.eef_link = self.move_group.get_end_effector_link()
     self.group_names = self.robot.get_group_names()
     

   def get_robot_joint_state(self):
     robot_joint_state = self.robot.get_current_state().joint_state.position
     print(robot_joint_state)

   def get_robot_task_state(self):
     robot_ee_pose = self.move_group.get_current_pose().pose
     print(robot_ee_pose)  
     


if __name__ == '__main__':
  robot = FrankaRobot()
  robot.get_robot_joint_state()
  robot.get_robot_task_state()