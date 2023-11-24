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

class FrankaRobot(object):
    def __init__(self):
        super(FrankaRobot, self).__init__()
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
        self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
        print(self.move_group.get_interface_description().name)    # Print the planner being used.

        self.move_group.set_max_velocity_scaling_factor(0.8)       # scaling down velocity
        self.move_group.set_max_acceleration_scaling_factor(0.8)  # scaling down acceleration  # 0.05

        self.pushing_z_height        = 0.15
        self.starting_depth          = 0.30 # was 35
        # self.finish_depth            = self.starting_depth + 0.05 + 0.25  # (0.5)
        self.finish_depth            = 0.40
        self.starting_position_width = [-0.08, 0.3]
        self.starting_position = -0.08
        self.finish_position = 0.3
        self.pushing_orientation     = [-0.9238638957839016, 0.3827149349905697, -0.0020559535525728366, 0.0007440814108405214]

        self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        self.joint_home = [0.18984723979820606, -0.977749801850428, -0.22761550468348588, -2.526835711730154, -0.20211957115956533, 3.1466847225824988, 0.7832720796780586]
        self.joint_push = [0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144]
        self.joint_via_point = [0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144]

        # JOINTS VALUES
        # STRAW SX: 0.17638642781659175, 0.24688739163446455, -0.21964895456542954, -1.075648374362316, 0.0059040641931806595, 2.49889777279577, -2.279363081662112
        # STEM SX HIGH: 0.12472952538415005, 0.4419523189257516, -0.2268521662653399, -0.7328628249317115, 0.09699661042123234, 2.787466859265501, -2.317040942416507
        # STRAW SX HIGH: 0.14518269428805297, 0.3999835010233113, -0.17861881098517943, -0.7155118474188835, -0.02258401060766644, 2.3282200174331664, -2.3087926862074264
        # STARW SX: 0.1717869929577175, 0.19109001349976099, -0.20248747353731603, -1.2955142998444407, 0.07144792380271715, 2.999680000212005, -2.3821880487596734
        # STRAW SX BOTTOM TOP: 0.14936706539772002, 0.22454546175018172, -0.22166286731824725, -1.1697368214590507, -0.17653950999771761, 2.881462726277297, -0.8408458503186679
        # STRAW DX: 0.04607601949356312, 0.17021933704864067, -0.13281828155595027, -1.3124015096343356, -0.019060995645703918, 3.009036840939284, 0.8235806091527144
        # STEM DX HIGH: 0.01877408689443296, 0.3938299077555545, -0.16231518721292756, -0.7594753556413555, 0.03801567551841853, 2.7554390427271525, 0.7919269511335517
        # BOTTOM TOP: 0.13647267212636702, 0.29775212166602205, -0.248174163759712, -1.0194844774789638, -0.09988637203640406, 2.723226729948717, -0.19643720480800225
        # UPDOWN 1 : 0.13005721391293038, 0.36349681467774525, -0.2825711837204216, -0.7972262237644334, -0.14252537727331427, 2.2318189776738473, 0.7055710180494495
        # DX TO SX TILTED UPDOWN TILTED RIGHT: 0.034334319390697345, 0.2556024921108534, -0.12514880304239012, -0.897488394277251, 0.045590534837670574, 2.240063419026217, 0.5793565670550617
        # DX TO SX TILTED UPDOWN: 0.20072787802470296, 0.08927335317092222, -0.2515315325526215, -1.295081142392075, -0.03537070164746708, 2.6857572598292583, 0.8061396811527981
        # DX TO SX MORE TILTED UPDOWN: 0.11018630737510891, 0.29260331744731105, -0.249527427401459, -0.8037662773968886, 0.1605739096339867, 2.076709148731154, 0.7302613615881454
        # DX TO SX TILTED DOWNUP: 0.17081756852437038, 0.2054913266299649, -0.2306200354733748, -1.547290352285954, 0.09144926861259672, 3.739156617994548, 0.7947664386266056
        # DX TO SX: 0.08121559405013135, 0.35921362119152783, -0.2169366269068474, -0.7805137910842356, 0.11388321365250481, 2.535152081065708, 0.7941936063667386
        # DX TO SX RIGHT TILTED: 0.1680918744577765, 0.0773544340474846, -0.2217802532101088, -1.4715670771433882, 0.00035040072463195444, 3.1577774621909236, 0.3101020713169118
        # STEM DX LOW: 0.06785820067488288, 0.2101602367814964, -0.1745633058570215, -1.0134326223248784, 0.20697346682680967, 2.620245136707027, 0.6744049262901147
        # STEM DX HIGH: 0.026501098037051005, 0.2841936486761172, -0.12279676709013398, -0.9124171410594404, 0.04548363351796108, 2.7795008895961524, 0.7966452993663676
        # STEM DX LOW DOWNUP: -0.1251889900993717, 0.3270095055398579, 0.08512084177934533, -0.7092821406294356, 0.10033687954478794, 2.284627084047795, 0.6933525089747952
        # STEM DX HIGH UPDOWN: -0.11640723545818496, 0.3310616827588127, 0.08585725122369772, -1.0829481084890533, 0.09251806390735183, 3.4788670709751264, 0.6767434285356915
        # SX TO DX TILTED UPDOWN: 0.09997277342944602, 0.1424868881011582, -0.08284607451333094, -1.1853227890553948, -0.05419205655323134, 2.5267731567998655, -2.310751636038224
        # SX TO DX TILTED DOWNUP: 0.027698455430022764, 0.2535964293982498, -0.023075784424417906, -1.4852780823213563, -0.052277590529786214, 3.738603407206074, -2.320943159254402
        # SX TO DX TILTED DOWNUP TILTED LEFT: 0.032542968651181774, 0.12695669291441064, -0.0016731429772380096, -1.5067610608093818, 0.05879627393397011, 3.3992760972976686, -2.0370848585633508
        # SX TO DX TILTED UPDOWN TILTED LEFT: 0.028869519334904593, 0.12850849893401048, -0.01205668957772608, -1.2477339517740242, 0.048265634313770234, 2.706446238345002, -1.922276667821066
        # SX TO DX:  0.030872797735378703, 0.08398333902859183, -0.015412248480787732, -1.455705059887902, -0.021846264771289297, 3.0628823733193147, -2.296158976303957
        # STEM SX LOW: 0.04087660735902801, 0.1774086123448491, -0.01748135073827725, -1.1678269272687143, -0.02135036701158065, 2.892645667076775, -2.3181141305267094
        # STEM SX HIGH: 0.05872839005498517, 0.39400592916023663, -0.036091494608409415, -0.6502329208473178, -0.02336838286243702, 2.607057501163771, -2.3263913018329667
        # STEM SX TILTED UPDOWN: 0.07973845165026815, 0.3538111918087231, -0.050502362228998184, -0.661024991177588, -0.023363940170033598, 2.2267511173884076, -2.2760901543536085
        # STEM SX TILTED DOWNUP: 0.06785179928729493, 0.3979851099625753, -0.07326815275297473, -1.0171059925032688, -0.018986619806951946, 3.5106008590062454, -2.346624771898869


        self.resets           = 5
        self.pushes_per_reset = 5

        # datasaving:
        self.datasave_folder = "/home/alessandro/Dataset/Pushing_Single_Strawberry/test/"
        self.robot_sub       = message_filters.Subscriber('/joint_states', JointState)
        self.fing_cam_sub = message_filters.Subscriber("/fing_camera/color/image_raw", Image)
        #self.front_color_sub = message_filters.Subscriber("/front_camera/color/image_raw", Image)
        #self.front_depth_sub = message_filters.Subscriber("/front_camera/color/depth_raw", Image)
        #self.hand_color_sub = message_filters.Subscriber("/hand_camera/color/image_raw", Image)
        #self.hand_depth_sub = message_filters.Subscriber("/hand_camera/color/depth_raw", Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.fing_cam_sub] , queue_size=1, slop=0.1, allow_headerless=True)
        #self.ts_hand = message_filters.ApproximateTimeSynchronizer([self.hand_color_sub, self.hand_depth_sub] , queue_size=1, slop=0.1, allow_headerless=True)


    def pushing_actions(self):
        start_position, start_ori = self.get_robot_task_state()

        total_pushes = 0
        failure_cases = 0
        for j in range(self.resets):
            for i in range(self.pushes_per_reset):
                print("  ")
                print("resets: {},   push: {},  total_pushes: {},   failure cases: {}".format(j, i, total_pushes, failure_cases))
                # self.go_via_point()
                self.go_home()
                # self.take_picture()
                self.go_push()
                
                # # 1. Move to random start position:
                # self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
                # # start_y_position = random.uniform(self.starting_position_width[0], self.starting_position_width[1])
                # start_y_position = self.starting_position
                # start_pose = self.move_group.get_current_pose().pose
                # start_pose.position.x = self.starting_depth
                # # start_pose.position.x = random.uniform(self.starting_depth, self.finish_depth)
                # start_pose.position.y = start_y_position
                # start_pose.position.z = self.pushing_z_height
                # start_pose.orientation.x = self.pushing_orientation[0]
                # start_pose.orientation.y = self.pushing_orientation[1]
                # start_pose.orientation.z = self.pushing_orientation[2]
                # start_pose.orientation.w = self.pushing_orientation[3]

                # target = self.move_group.set_pose_target(start_pose)
                # trajectory = self.move_group.plan(target)
                # if trajectory[0] == False:
                #     self.go_home()
                #     failure_cases += 1
                #     break
                # self.move_group.go(target, wait=True)
                

                # 2. Execute pushing action to second random position:
                # self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
                # start_pose = self.move_group.get_current_pose().pose
                # # finish_y_position = random.uniform(self.starting_position_width[0], self.starting_position_width[1])
                # finish_y_position = self.finish_position
                # finish_x_position = random.uniform(self.starting_depth, self.finish_depth)
                # # finish_y_position = random.uniform(self.starting_position_width[0], start_y_position)
                
                # start_position, start_ori = self.get_robot_task_state()    # calculate required angle of end effector:
                # euler_ori = euler_from_quaternion(start_ori)
                # euler_ori = list(euler_ori)
                # rotational_change = math.atan((start_y_position - finish_y_position) / (self.finish_depth - self.starting_depth))
                # # rotational_change = math.atan((start_y_position - finish_y_position) / (self.starting_depth - start_pose.position.x))
                # # rotational_change = math.atan((start_y_position - finish_y_position) / (finish_x_position - self.starting_depth))
                # # rotational_change = math.atan((start_y_position - finish_y_position) / (finish_x_position - start_pose.position.x))
                # print(f"rotational change: {rotational_change}")
                # euler_ori[2] += rotational_change
                # joint_goal = self.move_group.get_current_joint_values()
                # # joint_goal[-1] += rotational_change - pi/2
                # # joint_goal[-1] += - pi
                # self.move_group.go(joint_goal, wait=True)
                # self.move_group.stop()
                

                # a = input("start_collecting data - enter to continue:")
                # if a == "n" or a == "N":
                #    break


                # LINEAR PUSHING IN ONE DIRECTION

                # # 3. Make pushing action:
                # self.move_group.set_planner_id("LIN")
                # #finish_y_position += 0.1
                # _, final_ori_quat = self.get_robot_task_state()
                # finish_pose = self.move_group.get_current_pose().pose
                # # finish_pose.position.x = finish_pose.position.x - 0.2
                # finish_pose.position.x -= 0.02
                # finish_pose.position.y -= 0.12
                # # finish_pose.position.z += 0.05
                # finish_pose.orientation.x = final_ori_quat[0]
                # finish_pose.orientation.y = final_ori_quat[1]
                # finish_pose.orientation.z = final_ori_quat[2]
                # finish_pose.orientation.w = final_ori_quat[3]
                # target = self.move_group.set_pose_target(finish_pose)
                # trajectory = self.move_group.plan(target)
                # if trajectory[0] == False:
                #     print("False")
                #     self.go_home()
                #     failure_cases += 1
                #     break
                # time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))
                # self.move_group.go(target, wait=False)
                # self.data_saver(time_for_trajectory)

                # PUSHES THAT DESCRIBE AN ARC

                pilz_pose = MotionPlanRequest()
                pilz_pose.planner_id = "CIRC"
                pilz_pose.group_name = "panda_arm"
                pilz_pose.max_velocity_scaling_factor = 0.2 # 0.2
                pilz_pose.max_acceleration_scaling_factor = 0.06 # 0.05
                pilz_pose.start_state.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
                pilz_pose.start_state.joint_state.position = self.move_group.get_current_joint_values()
                # pilz_pose.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # pilz_pose.start_state.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                pilz_pose.start_state.joint_state.header.stamp = rospy.Time.now()
        
                pose = self.move_group.get_current_pose()
                pose.pose.position.z += 0.05  # RIGHT ONE DX !!!!
                pose.pose.position.y -= 0.12  # RIGHT ONE DX !!!!
                pose.pose.position.x -= 0.02  # RIGHT ONE DX !!!!
                # pose.pose.position.z += 0.02
                # pose.pose.position.y -= 0.12  
                # pose.pose.position.x -= 0.03
                # pose.pose.position.z += 0.02
                # pose.pose.position.y += 0.10 
                # pose.pose.position.x -= 0.02
                # pose.pose.position.z += 0.1
                # pose.pose.position.y = + 0.2
                pose.pose.orientation.x = 0.6847884219250332
                pose.pose.orientation.y = -0.018653069577975762
                pose.pose.orientation.z = 0.7265456014775064
                pose.pose.orientation.w = 0.05337011491865343 

                
                # Orientations
                # x: 0.6673118069487086  y: 0.22945693167932407 z: 0.637311613847233 w: 0.3096423350683734    circa - pi/2
                # x: 0.6088969914405341 y: -0.07194376739623143 z: 0.7896015255336476 w: 0.024453609599394147   
                # x: 0.24745165186593954 y: 0.6467114211426609 z: 0.30211974198520986 w: 0.6551760673706328   UpsideDown  circa - pi
                # x: -0.7000497491517786 y: -0.7000497491517786 z: -0.7099405225477211 w: 0.023204489216479676   circa - pi/3
                # x: 0.5463812985450763  y: 0.4736268452696545  z: 0.5478142323206663  w: 0.42076674643682344
                # x: 0.4625551611466434  y: 0.5603673982482279  z: 0.44805991436323167  w: 0.5208391450514528
                # x: 0.6847884219250332 y: -0.018653069577975762 z: 0.7265456014775064 w: 0.05337011491865343  RIGHT ONE FOR PUSHING DX TO SX
                # x: 0.31997031062443054 y: 0.6279902775550916 z: 0.4859878437063975 w: 0.5167814116091933     RIGHT ONE FOR PUSHING FIRST SX TO DX
                # x: 0.4741888250671071 y: 0.5006283387453521 z: 0.44245756688781357 w: 0.573365089737997
                # x: 0.6233901857452114 y: 0.37149289069959956 z: 0.5662414665957355  w: 0.3908302828386215   TILTED FOR BOTTOM TOP PUSHES
                # x: 0.7036374629506462 y: 0.16253800814247032 z: 0.6618960509841502 w: 0.20092121424340345   TILTED FOR BOTTOM TOP SX
                # x: 0.24527957854126883 y: 0.44075454322728563 z: 0.517805907940457 w: 0.6909778597589675
                # x: 0.5373964582218325 y: 0.7013220273628539 z: 0.3067053945180102 w: 0.35395516891244655

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
                time_for_trajectory = float(str(trajectory[1].joint_trajectory.points[-1].time_from_start.secs) + "." +str(trajectory[1].joint_trajectory.points[-1].time_from_start.nsecs))
                self.move_group.go(target, wait=False)
                self.data_saver(time_for_trajectory)

                total_pushes += 1
                
                self.go_via_point()
                
            self.go_home()

    def data_saver(self, time_for_trajectory):
        rate                = rospy.Rate(10)
        self.robot_states   = []
        self.camera_finger  = []
        # self.front_color  = []
        # self.front_depth  = []
        # self.hand_color   = []
        # self.hand_depth   = []
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

    def read_robot_data(self, robot_joint_data, fing_cam):#, front_col, front_dep, hand_col, hand_dep):
        if self.i != self.prev_i:
            self.prev_i = self.i
            ee_state = self.move_group.get_current_pose().pose            
            self.robot_states.append([robot_joint_data, ee_state])
            self.camera_finger.append(fing_cam)
            # self.front_color.append(front_col)
            # self.front_depth.append(front_dep)
            # self.hand_color.append(hand_col)
            # self.hand_depth.append(hand_dep)

    def format_data_for_saving(self):
        self.robot_states_formated = []

        for data_sample_index in range(len(self.robot_states)):
            robot_joint_data = self.robot_states[data_sample_index][0]
            ee_state = self.robot_states[data_sample_index][1]
            self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort) + 
                                              [ee_state.position.x, ee_state.position.y, ee_state.position.z,
                                               ee_state.orientation.x, ee_state.orientation.y, ee_state.orientation.z, ee_state.orientation.w])
            # self.robot_states_formated.append(list(robot_joint_data.position) + list(robot_joint_data.velocity) + list(robot_joint_data.effort))

    def save_data(self):
        
        print("robot_states", np.array(self.robot_states).shape)
        print("camera_finger", np.array(self.camera_finger).shape)
        # print("camera_front_color", np.array(self.front_color).shape)
        # print("camera_front_depth", np.array(self.front_depth).shape)
        # print("camera_hand_color", np.array(self.hand_color).shape)
        # print("camera_hand_depth", np.array(self.hand_depth).shape)
        
        
        
        # create new folder for this experiment:
        folder = str(self.datasave_folder + '/data_sample_' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        mydir = os.mkdir(folder)

        self.format_data_for_saving()
        T0 = pd.DataFrame(self.robot_states_formated)

        robot_states_col = ["position_panda_joint1", "position_panda_joint2", "position_panda_joint3", "position_panda_joint4", "position_panda_joint5", "position_panda_joint6", "position_panda_joint7", "position_panda_finger_joint1", "position_panda_finger_joint2",
        "velocity_panda_joint1", "velocity_panda_joint2", "velocity_panda_joint3", "velocity_panda_joint4", "velocity_panda_joint5", "velocity_panda_joint6", "velocity_panda_joint7", "velocity_panda_finger_joint1", "velocity_panda_finger_joint2",
        "effort_panda_joint1", "panda_joint2", "effort_panda_joint3", "effort_panda_joint4", "panda_joint5", "effort_panda_joint6", "effort_panda_joint7", "effort_panda_finger_joint1", "effort_panda_finger_joint2",
        "ee_state_position_x", "ee_state_position_y", "ee_state_position_z", "ee_state_orientation_x", "ee_state_orientation_y", "ee_state_orientation_z", "ee_state_orientation_w"]

        T0.to_csv(folder + '/robot_state.csv', header=robot_states_col, index=False)
        np.save(folder + '/camera_finger.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.camera_finger]))
        # np.save(folder + '/camera_front_color.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.front_color]))
        # np.save(folder + '/camera_front_depth.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.front_depth]))
        # np.save(folder + '/camera_hand_color.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.hand_color]))
        # np.save(folder + '/camera_hand_depth.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.hand_depth]))
        # # print(self.first_shot_rgb[0].shape)
        # hand_rgb = [self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.first_shot_rgb]
        # hand_depth = [self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.first_shot_depth]
        # hand_rgb[0] = cv2.cvtColor(hand_rgb[0], cv2.COLOR_BGR2RGB)


        # name = "first_shot_rgb.png"
        # # imsave(folder + '/name', self.first_shot_rgb[0])
        # name = "first_shot_depth.png"
        # imsave(folder + '/first_shot_rgb.png', hand_rgb[0])
        # imsave(folder + '/first_shot_depth.png', hand_depth[0])
        
        # i = np.load("camera_hand_color.npy")
        # first = i[0]
        # name = "first_shot" + str(i) + ".png"
        # imsave(name, first)
        # j = np.load("camera_hand_depth.npy")
        # first = j[0]
        # name = "first_shot" + str(j) + ".png"
        # imsave(name, first)

    
    def go_home(self):
        self.move_group.set_planner_id("PTP")                      # Set to be the straight line planner
        self.move_group.go(self.joint_home, wait=True)
        
    def go_push(self):
        self.move_group.set_planner_id("PTP")
        self.move_group.go(self.joint_push, wait=True)

    def go_via_point(self):
        self.move_group.set_planner_id("LIN")
        self.move_group.go(self.joint_via_point, wait=True)    

    # def take_picture(self):
    #     self.first_shot_rgb =   []
    #     self.first_shot_depth = []
    #     self.ts_hand.registerCallback(self.read_camera_hand_data)


    # def read_camera_hand_data(self, hand_col, hand_dep):
    #     self.first_shot_rgb.append(hand_col)
    #     self.first_shot_depth.append(hand_dep)

    def reset_objects(self):
        # move home first (to get the robot up high enough)
        self.move_group.set_named_target('ready')
        self.move_group.go()

        # Move above the pushback point:
        self.move_group.set_planner_id("PTP")                      # Set to be the straight line planner
        self.move_group.go([0.07130824142401027, 0.8614598760406538, -0.16447389670298204, -0.9640181670858131, 0.13709807091064682, 1.8604888006846219, 0.7095731452172556], wait=True)

        # Move down to the pushback point:
        self.move_group.go([0.06827783184260813, 1.0580782766974726, -0.16370692051658037, -0.9703260839895288, 0.13953893815809099, 2.066282942907594, 0.7095669528161733], wait=True)

        # Push the block back towards the robot to reset the objects.
        self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
        self.move_group.go([0.13260246841531056, 0.34646877631008416, -0.14746936818756937, -2.2979664011990875, 0.11272908575888033, 2.6471211875279743, 0.6693391410410404], wait=True)

        # move back home (to get the robot up high enough)
        self.move_group.set_planner_id("PTP")                      # Set to be the straight line planner
        self.move_group.go([0.12347067714783183, 0.0752346964597091, -0.17116823404775366, -1.9258203851829596, 0.0030568404994491065, 2.040577341397677, 0.7074800998503142], wait=True)

        # moveto the side to push the block back:
        self.move_group.go([0.5625205920607905, -0.12322370656475262, 0.1751817361488022, -2.1664951839073514, 0.07415023610326978, 2.071069740253273, 1.4965139810825394], wait=True)
        self.move_group.go([0.618022809069619, 0.41956730252173147, 0.1317212792416644, -2.189165379708273, -0.142555658538146, 2.645985071788768, 1.2043076761464278], wait=True)

        # # Push the block away from the robot again to ensure it's out of the way.
        self.move_group.set_planner_id("LIN")                      # Set to be the straight line planner
        self.move_group.go([0.08850941608377269, 0.6825085006845076, 0.01852105101081915, -1.6987854360623158, -0.025516628348523373, 2.4043064999103683, 0.9282770566848588], wait=True)

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


if __name__ == '__main__':
    robot = FrankaRobot()
    robot.pushing_actions()
    # robot.reset_objects()







        # # pilz_pose = MotionPlanRequest()
        # # pilz_pose.planner_id = "LIN"
        # # pilz_pose.group_name = "panda_arm"
        # # pilz_pose.max_velocity_scaling_factor = 0.2
        # # pilz_pose.max_acceleration_scaling_factor = 0.05
        # # pilz_pose.start_state.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
        # # pilz_pose.start_state.joint_state.position = self.move_group.get_current_joint_values()
        # # # pilz_pose.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # # # pilz_pose.start_state.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # # pilz_pose.start_state.joint_state.header.stamp = rospy.Time.now()
        
        # pose = self.move_group.get_current_pose()
        # pose.pose.position.z -= 0.2
        
        # # constraint = Constraints()
        # # position_constraints_pose = PositionConstraint()

        # # position_constraints_pose.header = pose.header

        # # position_constraints_pose.constraint_region.primitives = SolidPrimitive()
        # # position_constraints_pose.constraint_region.primitives.type = 1
        # # position_constraints_pose.constraint_region.primitives.dimensions = [0.1, 0.1, 0.1]
        # # position_constraints_pose.constraint_region.primitive_poses = Pose()
        # # position_constraints_pose.constraint_region.primitive_poses = pose
        # # constraint.position_constraints = [position_constraints_pose]
        # # pilz_pose.goal_constraints = constraint

        # target = self.move_group.set_pose_target(pose)
        # trajectory = self.move_group.plan(target)
        # self.move_group.go(target, wait=True)
