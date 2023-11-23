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

        self.move_group.set_max_velocity_scaling_factor(0.2)       # scaling down velocity
        self.move_group.set_max_acceleration_scaling_factor(0.02)  # scaling down acceleration  # 0.05

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
        self.joint_push = [0.14360167152468625, 0.1753777056788718, -0.2196985128072391, -1.365035858023108, -0.15087520535954108, 3.1017061897913636, -2.018819763140546]
        self.joint_via_point = [0.14360167152468625, 0.1753777056788718, -0.2196985128072391, -1.365035858023108, -0.15087520535954108, 3.1017061897913636, -2.018819763140546]

        # HOME Position : 0.18984723979820606, -0.977749801850428, -0.22761550468348588, -2.526835711730154, -0.20211957115956533, 3.1466847225824988, 0.7832720796780586, 0.04035545140504837, 0.04035545140504837
        # STEM 1 from DX:   0.056910959576521696, 0.2821489437897694, 0.03192101496657554, -1.0194771746635911, 0.12775754060597605, 2.895636680521177, 0.6672536889612675, 0.04036398604512215, 0.04036398604512215
        # STEM 1 from SX:   0.23086829590376073, 0.3470080969974167, -0.09854935223147986, -0.8710170411381125, -0.23082243248489165, 2.8180115306091844, -2.0942533056040604, 0.040361687541007996, 0.040361687541007996
        # STRAW 1 from SX: 0.3349699563764416, 0.13228792616670523, -0.20801186613048103, -1.4862533817998023, -0.2682172310269529, 3.2206274220148723, -2.0471692029734454, 0.040361687541007996, 0.040361687541007996
        # STRAW 1 from DX: 0.3499066223805411, 0.1767867084240142, -0.22260668660684726, -1.4581621506094675, 0.13011390588328195, 3.3216288816606787, 0.6681941866774614, 0.040361687541007996, 0.040361687541007996
        # STEM 2 from DX: 0.07799133335904118, 0.4674038597130084, -0.108937670476093, -0.7142617910535706, -0.04501549128436372, 2.947090336170119, 0.8252106761833031, 0.040358733385801315, 0.040358733385801315
        # STRAW 2 from DX: 0.1827175492044276, 0.14343199449541466, -0.175637748772877, -1.373292115629765, -0.09684765035353707, 3.1279476544062375, 0.8534871307909487, 0.040361687541007996, 0.040361687541007996
        # STEM 2 from SX: 0.20612881408465122, 0.39963921609457415, -0.1498813729009783, -0.7028731410233521, -0.15025881493753854, 2.6622977518898225, -2.2162056109306594, 0.04036135971546173, 0.04036135971546173
        # STRAW 2 from SX: 0.1952512429393539, 0.18755568553507881, -0.11358208760264214, -1.2957408742532859, -0.1376387489239375, 3.003782196134751, -2.2353463484315164, 0.04036135971546173, 0.04036135971546173
        # STEM 3 from DX: 0.031917930862250844, 0.4322014708543043, -0.10763223394105519, -0.6789915753527547, 0.1115955827392932, 2.7249351213798687, 0.7526366383186684, 0.040361687541007996, 0.040361687541007996
        # STRAW 3 FROM DX: 0.05728339801283313, 0.17515437521918836, -0.09864572214213335, -1.2625120425809893, 0.05963857436902755, 2.9751193265914946, 0.7070557986854781, 0.04036135971546173, 0.04036135971546173
        # STEM 3 FROM SX: 0.13623757296486355, 0.4639689223725398, -0.12745752431002136, -0.6091892340057775, -0.12040635004308486, 2.67342639734907, -2.268002437448694, 0.040361687541007996, 0.040361687541007996
        # STRAW 3 FROM SX:  0.137680316796183, 0.1809256827808397, -0.11138263298337253, -1.2432843718618123, -0.11715901978334371, 2.9070957382444798, -2.238090886331192, 0.04036135971546173, 0.04036135971546173
        # STEM 4 FROM DX: -0.07051167853084293, 0.43461049802570767, -0.006676113829584887, -0.7174060755232429, 0.0229308416230811, 2.8145218496349216, 0.7520828134531601, 0.040361687541007996, 0.040361687541007996
        # STRAW 4 FROM DX: -0.07449993729121623, 0.15696113059046488, 0.019139806555074242, -1.370367213065164, 0.026534897017810075, 3.1031429796198933, 0.7790997155408074, 0.040361687541007996, 0.040361687541007996
        # STEM 4 FROM SX: 0.06952138792571881, 0.43613283805328396, -0.11528172218065287, -0.693685478751121, -0.11701481939208848, 2.711087627092996, -2.225462300787361, 0.04036135971546173, 0.04036135971546173
        # STRAW 4 FROM SX: 0.07988892969337077, 0.17240528735152644, -0.09980363022373964, -1.302434041438656, -0.11668167786392164, 2.957962887763985, -2.2233506718809433, 0.040361687541007996, 0.040361687541007996
        # STEM 5 FROM DX: -0.15662673273629943, 0.38052099309099635, 0.02151623674971002, -0.8063942562968015, 0.0657528273794386, 2.8101843906339465, 0.6849977738179133, 0.040361687541007996, 0.040361687541007996
        # STRAW 5 FROM DX: -0.16591440073230815, 0.14222750758929936, 0.029302971614363453, -1.3899630065710706, 0.06545905252389023, 3.0899997718619043, 0.7466653501378969, 0.040361687541007996, 0.040361687541007996
        # STEM 5 FROM SX: 0.014050702805914507, 0.4511341127581077, -0.1329105964144558, -0.7033091418115717, -0.1144230171339829, 2.7748508391457185, -2.277548996259403, 0.040361687541007996, 0.040361687541007996
        # STRAW 5 FROM SX: 0.0908108790818455, 0.23189569035685842, 0.011064273440001304, -1.2914890065276832, -0.042418890045748815, 3.152559375931643, -2.2307931908130856, 0.04035840556025505, 0.04035840556025505
        
        # STRAW 4 SX TO DX: 0.14360167152468625, 0.1753777056788718, -0.2196985128072391, -1.365035858023108, -0.15087520535954108, 3.1017061897913636, -2.018819763140546
        # STEM 2 DX TO SX TILTED OTHER SIDE: 0.17286637920579997, 0.30759538698024946, -0.22964296138030216, -0.9595429770945211, 0.19548830838652803, 2.8854423233847784, 0.7767210286452101
        # STRAW 4 DX TO SX TILTED: 0.13245695563940701, 0.13515862398007752, -0.26857592199657687, -1.3874068785229166, -0.020533276576705603, 3.0412402976353965, 0.4969985379458285
        # STEM 4 DX TO SX: -0.06644365352875319, 0.41961213361297367, -0.10368871588245215, -0.6688759370305112, 0.039834172485053186, 2.612123664020012, 0.7237313946057228
        # STEM 3 SX TO DX: 0.09180091833535008, 0.5360718290297269, -0.0429847796561267, -0.5208448079389904, -0.18894511599350294, 2.6578027404149367, -2.209597450637081
        # STRAW 3 DX TO SX TILTED: 0.153261264639143, 0.2673715901326743, -0.2576103162057804, -1.1090046953107342, -0.0845398804762185, 2.960229460620814, 0.7132370636444382
        # STRAW 1 DX TO SX TILTED: 0.3014651247702147, 0.16454053941712385, -0.19813614845974903, -1.2788851631561298, -0.096734381285174, 2.8354239308883975, 0.5098889014336476
        # STEM 1 DX TO SX: 0.19388077179655408, 0.2878154102542944, -0.13469777921992757, -1.0346206111406024, -0.0020320504165646106, 2.9159676834742227, 0.8353728258413264
        # STEM 1 SX TO DX: 0.20183763307088995, 0.34887549828981934, -0.09534479508496846, -0.8856139660766518, -0.03285990125454522, 2.7697839655645766, -2.282296423127254
        # 0.02201575063074022, -0.49233325619014, -0.0642936407296283, -2.753725552868935, 0.03921446304303605, 3.74508875861538, -2.491776461331846, 0.040359389036893845, 0.040359389036893845
        # -0.028380306567355247, -0.1882516576070987, -0.018420622442629184, -2.4370903795853036, -0.018902496540436037, 3.7108533548386498, 0.7556894290431434
        # 0.2620320781595235, 0.24293832765749487, -0.08608154490233877, -1.1937964959056615, -0.03498531494999783, 3.125259281476339, -2.3099279934894956
        # RIGHT ONE FIRST STRAW: 0.44718648334989886, 0.17031041952000428, -0.3600997762578099, -1.4241713831907534, 0.042420956843437795, 3.1609172003367316, 0.7428947954809009, 0.040356434881687164, 0.040356434881687164
        # TILTED FIRST STRAW: 0.28230073427224855, 0.18456977357912124, -0.19082653819494305, -1.3566110317710003, -0.1953249317804972, 3.1093534971519747, 0.6138207626242953, 0.040356434881687164, 0.040356434881687164
        # PUSHING FIRST STEM SX TO DX: 0.284957139830286, 0.3266931948285856, -0.12599102023860181, -0.9077569347128018, -0.16277323310242756, 2.814360548821817, -2.0023340471885973, 0.040356434881687164, 0.040356434881687164
        # VIA POINT: 0.6656342871201437, 0.2789357422556844, -0.4233650336862402, -1.0885792098076752, 0.31706864590777284, 2.845376008033752, 0.6739281674023219
        # STRAW 2 FROM DX: 0.18940526163388155, 0.19375244140215658, -0.23083034143009054, -1.3427588089018796, -0.08661971096997276, 3.1369336061507194, 0.7454355565635253, 0.040358733385801315, 0.040358733385801315
        # STEM 2 FROM SX: 0.2536793952031543, 0.3901984130666181, -0.22820394962408838, -0.7929398933940001, -0.0200419598467037, 2.7553238865534464, -2.2475700418252997, 0.040358733385801315, 0.040358733385801315
        # VIA POINT UPSIDE DOWN: 0.18042257391156002, -0.6522656425240825, -0.2156567312896721, -2.3066735016226487, -0.19928004309466432, 3.2472437039088806, -2.2961177611450356, 0.04035840556025505, 0.04035840556025505
        # STRAW 2 FROM SX: 0.1708856008568102, 0.19565644599363252, -0.11424434159162816, -1.292285441859519, -0.19543196111843908, 3.038507807731628, -2.1016139622711303
        # STEM 3 FROM DX: 0.09212843131122364, 0.44661567791903733, -0.22812328743221055, -0.7322984858848001, 0.06593842372629376, 2.837358497745135, 0.7585206391154087, 0.04035545140504837, 0.04035545140504837
        # STRAW 3 FROM DX: 0.16097983259970608, 0.2603265409579978, -0.2577207282268299, -1.1744661346402083, -0.013477985550644158, 3.0821750960428798, 0.811908804517404, 0.04035545140504837, 0.04035545140504837
        # STEM 3 FROM SX: 0.15372636767152242, 0.41892496505675003, -0.12538422847219735, -0.7502830572250533, -0.02511013811530083, 2.885573949495951, -2.250338453461726
        # STEM 3 FROM SX: 0.16237277410457868, 0.3586529422661645, -0.13154176524647493, -0.8422542097147873, -0.09370410712047732, 2.79803170821732, -2.2513293329361237
        # STEM 4 FROM DX: 0.07374424271528376, 0.3151144011866599, -0.22942299118431347, -1.0231825072496812, 0.03309828684065076, 3.005636513710022, 0.7383653347858017
        # -0.01610546840121962, 0.32346509270500534, -0.09179775403209875, -0.9320981935199938, 0.06145193641858503, 2.912114173483986, 0.3103809757928136
        # 0.09706568603298003, 0.22557435770171114, -0.23846320776005234, -1.1086498673303558, -0.00856465394553852, 2.9478639249801635, 0.8400015711659091
        # -0.13100316853415087, 0.379948504538099, 0.015243983429786315, -0.7535999178727391, 0.3748246916229797, 2.682475224400965, 0.511016784975746
        # 0.12002381710644049, 0.20455473541167746, -0.26459884943960743, -1.1389099255110087, 0.09014678235186563, 2.9363728055842215, 0.7724942434606258
        # !!!!!!! STRAW 4 FROM DX: -0.011942022559371462, 0.19441319770136836, -0.09603791048077463, -1.2522120662477862, 0.06266558334276344, 2.922482471148173, 0.572274324884009
        # STEM 4 FROM SX: 0.020506841005358774, 0.2619643077101952, -0.029665900262097897, -1.1106172449742149, 0.04603671901987507, 3.0433808784937884, -2.4266548958276313
        # 0.026087582703982537, 0.29043843410373876, -0.030370379468591332, -1.0185327494102612, 0.046029044806513236, 3.0438362708997286, -2.4264842583085495
        # BELLO 0.1853608083285234, 0.18211179882261957, -0.21659788878109656, -1.1890621801372812, 0.029291484274180452, 2.9502520714659743, -2.3808960859439083
        # STRAW 4 FROM SX: 0.016847608791308956, 0.21888568731641436, -0.030299361740016583, -1.383950781786421, 0.05026245757337776, 3.337244836108316, -2.3248148036102454
        # STEM 5 FROM DX: -0.011982737409897764, 0.3579973640527001, -0.25360307013183014, -0.8607039473315505, 0.020545392412278388, 2.911550057411415, 0.7583041262527306
        # -0.11352658439212221, 0.28335470545224534, -0.12482561383818484, -1.0336447969368319, 0.13927138847609233, 2.905314467430115, 0.6809810805221398
        # STRAW 5 FROM DX: -0.08497953324412441, 0.16216272318479946, -0.10857081748742972, -1.1795155730803388, 0.13761092607588885, 2.55876302168499, 0.636710236019381
        # -0.08137120211962087, 0.11180050628637785, -0.08903413569154786, -1.4274294502356013, 0.13761741673284014, 3.0523609583752367, 0.6141713842401326
        # -0.08253127534138528, 0.21541979277159987, -0.07308042922354642, -1.5126442926900037, 0.12905245151402445, 3.6547554621615985, 0.6544594494402489
        # STEM 5 FROM SX: -0.04223727034110751, 0.3893088668658728, -0.07305176592024122, -0.801773119156815, -0.039527044778063784, 2.8097244460847643, -2.354411326215607
        # -0.06264355012126181, 0.5651246651850248, -0.0522807152126316, -0.5029841855594452, -0.03886264533392901, 2.8178649538884915, -2.3484774269676687
        # -0.043315946655022294, 0.31382055140199105, -0.04729739616889702, -0.9148788977673179, -0.032316891993775784, 2.661708317938711, -2.30816414449601
        # STRAW 5 FROM SX: -0.056873717604010526, 0.2614904073419571, -0.04667834231105557, -1.4572292590136338, -0.032813139827020006, 3.6372491007362866, -2.1748888023793698
        # -0.0524011911263219, 0.14924822218838557, -0.04833176878025386, -1.4612121518317323, -0.03343183066981612, 3.2183585227330522, -2.2738891927560405
        # -0.050274617290288294, 0.2799168936429616, -0.05849661117060214, -1.467082470203529, -0.03166473293579175, 3.7051970616832413, -2.265802170873956
        # -0.04335706094813625, 0.20005312136931197, -0.0513628261235345, -1.1229544356343666, -0.038312227866314816, 2.4847877691713083, -2.325956304442462

        self.resets           = 5
        self.pushes_per_reset = 5

        # datasaving:
        self.datasave_folder = "/home/gabriele/Dataset/Pushing_Strawberries/test/"
        self.robot_sub       = message_filters.Subscriber('/joint_states', JointState)
        self.fing_cam_sub = message_filters.Subscriber("/fing_camera/color/image_raw", Image)
        self.front_color_sub = message_filters.Subscriber("/front_camera/color/image_raw", Image)
        self.front_depth_sub = message_filters.Subscriber("/front_camera/color/depth_raw", Image)
        self.hand_color_sub = message_filters.Subscriber("/hand_camera/color/image_raw", Image)
        self.hand_depth_sub = message_filters.Subscriber("/hand_camera/color/depth_raw", Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.robot_sub, self.fing_cam_sub, self.front_color_sub, self.front_depth_sub, self.hand_color_sub, self.hand_depth_sub] , queue_size=1, slop=0.1, allow_headerless=True)
        self.ts_hand = message_filters.ApproximateTimeSynchronizer([self.hand_color_sub, self.hand_depth_sub] , queue_size=1, slop=0.1, allow_headerless=True)


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
                self.take_picture()
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
                # # finish_pose.position.x -= 0.02
                # finish_pose.position.y += 0.12
                # finish_pose.position.z += 0.02
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
                pilz_pose.max_velocity_scaling_factor = 0.4 # 0.2
                pilz_pose.max_acceleration_scaling_factor = 0.06 # 0.05
                pilz_pose.start_state.joint_state.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"] 
                pilz_pose.start_state.joint_state.position = self.move_group.get_current_joint_values()
                # pilz_pose.start_state.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # pilz_pose.start_state.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                pilz_pose.start_state.joint_state.header.stamp = rospy.Time.now()
        
                pose = self.move_group.get_current_pose()
                # pose.pose.position.z += 0.01  # RIGHT ONE DX !!!!
                # pose.pose.position.y -= 0.10  # RIGHT ONE DX !!!!
                # pose.pose.position.x -= 0.02  # RIGHT ONE DX !!!!
                # pose.pose.position.z += 0.02
                # pose.pose.position.y -= 0.12  
                # pose.pose.position.x -= 0.03
                pose.pose.position.z += 0.02
                pose.pose.position.y += 0.10 
                pose.pose.position.x -= 0.02
                # pose.pose.position.z += 0.1
                # pose.pose.position.y = + 0.2
                pose.pose.orientation.x = 0.31997031062443054
                pose.pose.orientation.y = 0.6279902775550916
                pose.pose.orientation.z = 0.4859878437063975
                pose.pose.orientation.w = 0.5167814116091933 
                
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
        self.camera_finger   = []
        self.front_color   = []
        self.front_depth   = []
        self.hand_color = []
        self.hand_depth = []
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

    def read_robot_data(self, robot_joint_data, fing_cam, front_col, front_dep, hand_col, hand_dep):
        if self.i != self.prev_i:
            self.prev_i = self.i
            ee_state = self.move_group.get_current_pose().pose            
            self.robot_states.append([robot_joint_data, ee_state])
            self.camera_finger.append(fing_cam)
            self.front_color.append(front_col)
            self.front_depth.append(front_dep)
            self.hand_color.append(hand_col)
            self.hand_depth.append(hand_dep)

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
        print("camera_front_color", np.array(self.front_color).shape)
        print("camera_front_depth", np.array(self.front_depth).shape)
        print("camera_hand_color", np.array(self.hand_color).shape)
        print("camera_hand_depth", np.array(self.hand_depth).shape)
        
        
        
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
        np.save(folder + '/camera_front_color.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.front_color]))
        np.save(folder + '/camera_front_depth.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.front_depth]))
        np.save(folder + '/camera_hand_color.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.hand_color]))
        np.save(folder + '/camera_hand_depth.npy', np.array([self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.hand_depth]))
        # print(self.first_shot_rgb[0].shape)
        hand_rgb = [self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.first_shot_rgb]
        hand_depth = [self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') for image in self.first_shot_depth]
        hand_rgb[0] = cv2.cvtColor(hand_rgb[0], cv2.COLOR_BGR2RGB)


        # name = "first_shot_rgb.png"
        # imsave(folder + '/name', self.first_shot_rgb[0])
        # name = "first_shot_depth.png"
        imsave(folder + '/first_shot_rgb.png', hand_rgb[0])
        imsave(folder + '/first_shot_depth.png', hand_depth[0])
        
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

    def take_picture(self):
        self.first_shot_rgb =   []
        self.first_shot_depth = []
        self.ts_hand.registerCallback(self.read_camera_hand_data)


    def read_camera_hand_data(self, hand_col, hand_dep):
        self.first_shot_rgb.append(hand_col)
        self.first_shot_depth.append(hand_dep)

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
