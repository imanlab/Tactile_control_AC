// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "examples_common.h"

std::array<double, 16> target_pose;


void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  // Extract pose components (position and orientation) from the received message
  double x = msg->x;
  double y = msg->y;
  double z = 0   //adjust
  // Populate the target_pose array
  target_pose = {{
      1.0, 0.0, 0.0, x,
      0.0, 1.0, 0.0, y,
      0.0, 0.0, 1.0, z,
      0.0, 0.0, 0.0, 1.0,
  }};
}


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle nh;
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    ros::Subscriber targetPoseSub = nh.subscribe("/opt_traj", 1, targetPoseCallback);


    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    // std::array<double, 7> q_goal = {{0.14360167152468625, 0.1753777056788718, -0.2196985128072391, -1.365035858023108, -0.15087520535954108, 3.1017061897913636, -2.018819763140546}};
    // std::array<double, 7> q_goal = {{-0.00635813,0.0372822,-0.0373876,-1.30535,0.0145356,2.85452,0.787199}};
    // std::array<double, 7> q_goal = {{0.000189747,-0.348658,-0.0224457,-1.67147,-0.0154906,2.95908,0.788193}};
    MotionGenerator motion_generator(0.5, q_goal);

    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    double time = 0.0;
    double T = 1.2;
    double a = 0.8 / pow(T, 2);
    

    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      std::array<double, 16> new_pose = getTargetPoseFromTopic();

      // Your motion generation logic here using initial_pose, target_pose, and time
      // ...

      // DON'T KNOW IF THE FOLLOWING ONE IS CORRECT --- MAYBE DEFINE A BANG-BANG TRAJECTORY
      if (time < T/2){
        new_pose[13] = - 0.5 * a * time * time + initial_pose[13];
      } else if (time < T){
        new_pose[13] = 0.5 * a * (time - T) * (time - T) - (0.2 - initial_pose[13]);
      }


      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
