#include <franka_example_controllers/cartesian_pose_example_controller.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

namespace franka_example_controllers {

class CartesianPoseExampleController : public controller_interface::ControllerBase {
 private:
  franka_hw::FrankaCartesianPoseInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Subscriber opt_traj_sub_;
  std::array<double, 16> initial_pose_;
  geometry_msgs::Point opt_traj_point_;

 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override {
    cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
    if (cartesian_pose_interface_ == nullptr) {
      ROS_ERROR("Could not get Cartesian Pose interface from hardware");
      return false;
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR("Could not get parameter arm_id");
      return false;
    }

    try {
      cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
          cartesian_pose_interface_->getHandle(arm_id + "_robot"));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("Exception getting Cartesian handle: " << e.what());
      return false;
    }

    opt_traj_sub_ = node_handle.subscribe("/opt_traj", 1, &CartesianPoseExampleController::optTrajCallback, this);

    return true;
  }

  void optTrajCallback(const geometry_msgs::Point::ConstPtr& msg) {
    opt_traj_point_ = *msg;
  }

  void starting(const ros::Time& /* time */) override {
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  }

  void update(const ros::Time& /* time */, const ros::Duration& period) override {
    // Here you can implement the logic to move the robot end effector to the opt_traj_point_
    std::array<double, 16> new_pose = initial_pose_;
    
    // Update the position part of the pose
    new_pose[12] = opt_traj_point_.x;
    new_pose[13] = opt_traj_point_.y;
    new_pose[14] = opt_traj_point_.z;

    cartesian_pose_handle_->setCommand(new_pose);
  }
};

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController, controller_interface::ControllerBase)