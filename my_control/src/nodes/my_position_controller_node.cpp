#include "my_position_controller_node.h"
#include "ros/ros.h"
#include "mav_msgs/default_topics.h"
#include "my_control/parameters_ros.h"
#include "my_control/RotorPWM.h"

namespace my_control {

MyPositionControllerNode::MyPositionControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
  InitializeParams();

  cmd_pose_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1, 
    &MyPositionControllerNode::CommandPoseCallback, this);
  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
    mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, 
    &MyPositionControllerNode::MultiDofJointTrajectoryCallback, this);
  odometry_sub_ = nh_.subscribe("/airsim_node/drone_1/pose_gt", 1, 
    &MyPositionControllerNode::OdometryCallback, this);
  motor_pwm_reference_pub_ = nh_.advertise<my_control::RotorPWM>(
    "/airsim_node/drone_1/rotor_pwm_cmd", 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), 
    &MyPositionControllerNode::TimedCommandCallback, this, true, false);
}

MyPositionControllerNode::~MyPositionControllerNode() {}

void MyPositionControllerNode::InitializeParams() {
  // 初始化参数
  // 获取无人机参数
  //  position_gain velocity_gain attitude_gain angular_rate_gain 
  GetRosParameter(private_nh_, "position_gain/x",
                  my_position_controller_.controller_parameters_.position_gain_.x(),
                  &my_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(private_nh_, "position_gain/y",
                  my_position_controller_.controller_parameters_.position_gain_.y(),
                  &my_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(private_nh_, "position_gain/z",
                  my_position_controller_.controller_parameters_.position_gain_.z(),
                  &my_position_controller_.controller_parameters_.position_gain_.z());  
  GetRosParameter(private_nh_, "velocity_gain/x",
                  my_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &my_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(private_nh_, "velocity_gain/y",
                  my_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &my_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(private_nh_, "velocity_gain/z",
                  my_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &my_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(private_nh_, "attitude_gain/x",
                  my_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &my_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  my_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &my_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  my_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &my_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  my_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &my_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  my_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &my_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  my_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &my_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(private_nh_, &my_position_controller_.vehicle_parameters_);
  my_position_controller_.InitializeParameters();
}

void MyPositionControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  my_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void MyPositionControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = trajectory_reference_msg->points.size();
  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(trajectory_reference_msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = trajectory_reference_msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = trajectory_reference_msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  my_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
} 

void MyPositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {
  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  my_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void MyPositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  ROS_INFO_ONCE("MyPositionController got first odometry message.");
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  my_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  my_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  // 将actuator_msg转换为FourPWM
  my_control::RotorPWM pwm_msg;
  pwm_msg.rotorPWM0 = ref_rotor_velocities[0];
  pwm_msg.rotorPWM1 = ref_rotor_velocities[1];
  pwm_msg.rotorPWM2 = ref_rotor_velocities[2];
  pwm_msg.rotorPWM3 = ref_rotor_velocities[3];
  motor_pwm_reference_pub_.publish(pwm_msg);
}

}  // namespace my_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_position_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  my_control::MyPositionControllerNode my_position_controller_node(nh, private_nh);

  ros::spin();
  return 0;
} 