#ifndef MY_CONTROL_MY_POSITION_CONTROLLER_H
#define MY_CONTROL_MY_POSITION_CONTROLLER_H

#include "my_control/common.h"
#include "my_control/parameters.h"

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

namespace my_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class MyPositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MyPositionControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class MyPositionController {
 public:
  MyPositionController();
  ~MyPositionController();

  void InitializeParameters();
  void SetOdometry(const EigenOdometry& odometry);
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;
  void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  MyPositionControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;

  void ComputeDesiredAngularAcc_Q(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration) const;
};

}  // namespace my_control

#endif  // MY_CONTROL_MY_POSITION_CONTROLLER_H 