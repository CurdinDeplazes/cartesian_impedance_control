#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <messages_fr3/srv/set_pose.hpp>
#include <messages_fr3/srv/set_param.hpp>
#include <messages_fr3/srv/set_stiffness.hpp>
#include <messages_fr3/srv/set_mode.hpp> 


namespace cartesian_impedance_control {

class UserInputServer {
public:
  UserInputServer (Eigen::Vector3d* position, 
                   Eigen::Vector3d* rotation, 
                   Eigen::Matrix<double, 6, 6>* stiffness, 
                   Eigen::Matrix<double, 6, 6>* damping, 
                   Eigen::Matrix<double, 6, 6>* inertia,
                   bool mode = false) :
  control_mode_(mode),
  position_d_target_(position),
  rotation_d_target_(rotation),
  K_(stiffness),
  D_(damping),
  T_(inertia){}
  int main(int argc, char **argv);

private:
  Eigen::Vector3d* position_d_target_;
  Eigen::Vector3d* rotation_d_target_;
  Eigen::Matrix<double, 6, 6>* K_;
  Eigen::Matrix<double, 6, 6>* D_;
  Eigen::Matrix<double, 6, 6>* T_;
  bool control_mode_ = false;
  
  //Eigen::VectorXd diag_values_inertia(6);
  void setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetPose::Response> response);

  void setParam(const std::shared_ptr<messages_fr3::srv::SetParam::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetParam::Response> response);

  void SetStiffness(const std::shared_ptr<messages_fr3::srv::SetStiffness::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetStiffness::Response> response);

  void SetMode(const std::shared_ptr<messages_fr3::srv::SetMode::Request> request,
    std::shared_ptr<messages_fr3::srv::SetMode::Response> response);
};

}

