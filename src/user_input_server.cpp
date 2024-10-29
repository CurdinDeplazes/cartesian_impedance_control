#include <chrono>
#include <iostream>
#include <cstdlib>
#include <memory>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_param.hpp"
#include "messages_fr3/srv/set_stiffness.hpp"
#include "cartesian_impedance_control/user_input_server.hpp"
#include "cartesian_impedance_control/cartesian_impedance_controller.hpp"

namespace cartesian_impedance_control{

void UserInputServer::setPose(const std::shared_ptr<messages_fr3::srv::SetPose::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetPose::Response> /*response*/)
{
    (*position_d_target_)[0] = request->x;
    (*position_d_target_)[1] = request->y;
    (*position_d_target_)[2] = request->z;
    (*rotation_d_target_)[0] = request->roll;
    (*rotation_d_target_)[1] = request->pitch;
    (*rotation_d_target_)[2] = request->yaw;
}

void UserInputServer::setParam(const std::shared_ptr<messages_fr3::srv::SetParam::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetParam::Response> /*response*/)
{   
    Eigen::VectorXd diag_values_inertia(6);
    diag_values_inertia << request->a, request->b, request->c, request->d, request->e, request->f;
    auto T_placeholder = diag_values_inertia.asDiagonal();
    (*T_) = T_placeholder;
    for (int i = 0; i < 6; ++i){
        (*D_)(i,i) = 2 * sqrt((*K_)(i,i)*(*T_)(i,i)); 
    }
    std::cout << "Went into setParam" << std::endl;
}

void UserInputServer::SetStiffness(const std::shared_ptr<messages_fr3::srv::SetStiffness::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetStiffness::Response> /*response*/)
{   
    Eigen::VectorXd diag_values_stiffness(6);
    diag_values_stiffness << request->a, request->b, request->c, request->d, request->e, request->f;
    auto K_placeholder = diag_values_stiffness.asDiagonal();
    (*K_) = K_placeholder;
    for (int i = 0; i < 6; ++i){
        (*D_)(i,i) = 2 * sqrt((*K_)(i,i)*(*T_)(i,i)); 
    }
    std::cout << "Went into setStiffness" << std::endl;
}

void UserInputServer::SetMode(const std::shared_ptr<messages_fr3::srv::SetMode::Request> request, 
    std::shared_ptr<messages_fr3::srv::SetMode::Response> /*response*/)
{   
    control_mode_ = request->mode;
    std::cout << "Went into setMode" << std::endl;
}

int UserInputServer::main(int /*argc*/, char** /***argv*/)
{    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("user_input_service");
    
    // Create the set_pose service
    rclcpp::Service<messages_fr3::srv::SetPose>::SharedPtr pose_service =
        node->create_service<messages_fr3::srv::SetPose> 
        ("set_pose", std::bind(&UserInputServer::setPose, this, std::placeholders::_1, std::placeholders::_2));

    // Create the set_param service    
    rclcpp::Service<messages_fr3::srv::SetParam>::SharedPtr param_service =
        node->create_service<messages_fr3::srv::SetParam> 
        ("set_param", std::bind(&UserInputServer::setParam, this, std::placeholders::_1, std::placeholders::_2));
    
    // Create the set_stiffness service    
    rclcpp::Service<messages_fr3::srv::SetStiffness>::SharedPtr stiffness_service =
        node->create_service<messages_fr3::srv::SetStiffness> 
        ("set_stiffness", std::bind(&UserInputServer::SetStiffness, this, std::placeholders::_1, std::placeholders::_2));

    // Create the set_mode service    
    rclcpp::Service<messages_fr3::srv::SetMode>::SharedPtr mode_service =
        node->create_service<messages_fr3::srv::SetMode> 
        ("set_mode", std::bind(&UserInputServer::SetMode, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to be called.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
}

    
