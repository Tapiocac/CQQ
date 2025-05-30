// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "robot_controller/my_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{  // utility


using ControllerReferenceMsg = robot_controller::MyController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg)
{
  if (!msg)
  {
    msg = std::make_shared<robot_controller::MyController::ControllerReferenceMsg>();
  }
  msg->linear.x = 0.0;
  msg->linear.y = 0.0;
  msg->linear.z = 0.0;
  msg->angular.x = 0.0;
  msg->angular.y = 0.0;
  msg->angular.z = 0.0;
}

}  // namespace

namespace robot_controller
{
MyController::MyController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MyController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<my_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();

    // 从参数服务器加载机器人参数，如果未定义则使用hpp中的默认值
    // wheel_radius_ = get_node()->get_parameter_or("wheel_radius", rclcpp::ParameterValue(wheel_radius_)).as_double();
    // wheel_separation_ = get_node()->get_parameter_or("wheel_separation", rclcpp::ParameterValue(wheel_separation_)).as_double();
    RCLCPP_INFO(get_node()->get_logger(), "Wheel radius: %.3f m, Wheel separation: %.3f m", wheel_radius_, wheel_separation_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return CallbackReturn::ERROR;
  }
  // 假设差速驱动至少需要两个关节
  // if (params_.joints.size() < 2) {
  //     RCLCPP_ERROR(get_node()->get_logger(), "Controller requires at least 2 joints for differential drive. Found %zu.", params_.joints.size());
  //     return CallbackReturn::ERROR;
  // }

  joint_names_ = params_.joints;

  if (!params_.state_joints.empty()) {
    state_joints_ = params_.state_joints;
  } else {
    state_joints_ = params_.joints;
  }

  if (params_.joints.size() != state_joints_.size())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "Size of 'joints' (%zu) and 'state_joints' (%zu) parameters has to be the same!",
      params_.joints.size(), state_joints_.size());
    return CallbackReturn::FAILURE;
  }


  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  // subscribers_qos.keep_last(1);
  // subscribers_qos.best_effort();


  // Reference Subscriber - Nav2 typically publishes to /cmd_vel
  // 初始化订阅者
  std::string cmd_vel_topic = "/cmd_vel"; 
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to command topic: %s", cmd_vel_topic.c_str());
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    cmd_vel_topic, subscribers_qos, // Use reliable QoS for commands
    std::bind(&MyController::reference_callback, this, std::placeholders::_1));

  // State Publisher
  try {
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>("~/joint_states", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during publisher creation: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // 初始化里程计发布者
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  odometry_.setWheelParams(wheel_separation_, wheel_radius_, wheel_radius_);
  odometry_.setVelocityRollingWindowSize(static_cast<size_t>(10));

  // 初始化PID控制器
  pid_controllers_.resize(joint_names_.size());
  // 从参数加载PID值
  for (size_t i = 0; i < joint_names_.size(); ++i) {
      pid_controllers_[i].kp = params_.pid.p;
      pid_controllers_[i].ki = params_.pid.i;
      pid_controllers_[i].kd = params_.pid.d;
      pid_controllers_[i].max_effort = params_.pid.max_effort;
      pid_controllers_[i].min_effort = params_.pid.min_effort;
  }


  target_wheel_velocities_.resize(joint_names_.size(), 0.0);
  current_wheel_velocities_.resize(joint_names_.size(), 0.0);
  
  last_received_twist_cmd_ = std::make_shared<geometry_msgs::msg::Twist>();
  reset_controller_reference_msg(last_received_twist_cmd_);
  input_ref_.writeFromNonRT(last_received_twist_cmd_); // Initialize buffer

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

// 记录接收到的指令
void MyController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
    input_ref_.writeFromNonRT(msg);
    // RCLCPP_DEBUG(get_node()->get_logger(), "New Twist received: lin_x=%.2f, ang_z=%.2f", msg->linear.x, msg->angular.z);
}

controller_interface::InterfaceConfiguration MyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 4轮是分开的
  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + params_.command_interfaces);// "left1_wheel_joint/effort ... right2_wheel_joint/effort"
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 4个轮子，每个轮子有3个状态接口position，velocity，effort
  state_interfaces_config.names.reserve(state_joints_.size() * params_.state_interfaces.size());
  for (const auto & joint : state_joints_)
  {
    for (const auto & iface : params_.state_interfaces)
    {
      state_interfaces_config.names.push_back(joint + "/" + iface);
    }
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn MyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset last command
  auto cmd = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(cmd);
  input_ref_.writeFromNonRT(cmd);
  last_received_twist_cmd_ = cmd;

  // Reset PID controllers
  for (auto & pid : pid_controllers_) {
    pid.integral = 0.0;
    pid.prev_error = 0.0;
  }
  // Initialize target velocities to 0
  std::fill(target_wheel_velocities_.begin(), target_wheel_velocities_.end(), 0.0);
  
  // Set command interfaces to 0
   for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// 将命令传递给command_interfaces_
// 从state_interfaces_中读取当前状态
controller_interface::return_type MyController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto current_twist_cmd_buffer = input_ref_.readFromRT();
  if (current_twist_cmd_buffer && *current_twist_cmd_buffer) {
    last_received_twist_cmd_ = *current_twist_cmd_buffer;
  }

  double linear_vel_x = last_received_twist_cmd_->linear.x;
  double angular_vel_z = last_received_twist_cmd_->angular.z;

  // 1. 读取当前轮速 (从 state_interfaces_)
  // Assumes state_interfaces are ordered per joint, and velocity is one of them.
  size_t num_cmd_joints = joint_names_.size(); // Joints we command
  size_t num_state_itfs_per_joint = params_.state_interfaces.size();

  for (size_t i = 0; i < num_cmd_joints; ++i) { // Iterate through commandable joints
    bool vel_found = false;
    // Find corresponding state joint index (can be different if state_joints_ != joint_names_)
    size_t state_joint_idx = i; // Simplified: assumes 1-to-1 mapping for now
                                // A more robust way is to find joint_names_[i] in state_joints_

    for (size_t j = 0; j < num_state_itfs_per_joint; ++j) {
      if (state_joints_[state_joint_idx] == joint_names_[i] && // ensure we are looking at the correct joint's state
          params_.state_interfaces[j] == hardware_interface::HW_IF_VELOCITY) {
        current_wheel_velocities_[i] = state_interfaces_[state_joint_idx * num_state_itfs_per_joint + j].get_value();
        vel_found = true;
        break;
      }
    }
     if (!vel_found) {
        // RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Velocity state interface not found for joint %s or mapping issue.", joint_names_[i].c_str());
     }
  }


  // 2. 根据Twist指令计算目标轮速 (差速驱动模型)
  //    v_left = (linear_x - angular_z * wheel_separation / 2.0)
  //    v_right = (linear_x + angular_z * wheel_separation / 2.0)
  //    Target wheel angular velocity = v_wheel / wheel_radius
  //    Assumes joint_names_[0] is left, joint_names_[1] is right for a 2-wheel diff drive.
  //    This needs to be more robust if you have >2 joints or different naming.
  if (num_cmd_joints >= 2 && wheel_radius_ > 1e-3) { // Avoid division by zero
    double target_vel_left_linear = linear_vel_x - (angular_vel_z * wheel_separation_ / 2.0);
    double target_vel_right_linear = linear_vel_x + (angular_vel_z * wheel_separation_ / 2.0);

    // Convert linear wheel velocity to angular wheel velocity
    target_wheel_velocities_[0] = target_vel_left_linear / wheel_radius_; // Left wheel (e.g., joint_names_[0])
    target_wheel_velocities_[2] = target_vel_right_linear / wheel_radius_; // Right wheel (e.g., joint_names_[1])

    // If 4WD skid steer, all left wheels get left speed, all right wheels get right speed
    if (num_cmd_joints == 4) {
        // Example: joint_names_ = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"}
        // This mapping depends on your URDF and joint_names_ order.
        // A common setup:
        target_wheel_velocities_[1] = target_wheel_velocities_[0]; // Rear left
        target_wheel_velocities_[3] = target_wheel_velocities_[2]; // Rear right
    }
  } else if (wheel_radius_ <= 1e-3) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Wheel radius is too small or zero!");
  }


  // 3. PID计算并下发力矩 (或速度，取决于command_interface类型)
  for (size_t i = 0; i < num_cmd_joints; ++i) {
    double error = target_wheel_velocities_[i] - current_wheel_velocities_[i];
    double command_value = pid_controllers_[i].compute(error, period.seconds());
    command_interfaces_[i].set_value(command_value);
    // command_interfaces_[i].set_value(0.01);
    // RCLCPP_DEBUG(get_node()->get_logger(), "Joint %s: Target=%.2f, Current=%.2f, Err=%.2f, Cmd=%.2f",
    //             joint_names_[i].c_str(), target_wheel_velocities_[i], current_wheel_velocities_[i], error, command_value);
  }

  // 4. 发布状态 (sensor_msgs::msg::JointState)
  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.name.resize(state_joints_.size());
    state_publisher_->msg_.position.assign(state_joints_.size(), std::numeric_limits<double>::quiet_NaN());
    state_publisher_->msg_.velocity.assign(state_joints_.size(), std::numeric_limits<double>::quiet_NaN());
    state_publisher_->msg_.effort.assign(state_joints_.size(), std::numeric_limits<double>::quiet_NaN());

    for (size_t i = 0; i < state_joints_.size(); ++i) {
      state_publisher_->msg_.name[i] = state_joints_[i];
      for (size_t j = 0; j < num_state_itfs_per_joint; ++j) {
        const auto & iface_name = params_.state_interfaces[j];
        double value = state_interfaces_[i * num_state_itfs_per_joint + j].get_value();
        if (iface_name == hardware_interface::HW_IF_POSITION) {
          state_publisher_->msg_.position[i] = value;
        } else if (iface_name == hardware_interface::HW_IF_VELOCITY) {
          state_publisher_->msg_.velocity[i] = value;
        } else if (iface_name == hardware_interface::HW_IF_EFFORT) {
          state_publisher_->msg_.effort[i] = value;
        }
      }
    }
    state_publisher_->unlockAndPublish();
  }

  // 5. 发布里程计 (geometry_msgs::msg::Odometry)

  odometry_.updateOpenLoop(linear_vel_x, angular_vel_z, time);

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }
  return controller_interface::return_type::OK;
}

}  // namespace robot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_controller::MyController, controller_interface::ControllerInterface)
