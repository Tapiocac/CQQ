// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef ROBOT_CONTROLLER__MY_CONTROLLER_HPP_
#define ROBOT_CONTROLLER__MY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
// #include "my_controller_parameters.hpp" 废弃
#include <robot_controller/my_controller_parameters.hpp> 
#include "robot_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
// #include "realtime_tools/realtime_publisher.h" 废弃
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
// #include "robot_controller/odometry.hpp"

// #include <control_toolbox/pid.hpp>

namespace robot_controller
{


// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

struct Pid
{
  double kp = 1.0;         // 比例系数
  double ki = 0.0;         // 积分系数
  double kd = 0.0;         // 微分系数
  double integral = 0.0;   // 积分项累计
  double prev_error = 0.0; // 上一次误差
  double max_effort = 0.1;// 输出最大限制
  double min_effort = -0.1;// 输出最小限制

  // 计算PID输出
  double compute(double error, double dt)
  {
    integral += error * dt;
    double derivative = (dt > 0) ? (error - prev_error) / dt : 0.0;
    prev_error = error;
    double output = kp * error + ki * integral + kd * derivative;
    return std::clamp(output, min_effort, max_effort);
  }
};

class MyController : public controller_interface::ControllerInterface
{
public:
  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  MyController();

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  // using ControllerReferenceMsg = std_msgs::msg::Float64MultiArray;
  using ControllerReferenceMsg = geometry_msgs::msg::Twist;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = sensor_msgs::msg::JointState;
  // using CmdType = std_msgs::msg::Float64MultiArray;

protected:
  std::vector<std::string> joint_names_;
  std::vector<Pid> pid_controllers_;
  std::vector<double> target_wheel_velocities_; // 目标轮速
  std::vector<double> current_wheel_velocities_; // 当前轮速
  std::mutex mutex_;

  // 机器人硬件参数 (从参数服务器加载)
  double wheel_radius_ = 0.03;     // 轮子半径 (m)
  double wheel_separation_ = 0.14; // 轮距 (m)
  std::shared_ptr<geometry_msgs::msg::Twist> last_received_twist_cmd_;

  // rclcpp::Subscription<CmdType>::SharedPtr cmd_sub_;
  std::shared_ptr<my_controller::ParamListener> param_listener_;
  my_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_ = nullptr;

  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;


  // rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
  // realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;

  std::vector<std::string> command_interface_types_;

  // 里程计相关
  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

private:
  // callback for topic interface
  ROBOT_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace robot_controller

#endif  // ROBOT_CONTROLLER__MY_CONTROLLER_HPP_
