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

#include <limits>
#include <vector>

#include "robot_hardware_interface/my_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface
{
hardware_interface::CallbackReturn MyHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  // info_ 信息来自于机器人ros2_control.xacro描述文件，包含了关节、传感器等信息
  // 计算状态接口总数
  size_t num_state_interfaces = 0;
  for (const auto & joint : info_.joints)
  {
    num_state_interfaces += joint.state_interfaces.size();
  }
  hw_states_.resize(num_state_interfaces, std::numeric_limits<double>::quiet_NaN());

  // 计算命令接口总数
  size_t num_command_interfaces = 0;
  for (const auto & joint : info_.joints)
  {
    num_command_interfaces += joint.command_interfaces.size();
  }
  hw_commands_.resize(num_command_interfaces, std::numeric_limits<double>::quiet_NaN());


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 为所有状态接口设置初始值
  for (double & state : hw_states_)
  {
    state = 0.0; 
  }
  // 为所有命令接口设置初始值
  for (double & command : hw_commands_)
  {
    command = 0.0; 
  }

  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  size_t state_interface_index = 0;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto & interface_info : info_.joints[i].state_interfaces)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, interface_info.name, &hw_states_[state_interface_index++]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  size_t command_interface_index = 0;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto & interface_info : info_.joints[i].command_interfaces)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, interface_info.name, &hw_commands_[command_interface_index++]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  for (double & command : hw_commands_)
  {
    command = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 当使用 gazebo_ros2_control 时，Gazebo 插件会直接更新 hw_states_ 指向的内存。

  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Reading states from Gazebo (handled by gazebo_ros2_control)");
  //打印读取到的第一个关节的第一个状态值 (如果存在)
  if (!hw_states_.empty()) {
    RCLCPP_DEBUG(rclcpp::get_logger("MyHardwareInterface"), "Current state [0]: %f", hw_states_[0]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 当使用 gazebo_ros2_control 时，Gazebo 插件会直接从 hw_commands_ 指向的内存读取命令。

  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Writing commands to Gazebo (handled by gazebo_ros2_control)");
  // 打印将要写入的第一个关节的第一个命令值 (如果存在)
  if (!hw_commands_.empty()) {
   RCLCPP_DEBUG(rclcpp::get_logger("MyHardwareInterface"), "Current command [0]: %f", hw_commands_[0]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::MyHardwareInterface, hardware_interface::SystemInterface)
