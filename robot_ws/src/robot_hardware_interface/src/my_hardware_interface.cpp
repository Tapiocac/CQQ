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
  
  hw_states_.resize(info_.joints.size() * info_.joints[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size() * info_.joints[0].command_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  joint_names_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    joint_names_[i] = info_.joints[i].name;
  }

  // 从 info_.hardware_parameters 读取硬件特定参数，例如串口号、波特率等
  // 示例:
  // if (info_.hardware_parameters.count("serial_port")) {
  //   serial_port_name_ = info_.hardware_parameters.at("serial_port");
  // }
  // if (info_.hardware_parameters.count("baud_rate")) {
  //   baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  // }

  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "on_init successful");
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


  // --- 初始化硬件连接 ---
  // 例如，打开串口
  // try {
  //   serial_port_.Open(serial_port_name_);
  //   serial_port_.SetBaudRate(static_cast<LibSerial::BaudRate>(baud_rate_));
  //   serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  //   serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  //   serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
  //   serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  // } catch (const LibSerial::OpenFailed& e) {
  //   RCLCPP_FATAL(rclcpp::get_logger("MyHardwareInterface"), "Failed to open serial port %s: %s", serial_port_name_.c_str(), e.what());
  //   return CallbackReturn::ERROR;
  // }
  // RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Serial port %s opened successfully at %d baud.", serial_port_name_.c_str(), baud_rate_);
  // -----------------------


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
      RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Exported state interface: %s/%s", info_.joints[i].name.c_str(), interface_info.name.c_str());
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
      RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Exported command interface: %s/%s", info_.joints[i].name.c_str(), interface_info.name.c_str());
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // --- 激活硬件 ---
  // 例如，发送启动电机或开始数据流的命令
  // std::string activate_cmd = "MOTORS_ON\n";
  // serial_port_.Write(activate_cmd);
  // -----------------

  // 将状态和指令向量初始化为0，以防万一
  for (double & state : hw_states_) { state = 0.0; }
  for (double & command : hw_commands_) { command = 0.0; }

  RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // --- 停用硬件 ---
  // 例如，发送停止电机的命令
  // std::string deactivate_cmd = "MOTORS_OFF\n";
  // serial_port_.Write(deactivate_cmd);
  // -----------------

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

  // --- 从实际硬件读取状态 ---
  // 您需要实现通过您的通信协议（例如串口）从硬件请求并解析数据。
  // 假设您的机器人有4个轮子，每个轮子都有位置、速度、力矩状态。
  // hw_states_ 的顺序与 export_state_interfaces 中定义的顺序一致。

  // 示例伪代码：
  // std::string request_data_cmd = "GET_STATES\n";
  // serial_port_.Write(request_data_cmd);
  // std::string response;
  // try {
  //   serial_port_.ReadLine(response, '\n', 100); // 100ms 超时
  // } catch (const LibSerial::ReadTimeout&) {
  //   RCLCPP_WARN(rclcpp::get_logger("MyHardwareInterface"), "Timeout reading from serial port.");
  //   return hardware_interface::return_type::ERROR;
  // }

  // // 解析 response 字符串，填充 hw_states_
  // // 例如，response 格式可能是 "pos1,vel1,eff1,pos2,vel2,eff2,..."
  // std::stringstream ss(response);
  // std::string token;
  // size_t state_idx = 0;
  // while(std::getline(ss, token, ',')) {
  //   if (state_idx < hw_states_.size()) {
  //     try {
  //       hw_states_[state_idx++] = std::stod(token);
  //     } catch (const std::invalid_argument& ia) {
  //       RCLCPP_ERROR(rclcpp::get_logger("MyHardwareInterface"), "Invalid argument: %s", ia.what());
  //       return hardware_interface::return_type::ERROR;
  //     } catch (const std::out_of_range& oor) {
  //       RCLCPP_ERROR(rclcpp::get_logger("MyHardwareInterface"), "Out of Range error: %s", oor.what());
  //       return hardware_interface::return_type::ERROR;
  //     }
  //   }
  // }
  // if (state_idx != hw_states_.size()){
  //    RCLCPP_WARN(rclcpp::get_logger("MyHardwareInterface"), "Did not receive all expected state values. Got %zu, expected %zu", state_idx, hw_states_.size());
  // }

  // RCLCPP_DEBUG(rclcpp::get_logger("MyHardwareInterface"), "Reading states from hardware.");
  // for (size_t i = 0; i < hw_states_.size(); ++i) {
  //   RCLCPP_DEBUG(rclcpp::get_logger("MyHardwareInterface"), "hw_states_[%zu]: %f", i, hw_states_[i]);
  // }






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


  // --- 将指令写入实际硬件 ---
  // 您需要实现通过您的通信协议将 hw_commands_ 中的指令发送到硬件。
  // hw_commands_ 的顺序与 export_command_interfaces 中定义的顺序一致。
  // 假设您的机器人有4个轮子，每个轮子接收一个力矩指令。

  // 示例伪代码：
  // std::stringstream cmd_stream;
  // cmd_stream << "SET_EFFORTS";
  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //   cmd_stream << "," << hw_commands_[i];
  // }
  // cmd_stream << "\n";
  // serial_port_.Write(cmd_stream.str());

  // RCLCPP_DEBUG(rclcpp::get_logger("MyHardwareInterface"), "Writing commands to hardware.");
  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //   RCLCPP_DEBUG(rclcpp::get_logger("MyHardwareInterface"), "hw_commands_[%zu]: %f", i, hw_commands_[i]);
  // }
  // --------------------------
}

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::MyHardwareInterface, hardware_interface::SystemInterface)
