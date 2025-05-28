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

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

// static constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
// static constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
// static constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
// static constexpr auto odom_frame_id = "odom";
// static constexpr auto base_frame_id = "base_link";

using ControllerReferenceMsg = robot_controller::MyController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  // msg->joint_names = joint_names;
  // msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  // msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  // msg->duration = std::numeric_limits<double>::quiet_NaN();

  // 只需要设置data，长度等于关节数，初值为NaN(未赋值)
  msg->data.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

}  // namespace

namespace robot_controller
{
MyController::MyController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MyController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<my_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // try
  // {
  //   // Explicitly set the interface parameter declared by the forward_command_controller
  //   // to match the value set in the JointGroupEffortController constructor.
  //   get_node()->set_parameter(
  //     rclcpp::Parameter("interface_name", hardware_interface::HW_IF_EFFORT));
  // }
  // catch (const std::exception & e)
  // {
  //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  //   return controller_interface::CallbackReturn::ERROR;
  // }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!params_.state_joints.empty())
  {
    state_joints_ = params_.state_joints;
  }
  else
  {
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
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber 接收数据,放入input_ref_中
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&MyController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  input_ref_.writeFromNonRT(msg);

  // auto set_slow_mode_service_callback =
  //   [&](
  //     const std::shared_ptr<ControllerModeSrvType::Request> request,
  //     std::shared_ptr<ControllerModeSrvType::Response> response)
  // {
  //   if (request->data)
  //   {
  //     control_mode_.writeFromNonRT(control_mode_type::SLOW);
  //   }
  //   else
  //   {
  //     control_mode_.writeFromNonRT(control_mode_type::FAST);
  //   }
  //   response->success = true;
  // };

  // set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
  //   "~/set_slow_control_mode", set_slow_mode_service_callback,
  //   rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  // 轮子状态将在update中填充
  // state_publisher_->lock();
  // state_publisher_->msg_.header.frame_id = params_.joints[0];
  // state_publisher_->unlock();

  // 初始化PID控制器
  pid_controllers_.resize(params_.joints.size());
  target_velocities_.resize(params_.joints.size(), 0.0);
  current_velocities_.resize(params_.joints.size(), 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

// 记录接收到的指令
void MyController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->data.size() == params_.joints.size())
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->data.size(), params_.joints.size());
  }
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
  // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
  // `on_activate` method in `JointTrajectoryController` for exemplary use of
  // `controller_interface::get_ordered_interfaces` helper function

  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

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
  
  auto current_ref = input_ref_.readFromRT();


  // 1. 读取当前速度
  size_t num_joints = params_.joints.size();
  size_t num_state_itfs = params_.state_interfaces.size();
  for (size_t i = 0; i < num_joints; ++i)
  {
    for (size_t j = 0; j < num_state_itfs; ++j)
    {
      if (params_.state_interfaces[j] == "velocity")
      {
        current_velocities_[i] = state_interfaces_[i * num_state_itfs + j].get_value();
      }
    }
  }

  // 2. 读取目标速度
  for (size_t i = 0; i < num_joints; ++i)
  {
    if (!std::isnan((*current_ref)->data[i]))
    {
      target_velocities_[i] = (*current_ref)->data[i];
      (*current_ref)->data[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }


  // 3. PID计算并下发力矩
  for (size_t i = 0; i < num_joints; ++i)
  {
    double error = target_velocities_[i] - current_velocities_[i];
    double effort = pid_controllers_[i].compute(error, period.seconds());
    command_interfaces_[i].set_value(effort);
  }




  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  // 将命令传递给硬件命令接口command_interfaces_，并将命令缓存初始化
  // for (size_t i = 0; i < command_interfaces_.size(); ++i)
  // {
  //   if (!std::isnan((*current_ref)->data[i]))
  //   {
  //     // if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
  //     // {
  //     //   (*current_ref)->displacements[i] /= 2;
  //     // }
  //     command_interfaces_[i].set_value((*current_ref)->data[i]);

  //     (*current_ref)->data[i] = std::numeric_limits<double>::quiet_NaN();
  //   }
  // }

  if (state_publisher_ && state_publisher_->trylock())
  {
    // 填充sensor_msgs::msg::JointState消息
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.name = params_.joints;
    state_publisher_->msg_.position.clear();
    state_publisher_->msg_.velocity.clear();
    state_publisher_->msg_.effort.clear();

    // 读取state_interfaces_中的数据
    for (size_t i = 0; i < state_joints_.size(); ++i)
    {
      for (size_t j = 0; j < params_.state_interfaces.size(); ++j)
      {
        const std::string &iface = params_.state_interfaces[j];
        double value = state_interfaces_[i * params_.state_interfaces.size() + j].get_value();
        if (iface == "position")
        {
          state_publisher_->msg_.position.push_back(value);
        }
        else if (iface == "velocity")
        {
          state_publisher_->msg_.velocity.push_back(value);
        }
        else if (iface == "effort")
        {
          state_publisher_->msg_.effort.push_back(value);
        }
      }
    }



    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace robot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_controller::MyController, controller_interface::ControllerInterface)
