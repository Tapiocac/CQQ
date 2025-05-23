#pragma once

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace my_pid_effort_controller {

class PIDEffortController : public controller_interface::ControllerInterface
{
public:
  PIDEffortController();
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update() override;

private:
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::LoanedCommandInterface> effort_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> velocity_interfaces_;

  double kp_, ki_, kd_;
  std::vector<double> integrals_;
  std::vector<double> prev_errors_;
  std::vector<double> targets_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
  void target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace my_pid_effort_controller
