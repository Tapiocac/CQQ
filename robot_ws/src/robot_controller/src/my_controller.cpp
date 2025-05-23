#include "my_pid_effort_controller/pid_effort_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace my_pid_effort_controller {

PIDEffortController::PIDEffortController() {}

controller_interface::CallbackReturn PIDEffortController::on_init() {
  auto logger = get_node()->get_logger();
  if (!get_node()->get_parameter("joints", joint_names_)) {
    RCLCPP_ERROR(logger, "Joint list 'joints' is not set!");
    return controller_interface::CallbackReturn::ERROR;
  }
  get_node()->declare_parameter("kp", 5.0);
  get_node()->declare_parameter("ki", 0.1);
  get_node()->declare_parameter("kd", 0.01);

  kp_ = get_node()->get_parameter("kp").as_double();
  ki_ = get_node()->get_parameter("ki").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();

  targets_.resize(joint_names_.size(), 0.0);
  integrals_.resize(joint_names_.size(), 0.0);
  prev_errors_.resize(joint_names_.size(), 0.0);

  target_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/targets", 10,
    std::bind(&PIDEffortController::target_callback, this, std::placeholders::_1)
  );

  return controller_interface::CallbackReturn::SUCCESS;
}

void PIDEffortController::target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != targets_.size()) return;
  for (size_t i = 0; i < targets_.size(); ++i) {
    targets_[i] = msg->data[i];
  }
}

controller_interface::InterfaceConfiguration PIDEffortController::command_interface_configuration() const {
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    [this]() {
      std::vector<std::string> interfaces;
      for (const auto &j : joint_names_)
        interfaces.push_back(j + "/effort");
      return interfaces;
    }()
  };
}

controller_interface::InterfaceConfiguration PIDEffortController::state_interface_configuration() const {
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    [this]() {
      std::vector<std::string> interfaces;
      for (const auto &j : joint_names_)
        interfaces.push_back(j + "/velocity");
      return interfaces;
    }()
  };
}

controller_interface::CallbackReturn PIDEffortController::on_configure(const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PIDEffortController::on_activate(const rclcpp_lifecycle::State &) {
  effort_interfaces_.resize(joint_names_.size());
  velocity_interfaces_.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    effort_interfaces_[i] = command_interfaces_[i];
    velocity_interfaces_[i] = state_interfaces_[i];
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PIDEffortController::on_deactivate(const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PIDEffortController::update() {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double error = targets_[i] - velocity_interfaces_[i].get_value();
    integrals_[i] += error;
    double derivative = error - prev_errors_[i];
    prev_errors_[i] = error;
    double effort = kp_ * error + ki_ * integrals_[i] + kd_ * derivative;
    effort_interfaces_[i].set_value(effort);
  }

  return controller_interface::return_type::OK;
}

}  // namespace my_pid_effort_controller

PLUGINLIB_EXPORT_CLASS(my_pid_effort_controller::PIDEffortController, controller_interface::ControllerInterface)
