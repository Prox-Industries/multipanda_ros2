#include <franka_example_controllers/subscriber/joint_velocity_controller.hpp>

#include <algorithm>
#include <cassert>
#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  for (int i = 0; i < num_joints; ++i) {
    const bool accepted = command_interfaces_[i].set_value(dq_d_(i));
    (void)accepted;
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("max_abs_velocity", 0.5);
    sub_desired_velocity_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_velocity/commands", 1,
        std::bind(&JointVelocityController::desiredVelocityCallback, this, std::placeholders::_1));
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  max_abs_velocity_ = get_node()->get_parameter("max_abs_velocity").as_double();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_d_.setZero();
  return CallbackReturn::SUCCESS;
}

void JointVelocityController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointVelocityController::desiredVelocityCallback(
    const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Expected %d velocity commands, got %zu", num_joints, msg.data.size());
    return;
  }
  for (int i = 0; i < num_joints; ++i) {
    dq_d_(i) = std::clamp(msg.data[i], -max_abs_velocity_, max_abs_velocity_);
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityController,
                       controller_interface::ControllerInterface)
