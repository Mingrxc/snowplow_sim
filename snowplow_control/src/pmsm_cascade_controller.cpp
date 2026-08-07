#include "snowplow_control/pmsm_cascade_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace snowplow_control
{

namespace
{
constexpr double kSmall = 1.0e-9;
}

PMSMController::PMSMController()
: velocity_p_(0.10),
  velocity_i_(1.50),
  current_p_(0.50),
  current_i_(10.0),
  feedforward_j_hat_(3.4),
  feedforward_b_hat_(1.25),
  feedforward_tc_(0.25),
  feedforward_observer_gain_(50.0),
  wheel_radius_(0.15),
  track_separation_(0.65),
  wheel_torque_constant_(0.06),
  max_torque_(30.0),
  target_linear_velocity_(0.0),
  target_angular_velocity_(0.0)
{
}

controller_interface::CallbackReturn PMSMController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints",
      {
        "left_drive_wheel_joint",
        "right_drive_wheel_joint",
        "left_idler_wheel_joint",
        "right_idler_wheel_joint",
      });
    auto_declare<double>("velocity_p", velocity_p_);
    auto_declare<double>("velocity_i", velocity_i_);
    auto_declare<double>("current_p", current_p_);
    auto_declare<double>("current_i", current_i_);
    auto_declare<double>("feedforward_j_hat", feedforward_j_hat_);
    auto_declare<double>("feedforward_b_hat", feedforward_b_hat_);
    auto_declare<double>("feedforward_tc", feedforward_tc_);
    auto_declare<double>("feedforward_observer_gain", feedforward_observer_gain_);
    auto_declare<double>("wheel_radius", wheel_radius_);
    auto_declare<double>("track_separation", track_separation_);
    auto_declare<double>("wheel_torque_constant", wheel_torque_constant_);
    auto_declare<double>("max_torque", max_torque_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PMSMController::on_configure(
  const rclcpp_lifecycle::State &)
{
  load_parameters();
  if (!validate_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      target_linear_velocity_.store(message->linear.x);
      target_angular_velocity_.store(message->angular.z);
    });

  std::string joint_list;
  for (const auto & joint_name : joint_names_) {
    if (!joint_list.empty()) {
      joint_list += ", ";
    }
    joint_list += joint_name;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured PMSMController for joints [%s], r=%.3f m, B=%.3f m, max torque=%.2f Nm",
    joint_list.c_str(), wheel_radius_, track_separation_, max_torque_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PMSMController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto expected_commands = joint_names_.size();
  const auto expected_states = joint_names_.size() * 3;
  if (command_interfaces_.size() != expected_commands ||
    state_interfaces_.size() != expected_states)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces and %zu state interfaces, got %zu and %zu",
      expected_commands, expected_states,
      command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  reset();
  RCLCPP_INFO(get_node()->get_logger(), "PMSMController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PMSMController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  reset();
  RCLCPP_INFO(get_node()->get_logger(), "PMSMController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PMSMController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
PMSMController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::return_type PMSMController::update(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= kSmall) {
    return controller_interface::return_type::OK;
  }

  const double omega_left = state_interfaces_[1].get_value();
  const double omega_right = state_interfaces_[4].get_value();

  const double linear = target_linear_velocity_.load();
  const double angular = target_angular_velocity_.load();
  const double half_track = track_separation_ * 0.5;
  const double omega_ref_left = (linear - angular * half_track) / wheel_radius_;
  const double omega_ref_right = (linear + angular * half_track) / wheel_radius_;

  const double command_left = control_wheel(
    omega_ref_left, omega_left, dt, left_);
  const double command_right = control_wheel(
    omega_ref_right, omega_right, dt, right_);
  const double command_scale = joint_names_.size() == 4 ? 0.5 : 1.0;
  const double front_left_command = command_left * command_scale;
  const double front_right_command = command_right * command_scale;

  command_interfaces_[0].set_value(front_left_command);
  command_interfaces_[1].set_value(front_right_command);
  if (joint_names_.size() == 4) {
    command_interfaces_[2].set_value(front_left_command);
    command_interfaces_[3].set_value(front_right_command);
  }
  left_.previous_effort = command_left;
  right_.previous_effort = command_right;

  return controller_interface::return_type::OK;
}

void PMSMController::load_parameters()
{
  auto node = get_node();
  joint_names_ = node->get_parameter("joints").as_string_array();
  velocity_p_ = node->get_parameter("velocity_p").as_double();
  velocity_i_ = node->get_parameter("velocity_i").as_double();
  current_p_ = node->get_parameter("current_p").as_double();
  current_i_ = node->get_parameter("current_i").as_double();
  feedforward_j_hat_ = node->get_parameter("feedforward_j_hat").as_double();
  feedforward_b_hat_ = node->get_parameter("feedforward_b_hat").as_double();
  feedforward_tc_ = node->get_parameter("feedforward_tc").as_double();
  feedforward_observer_gain_ =
    node->get_parameter("feedforward_observer_gain").as_double();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  track_separation_ = node->get_parameter("track_separation").as_double();
  wheel_torque_constant_ = node->get_parameter("wheel_torque_constant").as_double();
  max_torque_ = node->get_parameter("max_torque").as_double();
}

bool PMSMController::validate_parameters() const
{
  if (joint_names_.size() != 2 && joint_names_.size() != 4) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'joints' must contain two drive joints or four track proxy joints");
    return false;
  }
  if (wheel_radius_ <= kSmall || track_separation_ <= kSmall ||
    wheel_torque_constant_ <= kSmall || max_torque_ <= kSmall)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "wheel_radius, track_separation, wheel_torque_constant, and max_torque must be positive");
    return false;
  }
  return true;
}

void PMSMController::reset()
{
  left_ = WheelMemory{};
  right_ = WheelMemory{};
  target_linear_velocity_.store(0.0);
  target_angular_velocity_.store(0.0);
}

double PMSMController::control_wheel(
  double omega_ref,
  double omega,
  double dt,
  WheelMemory & memory)
{
  // Gazebo's effort command interface expects wheel torque directly. The
  // current loop remains in the Python PMSM prototype; using Gazebo effort
  // state as current feedback makes the simulated torque alternate signs.
  const double velocity_torque = pi_step(
    omega_ref, omega,
    velocity_p_, velocity_i_,
    dt, -max_torque_, max_torque_,
    memory.speed_integral);

  const double d_omega_ref_dt = (omega_ref - memory.previous_omega_ref) / dt;
  const double feedforward_torque = compute_feedforward(
    d_omega_ref_dt, omega, memory.previous_effort, dt, memory);

  const double effort_command = clamp(
    velocity_torque + feedforward_torque,
    -max_torque_, max_torque_);

  memory.previous_omega_ref = omega_ref;
  return effort_command;
}

double PMSMController::pi_step(
  double setpoint,
  double measurement,
  double kp,
  double ki,
  double dt,
  double output_min,
  double output_max,
  double & integral) const
{
  const double error = setpoint - measurement;
  const double old_integral = integral;
  integral += ki * error * dt;

  double output = kp * error + integral;
  if (output > output_max) {
    output = output_max;
    integral = old_integral;
  } else if (output < output_min) {
    output = output_min;
    integral = old_integral;
  }
  return output;
}

double PMSMController::compute_feedforward(
  double d_omega_ref_dt,
  double omega,
  double electrical_torque,
  double dt,
  WheelMemory & memory)
{
  const double sign = (omega > 0.0) - (omega < 0.0);
  const double friction_torque = feedforward_tc_ * sign + feedforward_b_hat_ * omega;
  const double acceleration_torque = feedforward_j_hat_ * d_omega_ref_dt;
  const double d_omega_dt = (omega - memory.previous_omega) / dt;

  const double load_error =
    electrical_torque - memory.load_torque_hat - feedforward_j_hat_ * d_omega_dt;
  memory.load_torque_hat += feedforward_observer_gain_ * load_error * dt;
  memory.load_torque_hat = clamp(memory.load_torque_hat, -max_torque_, max_torque_);
  memory.previous_omega = omega;

  return clamp(
    acceleration_torque + friction_torque + memory.load_torque_hat,
    -max_torque_, max_torque_);
}

double PMSMController::clamp(double value, double lower, double upper) const
{
  return std::min(std::max(value, lower), upper);
}

}  // namespace snowplow_control

PLUGINLIB_EXPORT_CLASS(
  snowplow_control::PMSMController,
  controller_interface::ControllerInterface)
