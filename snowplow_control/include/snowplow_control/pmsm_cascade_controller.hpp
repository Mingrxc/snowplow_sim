#ifndef SNOWPLOW_CONTROL__PMSM_CASCADE_CONTROLLER_HPP_
#define SNOWPLOW_CONTROL__PMSM_CASCADE_CONTROLLER_HPP_

#include <atomic>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace snowplow_control
{

class PMSMController : public controller_interface::ControllerInterface
{
public:
  PMSMController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct WheelMemory
  {
    double speed_integral = 0.0;
    double current_integral = 0.0;
    double load_torque_hat = 0.0;
    double previous_omega = 0.0;
    double previous_omega_ref = 0.0;
    double previous_effort = 0.0;
  };

  void load_parameters();
  bool validate_parameters() const;
  void reset();

  double control_wheel(
    double omega_ref,
    double omega,
    double effort_measured,
    double dt,
    WheelMemory & memory);

  double pi_step(
    double setpoint,
    double measurement,
    double kp,
    double ki,
    double dt,
    double output_min,
    double output_max,
    double & integral) const;

  double compute_feedforward(
    double d_omega_ref_dt,
    double omega,
    double electrical_torque,
    double dt,
    WheelMemory & memory);

  double clamp(double value, double lower, double upper) const;
  double measured_current_from_effort(double effort, double fallback_effort) const;

  std::vector<std::string> joint_names_;

  double velocity_p_;
  double velocity_i_;
  double current_p_;
  double current_i_;
  double feedforward_j_hat_;
  double feedforward_b_hat_;
  double feedforward_tc_;
  double feedforward_observer_gain_;
  double wheel_radius_;
  double track_separation_;
  double wheel_torque_constant_;
  double max_torque_;

  WheelMemory left_;
  WheelMemory right_;

  std::atomic<double> target_linear_velocity_;
  std::atomic<double> target_angular_velocity_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace snowplow_control

#endif  // SNOWPLOW_CONTROL__PMSM_CASCADE_CONTROLLER_HPP_
