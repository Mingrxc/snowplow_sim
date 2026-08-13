// Copyright 2026 Mingrxc
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
    double previous_omega_ref = 0.0;
  };

  void load_parameters();
  bool validate_parameters() const;
  void reset();

  double control_wheel(
    double omega_ref,
    double omega,
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
    double omega_ref);

  double clamp(double value, double lower, double upper) const;

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
