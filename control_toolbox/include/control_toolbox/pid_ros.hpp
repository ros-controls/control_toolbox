// Copyright (c) 2020, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef CONTROL_TOOLBOX__PID_ROS_HPP_
#define CONTROL_TOOLBOX__PID_ROS_HPP_

#include <memory>
#include <string>

#include "control_msgs/msg/pid_state.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "control_toolbox/pid.hpp"

namespace control_toolbox
{

class PidROS
{
public:
  /*!
   * \brief Constructor of PidROS class.
   *
   * The node is passed to this class to handler the ROS parameters, this class allows
   * to add a prefix to the pid parameters
   *
   * \param node ROS node
   * \param prefix prefix to add to the pid parameters.
   *               Per default is prefix interpreted as prefix for topics.
   * \param prefix_is_for_params provided prefix should be interpreted as prefix for parameters.
   *        If the parameter is `true` then "/" in the middle of the string will not be replaced
   *        with "." for parameters prefix. "/" or "~/" at the beginning will be removed.
   *
   */
  template <class NodeT>
  explicit PidROS(
    std::shared_ptr<NodeT> node_ptr, std::string prefix = std::string(""),
    bool prefix_is_for_params = false)
  : PidROS(
      node_ptr->get_node_base_interface(), node_ptr->get_node_logging_interface(),
      node_ptr->get_node_parameters_interface(), node_ptr->get_node_topics_interface(), prefix,
      prefix_is_for_params)
  {
  }

  PidROS(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    std::string prefix = std::string(""), bool prefix_is_for_params = false);

  /*!
   * \brief Initialize the PID controller and set the parameters
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   *
   * \note New gains are not applied if i_min_ > i_max_
   */
  [[deprecated("Use initialize_from_args with AntiwindupStrategy instead.")]]
  void initialize_from_args(
    double p, double i, double d, double i_max, double i_min, bool antiwindup);

  /*!
   * \brief Initialize the PID controller and set the parameters
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   * \param save_i_term save integrator output between resets.
   *
   * \note New gains are not applied if i_min_ > i_max_
   */
  [[deprecated("Use initialize_from_args with AntiwindupStrategy instead.")]]
  void initialize_from_args(
    double p, double i, double d, double i_max, double i_min, bool antiwindup, bool save_i_term);

  /*!
   * \brief Initialize the PID controller and set the parameters
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param trk_tc Specifies the tracking time constant for the 'back_calculation' strategy. If set
   *    to 0.0 when this strategy is selected, a recommended default value will be applied.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior. When a strategy other
        than 'none' is selected, it will override the controller's default anti-windup behavior.
   * \deprecated{only when `antiwindup_strat == AntiwindupStrategy::NONE`:}
   *     Old anti-windup technique is deprecated and will be removed by
   *     the ROS 2 Kilted Kaiju release.
   * \warning{If you pass `AntiwindupStrategy::NONE`, at runtime a warning will be printed:}
   *     `"Old anti-windup technique is deprecated. This option will be removed by the ROS 2 Kilted Kaiju release."`
   * \param save_i_term save integrator output between resets.
   *
   * \note New gains are not applied if i_min_ > i_max_ or if u_min_ > u_max_.
   */
  [[deprecated("Use initialize_from_args with AntiwindupStrategy only.")]]
  void initialize_from_args(
    double p, double i, double d, double i_max, double i_min, double u_max, double u_min,
    double trk_tc, bool antiwindup, AntiwindupStrategy antiwindup_strat, bool save_i_term);

  /*!
   * \brief Initialize the PID controller and set the parameters.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param trk_tc Specifies the tracking time constant for the 'back_calculation' strategy. If set
   *    to 0.0 when this strategy is selected, a recommended default value will be applied.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   * \param save_i_term save integrator output between resets.
   *
   * \note New gains are not applied if u_min_ > u_max_.
   */
  void initialize_from_args(
    double p, double i, double d, double u_max, double u_min, double trk_tc,
    AntiwindupStrategy antiwindup_strat, bool save_i_term);

  /*!
   * \brief Initialize the PID controller based on already set parameters
   * \return True if all parameters are set (p, i, d, i_max, i_min, u_max, u_min and trk_tc), False otherwise
   */
  bool initialize_from_ros_parameters();

  /*!
   * \brief Reset the state of this PID controller
   *
   * @note save_i_term parameter is read from ROS parameters
   */
  void reset();

  /*!
   * \brief Reset the state of this PID controller
   *
   * \param save_i_term boolean indicating if integral term is retained on reset()
   */
  void reset(bool save_i_term);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time since last call in seconds
   *
   * \returns PID command
   */
  double compute_command(double error, const rclcpp::Duration & dt);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call in seconds
   *
   * \returns PID command
   */
  double compute_command(double error, double error_dot, const rclcpp::Duration & dt);

  /*!
   * \brief Get PID gains for the controller.
   * \return gains A struct of the PID gain values
   */
  Pid::Gains get_gains();

  /*!
   * \brief Set PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Antiwindup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   *
   * \note New gains are not applied if i_min > i_max
   */
  [[deprecated("Use set_gains with AntiwindupStrategy instead.")]]
  void set_gains(double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

  /*!
   * \brief Initialize the PID controller and set the parameters
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param trk_tc Specifies the tracking time constant for the 'back_calculation' strategy. If set
   *    to 0.0 when this strategy is selected, a recommended default value will be applied.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior. When a strategy other
        than 'none' is selected, it will override the controller's default anti-windup behavior.
   * \deprecated{only when `antiwindup_strat == AntiwindupStrategy::NONE`:}
   *     Old anti-windup technique is deprecated and will be removed by
   *     the ROS 2 Kilted Kaiju release.
   * \warning{If you pass `AntiwindupStrategy::NONE`, at runtime a warning will be printed:}
   *   `"Old anti-windup technique is deprecated. This option will be removed by
   *     the ROS 2 Kilted Kaiju release."`
   *
   * \note New gains are not applied if i_min > i_max or if u_min_ > u_max_.
   */
  [[deprecated("Use set_gains with AntiwindupStrategy only.")]]
  void set_gains(
    double p, double i, double d, double i_max, double i_min, double u_max, double u_min,
    double trk_tc = 0.0, bool antiwindup = false,
    AntiwindupStrategy antiwindup_strat = AntiwindupStrategy::NONE);

  /*!
   * \brief Set PID gains for the controller (preferred).
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param trk_tc Specifies the tracking time constant for the 'back_calculation' strategy. If set
   *    to 0.0 when this strategy is selected, a recommended default value will be applied.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   *
   * \note New gains are not applied if u_min_ > u_max_.
   */
  void set_gains(
    double p, double i, double d, double u_max, double u_min, double trk_tc,
    AntiwindupStrategy antiwindup_strat);

  /*!
   * \brief Set PID gains for the controller.
   * \param gains A struct of the PID gain values
   *
   * \note New gains are not applied if gains.i_min_ > gains.i_max_
   */
  void set_gains(const Pid::Gains & gains);

  /*!
   * \brief Set current command for this PID controller
   * \param cmd command to set
   */
  void set_current_cmd(double cmd);

  /*!
   * \brief Return current command for this PID controller
   * \return current cmd
   */
  double get_current_cmd();

  /*!
   * \brief Return PID state publisher
   * \return shared_ptr to the PID state publisher
   */
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> get_pid_state_publisher();

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe[out] The proportional error.
   * \param ie[out] The weighted integral error.
   * \param de[out] The derivative error.
   */
  void get_current_pid_errors(double & pe, double & ie, double & de);

  /*!
   * \brief Print to console the current parameters
   */
  void print_values();

  /*!
   * \brief Return PID parameters callback handle
   * \return shared_ptr to the PID parameters callback handle
   */
  inline rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  get_parameters_callback_handle()
  {
    return parameter_callback_;
  }

protected:
  std::string topic_prefix_;
  std::string param_prefix_;

private:
  void set_parameter_event_callback();

  void publish_pid_state(double cmd, double error, rclcpp::Duration dt);

  void declare_param(const std::string & param_name, rclcpp::ParameterValue param_value);

  bool get_double_param(const std::string & param_name, double & value);

  bool get_boolean_param(const std::string & param_name, bool & value);

  bool get_string_param(const std::string & param_name, std::string & value);

  /*!
   * \brief Set prefix for topic and parameter names
   * \param[in] topic_prefix prefix to add to the pid parameters.
   *               Per default is prefix interpreted as prefix for topics.
   *               If not stated explicitly using "/" or "~", prefix is interpreted as global, i.e.,
   *               "/" will be added in front of topic prefix
   */
  void set_prefixes(const std::string & topic_prefix);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;

  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::PidState>> rt_state_pub_;
  control_msgs::msg::PidState pid_state_msg_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> state_pub_;

  Pid pid_;
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__PID_ROS_HPP_
