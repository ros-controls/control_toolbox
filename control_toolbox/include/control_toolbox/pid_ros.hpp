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
   * \param node Any ROS node
   * \param param_prefix prefix to add to the pid parameters.
   * \param topic_prefix prefix to add to the state publisher.
   *      If it starts with `~/`, topic will be local under the namespace of the node.
   *      If it starts with `/` or an alphanumeric character, topic will be in global namespace.
   *
   * state publisher is not activated if topic_prefix is empty,
   *
   */
  template <class NodeT>
  explicit PidROS(
    std::shared_ptr<NodeT> node_ptr, const std::string & param_prefix,
    const std::string & topic_prefix)
  : PidROS(
      node_ptr->get_node_base_interface(), node_ptr->get_node_logging_interface(),
      node_ptr->get_node_parameters_interface(), node_ptr->get_node_topics_interface(),
      param_prefix, topic_prefix, !topic_prefix.empty())
  {
  }

  /*!
   * \brief Constructor of PidROS class.
   *
   * The node is passed to this class to handler the ROS parameters, this class allows
   * to add a prefix to the pid parameters
   *
   * \param node Any ROS node
   * \param param_prefix prefix to add to the pid parameters.
   * \param topic_prefix prefix to add to the state publisher.
   *      If it starts with `~/`, topic will be local under the namespace of the node.
   *      If it starts with `/` or an alphanumeric character, topic will be in global namespace.
   * \param activate_state_publisher If true, the publisher will be enabled after initialization.
   *
   */
  template <class NodeT>
  explicit PidROS(
    std::shared_ptr<NodeT> node_ptr, std::string param_prefix, std::string topic_prefix,
    bool activate_state_publisher)
  : PidROS(
      node_ptr->get_node_base_interface(), node_ptr->get_node_logging_interface(),
      node_ptr->get_node_parameters_interface(), node_ptr->get_node_topics_interface(),
      param_prefix, topic_prefix, activate_state_publisher)
  {
  }

  /*!
   * \brief Constructor of PidROS class with node_interfaces
   *
   * \param node_base Node base interface pointer.
   * \param node_logging Node logging interface pointer.
   * \param node_params Node parameters interface pointer.
   * \param topics_interface Node topics interface pointer.
   * \param param_prefix Prefix to add to the PID parameters. This string is not manipulated, i.e., probably should end with `.`.
   * \param topic_prefix Prefix to add to the state publisher. This string is not manipulated, i.e., probably should end with `/`. If it starts with `~/`, topic will be local under the namespace of the node. If it starts with `/` or an alphanumeric character, topic will be in global namespace.
   * \param activate_state_publisher If true, the publisher will be enabled after initialization.
   */
  PidROS(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string & param_prefix, const std::string & topic_prefix,
    bool activate_state_publisher);

  /*!
   * \brief Initialize the PID controller and set the parameters.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   * \param save_i_term save integrator output between resets.
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if antiwindup_strat.i_min > antiwindup_strat.i_max or u_min > u_max.
   */
  bool initialize_from_args(
    double p, double i, double d, double u_max, double u_min,
    const AntiWindupStrategy & antiwindup_strat, bool save_i_term);

  /*!
   * \brief Initialize the PID controller based on already set parameters
   * \return True if all parameters are set (p, i, d, i_max, i_min, u_max, u_min), False otherwise
   * \return False if the parameters are not set or if the parameters are invalid
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
   *
   * \note This method is not RT safe
   */
  Pid::Gains get_gains();

  /*!
   * \brief Set PID gains for the controller.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if antiwindup_strat.i_min > antiwindup_strat.i_max or u_min > u_max.
   * \note This method is not RT safe
   */
  bool set_gains(
    double p, double i, double d, double u_max, double u_min,
    const AntiWindupStrategy & antiwindup_strat);

  /*!
   * \brief Set PID gains for the controller.
   * \param gains A struct of the PID gain values
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if gains.i_min_ > gains.i_max_
   * \note This method is not RT safe
   */
  bool set_gains(const Pid::Gains & gains);

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
