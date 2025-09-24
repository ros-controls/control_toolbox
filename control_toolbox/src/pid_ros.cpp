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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_toolbox/pid_ros.hpp"

namespace control_toolbox
{
constexpr double UMAX_INFINITY = std::numeric_limits<double>::infinity();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
PidROS::PidROS(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface, std::string prefix,
  bool prefix_is_for_params)
: node_base_(node_base),
  node_logging_(node_logging),
  node_params_(node_params),
  topics_interface_(topics_interface)
{
  // note: deprecation on templated constructor does not show up
  RCLCPP_WARN(
    node_logging->get_logger(),
    "PidROS constructor with node and prefix is deprecated, use overloads with explicit "
    "prefixes for params and topics");

  if (prefix_is_for_params)
  {
    param_prefix_ = prefix;
    // If it starts with a "~", remove it
    if (param_prefix_.compare(0, 1, "~") == 0)
    {
      param_prefix_.erase(0, 1);
    }
    // If it starts with a "/" or a "~/", remove those as well
    if (param_prefix_.compare(0, 1, "/") == 0)
    {
      param_prefix_.erase(0, 1);
    }
    // Add a trailing "."
    if (!param_prefix_.empty() && param_prefix_.back() != '.')
    {
      param_prefix_.append(".");
    }

    topic_prefix_ = prefix;
    // Replace parameter separator from "." to "/" in topics
    std::replace(topic_prefix_.begin(), topic_prefix_.end(), '.', '/');
    // Add a trailing "/"
    if (!topic_prefix_.empty() && topic_prefix_.back() != '/')
    {
      topic_prefix_.append("/");
    }
    // Add global namespace if none is defined
    if (topic_prefix_.compare(0, 1, "~") != 0 && topic_prefix_.compare(0, 1, "/") != 0)
    {
      topic_prefix_ = "/" + topic_prefix_;
    }
  }
  else
  {
    set_prefixes(prefix);
  }

  state_pub_ = rclcpp::create_publisher<control_msgs::msg::PidState>(
    topics_interface_, topic_prefix_ + "pid_state", rclcpp::SensorDataQoS());
  rt_state_pub_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));
}

PidROS::PidROS(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const std::string & param_prefix, const std::string & topic_prefix, bool activate_state_publisher)
: topic_prefix_(topic_prefix),
  param_prefix_(param_prefix),
  node_base_(node_base),
  node_logging_(node_logging),
  node_params_(node_params),
  topics_interface_(topics_interface)
{
  // Add a trailing "."
  if (!param_prefix_.empty() && param_prefix_.back() != '.')
  {
    param_prefix_.append(".");
  }
  // Add a trailing "/"
  if (!topic_prefix_.empty() && topic_prefix_.back() != '/')
  {
    topic_prefix_.append("/");
  }

  if (activate_state_publisher)
  {
    state_pub_ = rclcpp::create_publisher<control_msgs::msg::PidState>(
      topics_interface_, topic_prefix_ + "pid_state", rclcpp::SensorDataQoS());
    rt_state_pub_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));
  }
}
#pragma GCC diagnostic pop

void PidROS::set_prefixes(const std::string & topic_prefix)
{
  param_prefix_ = topic_prefix;
  // If it starts with a "~", remove it
  if (param_prefix_.compare(0, 1, "~") == 0)
  {
    param_prefix_.erase(0, 1);
  }
  // If it starts with a "/" or a "~/", remove those as well
  if (param_prefix_.compare(0, 1, "/") == 0)
  {
    param_prefix_.erase(0, 1);
  }
  // Replace namespacing separator from "/" to "." in parameters
  std::replace(param_prefix_.begin(), param_prefix_.end(), '/', '.');
  // Add a trailing "."
  if (!param_prefix_.empty() && param_prefix_.back() != '.')
  {
    param_prefix_.append(".");
  }

  topic_prefix_ = topic_prefix;
  // Replace parameter separator from "." to "/" in topics
  std::replace(topic_prefix_.begin(), topic_prefix_.end(), '.', '/');
  // Add a trailing "/"
  if (!topic_prefix_.empty() && topic_prefix_.back() != '/')
  {
    topic_prefix_.append("/");
  }
}

bool PidROS::get_boolean_param(const std::string & param_name, bool & value)
{
  declare_param(param_name, rclcpp::ParameterValue(value));
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name))
  {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_BOOL != param.get_type())
    {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Wrong parameter type '%s', not boolean", param_name.c_str());
      return false;
    }
    value = param.as_bool();
    return true;
  }
  else
  {
    return false;
  }
}

// TODO(anyone): to-be-removed once this functionality becomes supported by the param API directly
bool PidROS::get_double_param(const std::string & param_name, double & value)
{
  declare_param(param_name, rclcpp::ParameterValue(value));
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name))
  {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_DOUBLE != param.get_type())
    {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Wrong parameter type '%s', not double", param_name.c_str());
      return false;
    }
    value = param.as_double();
    RCLCPP_DEBUG_STREAM(
      node_logging_->get_logger(), "parameter '" << param_name << "' in node '"
                                                 << node_base_->get_name() << "' value is " << value
                                                 << std::endl);
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(), "parameter '" << param_name << "' in node '"
                                                 << node_base_->get_name() << "' does not exists"
                                                 << std::endl);
    return false;
  }
}

bool PidROS::get_string_param(const std::string & param_name, std::string & value)
{
  declare_param(param_name, rclcpp::ParameterValue(value));
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name))
  {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_STRING != param.get_type())
    {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Wrong parameter type '%s', not string", param_name.c_str());
      return false;
    }
    value = param.as_string();
    RCLCPP_DEBUG_STREAM(
      node_logging_->get_logger(), "parameter '" << param_name << "' in node '"
                                                 << node_base_->get_name() << "' value is " << value
                                                 << std::endl);
    return true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(), "parameter '" << param_name << "' in node '"
                                                 << node_base_->get_name() << "' does not exists"
                                                 << std::endl);
    return false;
  }
}

bool PidROS::initialize_from_ros_parameters()
{
  double p, i, d, i_max, i_min, u_max, u_min, tracking_time_constant, error_deadband;
  p = i = d = i_max = i_min = tracking_time_constant = std::numeric_limits<double>::quiet_NaN();
  error_deadband = std::numeric_limits<double>::epsilon();
  u_max = UMAX_INFINITY;
  u_min = -UMAX_INFINITY;
  bool antiwindup = false;
  std::string antiwindup_strat_str = "legacy";
  bool all_params_available = true;

  all_params_available &= get_double_param(param_prefix_ + "p", p);
  all_params_available &= get_double_param(param_prefix_ + "i", i);
  all_params_available &= get_double_param(param_prefix_ + "d", d);
  all_params_available &= get_double_param(param_prefix_ + "i_clamp_max", i_max);
  all_params_available &= get_double_param(param_prefix_ + "i_clamp_min", i_min);
  all_params_available &= get_double_param(param_prefix_ + "u_clamp_max", u_max);
  all_params_available &= get_double_param(param_prefix_ + "u_clamp_min", u_min);
  all_params_available &= get_double_param(param_prefix_ + "error_deadband", error_deadband);
  all_params_available &=
    get_double_param(param_prefix_ + "tracking_time_constant", tracking_time_constant);

  bool saturation = std::isfinite(u_max) || std::isfinite(u_min);
  get_boolean_param(param_prefix_ + "saturation", saturation);
  if (!saturation)
  {
    u_max = UMAX_INFINITY;
    u_min = -UMAX_INFINITY;
  }
  get_boolean_param(param_prefix_ + "antiwindup", antiwindup);
  get_string_param(param_prefix_ + "antiwindup_strategy", antiwindup_strat_str);
  declare_param(param_prefix_ + "save_i_term", rclcpp::ParameterValue(false));
  declare_param(
    param_prefix_ + "activate_state_publisher", rclcpp::ParameterValue(rt_state_pub_ != nullptr));

  if (all_params_available)
  {
    set_parameter_event_callback();
  }

  RCLCPP_WARN_EXPRESSION(
    node_logging_->get_logger(), antiwindup_strat_str == "legacy",
    "Using the legacy anti-windup technique is deprecated. This option will be removed by the ROS "
    "2 Kilted Kaiju release.");

  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.set_type(antiwindup_strat_str);
  antiwindup_strat.i_max = i_max;
  antiwindup_strat.i_min = i_min;
  antiwindup_strat.tracking_time_constant = tracking_time_constant;
  antiwindup_strat.legacy_antiwindup = antiwindup;
  antiwindup_strat.error_deadband = error_deadband;

  try
  {
    antiwindup_strat.validate();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node_logging_->get_logger(), "Invalid antiwindup strategy: %s", e.what());
    return false;
  }

  if (pid_.initialize(p, i, d, u_max, u_min, antiwindup_strat))
  {
    return all_params_available;
  }

  return false;
}

void PidROS::declare_param(const std::string & param_name, rclcpp::ParameterValue param_value)
{
  if (!node_params_->has_parameter(param_name))
  {
    node_params_->declare_parameter(param_name, param_value);
  }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
bool PidROS::initialize_from_args(
  double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = i_max;
  antiwindup_strat.i_min = i_min;
  antiwindup_strat.legacy_antiwindup = antiwindup;

  return initialize_from_args(p, i, d, UMAX_INFINITY, -UMAX_INFINITY, antiwindup_strat, false);
}
#pragma GCC diagnostic pop

bool PidROS::initialize_from_args(
  double p, double i, double d, double i_max, double i_min, bool antiwindup, bool save_i_term)
{
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = i_max;
  antiwindup_strat.i_min = i_min;
  antiwindup_strat.legacy_antiwindup = antiwindup;

  return initialize_from_args(
    p, i, d, UMAX_INFINITY, -UMAX_INFINITY, antiwindup_strat, save_i_term);
}

bool PidROS::initialize_from_args(
  double p, double i, double d, double u_max, double u_min,
  const AntiWindupStrategy & antiwindup_strat, bool save_i_term)
{
  Pid::Gains verify_gains(p, i, d, u_max, u_min, antiwindup_strat);
  std::string error_msg = "";
  if (!verify_gains.validate(error_msg))
  {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Received invalid gains: %s. Skipping new gains.",
      error_msg.c_str());
    return false;
  }
  else
  {
    if (pid_.initialize(p, i, d, u_max, u_min, antiwindup_strat))
    {
      const Pid::Gains gains = pid_.get_gains();
      declare_param(param_prefix_ + "p", rclcpp::ParameterValue(gains.p_gain_));
      declare_param(param_prefix_ + "i", rclcpp::ParameterValue(gains.i_gain_));
      declare_param(param_prefix_ + "d", rclcpp::ParameterValue(gains.d_gain_));
      declare_param(
        param_prefix_ + "i_clamp_max", rclcpp::ParameterValue(gains.antiwindup_strat_.i_max));
      declare_param(
        param_prefix_ + "i_clamp_min", rclcpp::ParameterValue(gains.antiwindup_strat_.i_min));
      declare_param(param_prefix_ + "u_clamp_max", rclcpp::ParameterValue(gains.u_max_));
      declare_param(param_prefix_ + "u_clamp_min", rclcpp::ParameterValue(gains.u_min_));
      declare_param(
        param_prefix_ + "antiwindup",
        rclcpp::ParameterValue(gains.antiwindup_strat_.legacy_antiwindup));
      declare_param(
        param_prefix_ + "tracking_time_constant",
        rclcpp::ParameterValue(antiwindup_strat.tracking_time_constant));
      declare_param(
        param_prefix_ + "error_deadband", rclcpp::ParameterValue(antiwindup_strat.error_deadband));
      declare_param(
        param_prefix_ + "saturation",
        rclcpp::ParameterValue(std::isfinite(gains.u_max_) || std::isfinite(gains.u_min_)));
      declare_param(
        param_prefix_ + "antiwindup_strategy",
        rclcpp::ParameterValue(gains.antiwindup_strat_.to_string()));
      declare_param(param_prefix_ + "save_i_term", rclcpp::ParameterValue(save_i_term));
      declare_param(
        param_prefix_ + "activate_state_publisher",
        rclcpp::ParameterValue(rt_state_pub_ != nullptr));

      set_parameter_event_callback();
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_logging_->get_logger(), "Failed to initialize PID controller");
      return false;
    }
  }
  return false;
}

void PidROS::reset()
{
  bool save_i_term = false;
  get_boolean_param(param_prefix_ + "save_i_term", save_i_term);
  reset(save_i_term);
}

void PidROS::reset(bool save_i_term) { pid_.reset(save_i_term); }

std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> PidROS::get_pid_state_publisher()
{
  return state_pub_;
}

double PidROS::compute_command(double error, const rclcpp::Duration & dt)
{
  double cmd = pid_.compute_command(error, dt);
  publish_pid_state(cmd, error, dt);
  return cmd;
}

double PidROS::compute_command(double error, double error_dot, const rclcpp::Duration & dt)
{
  double cmd = pid_.compute_command(error, error_dot, dt);
  publish_pid_state(cmd, error, dt);
  return cmd;
}

Pid::Gains PidROS::get_gains() { return pid_.get_gains(); }

bool PidROS::set_gains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = i_max;
  antiwindup_strat.i_min = i_min;
  antiwindup_strat.legacy_antiwindup = antiwindup;
  return set_gains(p, i, d, UMAX_INFINITY, -UMAX_INFINITY, antiwindup_strat);
}

bool PidROS::set_gains(
  double p, double i, double d, double u_max, double u_min,
  const AntiWindupStrategy & antiwindup_strat)
{
  Pid::Gains gains(p, i, d, u_max, u_min, antiwindup_strat);

  return set_gains(gains);
}

bool PidROS::set_gains(const Pid::Gains & gains)
{
  std::string error_msg = "";
  if (!gains.validate(error_msg))
  {
    RCLCPP_ERROR(
      node_logging_->get_logger(), "Received invalid gains: %s. Skipping new gains.",
      error_msg.c_str());
    return false;
  }
  else
  {
    if (pid_.set_gains(gains))
    {
      node_params_->set_parameters(
        {rclcpp::Parameter(param_prefix_ + "p", gains.p_gain_),
         rclcpp::Parameter(param_prefix_ + "i", gains.i_gain_),
         rclcpp::Parameter(param_prefix_ + "d", gains.d_gain_),
         rclcpp::Parameter(param_prefix_ + "i_clamp_max", gains.antiwindup_strat_.i_max),
         rclcpp::Parameter(param_prefix_ + "i_clamp_min", gains.antiwindup_strat_.i_min),
         rclcpp::Parameter(param_prefix_ + "u_clamp_max", gains.u_max_),
         rclcpp::Parameter(param_prefix_ + "u_clamp_min", gains.u_min_),
         rclcpp::Parameter(
           param_prefix_ + "tracking_time_constant",
           gains.antiwindup_strat_.tracking_time_constant),
         rclcpp::Parameter(param_prefix_ + "antiwindup", gains.antiwindup_strat_.legacy_antiwindup),
         rclcpp::Parameter(
           param_prefix_ + "error_deadband", gains.antiwindup_strat_.error_deadband),
         rclcpp::Parameter(param_prefix_ + "saturation", true),
         rclcpp::Parameter(
           param_prefix_ + "antiwindup_strategy", gains.antiwindup_strat_.to_string())});
      return true;
    }
  }
  return false;
}

void PidROS::publish_pid_state(double cmd, double error, rclcpp::Duration dt)
{
  Pid::Gains gains = pid_.get_gains_rt();

  double p_error, i_term, d_error;
  get_current_pid_errors(p_error, i_term, d_error);

  // Publish controller state if configured
  if (rt_state_pub_)
  {
    pid_state_msg_.header.stamp = rclcpp::Clock().now();
    pid_state_msg_.timestep = dt;
    pid_state_msg_.error = error;
    pid_state_msg_.error_dot = d_error;
    pid_state_msg_.p_error = p_error;
    pid_state_msg_.i_error = i_term;
    pid_state_msg_.d_error = d_error;
    pid_state_msg_.p_term = gains.p_gain_;
    pid_state_msg_.i_term = gains.i_gain_;
    pid_state_msg_.d_term = gains.d_gain_;
    pid_state_msg_.i_max = gains.i_max_;
    pid_state_msg_.i_min = gains.i_min_;
    pid_state_msg_.output = cmd;
    rt_state_pub_->try_publish(pid_state_msg_);
  }
}

void PidROS::set_current_cmd(double cmd) { pid_.set_current_cmd(cmd); }

double PidROS::get_current_cmd() { return pid_.get_current_cmd(); }

void PidROS::get_current_pid_errors(double & pe, double & ie, double & de)
{
  double _pe, _ie, _de;
  pid_.get_current_pid_errors(_pe, _ie, _de);
  pe = _pe;
  ie = _ie;
  de = _de;
}

void PidROS::print_values()
{
  Pid::Gains gains = pid_.get_gains();

  double p_error, i_term, d_error;
  get_current_pid_errors(p_error, i_term, d_error);

  RCLCPP_INFO_STREAM(
    node_logging_->get_logger(),
    "Current Values of PID template:\n"
      << "  P Gain:       " << gains.p_gain_ << "\n"
      << "  I Gain:       " << gains.i_gain_ << "\n"
      << "  D Gain:       " << gains.d_gain_ << "\n"
      << "  I Max:        " << gains.i_max_ << "\n"
      << "  I Min:        " << gains.i_min_ << "\n"
      << "  U_Max:                  " << gains.u_max_ << "\n"
      << "  U_Min:                  " << gains.u_min_ << "\n"
      << "  Tracking_Time_Constant: " << gains.antiwindup_strat_.tracking_time_constant << "\n"
      << "  Antiwindup:             " << gains.antiwindup_strat_.legacy_antiwindup << "\n"
      << "  Antiwindup_Strategy:    " << gains.antiwindup_strat_.to_string() << "\n"
      << "\n"
      << "  P Error:      " << p_error << "\n"
      << "  I Term:       " << i_term << "\n"
      << "  D Error:      " << d_error << "\n"
      << "  Command:      " << get_current_cmd(););
}

void PidROS::set_parameter_event_callback()
{
  auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    Pid::Gains gains = pid_.get_gains();
    bool changed = false;
    // The saturation parameter is special, it can change the u_min and u_max parameters
    // so we need to check it first and then proceed with the loop, as if we update only one
    // parameter, we need to keep this logic up-to-date. So, do not move it inside the loop
    bool saturation = true;  // default value
    try
    {
      // we can't use get_parameter_or, because we don't have access to a rclcpp::Node
      if (node_params_->has_parameter(param_prefix_ + "saturation"))
      {
        saturation = node_params_->get_parameter(param_prefix_ + "saturation").get_value<bool>();
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(
        node_logging_->get_logger(), "Error with saturation parameter: " << e.what());
    }

    std::for_each(
      parameters.begin(), parameters.end(),
      [&, this](const rclcpp::Parameter & parameter)
      {
        if (parameter.get_name() == param_prefix_ + "saturation")
        {
          saturation = parameter.get_value<bool>();
          changed = true;
          if (!saturation)
          {
            RCLCPP_WARN(
              node_logging_->get_logger(),
              "Saturation is set to false, Changing the u_min and u_max to -inf and inf");
            gains.u_min_ = -UMAX_INFINITY;
            gains.u_max_ = UMAX_INFINITY;
          }
          else
          {
            RCLCPP_INFO(
              node_logging_->get_logger(),
              "Saturation is set to true, using u_min and u_max from parameters");
            gains.u_min_ =
              node_params_->get_parameter(param_prefix_ + "u_clamp_min").get_value<double>();
            gains.u_max_ =
              node_params_->get_parameter(param_prefix_ + "u_clamp_max").get_value<double>();
          }
          return;
        }
      });

    for (auto & parameter : parameters)
    {
      const std::string param_name = parameter.get_name();
      try
      {
        if (param_name == param_prefix_ + "p")
        {
          gains.p_gain_ = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "i")
        {
          gains.i_gain_ = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "d")
        {
          gains.d_gain_ = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "i_clamp_max")
        {
          gains.i_max_ = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "i_clamp_min")
        {
          gains.i_min_ = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "u_clamp_max")
        {
          gains.u_max_ = saturation ? parameter.get_value<double>() : UMAX_INFINITY;
          RCLCPP_WARN_EXPRESSION(
            node_logging_->get_logger(), !saturation,
            "Saturation is set to false, Changing the u_clamp_max inf");
          changed = true;
        }
        else if (param_name == param_prefix_ + "u_clamp_min")
        {
          gains.u_min_ = saturation ? parameter.get_value<double>() : -UMAX_INFINITY;
          RCLCPP_WARN_EXPRESSION(
            node_logging_->get_logger(), !saturation,
            "Saturation is set to false, Changing the u_clamp_min -inf");
          changed = true;
        }
        else if (param_name == param_prefix_ + "tracking_time_constant")
        {
          gains.antiwindup_strat_.tracking_time_constant = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "antiwindup")
        {
          gains.antiwindup_strat_.legacy_antiwindup = parameter.get_value<bool>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "error_deadband")
        {
          gains.antiwindup_strat_.error_deadband = parameter.get_value<double>();
          changed = true;
        }
        else if (param_name == param_prefix_ + "activate_state_publisher")
        {
          if (parameter.get_value<bool>())
          {
            std::string topic_name = topic_prefix_ + "pid_state";
            RCLCPP_INFO(
              node_logging_->get_logger(), "Activate publisher: `%s` ...", topic_name.c_str());
            state_pub_ = rclcpp::create_publisher<control_msgs::msg::PidState>(
              topics_interface_, topic_name, rclcpp::SensorDataQoS());
            rt_state_pub_.reset(
              new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));
          }
          else
          {
            RCLCPP_INFO(node_logging_->get_logger(), "Deactivate publisher...");
            state_pub_.reset();
            rt_state_pub_.reset();
          }
        }
      }
      catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
      {
        RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Please use the right type: " << e.what());
      }
    }

    if (changed)
    {
      pid_.set_gains(gains);
    }

    return result;
  };
  /// @note this gets called whenever a parameter changes.
  /// Any parameter under that node. Not just PidROS.
  parameter_callback_ = node_params_->add_on_set_parameters_callback(on_parameter_event_callback);
}

// TODO(christophfroehlich): Remove deprecated functions
// BEGIN DEPRECATED
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void PidROS::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  initialize_from_args(p, i, d, i_max, i_min, antiwindup);
}

void PidROS::initPid(
  double p, double i, double d, double i_max, double i_min, bool antiwindup, bool save_i_term)
{
  initialize_from_args(p, i, d, i_max, i_min, antiwindup, save_i_term);
}

bool PidROS::initPid() { return initialize_from_ros_parameters(); }

double PidROS::computeCommand(double error, rclcpp::Duration dt)
{
  double cmd = pid_.compute_command(error, dt);
  publish_pid_state(cmd, error, dt);
  return cmd;
}

double PidROS::computeCommand(double error, double error_dot, rclcpp::Duration dt)
{
  double cmd = pid_.compute_command(error, error_dot, dt);
  publish_pid_state(cmd, error, dt);
  return cmd;
}

Pid::Gains PidROS::getGains() { return get_gains(); }
void PidROS::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  set_gains(p, i, d, i_max, i_min, antiwindup);
}

void PidROS::setGains(const Pid::Gains & gains) { set_gains(gains); }

void PidROS::setCurrentCmd(double cmd) { set_current_cmd(cmd); }

double PidROS::getCurrentCmd() { return get_current_cmd(); }

std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> PidROS::getPidStatePublisher()
{
  return get_pid_state_publisher();
}

void PidROS::getCurrentPIDErrors(double & pe, double & ie, double & de)
{
  get_current_pid_errors(pe, ie, de);
}

void PidROS::printValues() { print_values(); }

void PidROS::setParameterEventCallback() { set_parameter_event_callback(); }

void PidROS::publishPIDState(double cmd, double error, rclcpp::Duration dt)
{
  publish_pid_state(cmd, error, dt);
}

void PidROS::declareParam(const std::string & param_name, rclcpp::ParameterValue param_value)
{
  declare_param(param_name, param_value);
}

bool PidROS::getDoubleParam(const std::string & param_name, double & value)
{
  return get_double_param(param_name, value);
}

bool PidROS::getBooleanParam(const std::string & param_name, bool & value)
{
  return get_boolean_param(param_name, value);
}

void PidROS::initialize(std::string topic_prefix) { set_prefixes(topic_prefix); }
#pragma GCC diagnostic pop
// END DEPRECATED

}  // namespace control_toolbox
