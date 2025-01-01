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

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_toolbox/pid_ros.hpp"

namespace control_toolbox
{

PidROS::PidROS(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  std::string prefix, bool prefix_is_for_params)
: node_base_(node_base), node_logging_(node_logging), node_params_(node_params),
  topics_interface_(topics_interface)
{
  if (prefix_is_for_params)
  {
    param_prefix_ = prefix;
    // If it starts with a "~", remove it
    if (param_prefix_.compare(0, 1, "~") == 0) {
      param_prefix_.erase(0, 1);
    }
    // If it starts with a "/" or a "~/", remove those as well
    if (param_prefix_.compare(0, 1, "/") == 0) {
      param_prefix_.erase(0, 1);
    }
    // Add a trailing "."
    if (!param_prefix_.empty() && param_prefix_.back() != '.') {
      param_prefix_.append(".");
    }

    topic_prefix_ = prefix;
    // Replace parameter separator from "." to "/" in topics
    std::replace(topic_prefix_.begin(), topic_prefix_.end(), '.', '/');
    // Add a trailing "/"
    if (!topic_prefix_.empty() && topic_prefix_.back() != '/') {
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
    initialize(prefix);
  }

  state_pub_ = rclcpp::create_publisher<control_msgs::msg::PidState>(
    topics_interface_, topic_prefix_ + "pid_state", rclcpp::SensorDataQoS());
  rt_state_pub_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));
}

void PidROS::initialize(std::string topic_prefix)
{
  param_prefix_ = topic_prefix;
  // If it starts with a "~", remove it
  if (param_prefix_.compare(0, 1, "~") == 0) {
    param_prefix_.erase(0, 1);
  }
  // If it starts with a "/" or a "~/", remove those as well
  if (param_prefix_.compare(0, 1, "/") == 0) {
    param_prefix_.erase(0, 1);
  }
  // Replace namespacing separator from "/" to "." in parameters
  std::replace(param_prefix_.begin(), param_prefix_.end(), '/', '.');
  // Add a trailing "."
  if (!param_prefix_.empty() && param_prefix_.back() != '.') {
    param_prefix_.append(".");
  }

  topic_prefix_ = topic_prefix;
  // Replace parameter separator from "." to "/" in topics
  std::replace(topic_prefix_.begin(), topic_prefix_.end(), '.', '/');
  // Add a trailing "/"
  if (!topic_prefix_.empty() && topic_prefix_.back() != '/') {
    topic_prefix_.append("/");
  }
}

bool PidROS::getBooleanParam(const std::string & param_name, bool & value)
{
  declareParam(param_name, rclcpp::ParameterValue(value));
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name)) {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_BOOL != param.get_type()) {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Wrong parameter type '%s', not boolean", param_name.c_str());
      return false;
    }
    value = param.as_bool();
    return true;
  } else {
    return false;
  }
}

// TODO(anyone): to-be-removed once this functionality becomes supported by the param API directly
bool PidROS::getDoubleParam(const std::string & param_name, double & value)
{
  declareParam(param_name, rclcpp::ParameterValue(value));
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name)) {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_DOUBLE != param.get_type()) {
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
  } else {
    RCLCPP_ERROR_STREAM(
      node_logging_->get_logger(), "parameter '" << param_name << "' in node '"
                                                 << node_base_->get_name() << "' does not exists"
                                                 << std::endl);
    return false;
  }
}

bool PidROS::initPid()
{
  double p, i, d, i_min, i_max;
  p = i = d = i_min = i_max = std::numeric_limits<double>::quiet_NaN();
  bool antiwindup = false;
  bool all_params_available = true;
  all_params_available &= getDoubleParam(param_prefix_ + "p", p);
  all_params_available &= getDoubleParam(param_prefix_ + "i", i);
  all_params_available &= getDoubleParam(param_prefix_ + "d", d);
  all_params_available &= getDoubleParam(param_prefix_ + "i_clamp_max", i_max);
  all_params_available &= getDoubleParam(param_prefix_ + "i_clamp_min", i_min);

  getBooleanParam(param_prefix_ + "antiwindup", antiwindup);

  if (all_params_available) {
    setParameterEventCallback();
  }

  pid_.initPid(p, i, d, i_max, i_min, antiwindup);

  return all_params_available;
}

void PidROS::declareParam(const std::string & param_name, rclcpp::ParameterValue param_value)
{
  if (!node_params_->has_parameter(param_name)) {
    node_params_->declare_parameter(param_name, param_value);
  }
}

void PidROS::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  if (i_min > i_max) {
    RCLCPP_ERROR(node_logging_->get_logger(), "received i_min > i_max, skip new gains");
  } else {
    pid_.initPid(p, i, d, i_max, i_min, antiwindup);

    declareParam(param_prefix_ + "p", rclcpp::ParameterValue(p));
    declareParam(param_prefix_ + "i", rclcpp::ParameterValue(i));
    declareParam(param_prefix_ + "d", rclcpp::ParameterValue(d));
    declareParam(param_prefix_ + "i_clamp_max", rclcpp::ParameterValue(i_max));
    declareParam(param_prefix_ + "i_clamp_min", rclcpp::ParameterValue(i_min));
    declareParam(param_prefix_ + "antiwindup", rclcpp::ParameterValue(antiwindup));

    setParameterEventCallback();
  }
}

void PidROS::reset() { pid_.reset(); }

std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> PidROS::getPidStatePublisher()
{
  return state_pub_;
}

double PidROS::computeCommand(double error, rclcpp::Duration dt)
{
  double cmd_ = pid_.computeCommand(error, static_cast<uint64_t>(dt.nanoseconds()));
  publishPIDState(cmd_, error, dt);

  return cmd_;
}

double PidROS::computeCommand(double error, double error_dot, rclcpp::Duration dt)
{
  double cmd_ = pid_.computeCommand(error, error_dot, static_cast<uint64_t>(dt.nanoseconds()));
  publishPIDState(cmd_, error, dt);

  return cmd_;
}

Pid::Gains PidROS::getGains() { return pid_.getGains(); }

void PidROS::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  if (i_min > i_max) {
    RCLCPP_ERROR(node_logging_->get_logger(), "received i_min > i_max, skip new gains");
  } else {
    node_params_->set_parameters(
      {rclcpp::Parameter(param_prefix_ + "p", p), rclcpp::Parameter(param_prefix_ + "i", i),
      rclcpp::Parameter(param_prefix_ + "d", d),
      rclcpp::Parameter(param_prefix_ + "i_clamp_max", i_max),
      rclcpp::Parameter(param_prefix_ + "i_clamp_min", i_min),
      rclcpp::Parameter(param_prefix_ + "antiwindup", antiwindup)});

    pid_.setGains(p, i, d, i_max, i_min, antiwindup);
  }
}

void PidROS::publishPIDState(double cmd, double error, rclcpp::Duration dt)
{
  Pid::Gains gains = pid_.getGains();

  double p_error_, i_error_, d_error_;
  getCurrentPIDErrors(p_error_, i_error_, d_error_);

  // Publish controller state if configured
  if (rt_state_pub_) {
    if (rt_state_pub_->trylock()) {
      rt_state_pub_->msg_.header.stamp = rclcpp::Clock().now();
      rt_state_pub_->msg_.timestep = dt;
      rt_state_pub_->msg_.error = error;
      rt_state_pub_->msg_.error_dot = pid_.getDerivativeError();
      rt_state_pub_->msg_.p_error = p_error_;
      rt_state_pub_->msg_.i_error = i_error_;
      rt_state_pub_->msg_.d_error = d_error_;
      rt_state_pub_->msg_.p_term = gains.p_gain_;
      rt_state_pub_->msg_.i_term = gains.i_gain_;
      rt_state_pub_->msg_.d_term = gains.d_gain_;
      rt_state_pub_->msg_.i_max = gains.i_max_;
      rt_state_pub_->msg_.i_min = gains.i_min_;
      rt_state_pub_->msg_.output = cmd;
      rt_state_pub_->unlockAndPublish();
    }
  }
}

void PidROS::setCurrentCmd(double cmd) { pid_.setCurrentCmd(cmd); }

double PidROS::getCurrentCmd() { return pid_.getCurrentCmd(); }

void PidROS::getCurrentPIDErrors(double & pe, double & ie, double & de)
{
  double _pe, _ie, _de;
  pid_.getCurrentPIDErrors(_pe, _ie, _de);
  pe = _pe;
  ie = _ie;
  de = _de;
}

void PidROS::printValues()
{
  Pid::Gains gains = pid_.getGains();

  double p_error_, i_error_, d_error_;
  getCurrentPIDErrors(p_error_, i_error_, d_error_);

  RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Current Values of PID template:\n"
                                                    << "  P Gain:       " << gains.p_gain_ << "\n"
                                                    << "  I Gain:       " << gains.i_gain_ << "\n"
                                                    << "  D Gain:       " << gains.d_gain_ << "\n"
                                                    << "  I_Max:        " << gains.i_max_ << "\n"
                                                    << "  I_Min:        " << gains.i_min_ << "\n"
                                                    << "  Antiwindup:   " << gains.antiwindup_
                                                    << "\n"
                                                    << "  P_Error:      " << p_error_ << "\n"
                                                    << "  I_Error:      " << i_error_ << "\n"
                                                    << "  D_Error:      " << d_error_ << "\n"
                                                    << "  Command:      " << getCurrentCmd(););
}

void PidROS::setGains(const Pid::Gains & gains)
{
  if (gains.i_min_ > gains.i_max_) {
    RCLCPP_ERROR(node_logging_->get_logger(), "received i_min > i_max, skip new gains");
  } else {
  pid_.setGains(gains);
  }
}

void PidROS::setParameterEventCallback()
{
  auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    /// @note don't use getGains, it's rt
    Pid::Gains gains = pid_.getGains();
    bool changed = false;

    for (auto & parameter : parameters) {
      const std::string param_name = parameter.get_name();
      try {
        if (param_name == param_prefix_ + "p") {
          gains.p_gain_ = parameter.get_value<double>();
          changed = true;
        } else if (param_name == param_prefix_ + "i") {
          gains.i_gain_ = parameter.get_value<double>();
          changed = true;
        } else if (param_name == param_prefix_ + "d") {
          gains.d_gain_ = parameter.get_value<double>();
          changed = true;
        } else if (param_name == param_prefix_ + "i_clamp_max") {
          gains.i_max_ = parameter.get_value<double>();
          changed = true;
        } else if (param_name == param_prefix_ + "i_clamp_min") {
          gains.i_min_ = parameter.get_value<double>();
          changed = true;
        } else if (param_name == param_prefix_ + "antiwindup") {
          gains.antiwindup_ = parameter.get_value<bool>();
          changed = true;
        }
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        RCLCPP_INFO_STREAM(node_logging_->get_logger(), "Please use the right type: " << e.what());
      }
    }

    if (changed) {
      /// @note don't call setGains() from inside a callback
      if (gains.i_min_ > gains.i_max_) {
        RCLCPP_ERROR(node_logging_->get_logger(), "received i_min > i_max, skip new gains");
      } else {
          pid_.setGains(gains);
      }
    }

    return result;
  };
  /// @note this gets called whenever a parameter changes.
  /// Any parameter under that node. Not just PidROS.
  parameter_callback_ = node_params_->add_on_set_parameters_callback(on_parameter_event_callback);
}

}  // namespace control_toolbox
