/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_toolbox/pid_ros.hpp"

namespace control_toolbox
{

void PidROS::initialize(std::string topic_prefix)
{
  if (topic_prefix.back() != '.' && !topic_prefix.empty()) {
    topic_prefix_ = topic_prefix + ".";
  } else {
    topic_prefix_ = topic_prefix;
  }

  state_pub_ = rclcpp::create_publisher<control_msgs::msg::PidState>(
    topics_interface_,
    topic_prefix + "/pid_state",
    rclcpp::SensorDataQoS());
  rt_state_pub_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));
}

bool
PidROS::getBooleanParam(const std::string & param_name, bool & value)
{
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name)) {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_BOOL != param.get_type()) {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Wrong parameter type '%s', not boolean",
        param_name.c_str());
      return false;
    }
    value = param.as_bool();
    return true;
  } else {
    return false;
  }
}

// TODO(anyone): to-be-removed once this functionality becomes supported by the param API directly
bool
PidROS::getDoubleParam(const std::string & param_name, double & value)
{
  rclcpp::Parameter param;
  if (node_params_->has_parameter(param_name)) {
    node_params_->get_parameter(param_name, param);
    if (rclcpp::PARAMETER_DOUBLE != param.get_type()) {
      RCLCPP_ERROR(
        node_logging_->get_logger(), "Wrong parameter type '%s', not double",
        param_name.c_str());
      return false;
    }
    value = param.as_double();
    RCLCPP_INFO_STREAM(
      node_logging_->get_logger(),
      "parameter '" << param_name << "' in node '" << node_base_->get_name() <<
        "' value is " << value << std::endl);
    return true;
  } else {
    RCLCPP_INFO_STREAM(
      node_logging_->get_logger(),
      "parameter '" << param_name << "' in node '" << node_base_->get_name() <<
        "' does not exists" << std::endl);
    return false;
  }
}

bool
PidROS::initPid()
{
  double p, i, d, i_min, i_max;
  bool antiwindup = false;
  bool all_params_available = true;
  all_params_available &= getDoubleParam(topic_prefix_ + "p", p);
  all_params_available &= getDoubleParam(topic_prefix_ + "i", i);
  all_params_available &= getDoubleParam(topic_prefix_ + "d", d);
  all_params_available &= getDoubleParam(topic_prefix_ + "i_clamp_max", i_max);
  all_params_available &= getDoubleParam(topic_prefix_ + "i_clamp_min", i_min);

  getBooleanParam(topic_prefix_ + "antiwindup", antiwindup);

  if (all_params_available) {
    setParameterEventCallback();
  }

  pid_.initPid(p, i, d, i_max, i_min, antiwindup);

  return all_params_available;
}

void
PidROS::declareParam(const std::string & param_name, rclcpp::ParameterValue param_value)
{
  if (!node_params_->has_parameter(param_name)) {
    node_params_->declare_parameter(param_name, param_value);
  }
}

void
PidROS::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  pid_.initPid(p, i, d, i_max, i_min, antiwindup);

  declareParam("p", rclcpp::ParameterValue(p));
  declareParam("i", rclcpp::ParameterValue(i));
  declareParam("d", rclcpp::ParameterValue(d));
  declareParam("i_clamp_max", rclcpp::ParameterValue(i_max));
  declareParam("i_clamp_min", rclcpp::ParameterValue(i_min));
  declareParam("antiwindup", rclcpp::ParameterValue(antiwindup));

  setParameterEventCallback();
}

void
PidROS::reset()
{
  pid_.reset();
}


std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>>
PidROS::getPidStatePublisher()
{
  return state_pub_;
}

double
PidROS::computeCommand(double error, rclcpp::Duration dt)
{
  double cmd_ = pid_.computeCommand(error, dt.nanoseconds());
  publishPIDState(cmd_, error, dt);

  return cmd_;
}

Pid::Gains
PidROS::getGains()
{
  return pid_.getGains();
}

void
PidROS::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  node_params_->set_parameters(
    {
      rclcpp::Parameter(topic_prefix_ + "p", p),
      rclcpp::Parameter(topic_prefix_ + "i", i),
      rclcpp::Parameter(topic_prefix_ + "d", d),
      rclcpp::Parameter(topic_prefix_ + "i_clamp_max", i_max),
      rclcpp::Parameter(topic_prefix_ + "i_clamp_min", i_min),
      rclcpp::Parameter(topic_prefix_ + "antiwindup", antiwindup)
    }
  );

  pid_.setGains(p, i, d, i_max, i_min, antiwindup);
}

void
PidROS::publishPIDState(double cmd, double error, rclcpp::Duration dt)
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

void
PidROS::setCurrentCmd(double cmd)
{
  pid_.setCurrentCmd(cmd);
}

double
PidROS::getCurrentCmd()
{
  return pid_.getCurrentCmd();
}

void
PidROS::getCurrentPIDErrors(double & pe, double & ie, double & de)
{
  double _pe, _ie, _de;
  pid_.getCurrentPIDErrors(_pe, _ie, _de);
  pe = _pe;
  ie = _ie;
  de = _de;
}

void
PidROS::printValues()
{
  Pid::Gains gains = pid_.getGains();

  double p_error_, i_error_, d_error_;
  getCurrentPIDErrors(p_error_, i_error_, d_error_);

  RCLCPP_INFO_STREAM(
    node_logging_->get_logger(),
    "Current Values of PID template:\n" <<
      "  P Gain:       " << gains.p_gain_ << "\n" <<
      "  I Gain:       " << gains.i_gain_ << "\n" <<
      "  D Gain:       " << gains.d_gain_ << "\n" <<
      "  I_Max:        " << gains.i_max_ << "\n" <<
      "  I_Min:        " << gains.i_min_ << "\n" <<
      "  Antiwindup:   " << gains.antiwindup_ << "\n" <<
      "  P_Error:      " << p_error_ << "\n" <<
      "  I_Error:      " << i_error_ << "\n" <<
      "  D_Error:      " << d_error_ << "\n" <<
      "  Command:      " << getCurrentCmd();
  );
}

void
PidROS::setGains(const Pid::Gains & gains)
{
  pid_.setGains(gains);
}

void
PidROS::setParameterEventCallback()
{
  auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      /// @note don't use getGains, it's rt
      Pid::Gains gains = pid_.getGains();

      for (auto & parameter : parameters) {
        const std::string param_name = parameter.get_name();
        try {
          if (param_name == "p") {
            gains.p_gain_ = parameter.get_value<double>();
          } else if (param_name == "i") {
            gains.i_gain_ = parameter.get_value<double>();
          } else if (param_name == "d") {
            gains.d_gain_ = parameter.get_value<double>();
          } else if (param_name == "i_clamp_max") {
            gains.i_max_ = parameter.get_value<double>();
          } else if (param_name == "i_clamp_min") {
            gains.i_min_ = parameter.get_value<double>();
          } else if (param_name == "antiwindup") {
            gains.antiwindup_ = parameter.get_value<bool>();
          } else {
            result.successful = false;
            result.reason = "Invalid parameter";
          }
        } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
          RCLCPP_INFO_STREAM(
            node_logging_->get_logger(), "Please use the right type: " << e.what());
        }
      }

      if (result.successful) {
        /// @note don't call setGains() from inside a callback
        pid_.setGains(gains);
      }

      return result;
    };

  parameter_callback_ =
    node_params_->add_on_set_parameters_callback(
    on_parameter_event_callback);
}

}  // namespace control_toolbox
