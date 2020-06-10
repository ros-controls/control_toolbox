/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/*
  Author: Melonee Wise
  Contributors: Dave Coleman, Jonathan Bohren, Bob Holmberg, Wim Meeussen
  Desc: Implements a standard proportional-integral-derivative controller
*/

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "control_toolbox/pid.hpp"

namespace control_toolbox
{
/// @note replace with std::clamp once its supported
template<typename T>
T clamp(T val, T low, T high)
{
  if (val < low) {
    return low;
  } else if (val > high) {
    return high;
  }
  return val;
}

Pid::Pid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
: gains_buffer_(),
  node_param_iface_(nullptr), parameter_callback_(nullptr)
{
  setGains(p, i, d, i_max, i_min, antiwindup);

  reset();
}

Pid::Pid(const Pid & source)
{
  // Copy the realtime buffer to then new PID class
  gains_buffer_ = source.gains_buffer_;

  // Reset the state of this PID controller
  reset();
}

Pid::~Pid() {}

void Pid::initPid(
  double p, double i, double d, double i_max, double i_min, NodeParamsIfacePtr node_param_iface)
{
  const Pid::Gains gains = getGains();
  initPid(p, i, d, i_max, i_min, gains.antiwindup_, node_param_iface);
}

void Pid::initPid(
  double p, double i, double d, double i_max, double i_min, bool antiwindup,
  NodeParamsIfacePtr node_param_iface)
{
  initPid(p, i, d, i_max, i_min, antiwindup);

  node_param_iface_ = node_param_iface;

  // declare parameters if necessary
  if (node_param_iface_) {
    auto declare_param = [this](
      const std::string & param_name, rclcpp::ParameterValue param_value) {
        if (!node_param_iface_->has_parameter(param_name)) {
          node_param_iface_->declare_parameter(param_name, param_value);
        }
      };

    declare_param("p", rclcpp::ParameterValue(p));
    declare_param("i", rclcpp::ParameterValue(i));
    declare_param("d", rclcpp::ParameterValue(d));
    declare_param("i_clamp_max", rclcpp::ParameterValue(i_max));
    declare_param("i_clamp_min", rclcpp::ParameterValue(i_min));
    declare_param("antiwindup", rclcpp::ParameterValue(antiwindup));
  }

  setParameterEventCallback();
}

void Pid::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  setGains(p, i, d, i_max, i_min, antiwindup);

  reset();
}

void Pid::initPublisher(NodePtr node, std::string topic_prefix)
{
  rclcpp::QoS qos(10);
  qos.reliable().transient_local();

  state_pub_ = node->create_publisher<PidStateMsg>(topic_prefix + "/pid_state", qos);
  rt_state_pub_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));

  clock_ = node->get_clock();
}

void Pid::reset()
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  cmd_ = 0.0;
}

void Pid::getGains(double & p, double & i, double & d, double & i_max, double & i_min)
{
  bool antiwindup;
  getGains(p, i, d, i_max, i_min, antiwindup);
}

void Pid::getGains(
  double & p, double & i, double & d, double & i_max, double & i_min, bool & antiwindup)
{
  Gains gains = *gains_buffer_.readFromRT();

  p = gains.p_gain_;
  i = gains.i_gain_;
  d = gains.d_gain_;
  i_max = gains.i_max_;
  i_min = gains.i_min_;
  antiwindup = gains.antiwindup_;
}

Pid::Gains Pid::getGains()
{
  return *gains_buffer_.readFromRT();
}

void Pid::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  Gains gains(p, i, d, i_max, i_min, antiwindup);

  setGains(gains);
}

void Pid::setGains(const Gains & gains)
{
  gains_buffer_.writeFromNonRT(gains);

  // update node parameters
  if (node_param_iface_) {
    node_param_iface_->set_parameters(
      {rclcpp::Parameter("p", gains.p_gain_), rclcpp::Parameter("i", gains.i_gain_),
        rclcpp::Parameter("d", gains.d_gain_), rclcpp::Parameter("i_clamp_max", gains.i_max_),
        rclcpp::Parameter("i_clamp_min", gains.i_min_),
        rclcpp::Parameter("antiwindup", gains.antiwindup_)});
  }
}

double Pid::computeCommand(double error, rclcpp::Duration dt)
{
  if (dt == rclcpp::Duration(0, 0) || std::isnan(error) || std::isinf(error)) {
    return 0.0;
  }

  double error_dot = d_error_;

  // Calculate the derivative error
  if (dt.seconds() > 0.0) {
    error_dot = (error - p_error_last_) / dt.seconds();
    p_error_last_ = error;
  }

  return computeCommand(error, error_dot, dt);
}

double Pid::computeCommand(double error, double error_dot, rclcpp::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term, i_term;
  p_error_ = error;  // this is error = target - state
  d_error_ = error_dot;

  if (
    dt == rclcpp::Duration(0, 0) || std::isnan(error) || std::isinf(error) ||
    std::isnan(error_dot) || std::isinf(error_dot))
  {
    return 0.0;
  }

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  // Calculate the integral of the position error
  i_error_ += dt.seconds() * p_error_;

  if (gains.antiwindup_ && gains.i_gain_ != 0) {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    std::pair<double, double> bounds =
      std::minmax<double>(gains.i_min_ / gains.i_gain_, gains.i_max_ / gains.i_gain_);
    i_error_ = clamp(i_error_, bounds.first, bounds.second);
  }

  // Calculate integral contribution to command
  i_term = gains.i_gain_ * i_error_;

  if (!gains.antiwindup_) {
    // Limit i_term so that the limit is meaningful in the output
    i_term = clamp(i_term, gains.i_min_, gains.i_max_);
  }

  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;

  // Compute the command
  cmd_ = p_term + i_term + d_term;

  // Publish controller state if configured
  if (rt_state_pub_) {
    if (rt_state_pub_->trylock()) {
      rt_state_pub_->msg_.header.stamp = clock_->now();
      rt_state_pub_->msg_.timestep = dt;
      rt_state_pub_->msg_.error = error;
      rt_state_pub_->msg_.error_dot = error_dot;
      rt_state_pub_->msg_.p_error = p_error_;
      rt_state_pub_->msg_.i_error = i_error_;
      rt_state_pub_->msg_.d_error = d_error_;
      rt_state_pub_->msg_.p_term = p_term;
      rt_state_pub_->msg_.i_term = i_term;
      rt_state_pub_->msg_.d_term = d_term;
      rt_state_pub_->msg_.i_max = gains.i_max_;
      rt_state_pub_->msg_.i_min = gains.i_min_;
      rt_state_pub_->msg_.output = cmd_;
      rt_state_pub_->unlockAndPublish();
    }
  }

  return cmd_;
}

void Pid::setCurrentCmd(double cmd) {cmd_ = cmd;}

double Pid::getCurrentCmd() {return cmd_;}

void Pid::getCurrentPIDErrors(double * pe, double * ie, double * de)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  *pe = p_error_;
  *ie = i_error_;
  *de = d_error_;
}

void Pid::printValues(const rclcpp::Logger & logger)
{
  Gains gains = getGains();

  RCLCPP_INFO_STREAM(logger,
    "Current Values of PID Class:\n" <<
    "  P Gain:       " << gains.p_gain_ << "\n" <<
    "  I Gain:       " << gains.i_gain_ << "\n" <<
    "  D Gain:       " << gains.d_gain_ << "\n" <<
    "  I_Max:        " << gains.i_max_ << "\n" <<
    "  I_Min:        " << gains.i_min_ << "\n" <<
    "  Antiwindup:   " << gains.antiwindup_ << "\n" <<
    "  P_Error_Last: " << p_error_last_ << "\n" <<
    "  P_Error:      " << p_error_ << "\n" <<
    "  I_Error:      " << i_error_ << "\n" <<
    "  D_Error:      " << d_error_ << "\n" <<
    "  Command:      " << cmd_
  );
}

void Pid::setParameterEventCallback()
{
  auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      /// @note don't use getGains, it's rt
      Gains gains = *gains_buffer_.readFromNonRT();

      for (auto & parameter : parameters) {
        const std::string param_name = parameter.get_name();
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
      }

      if (result.successful) {
        /// @note don't call setGains() from inside a callback
        gains_buffer_.writeFromNonRT(gains);
      }

      return result;
    };

  if (node_param_iface_) {
    parameter_callback_ =
      node_param_iface_->add_on_set_parameters_callback(on_parameter_event_callback);
  }
}

}  // namespace control_toolbox
