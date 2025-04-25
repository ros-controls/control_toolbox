// Copyright (c) 2008, Willow Garage, Inc.
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

/*
  Author: Melonee Wise
  Contributors: Dave Coleman, Jonathan Bohren, Bob Holmberg, Wim Meeussen
  Desc: Implements a standard proportional-integral-derivative controller
*/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "control_toolbox/pid.hpp"

namespace control_toolbox
{
Pid::Pid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
: gains_buffer_()
{
  if (i_min > i_max)
  {
    throw std::invalid_argument("received i_min > i_max");
  }
  set_gains(p, i, d, i_max, i_min, antiwindup);

  // Initialize saved i-term values
  clear_saved_iterm();

  reset();
}

Pid::Pid(const Pid & source)
{
  // Copy the realtime buffer to the new PID class
  gains_buffer_ = source.gains_buffer_;

  // Initialize saved i-term values
  clear_saved_iterm();

  // Reset the state of this PID controller
  reset();
}

Pid::~Pid() {}

void Pid::initialize(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  set_gains(p, i, d, i_max, i_min, antiwindup);

  reset();
}

void Pid::reset() { reset(false); }

void Pid::reset(bool save_i_term)
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  cmd_ = 0.0;

  // Check to see if we should reset integral error here
  if (!save_i_term)
  {
    clear_saved_iterm();
  }
}

void Pid::clear_saved_iterm() { i_term_ = 0.0; }

void Pid::get_gains(double & p, double & i, double & d, double & i_max, double & i_min)
{
  bool antiwindup;
  get_gains(p, i, d, i_max, i_min, antiwindup);
}

void Pid::get_gains(
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

Pid::Gains Pid::get_gains() { return *gains_buffer_.readFromRT(); }

void Pid::set_gains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  Gains gains(p, i, d, i_max, i_min, antiwindup);

  set_gains(gains);
}

void Pid::set_gains(const Gains & gains)
{
  if (gains.i_min_ > gains.i_max_)
  {
    std::cout << "received i_min > i_max, skip new gains\n";
  }
  else
  {
    gains_buffer_.writeFromNonRT(gains);
  }
}

double Pid::compute_command(double error, const double & dt_s)
{
  if (std::abs(dt_s) <= std::numeric_limits<float>::epsilon())
  {
    // don't update anything
    return cmd_;
  }
  else if (dt_s < 0.0)
  {
    throw std::invalid_argument("Pid is called with negative dt");
  }

  // don't reset controller but return NaN
  if (!std::isfinite(error))
  {
    std::cout << "Received a non-finite error value\n";
    return cmd_ = std::numeric_limits<float>::quiet_NaN();
  }

  // Calculate the derivative error
  d_error_ = (error - p_error_last_) / dt_s;
  p_error_last_ = error;

  return compute_command(error, d_error_, dt_s);
}

double Pid::compute_command(double error, const rcl_duration_value_t & dt_ns)
{
  return compute_command(error, static_cast<double>(dt_ns) / 1.e9);
}

double Pid::compute_command(double error, const rclcpp::Duration & dt)
{
  return compute_command(error, dt.seconds());
}

double Pid::compute_command(double error, const std::chrono::nanoseconds & dt_ns)
{
  return compute_command(error, static_cast<double>(dt_ns.count()) / 1.e9);
}

double Pid::compute_command(double error, double error_dot, const rcl_duration_value_t & dt_ns)
{
  return compute_command(error, error_dot, static_cast<double>(dt_ns) / 1.e9);
}

double Pid::compute_command(double error, double error_dot, const rclcpp::Duration & dt)
{
  return compute_command(error, error_dot, dt.seconds());
}

double Pid::compute_command(double error, double error_dot, const std::chrono::nanoseconds & dt_ns)
{
  return compute_command(error, error_dot, static_cast<double>(dt_ns.count()) / 1.e9);
}

double Pid::compute_command(double error, double error_dot, const double & dt_s)
{
  if (std::abs(dt_s) <= std::numeric_limits<float>::epsilon())
  {
    // don't update anything
    return cmd_;
  }
  else if (dt_s < 0.0)
  {
    throw std::invalid_argument("Pid is called with negative dt");
  }
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term;
  p_error_ = error;  // this is error = target - state
  d_error_ = error_dot;

  // don't reset controller but return NaN
  if (!std::isfinite(error) || !std::isfinite(error_dot))
  {
    std::cout << "Received a non-finite error/error_dot value\n";
    return cmd_ = std::numeric_limits<float>::quiet_NaN();
  }

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  // Calculate integral contribution to command
  if (gains.antiwindup_)
  {
    // Prevent i_term_ from climbing higher than permitted by i_max_/i_min_
    i_term_ = std::clamp(i_term_ + gains.i_gain_ * dt_s * p_error_, gains.i_min_, gains.i_max_);
  }
  else
  {
    i_term_ += gains.i_gain_ * dt_s * p_error_;
  }

  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;

  // Compute the command
  // Limit i_term so that the limit is meaningful in the output
  cmd_ = p_term + std::clamp(i_term_, gains.i_min_, gains.i_max_) + d_term;

  return cmd_;
}

void Pid::set_current_cmd(double cmd) { cmd_ = cmd; }

double Pid::get_current_cmd() { return cmd_; }

void Pid::get_current_pid_errors(double & pe, double & ie, double & de)
{
  pe = p_error_;
  ie = i_term_;
  de = d_error_;
}

}  // namespace control_toolbox
