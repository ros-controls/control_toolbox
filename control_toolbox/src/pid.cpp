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
#include <stdexcept>

#include "control_toolbox/pid.hpp"

namespace control_toolbox
{

Pid::Pid(
  double p, double i, double d, double u_max, double u_min,
  const AntiWindupStrategy & antiwindup_strat)
{
  if (u_min > u_max)
  {
    throw std::invalid_argument("received u_min > u_max");
  }
  set_gains(p, i, d, u_max, u_min, antiwindup_strat);

  // Initialize saved i-term values
  clear_saved_iterm();

  reset();
}

Pid::Pid(const Pid & source)
{
  // Copy the realtime box to the new PID class
  gains_box_ = source.gains_box_;

  // Initialize saved i-term values
  clear_saved_iterm();

  // Reset the state of this PID controller
  reset();
}

Pid::~Pid() {}

bool Pid::initialize(
  double p, double i, double d, double u_max, double u_min,
  const AntiWindupStrategy & antiwindup_strat)
{
  if (set_gains(p, i, d, u_max, u_min, antiwindup_strat))
  {
    reset();
    return true;
  }
  return false;
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

  // blocking, as reset() is not called in the RT thread
  gains_ = gains_box_.get();
}

void Pid::clear_saved_iterm() { i_term_ = 0.0; }

void Pid::get_gains(
  double & p, double & i, double & d, double & u_max, double & u_min,
  AntiWindupStrategy & antiwindup_strat)
{
  Gains gains = get_gains();
  p = gains.p_gain_;
  i = gains.i_gain_;
  d = gains.d_gain_;
  u_max = gains.u_max_;
  u_min = gains.u_min_;
  antiwindup_strat = gains.antiwindup_strat_;
}

Pid::Gains Pid::get_gains()
{
  // blocking, as get_gains() is called from non-RT thread
  return gains_box_.get();
}

bool Pid::set_gains(
  double p, double i, double d, double u_max, double u_min,
  const AntiWindupStrategy & antiwindup_strat)
{
  try
  {
    Gains gains(p, i, d, u_max, u_min, antiwindup_strat);
    if (set_gains(gains))
    {
      return true;
    }
  }
  catch (const std::exception & e)
  {
    std::cerr << e.what() << '\n';
  }
  return false;
}

bool Pid::set_gains(const Gains & gains_in)
{
  std::string error_msg = "";
  if (!gains_in.validate(error_msg))
  {
    std::cerr << "PID: Invalid gains: " << error_msg << ". Skipping new gains." << std::endl;
    return false;
  }
  else
  {
    Gains gains = gains_in;

    if (gains.antiwindup_strat_.type == AntiWindupStrategy::BACK_CALCULATION)
    {
      if (is_zero(gains.antiwindup_strat_.tracking_time_constant) && !is_zero(gains.d_gain_))
      {
        // Default value for tracking time constant for back calculation technique
        gains.antiwindup_strat_.tracking_time_constant = std::sqrt(gains.d_gain_ / gains.i_gain_);
      }
      else if (is_zero(gains.antiwindup_strat_.tracking_time_constant) && is_zero(gains.d_gain_))
      {
        // Default value for tracking time constant for back calculation technique
        gains.antiwindup_strat_.tracking_time_constant = gains.p_gain_ / gains.i_gain_;
      }
    }
    // blocking, as set_gains() is called from non-RT thread
    gains_box_.set(gains);
    return true;
  }
  return false;
}

double Pid::compute_command(double error, const double & dt_s)
{
  if (is_zero(dt_s))
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
    std::cerr << "Received a non-finite error value\n";
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
  if (is_zero(dt_s))
  {
    // Don't update anything
    return cmd_;
  }
  else if (dt_s < 0.0)
  {
    throw std::invalid_argument("Pid is called with negative dt");
  }
  // Get the gain parameters from the realtime box
  auto gains_opt = gains_box_.try_get();
  if (gains_opt.has_value())
  {
    gains_ = gains_opt.value();
  }

  double p_term, d_term;
  p_error_ = error;      // This is error = target - state
  d_error_ = error_dot;  // This is the derivative of error

  // Don't reset controller but return NaN
  if (!std::isfinite(error) || !std::isfinite(error_dot))
  {
    std::cerr << "Received a non-finite error/error_dot value\n";
    return cmd_ = std::numeric_limits<double>::quiet_NaN();
  }

  // Calculate proportional contribution to command
  p_term = gains_.p_gain_ * p_error_;

  // Calculate derivative contribution to command
  d_term = gains_.d_gain_ * d_error_;

  if (gains_.antiwindup_strat_.type == AntiWindupStrategy::UNDEFINED)
  {
    throw std::runtime_error(
      "PID: Antiwindup strategy cannot be UNDEFINED. Please set a valid antiwindup strategy.");
  }

  const bool is_error_in_deadband_zone =
    control_toolbox::is_zero(error, gains_.antiwindup_strat_.error_deadband);

  // Compute the command
  cmd_unsat_ = p_term + i_term_ + d_term;

  if (std::isfinite(gains_.u_min_) || std::isfinite(gains_.u_max_))
  {
    if (gains_.u_min_ > gains_.u_max_)
    {
      throw std::runtime_error("Pid: Error while saturating the command : u_min > u_max");
    }
    if (std::isnan(gains_.u_min_) || std::isnan(gains_.u_max_))
    {
      throw std::runtime_error("Pid: Error while saturating the command : u_min or u_max is NaN");
    }
    cmd_ = std::clamp(cmd_unsat_, gains_.u_min_, gains_.u_max_);
  }
  else
  {
    cmd_ = cmd_unsat_;
  }
  if (!is_error_in_deadband_zone)
  {
    if (
      gains_.antiwindup_strat_.type == AntiWindupStrategy::BACK_CALCULATION &&
      !is_zero(gains_.i_gain_))
    {
      i_term_ += dt_s * (gains_.i_gain_ * error +
                         1 / gains_.antiwindup_strat_.tracking_time_constant * (cmd_ - cmd_unsat_));
    }
    else if (gains_.antiwindup_strat_.type == AntiWindupStrategy::CONDITIONAL_INTEGRATION)
    {
      if (!(!is_zero(cmd_unsat_ - cmd_) && error * cmd_unsat_ > 0))
      {
        i_term_ += dt_s * gains_.i_gain_ * error;
      }
    }
    else if (gains_.antiwindup_strat_.type == AntiWindupStrategy::NONE)
    {
      // No anti-windup strategy, so just integrate the error
      i_term_ += dt_s * gains_.i_gain_ * error;
    }
  }

  i_term_ = std::clamp(i_term_, gains_.i_min_, gains_.i_max_);

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
