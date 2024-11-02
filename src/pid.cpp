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
#include <string>
#include <utility>
#include <vector>

#include "control_toolbox/pid.hpp"

namespace control_toolbox
{
Pid::Pid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
: gains_buffer_()
{
  if (i_min > i_max) {
    throw std::invalid_argument("received i_min > i_max");
  }
  setGains(p, i, d, i_max, i_min, antiwindup);

  reset();
}

Pid::Pid(const Pid & source)
{
  // Copy the realtime buffer to the new PID class
  gains_buffer_ = source.gains_buffer_;

  // Reset the state of this PID controller
  reset();
}

Pid::~Pid() {}

void Pid::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  setGains(p, i, d, i_max, i_min, antiwindup);

  reset();
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

Pid::Gains Pid::getGains() { return *gains_buffer_.readFromRT(); }

void Pid::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  Gains gains(p, i, d, i_max, i_min, antiwindup);

  setGains(gains);
}

void Pid::setGains(const Gains & gains)
{
  if (gains.i_min_ > gains.i_max_) {
    std::cout << "received i_min > i_max, skip new gains\n";
  } else {
    gains_buffer_.writeFromNonRT(gains);
  }
}

double Pid::computeCommand(double error, uint64_t dt)
{
  if (dt == 0 || std::isnan(error) || std::isinf(error)) {
    return 0.0;
  }

  error_dot_ = d_error_;

  // Calculate the derivative error
  error_dot_ = (error - p_error_last_) / (static_cast<double>(dt) / 1e9);
  p_error_last_ = error;

  return computeCommand(error, error_dot_, dt);
}

double Pid::computeCommand(double error, double error_dot, uint64_t dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term, i_term;
  p_error_ = error;  // this is error = target - state
  d_error_ = error_dot;

  if (
    dt == 0 || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) ||
    std::isinf(error_dot)) {
    return 0.0;
  }

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  // Calculate the integral of the position error
  i_error_ += (static_cast<double>(dt) / 1e9) * p_error_;

  if (gains.antiwindup_ && gains.i_gain_ != 0) {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    std::pair<double, double> bounds =
      std::minmax<double>(gains.i_min_ / gains.i_gain_, gains.i_max_ / gains.i_gain_);
    i_error_ = std::clamp(i_error_, bounds.first, bounds.second);
  }

  // Calculate integral contribution to command
  i_term = gains.i_gain_ * i_error_;

  if (!gains.antiwindup_) {
    // Limit i_term so that the limit is meaningful in the output
    i_term = std::clamp(i_term, gains.i_min_, gains.i_max_);
  }

  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;

  // Compute the command
  cmd_ = p_term + i_term + d_term;

  return cmd_;
}

void Pid::setCurrentCmd(double cmd) { cmd_ = cmd; }

double Pid::getDerivativeError() { return error_dot_; }

double Pid::getCurrentCmd() { return cmd_; }

void Pid::getCurrentPIDErrors(double & pe, double & ie, double & de)
{
  pe = p_error_;
  ie = i_error_;
  de = d_error_;
}
}  // namespace control_toolbox
