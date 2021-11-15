// Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <cmath>
#include <iostream>

#include "control_toolbox/sinusoid.hpp"

namespace control_toolbox
{
Sinusoid::Sinusoid(double offset, double amplitude, double frequency, double phase)
: offset_(offset), amplitude_(amplitude), frequency_(frequency), phase_(phase)
{
}

Sinusoid::~Sinusoid() {}

Sinusoid::Sinusoid() {}

double Sinusoid::update(double time, double & qd, double & qdd)
{
  double angular_frequency = 2.0 * M_PI * frequency_;
  double p = phase_ + angular_frequency * time;
  double sin_p = sin(p);
  double cos_p = cos(p);
  double q = offset_ + amplitude_ * sin_p;
  qd = angular_frequency * amplitude_ * cos_p;
  qdd = -angular_frequency * angular_frequency * amplitude_ * sin_p;
  return q;
}

void Sinusoid::debug()
{
  std::cout << "offset=" << offset_ << " amplitude=" << amplitude_ << " phase=" << phase_ <<
    " frequency=" << frequency_ << std::endl;
}

}  // namespace control_toolbox
