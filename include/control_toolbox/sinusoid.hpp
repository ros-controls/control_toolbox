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

#ifndef CONTROL_TOOLBOX__SINUSOID_HPP_
#define CONTROL_TOOLBOX__SINUSOID_HPP_

namespace control_toolbox
{
/**
 * \class Sinusoid
 * \brief A basic sine class
 *
 * This class calculates the output for a sine wave and its derivatives, given the amplitude,
 * phase, frequency and offset.<br>
 *
 */
class Sinusoid
{
public:
  /**
   * \brief Constructor
   */
  Sinusoid();

  /**
   * \brief Constructor which initializes values
   *
   * \param offset A DC offset to be added to the sine wave
   * \param amplitude Amplitude of the sine wave
   * \param frequency Frequency of the sine wave
   * \param phase Phase (in radians) of the sine wave at t=0
   */
  Sinusoid(double offset, double amplitude, double frequency, double phase);

  /**
   * Destructor
   */
  virtual ~Sinusoid();

  /**
   * Prints the parameters of the sine wave to stdout (for debugging)
   */
  void debug();

  /**
   * \brief Gets the value and derivatives of the sinusoid at a given time
   *
   * \param time Time at which to sample the sine wave
   * \param qd (output) The derivative of the sine wave
   * \param qdd (output) Second derivative of the sine wave
   * \return The sampled value of the sine wave
   */
  double update(double time, double & qd, double & qdd);

private:
  double offset_;    /**< DC offset of the sine wave. */
  double amplitude_; /**< Amplitude of the sine wave. */
  double frequency_; /**< Frequency of the sine wave. */
  double phase_;     /**< Phase of the sine wave at t=0. */
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__SINUSOID_HPP_
