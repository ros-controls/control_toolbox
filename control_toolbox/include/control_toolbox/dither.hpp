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

// \author Kevin Watts

#ifndef CONTROL_TOOLBOX__DITHER_HPP_
#define CONTROL_TOOLBOX__DITHER_HPP_

#include <math.h>
#include <cstdlib>
#include <ctime>
#include <random>

#include "rcutils/logging_macros.h"

namespace control_toolbox
{
/***************************************************/
/*! \class Dither
 *
 * \brief Gives white noise at specified amplitude.
 *
 * This class gives white noise at the given amplitude when
 * update() is called. It can be used to vibrate joints or
 * to break static friction.
 *
 */
class Dither
{
public:
  Dither();

  /*!
   * \brief Get next Gaussian white noise point. Called in RT loop.
   *\return White noise of given amplitude.
   */
  double update();

  /*
  *\brief Dither gets an amplitude, must be >0 to initialize
  *
  *\param amplitude Amplitude of white noise output
  *\param seed Random seed for white noise
  */
  bool init(const double & amplitude, const double & seed)
  {
    if (amplitude < 0.0) {
      RCUTILS_LOG_ERROR("Dither amplitude not set properly. Amplitude must be >0.");
      return false;
    }

    amplitude_ = amplitude;

    // seed generator for reproducible sequence of random numbers
    generator_.seed(static_cast<unsigned int>(seed));

    return true;
  }

  /*
  *\brief Generate a random number with random_device for non-deterministic random numbers
  */
  static double generateRandomSeed()
  {
    std::random_device rdev{};
    return static_cast<double>(rdev());
  }

private:
  double amplitude_; /**< Amplitude of the sweep. */
  double saved_value_;
  bool has_saved_value_;
  std::mt19937 generator_; /**< random number generator for white noise. */
};
}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__DITHER_HPP_
