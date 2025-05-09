// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROL_TOOLBOX__LOW_PASS_FILTER_HPP_
#define CONTROL_TOOLBOX__LOW_PASS_FILTER_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "control_toolbox/filter_traits.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_toolbox
{

/***************************************************/
/*! \class LowPassFilter
  \brief A Low-pass filter class.

  This class implements a low-pass filter for
  various data types based on an Infinite Impulse Response Filter.
  For vector elements, the filtering is applied separately on
  each element of the vector.

  In particular, this class implements a simplified version of
  an IIR filter equation :

  \f$y(n) = b x(n-1) + a y(n-1)\f$

  where: <br>
  <UL TYPE="none">
  <LI>  \f$ x(n)\f$ is the input signal
  <LI>  \f$ y(n)\f$ is the output signal (filtered)
  <LI>  \f$ b \f$ is the feedforward filter coefficient
  <LI>  \f$ a \f$ is the feedback filter coefficient
  </UL>

  and the Low-Pass coefficient equation:
  <br>
  <UL TYPE="none">
  <LI>  \f$ a = e^{\frac{-1}{sf} \frac{2\pi df}{10^{\frac{di}{-10}}}} \f$
  <LI>  \f$ b = 1 - a \f$
  </UL>

    where: <br>
  <UL TYPE="none">
  <LI>  \f$ sf \f$ is the sampling frequency
  <LI>  \f$ df \f$ is the damping frequency
  <LI>  \f$ di \f$ is the damping intensity (amplitude)
  </UL>

  \section Usage

  For manual instantiation, you should first call configure()
  (in non-realtime) and then call update() at every update step.

*/
/***************************************************/

template <typename T>
class LowPassFilter
{
public:
  // Default constructor
  LowPassFilter();

  LowPassFilter(double sampling_frequency, double damping_frequency, double damping_intensity)
  {
    set_params(sampling_frequency, damping_frequency, damping_intensity);
  }

  /*!
   * \brief Destructor of LowPassFilter class.
   */
  ~LowPassFilter();

  /*!
   * \brief Configure the LowPassFilter (access and process params).
   */
  bool configure();

  /*!
   * \brief Applies one iteration of the IIR filter.
   *
   * \param data_in input to the filter
   * \param data_out filtered output
   *
   * \returns false if filter is not configured, true otherwise
   */
  bool update(const T & data_in, T & data_out);

  bool is_configured() const { return configured_; }

  /*!
   * \brief Internal computation of the feedforward and feedbackward coefficients
   * according to the LowPassFilter parameters.
   */
  void set_params(double sampling_frequency, double damping_frequency, double damping_intensity)
  {
    a1_ = exp(
      -1.0 / sampling_frequency * (2.0 * M_PI * damping_frequency) /
      (pow(10.0, damping_intensity / -10.0)));
    b1_ = 1.0 - a1_;
  };

private:
  // Filter parameters
  double a1_; /** feedbackward coefficient. */
  double b1_; /** feedforward coefficient. */

  // Define the storage type based on T
  using Traits = FilterTraits<T>;
  using StorageType = typename Traits::StorageType;

  StorageType filtered_value_, filtered_old_value_, old_value_;

  bool configured_ = false;
};

template <typename T>
LowPassFilter<T>::LowPassFilter() : a1_(1.0), b1_(0.0)
{
}

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}

template <typename T>
bool LowPassFilter<T>::configure()
{
  Traits::initialize(filtered_value_);
  Traits::initialize(filtered_old_value_);
  Traits::initialize(old_value_);

  return configured_ = true;
}

template <typename T>
bool LowPassFilter<T>::update(const T & data_in, T & data_out)
{
  if (!configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }
  // If this is the first call to update initialize the filter at the current state
  // so that we dont apply an impulse to the data.
  if (Traits::is_nan(filtered_value_) || Traits::is_empty(filtered_value_))
  {
    if (!Traits::is_finite(data_in))
    {
      return false;
    }

    Traits::assign(filtered_value_, data_in);
    Traits::assign(filtered_old_value_, data_in);
    Traits::assign(old_value_, data_in);
  }
  else
  {
    // Generic validation for all types
    Traits::validate_input(data_in, filtered_value_, data_out);
  }

  // Filter
  filtered_value_ = old_value_ * b1_ + filtered_old_value_ * a1_;
  filtered_old_value_ = filtered_value_;

  Traits::assign(old_value_, data_in);
  Traits::assign(data_out, filtered_value_);

  if (Traits::is_finite(data_in))
  {
    Traits::assign(old_value_, data_in);
  }

  Traits::add_metadata(data_out, data_in);

  return true;
}

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__LOW_PASS_FILTER_HPP_
