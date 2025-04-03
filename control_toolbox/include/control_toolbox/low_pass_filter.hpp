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

#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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

  LowPassFilter(double sampling_frequency, double damping_frequency, double damping_intensity){
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

  bool is_configured() const
  {
    return configured_;
  }

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
  /** internal data storage (double). */
  double filtered_value, filtered_old_value, old_value;
  /** internal data storage (wrench). */
  Eigen::Matrix<double, 6, 1> msg_filtered, msg_filtered_old, msg_old;
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
  // Initialize storage Vectors
  filtered_value = filtered_old_value = old_value = 0;
  // TODO(destogl): make the size parameterizable and more intelligent is using complex types
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    msg_filtered[i] = msg_filtered_old[i] = msg_old[i] = 0;
  }

  return configured_ = true;
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // IIR Filter
  msg_filtered = b1_ * msg_old + a1_ * msg_filtered_old;
  msg_filtered_old = msg_filtered;

  // TODO(destogl): use wrenchMsgToEigen
  msg_old[0] = data_in.wrench.force.x;
  msg_old[1] = data_in.wrench.force.y;
  msg_old[2] = data_in.wrench.force.z;
  msg_old[3] = data_in.wrench.torque.x;
  msg_old[4] = data_in.wrench.torque.y;
  msg_old[5] = data_in.wrench.torque.z;

  data_out.wrench.force.x = msg_filtered[0];
  data_out.wrench.force.y = msg_filtered[1];
  data_out.wrench.force.z = msg_filtered[2];
  data_out.wrench.torque.x = msg_filtered[3];
  data_out.wrench.torque.y = msg_filtered[4];
  data_out.wrench.torque.z = msg_filtered[5];

  // copy the header
  data_out.header = data_in.header;
  return true;
}

template <typename T>
bool LowPassFilter<T>::update(const T & data_in, T & data_out)
{
  if (!configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Filter
  data_out = b1_ * old_value + a1_ * filtered_old_value;
  filtered_old_value = data_out;
  old_value = data_in;

  return true;
}

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__LOW_PASS_FILTER_HPP_
