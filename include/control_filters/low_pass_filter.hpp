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

#ifndef CONTROL_FILTERS__LOW_PASS_FILTER_HPP_
#define CONTROL_FILTERS__LOW_PASS_FILTER_HPP_

#include <memory>
#include <string>

#include "filters/filter_base.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "control_toolbox/low_pass_filter.hpp"
#include "control_toolbox/low_pass_filter_parameters.hpp"

namespace control_filters
{

/***************************************************/
/*! \class LowPassFilter
  \brief A Low-pass filter class.

  This class implements a low-pass filter for
  various data types based on an Infinite Impulse Response Filter.

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

  The LowPassFilter class is meant to be instantiated as a filter in
  a controller but can also be used elsewhere.
  For manual instantiation, you should first call configure()
  (in non-realtime) and then call update() at every update step.

*/
/***************************************************/

template <typename T>
class LowPassFilter : public filters::FilterBase<T>
{
public:
  /*!
   * \brief Configure the LowPassFilter (access and process params).
   */
  bool configure() override;

  /*!
   * \brief Applies one iteration of the IIR filter.
   *
   * \param data_in input to the filter
   * \param data_out filtered output
   *
   * \returns false if filter is not configured, true otherwise
   */
  bool update(const T & data_in, T & data_out) override;

private:
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<low_pass_filter::ParamListener> parameter_handler_;
  low_pass_filter::Params parameters_;
  std::shared_ptr<control_toolbox::LowPassFilter<T>> lpf_;
};

template <typename T>
bool LowPassFilter<T>::configure()
{
  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));

  // Initialize the parameters once
  if (!parameter_handler_)
  {
    try
    {
      parameter_handler_ =
        std::make_shared<low_pass_filter::ParamListener>(this->params_interface_,
                                                         this->param_prefix_);
    }
    catch (const std::exception & ex) {
      RCLCPP_ERROR((*logger_),
        "LowPass filter cannot be configured: %s (type : %s)", ex.what(), typeid(ex).name());
      parameter_handler_.reset();
      return false;
    }
    catch (...) {
      RCLCPP_ERROR((*logger_), "Caught unknown exception while configuring LowPass filter");
      parameter_handler_.reset();
      return false;
    }
  }
  parameters_ = parameter_handler_->get_params();
  lpf_ = std::make_shared<control_toolbox::LowPassFilter<T>>(
    parameters_.sampling_frequency,
    parameters_.damping_frequency,
    parameters_.damping_intensity);

  return lpf_->configure();
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_ || !lpf_ || !lpf_->is_configured())
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    lpf_->set_params(
      parameters_.sampling_frequency,
      parameters_.damping_frequency,
      parameters_.damping_intensity);
  }

  return lpf_->update(data_in, data_out);
}

template <typename T>
bool LowPassFilter<T>::update(const T & data_in, T & data_out)
{
  if (!this->configured_ || !lpf_ || !lpf_->is_configured())
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    lpf_->set_params(
      parameters_.sampling_frequency,
      parameters_.damping_frequency,
      parameters_.damping_intensity);
  }

  return lpf_->update(data_in, data_out);
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__LOW_PASS_FILTER_HPP_
