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

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "low_pass_filter_parameters.hpp"
#include "filters/filter_base.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_filters
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
  // Default constructor
  LowPassFilter();

  /*!
   * \brief Destructor of LowPassFilter class.
   */
  ~LowPassFilter() override;

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

protected:
  /*!
   * \brief Internal computation of the feedforward and feedbackward coefficients
   * according to the LowPassFilter parameters.
   */
  void compute_internal_params()
  {
    a1_ = exp(
      -1.0 / parameters_.sampling_frequency * (2.0 * M_PI * parameters_.damping_frequency) /
      (pow(10.0, parameters_.damping_intensity / -10.0)));
    b1_ = 1.0 - a1_;
  };

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<low_pass_filter::ParamListener> parameter_handler_;
  low_pass_filter::Params parameters_;

  // Filter parameters
  /** internal data storage (double). */
  double filtered_value, filtered_old_value, old_value;
  /** internal data storage (wrench). */
  Eigen::Matrix<double, 6, 1> msg_filtered, msg_filtered_old, msg_old;
  double a1_; /**< feedbackward coefficient. */
  double b1_; /**< feedforward coefficient. */
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
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
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
    catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
      RCLCPP_ERROR((*logger_), "LowPass filter cannot be configured: %s", ex.what());
      parameter_handler_.reset();
      return false;
    }
    catch (rclcpp::exceptions::InvalidParameterValueException & ex)  {
      RCLCPP_ERROR((*logger_), "LowPass filter cannot be configured: %s", ex.what());
      parameter_handler_.reset();
      return false;
    }
  }
  parameters_ = parameter_handler_->get_params();
  compute_internal_params();

  // Initialize storage Vectors
  filtered_value = filtered_old_value = old_value = 0;
  // TODO(destogl): make the size parameterizable and more intelligent is using complex types
  for (size_t i = 0; i < 6; ++i)
  {
    msg_filtered[i] = msg_filtered_old[i] = msg_old[i] = 0;
  }

  return true;
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_)
  {
    if (logger_)
      RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    compute_internal_params();
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
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    compute_internal_params();
  }

  // Filter
  data_out = b1_ * old_value + a1_ * filtered_old_value;
  filtered_old_value = data_out;
  old_value = data_in;

  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__LOW_PASS_FILTER_HPP_
