// Copyright 2024 AIT - Austrian Institute of Technology GmbH
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

#ifndef CONTROL_FILTERS__RATE_LIMITER_HPP_
#define CONTROL_FILTERS__RATE_LIMITER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "filters/filter_base.hpp"

#include "control_toolbox/rate_limiter.hpp"
#include "control_toolbox/rate_limiter_parameters.hpp"

namespace control_filters
{

/***************************************************/
/*! \class RateLimiter

  \section Usage

  The RateLimiter class is meant to be instantiated as a filter in
  a controller but can also be used elsewhere.
  For manual instantiation, you should first call configure()
  (in non-realtime) and then call update() at every update step.

*/
/***************************************************/

template <typename T>
class RateLimiter : public filters::FilterBase<T>
{
public:
  /*!
   * \brief Configure the RateLimiter (access and process params).
   */
  bool configure() override;

  /*!
   * \brief Applies one step of the rate limiter
   *
   * \param data_in input to the limiter
   * \param data_out limited output
   *
   * \returns false if filter is not configured, true otherwise
   */
  bool update(const T & data_in, T & data_out) override;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<rate_limiter::ParamListener> parameter_handler_;
  rate_limiter::Params parameters_;
  std::shared_ptr<control_toolbox::RateLimiter<T>> limiter;

  T v1, v0;
};

template <typename T>
bool RateLimiter<T>::configure()
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));

  v0 = v1 = std::numeric_limits<T>::quiet_NaN();

  // Initialize the parameters once
  if (!parameter_handler_)
  {
    try
    {
      parameter_handler_ =
        std::make_shared<rate_limiter::ParamListener>(this->params_interface_,
                                                         this->param_prefix_);
    }
    catch (const std::exception & ex) {
      RCLCPP_ERROR((*logger_),
        "Rate Limiter filter cannot be configured: %s (type : %s)", ex.what(), typeid(ex).name());
      parameter_handler_.reset();
      return false;
    }
    catch (...) {
      RCLCPP_ERROR((*logger_), "Caught unknown exception while configuring Rate Limiter filter");
      parameter_handler_.reset();
      return false;
    }
  }
  parameters_ = parameter_handler_->get_params();
  limiter = std::make_shared<control_toolbox::RateLimiter<T>>(
    parameters_.min_value, parameters_.max_value,
    parameters_.min_first_derivative_neg, parameters_.max_first_derivative_pos,
    parameters_.min_first_derivative_pos, parameters_.max_first_derivative_neg,
    parameters_.min_second_derivative, parameters_.max_second_derivative
  );

  return true;
}

template <typename T>
bool RateLimiter<T>::update(const T & data_in, T & data_out)
{
  if (!this->configured_ || !limiter)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    limiter->set_params(
      parameters_.min_value, parameters_.max_value,
      parameters_.min_first_derivative_neg, parameters_.max_first_derivative_pos,
      parameters_.min_first_derivative_pos, parameters_.max_first_derivative_neg,
      parameters_.min_second_derivative, parameters_.max_second_derivative
    );
  }
  T v = data_in;
  if (std::isnan(v0))
  {
    // not initialized yet
    v1 = v0 = v;
  }
  limiter->limit(v, v0, v1, static_cast<T>(parameters_.sampling_interval));
  // shift the values for the next update call
  v1 = v0;
  v0 = v;  // use the limited value
  data_out = v;
  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__RATE_LIMITER_HPP_
