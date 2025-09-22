// Copyright (c) 2024, AIT Austrian Institute of Technology GmbH
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

#ifndef CONTROL_FILTERS__EXPONENTIAL_FILTER_HPP_
#define CONTROL_FILTERS__EXPONENTIAL_FILTER_HPP_

#include <limits>
#include <memory>
#include <string>

#include "filters/filter_base.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "control_toolbox/exponential_filter_parameters.hpp"
#include "control_toolbox/filters.hpp"
#include "control_toolbox/exponential_filter.hpp"

namespace control_filters
{

/***************************************************/
/*! \class ExponentialFilter
  \brief A exponential filter class.

  This class implements a low-pass filter for
  various data types.

  \section Usage

  The ExponentialFilter class is meant to be instantiated as a filter in
  a controller but can also be used elsewhere.
  For manual instantiation, you should first call configure()
  (in non-realtime) and then call update() at every update step.

*/
/***************************************************/

template <typename T>
class ExponentialFilter : public filters::FilterBase<T>
{
public:
  /*!
   * \brief Configure the ExponentialFilter (access and process params).
   */
  bool configure() override;

  /*!
   * \brief Applies one iteration of the exponential filter.
   *
   * \param data_in input to the filter
   * \param data_out filtered output
   *
   * \returns false if filter is not configured, true otherwise
   */
  bool update(const T & data_in, T & data_out) override;

private:
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<exponential_filter::ParamListener> parameter_handler_;
  exponential_filter::Params parameters_;
  std::shared_ptr<control_toolbox::ExponentialFilter<T>> expo_;
};

template <typename T>
bool ExponentialFilter<T>::configure()
{
  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));

  // Initialize the parameters once
  if (!parameter_handler_)
  {
    try
    {
      parameter_handler_ = std::make_shared<exponential_filter::ParamListener>(
        this->params_interface_, this->param_prefix_);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        (*logger_), "Exponential filter cannot be configured: %s (type : %s)", ex.what(),
        typeid(ex).name());
      parameter_handler_.reset();
      return false;
    }
    catch (...)
    {
      RCLCPP_ERROR((*logger_), "Caught unknown exception while configuring Exponential filter");
      parameter_handler_.reset();
      return false;
    }
  }
  parameters_ = parameter_handler_->get_params();
  expo_ = std::make_shared<control_toolbox::ExponentialFilter<T>>(parameters_.alpha);
  expo_->configure();
  return true;
}

template <>
inline bool ExponentialFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_ || !expo_ || !expo_->is_configured())
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    expo_ = std::make_shared<control_toolbox::ExponentialFilter<geometry_msgs::msg::WrenchStamped>>(parameters_.alpha);
  }

  // Delegate filtering to toolbox filter instance
  return expo_->update(data_in, data_out);
}

template <>
inline bool ExponentialFilter<std::vector<double>>::update(
  const std::vector<double> & data_in, std::vector<double> & data_out)
{
  if (!this->configured_ || !expo_ || !expo_->is_configured())
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    expo_->set_alpha(parameters_.alpha);
  }

  // Delegate filtering to toolbox filter instance
  return expo_->update(data_in, data_out);
}

template <typename T>
bool ExponentialFilter<T>::update(const T & data_in, T & data_out)
{
  if (!this->configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    expo_->set_alpha(parameters_.alpha);
  }

  // Delegate filtering to toolbox filter instance
  return expo_->update(data_in, data_out);
}
}  // namespace control_filters

#endif  // CONTROL_FILTERS__EXPONENTIAL_FILTER_HPP_
