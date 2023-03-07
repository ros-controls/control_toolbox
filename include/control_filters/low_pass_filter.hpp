// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

template <typename T>
class LowPassFilter : public filters::FilterBase<T>
{
public:
  LowPassFilter();

  ~LowPassFilter() override;

  bool configure() override;

  bool update(const T & data_in, T & data_out) override;

protected:
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
  // TODO(destogl): we should do this more intelligently using only one set of types
  double filtered_value, filtered_old_value, old_value;
  Eigen::Matrix<double, 6, 1> msg_filtered, msg_filtered_old, msg_old;
  double a1_;
  double b1_;
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

  // Initialize the parameters
  try
  {
    parameter_handler_ = std::make_shared<low_pass_filter::ParamListener>(this->params_interface_);
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
