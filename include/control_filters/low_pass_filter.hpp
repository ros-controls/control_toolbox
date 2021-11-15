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

#include "control_toolbox/parameter_handler.hpp"
#include "filters/filter_base.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_filters
{
class LowPassParameters : public control_toolbox::ParameterHandler
{
public:
  explicit LowPassParameters(const std::string & params_prefix)
  : control_toolbox::ParameterHandler(params_prefix, 0, 1, 3)
  {
    add_double_parameter("sampling_frequency", false);
    add_double_parameter("damping_frequency", false);
    add_double_parameter("damping_intensity", false);

    add_integer_parameter("divider", false);
  }

  bool check_if_parameters_are_valid() override
  {
    bool ret = true;

    // Check if any string parameter is empty
    ret = !empty_parameter_in_list(string_parameters_);

    for (size_t i = 0; i < 3; ++i)
    {
      if (std::isnan(double_parameters_[i].second))
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "Parameter '%s' has to be set",
          double_parameters_[i].first.name.c_str());
        ret = false;
      }
    }

    if (integer_parameters_[0].second < 0)
    {
      RCUTILS_LOG_ERROR_NAMED(
        logger_name_.c_str(), "Parameter '%s' has to be positive",
        integer_parameters_[0].first.name.c_str());
    }

    return ret;
  }

  void update_storage() override
  {
    sampling_frequency_ = double_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Sampling frequency is %e", sampling_frequency_);
    damping_frequency_ = double_parameters_[1].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Damping frequency is %e", damping_frequency_);
    damping_intensity_ = double_parameters_[2].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Damping intensity is %e", damping_intensity_);

    divider_ = integer_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Divider %d", divider_);

    a1_ = exp(
      -1.0 / sampling_frequency_ * (2.0 * M_PI * damping_frequency_) /
      (pow(10.0, damping_intensity_ / -10.0)));
    b1_ = 1.0 - a1_;
  }

  // Parameters from parameter server
  double sampling_frequency_;
  double damping_frequency_;
  double damping_intensity_;

  int divider_;

  // Filter Parameters
  double a1_;
  double b1_;
};

template <typename T>
class LowPassFilter : public filters::FilterBase<T>
{
public:
  LowPassFilter();

  ~LowPassFilter() override;

  bool configure() override;

  bool update(const T & data_in, T & data_out) override;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<LowPassParameters> parameters_;

  // Callback for updating dynamic parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;

  // Filter parameters
  // TODO(destogl): we should do this more intelligently using only one set of types
  double filtered_value, filtered_old_value, old_value;
  Eigen::Matrix<double, 6, 1> msg_filtered, msg_filtered_old, msg_old;
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
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
  parameters_.reset(new LowPassParameters(this->param_prefix_));

  parameters_->initialize(this->params_interface_, logger_->get_name());

  parameters_->declare_parameters();

  if (!parameters_->get_parameters())
  {
    return false;
  }

  // Initialize storage Vectors
  filtered_value = filtered_old_value = old_value = 0;
  // TODO(destogl): make the size parameterizable and more intelligent is using complex types
  for (size_t i = 0; i < 6; ++i)
  {
    msg_filtered[i] = msg_filtered_old[i] = msg_old[i] = 0;
  }

  // Add callback to dynamically update parameters
  on_set_callback_handle_ = this->params_interface_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return parameters_->set_parameter_callback(parameters);
    });

  return true;
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  parameters_->update();

  // IIR Filter
  msg_filtered = parameters_->b1_ * msg_old + parameters_->a1_ * msg_filtered_old;
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

  parameters_->update();

  // Filter
  data_out = parameters_->b1_ * old_value + parameters_->a1_ * filtered_old_value;
  filtered_old_value = data_out;
  old_value = data_in;

  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__LOW_PASS_FILTER_HPP_
