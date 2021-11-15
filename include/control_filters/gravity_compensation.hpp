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

#ifndef CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP_
#define CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/parameter_handler.hpp"
#include "filters/filter_base.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace control_filters
{
class GravityCompensationParameters : public control_toolbox::ParameterHandler
{
public:
  explicit GravityCompensationParameters(const std::string & params_prefix)
  : control_toolbox::ParameterHandler(params_prefix, 0, 0, 4, 3)
  {
    add_string_parameter("world_frame", false);
    add_string_parameter("sensor_frame", false);
    add_string_parameter("force_frame", false);

    add_double_parameter("CoG.x", true);
    add_double_parameter("CoG.y", true);
    add_double_parameter("CoG.z", true);
    add_double_parameter("force", true);
  }

  bool check_if_parameters_are_valid() override
  {
    bool ret = true;

    // Check if any string parameter is empty
    ret = !empty_parameter_in_list(string_parameters_);

    for (size_t i = 0; i < 4; ++i)
    {
      if (std::isnan(double_parameters_[i].second))
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "Parameter '%s' has to be set",
          double_parameters_[i].first.name.c_str());
        ret = false;
      }
    }

    return ret;
  }

  void update_storage() override
  {
    world_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "World frame: %s", world_frame_.c_str());
    sensor_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Sensor frame: %s", sensor_frame_.c_str());
    force_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Force frame: %s", force_frame_.c_str());

    cog_.vector.x = double_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "CoG X is %e", cog_.vector.x);
    cog_.vector.y = double_parameters_[1].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "CoG Y is %e", cog_.vector.y);
    cog_.vector.z = double_parameters_[2].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "CoG Z is %e", cog_.vector.z);

    force_z_ = double_parameters_[3].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Force is %e", force_z_);
  }

  // Frames for Transformation of Gravity / CoG Vector
  std::string world_frame_;
  std::string sensor_frame_;
  std::string force_frame_;

  // Storage for Calibration Values
  geometry_msgs::msg::Vector3Stamped cog_;  // Center of Gravity Vector (wrt Sensor Frame)
  double force_z_;                          // Gravitational Force
};

template <typename T>
class GravityCompensation : public filters::FilterBase<T>
{
public:
  /** \brief Constructor */
  GravityCompensation();

  /** \brief Destructor */
  ~GravityCompensation();

  /** @brief Configure filter parameters  */
  bool configure() override;

  /** \brief Update the filter and return the data separately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  bool update(const T & data_in, T & data_out) override;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<GravityCompensationParameters> parameters_;

  // Callback for updating dynamic parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;

  // Filter objects
  std::unique_ptr<tf2_ros::Buffer> p_tf_Buffer_;
  std::unique_ptr<tf2_ros::TransformListener> p_tf_Listener_;
  geometry_msgs::msg::TransformStamped transform_, transform_back_, transform_cog_;
};

template <typename T>
GravityCompensation<T>::GravityCompensation()
{
}

template <typename T>
GravityCompensation<T>::~GravityCompensation()
{
}

template <typename T>
bool GravityCompensation<T>::configure()
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  p_tf_Buffer_.reset(new tf2_ros::Buffer(clock_));
  p_tf_Listener_.reset(new tf2_ros::TransformListener(*p_tf_Buffer_.get(), true));

  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));
  parameters_.reset(new GravityCompensationParameters(this->param_prefix_));

  parameters_->initialize(this->params_interface_, logger_->get_name());

  parameters_->declare_parameters();

  if (!parameters_->get_parameters())
  {
    return false;
  }

  // Add callback to dynamically update parameters
  on_set_callback_handle_ = this->params_interface_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return parameters_->set_parameter_callback(parameters);
    });

  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP_
