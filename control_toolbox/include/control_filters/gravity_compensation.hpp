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

#ifndef CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP_
#define CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP_

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "filters/filter_base.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "control_toolbox/gravity_compensation_filter_parameters.hpp"

namespace control_filters
{

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

protected:
  void compute_internal_params()
  {
    cog_.vector.x = parameters_.tool.CoG[0];
    cog_.vector.y = parameters_.tool.CoG[1];
    cog_.vector.z = parameters_.tool.CoG[2];
    cst_ext_force_.vector.x = parameters_.tool.gravity_field[0] * parameters_.tool.mass;
    cst_ext_force_.vector.y = parameters_.tool.gravity_field[1] * parameters_.tool.mass;
    cst_ext_force_.vector.z = parameters_.tool.gravity_field[2] * parameters_.tool.mass;
  };

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<gravity_compensation_filter::ParamListener> parameter_handler_;
  gravity_compensation_filter::Params parameters_;

  // Frames for Transformation of Gravity / CoG Vector
  std::string world_frame_;   // frame in which gravity is given
  std::string sensor_frame_;  // frame in which Cog is given and compution occur
  // Storage for Calibration Values
  geometry_msgs::msg::Vector3Stamped cog_;            // Center of Gravity Vector (wrt sensor frame)
  geometry_msgs::msg::Vector3Stamped cst_ext_force_;  // Gravity Force Vector (wrt world frame)

  // Filter objects
  std::unique_ptr<tf2_ros::Buffer> p_tf_Buffer_;
  std::unique_ptr<tf2_ros::TransformListener> p_tf_Listener_;
  geometry_msgs::msg::TransformStamped transform_sensor_datain_, transform_world_dataout_,
    transform_data_out_sensor_, transform_sensor_world_;
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
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  p_tf_Buffer_.reset(new tf2_ros::Buffer(clock_));
  p_tf_Listener_.reset(new tf2_ros::TransformListener(*p_tf_Buffer_.get(), true));

  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));

  // Initialize the parameter_listener once
  if (!parameter_handler_)
  {
    try
    {
      parameter_handler_ = std::make_shared<gravity_compensation_filter::ParamListener>(
        this->params_interface_, this->param_prefix_);
    }
    catch (std::exception & ex)
    {
      RCLCPP_ERROR((*logger_), "GravityCompensation filter cannot be configured: %s", ex.what());
      parameter_handler_.reset();
      throw;
    }
  }
  parameters_ = parameter_handler_->get_params();
  compute_internal_params();

  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP_
