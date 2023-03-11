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

#include "control_filters/gravity_compensation.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace control_filters
{
template <>
bool GravityCompensation<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
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

  try
  {
    transform_ = p_tf_Buffer_->lookupTransform(
      parameters_.world_frame, data_in.header.frame_id, rclcpp::Time());
    transform_back_ = p_tf_Buffer_->lookupTransform(
      data_in.header.frame_id, parameters_.world_frame, rclcpp::Time());
    transform_cog_ = p_tf_Buffer_->lookupTransform(
      parameters_.world_frame, parameters_.force_frame, rclcpp::Time());
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 5000, "%s", ex.what());
  }

  geometry_msgs::msg::Vector3Stamped temp_force_transformed, temp_torque_transformed,
    temp_vector_in, temp_vector_out;

  // TODO(destogl): change this when `doTransform` for wrenches is merged into geometry2
  temp_vector_in.vector = data_in.wrench.force;
  tf2::doTransform(temp_vector_in, temp_force_transformed, transform_);

  temp_vector_in.vector = data_in.wrench.torque;
  tf2::doTransform(temp_vector_in, temp_torque_transformed, transform_);

  // Transform CoG Vector
  geometry_msgs::msg::Vector3Stamped cog_transformed;
  tf2::doTransform(cog_, cog_transformed, transform_cog_);

  // TODO(guihomework): use the full force vector and not only its z component
  // Compensate for gravity force
  temp_force_transformed.vector.z -= force_z_;
  // Compensation Values for torque result from cross-product of cog Vector and (0 0 G)
  temp_torque_transformed.vector.x -= (force_z_ * cog_transformed.vector.y);
  temp_torque_transformed.vector.y += (force_z_ * cog_transformed.vector.x);

  // Copy Message and Compensate values for Gravity Force and Resulting Torque
  data_out = data_in;

  // TODO(destogl): change this when `doTransform` for wrenches is merged into geometry2
  tf2::doTransform(temp_force_transformed, temp_vector_out, transform_back_);
  data_out.wrench.force = temp_vector_out.vector;

  tf2::doTransform(temp_torque_transformed, temp_vector_out, transform_back_);
  data_out.wrench.torque = temp_vector_out.vector;

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::GravityCompensation<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
