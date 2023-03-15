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
    // use data_out frame id for the back transformation, otherwise same is data_in
    if (!data_out.header.frame_id.empty())
    {
      data_out.header.stamp = data_in.header.stamp;  // only copy the timestamp
      transform_back_ = p_tf_Buffer_->lookupTransform(
        data_out.header.frame_id, parameters_.world_frame, rclcpp::Time());
    }
    else
    {
      data_out.header = data_in.header;  // keep the same header and same frame_id
      transform_back_ = p_tf_Buffer_->lookupTransform(
        data_in.header.frame_id, parameters_.world_frame, rclcpp::Time());
    }
    transform_cog_ = p_tf_Buffer_->lookupTransform(
      parameters_.world_frame, parameters_.force_frame, rclcpp::Time());
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 5000, "%s", ex.what());
    return false;  // if cannot transform, result of subsequent computations is invalid
  }

  // Transform data_in to world_frame frame
  geometry_msgs::msg::Wrench wrench_world;
  tf2::doTransform(data_in.wrench, wrench_world, transform_);

  // Transform CoG Vector to world_frame frame
  geometry_msgs::msg::Vector3Stamped cog_transformed;
  tf2::doTransform(cog_, cog_transformed, transform_cog_);

  // TODO(guihomework): use the full force vector and not only its z component
  // Compensate for gravity force
  wrench_world.force.z -= force_z_;
  // Compensation Values for torque result from cross-product of cog Vector and (0 0 G)
  wrench_world.torque.x -= (force_z_ * cog_transformed.vector.y);
  wrench_world.torque.y += (force_z_ * cog_transformed.vector.x);
  // Transform wrench_world to data_out frame_id if not empty otherwise to data_in frame id
  tf2::doTransform(wrench_world, data_out.wrench, transform_back_);

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::GravityCompensation<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
