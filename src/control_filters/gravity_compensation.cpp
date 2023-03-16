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
#include "tf2/LinearMath/Vector3.h"

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
    transform_datain_world_ = p_tf_Buffer_->lookupTransform(
      parameters_.world_frame, data_in.header.frame_id, rclcpp::Time());
    // use data_out frame id for the back transformation, otherwise same is data_in
    if (!data_out.header.frame_id.empty())
    {
      data_out.header.stamp = data_in.header.stamp;  // only copy the timestamp
      transform_world_dataout_ = p_tf_Buffer_->lookupTransform(
        data_out.header.frame_id, parameters_.world_frame, rclcpp::Time());
    }
    else
    {
      data_out.header = data_in.header;  // keep the same header and same frame_id
      transform_world_dataout_ = p_tf_Buffer_->lookupTransform(
        data_in.header.frame_id, parameters_.world_frame, rclcpp::Time());
    }
    transform_force_world_ = p_tf_Buffer_->lookupTransform(
      parameters_.world_frame, parameters_.force_frame, rclcpp::Time());
    transform_cog_world_ = p_tf_Buffer_->lookupTransform(
      parameters_.world_frame, parameters_.sensor_frame, rclcpp::Time());
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 5000, "%s", ex.what());
    return false;  // if cannot transform, result of subsequent computations is invalid
  }

  // Transform data_in to world_frame frame
  geometry_msgs::msg::Wrench wrench_world;
  tf2::doTransform(data_in.wrench, wrench_world, transform_datain_world_);

  // Transform CoG Vector to world_frame frame
  geometry_msgs::msg::Vector3Stamped cog_transformed;
  tf2::doTransform(cog_, cog_transformed, transform_cog_world_);

  // Transform Ext force vector to world_frame frame
  geometry_msgs::msg::Vector3Stamped force_transformed;
  tf2::doTransform(force_, force_transformed, transform_force_world_);

  // Compensate for gravity force
  wrench_world.force.x -= force_transformed.vector.x;
  wrench_world.force.y -= force_transformed.vector.y;
  wrench_world.force.z -= force_transformed.vector.z;
  // Compensation values for torque result from cross-product of cog Vector and force
  tf2::Vector3 cog_vector = {cog_transformed.vector.x, cog_transformed.vector.y,
    cog_transformed.vector.z};
  auto added_torque = cog_vector.cross({force_transformed.vector.x,
                                        force_transformed.vector.y,
                                        force_transformed.vector.z});
  wrench_world.torque.x -= added_torque.getX();
  wrench_world.torque.y -= added_torque.getY();
  wrench_world.torque.z -= added_torque.getZ();
  // Transform wrench_world to data_out frame_id if not empty otherwise to data_in frame id
  tf2::doTransform(wrench_world, data_out.wrench, transform_world_dataout_);

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::GravityCompensation<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
