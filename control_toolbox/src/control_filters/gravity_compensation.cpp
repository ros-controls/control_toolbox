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
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace control_filters
{
template <>
bool GravityCompensation<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    compute_internal_params();
  }

  try
  {
    // transform from data_in frame to sensor_frame
    transform_sensor_datain_ = p_tf_Buffer_->lookupTransform(
      parameters_.sensor_frame, data_in.header.frame_id, rclcpp::Time());

    // use data_out frame id for the back transformation, otherwise same is data_in
    if (!data_out.header.frame_id.empty())
    {
      data_out.header.stamp = data_in.header.stamp;  // only copy the timestamp
    }
    else
    {
      data_out.header = data_in.header;  // keep the same header and same frame_id
    }
    transform_data_out_sensor_ = p_tf_Buffer_->lookupTransform(
      data_out.header.frame_id, parameters_.sensor_frame, rclcpp::Time());
    // transform from world (gravity) frame to sensor frame
    transform_sensor_world_ = p_tf_Buffer_->lookupTransform(
      parameters_.sensor_frame, parameters_.world_frame, rclcpp::Time());
  }
  catch (const tf2::TransformException & ex)
  {
    std::stringstream frames_sstr;
    frames_sstr << "datain:" << data_in.header.frame_id << " dataout:" << data_out.header.frame_id;
    frames_sstr << " world:" << parameters_.world_frame << " sensor:" << parameters_.sensor_frame;
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      (*logger_), *clock_, 5000, "GravityCompensation update failed:%s, given frames are %s",
      ex.what(), frames_sstr.str().c_str());
    return false;  // if cannot transform, result of subsequent computations is invalid
  }

  // Transform data_in to sensor_frame frame
  geometry_msgs::msg::Wrench wrench_sensor;
  tf2::doTransform(data_in.wrench, wrench_sensor, transform_sensor_datain_);

  // CoG is already in sensor_frame

  // Rotate (no wrench, just a force) the gravity force to sensor frame
  geometry_msgs::msg::Vector3Stamped cst_ext_force_transformed;
  tf2::doTransform(cst_ext_force_, cst_ext_force_transformed, transform_sensor_world_);

  // Compensate for gravity force in sensor frame
  wrench_sensor.force.x -= cst_ext_force_transformed.vector.x;
  wrench_sensor.force.y -= cst_ext_force_transformed.vector.y;
  wrench_sensor.force.z -= cst_ext_force_transformed.vector.z;
  // Compensate for torque produced by offset CoG in sensor frame
  // result from cross-product of cog Vector and force
  tf2::Vector3 cog_vector = {cog_.vector.x, cog_.vector.y, cog_.vector.z};
  auto added_torque = cog_vector.cross(
    {cst_ext_force_transformed.vector.x, cst_ext_force_transformed.vector.y,
     cst_ext_force_transformed.vector.z});
  wrench_sensor.torque.x -= added_torque.getX();
  wrench_sensor.torque.y -= added_torque.getY();
  wrench_sensor.torque.z -= added_torque.getZ();
  // Transform wrench_world to data_out frame_id if not empty otherwise to data_in frame id
  tf2::doTransform(wrench_sensor, data_out.wrench, transform_data_out_sensor_);

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::GravityCompensation<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
