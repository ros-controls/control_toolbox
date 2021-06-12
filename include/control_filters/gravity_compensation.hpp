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

#ifndef CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP
#define CONTROL_FILTERS__GRAVITY_COMPENSATION_HPP

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "filters/filter_base.hpp"

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
    virtual bool configure() override;

    /** \brief Update the filter and return the data seperately
     * \param data_in T array with length width
     * \param data_out T array with length width
     */
    virtual bool update(const T& data_in, T& data_out) override;

    /** \brief Get most recent parameters */
    bool updateParameters();

  private:
    /** \brief Dynamic parameter callback activated when parameters change */
    //  void parameterCallback();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    // Storage for Calibration Values
    geometry_msgs::msg::Vector3Stamped cog_;  // Center of Gravity Vector (wrt Sensor Frame)
    double force_z_;                          // Gravitational Force

    // Frames for Transformation of Gravity / CoG Vector
    std::string world_frame_;
    std::string sensor_frame_;
    std::string force_frame_;

    // tf2 objects
    std::unique_ptr<tf2_ros::Buffer> p_tf_Buffer_;
    std::unique_ptr<tf2_ros::TransformListener> p_tf_Listener_;
    geometry_msgs::msg::TransformStamped transform_, transform_back_, transform_cog_;

    bool configured_;

    uint num_transform_errors_;
  };

  template <typename T>
  GravityCompensation<T>::GravityCompensation()
  : logger_(rclcpp::get_logger("GravityCompensation"))
  , configured_(false)
  , num_transform_errors_(0)
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

    if(!updateParameters()){
      return false;
    }
    else{
      configured_ = true;
    }
    RCLCPP_INFO(logger_,
                "Gravity Compensation Params: world frame: %s; sensor frame: %s; force frame: %s; CoG x:%f; "
                "CoG y:%f; CoG z:%f; force: %f.",
                world_frame_.c_str(), sensor_frame_.c_str(), force_frame_.c_str(), cog_.vector.x, cog_.vector.y, cog_.vector.z, force_z_);

    return true;
  }

  template <typename T>
  bool GravityCompensation<T>::update(const T& data_in, T& data_out)
  {
    if (!configured_)
    {
      RCLCPP_ERROR(logger_, "Filter is not configured");
      return false;
    }

    //if (params_updated_)
    //{
    //  updateParameters();
    //}

    try
    {
      transform_ = p_tf_Buffer_->lookupTransform(world_frame_, data_in.header.frame_id, rclcpp::Time());
      transform_back_ = p_tf_Buffer_->lookupTransform(data_in.header.frame_id, world_frame_, rclcpp::Time());
      transform_cog_ = p_tf_Buffer_->lookupTransform(world_frame_,  force_frame_, rclcpp::Time());
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger_, *clock_, 5000, "%s", ex.what());
    }

    geometry_msgs::msg::Vector3Stamped temp_force_transformed, temp_torque_transformed, temp_vector_in, temp_vector_out;

    temp_vector_in.vector = data_in.wrench.force;
    tf2::doTransform(temp_vector_in, temp_force_transformed, transform_);

    temp_vector_in.vector = data_in.wrench.torque;
    tf2::doTransform(temp_vector_in, temp_torque_transformed, transform_);

    // Transform CoG Vector
    geometry_msgs::msg::Vector3Stamped cog_transformed;
    tf2::doTransform(cog_, cog_transformed, transform_cog_);

    // Compensate for gravity force
    temp_force_transformed.vector.z += force_z_;
    // Compensation Values for torque result from Crossprod of cog Vector and (0 0 G)
    temp_torque_transformed.vector.x += (force_z_ * cog_transformed.vector.y);
    temp_torque_transformed.vector.y -= (force_z_ * cog_transformed.vector.x);

    // Copy Message and Compensate values for Gravity Force and Resulting Torque
    // geometry_msgs::WrenchStamped compensated_wrench;
    data_out = data_in;

    tf2::doTransform(temp_force_transformed, temp_vector_out, transform_back_);
    data_out.wrench.force = temp_vector_out.vector;

    tf2::doTransform(temp_torque_transformed, temp_vector_out, transform_back_);
    data_out.wrench.torque = temp_vector_out.vector;

    return true;
  }

  template <typename T>
  bool GravityCompensation<T>::updateParameters()
  {
    //params_updated_ = false;

    if (!filters::FilterBase<T>::getParam("world_frame", world_frame_)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param world_frame_");
      return false;
    }
    if (!filters::FilterBase<T>::getParam("sensor_frame", sensor_frame_)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param sensor_frame");
      return false;
    }
    if (!filters::FilterBase<T>::getParam("force_frame", force_frame_)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param force_frame");
      return false;
    }
    if (!filters::FilterBase<T>::getParam("CoG_x", cog_.vector.x)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param CoG_x");
      return false;
    }
    if (!filters::FilterBase<T>::getParam("CoG_y", cog_.vector.y)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param CoG_y");
      return false;
    }
    if (!filters::FilterBase<T>::getParam("CoG_z", cog_.vector.z)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param CoG_z");
      return false;
    }
    if (!filters::FilterBase<T>::getParam("force", force_z_)) {
      RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
                   "Gravitiy Compensator did not find param force");
      return false;
    }
    return true;
  }

  //template <typename T>
  //void GravityCompensation<T>::parameterCallback()
  //{
  //  params_updated_ = true;
  //}

}  // namespace iirob_filters
#endif
