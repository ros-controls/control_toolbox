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

#ifndef CONTROL_FILTERS__LOW_PASS_FILTER_HPP
#define CONTROL_FILTERS__LOW_PASS_FILTER_HPP

#include <cmath>
#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "filters/filter_base.hpp"
#include "filters/filter_chain.hpp"
#include "rclcpp/node.hpp"


namespace control_filters
{
template <typename T>
class LowPassFilter : public filters::FilterBase<T>
{
public:
    LowPassFilter();

    ~LowPassFilter() override;

    bool configure() override;

    bool update(const T& data_in, T& data_out) override;

    /** \brief Get most recent parameters */
    bool updateParameters();

private:
    /** \brief Dynamic parameter callback activated when parameters change */
    void parameterCallback();
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;

    // Parameters
    double sampling_frequency_;
    double damping_frequency_;
    double damping_intensity_;
    int divider_;

    // Filter parameters
    double b1;
    double a1;
    double filtered_value, filtered_old_value, old_value;
    Eigen::Matrix<double, 6, 1> msg_filtered, msg_filtered_old, msg_old;

    // dynamic parameters
    bool params_updated_;
    bool initialized_;
    bool configured_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
  : logger_(rclcpp::get_logger("LowPassFilter")), params_updated_(false), initialized_(false), configured_(false)
{
}

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}

template <typename T>
bool LowPassFilter<T>::configure()
{
    if (!initialized_)
    {
        RCLCPP_INFO(logger_, "Node is not initialized... call setNode()");
        return false;
    }
    
    if (!updateParameters())
    {
        return false;
    }
    
    configured_ = true; 

    RCLCPP_INFO(logger_,
                "Low Pass Filter Params: sampling frequency: %f; damping frequency: %f; damping intensity :%f; "
                "divider :%d",
                sampling_frequency_, damping_frequency_, damping_intensity_, divider_);

    // Initialize storage Vectors
    filtered_value = filtered_old_value = old_value = 0;
    for (unsigned int i = 0; i < 6; i++)
    {
        msg_filtered(i) = msg_filtered_old(i) = msg_old(i) = 0;
    }

    return true;
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(const geometry_msgs::msg::WrenchStamped& data_in,
                                                                     geometry_msgs::msg::WrenchStamped& data_out)
{
    if (!configured_)
    {
        RCLCPP_ERROR(logger_, "Filter is not configured");
        return false;
    }

    if (params_updated_)
    {
        updateParameters();
    }

    // IIR Filter
    msg_filtered = b1 * msg_old + a1 * msg_filtered_old;
    msg_filtered_old = msg_filtered;

    // TODO use wrenchMsgToEigen
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
bool LowPassFilter<T>::update(const T& data_in, T& data_out)
{
    if (!configured_)
    {
        RCLCPP_ERROR(logger_, "Filter is not configured");
        return false;
    }

    if (params_updated_)
    {
        if (!updateParameters()) 
        {
            RCLCPP_ERROR(logger_, "Unable to update as not all parameters are set");
            return false;
        }
    }

    data_out = b1 * old_value + a1 * filtered_old_value;
    filtered_old_value = data_out;
    old_value = data_in;

    return true;
}

template <typename T>
bool LowPassFilter<T>::updateParameters()
{
    
    if (!filters::FilterBase<T>::getParam("sampling_frequency", sampling_frequency_)) {
        RCLCPP_ERROR(logger_, "Low pass filter did not find parameter sampling_frequency");
        return false;
    }
    if (!filters::FilterBase<T>::getParam("damping_frequency", damping_frequency_)) {
        RCLCPP_ERROR(logger_, "Low pass filter did not find parameter damping_frequency");
        return false;
    }    
    if (!filters::FilterBase<T>::getParam("damping_intensity", damping_intensity_)) {
        RCLCPP_ERROR(logger_, "Low pass filter did not find parameter damping_intensity");
        return false;
    }    
    if (!filters::FilterBase<T>::getParam("divider", divider_)) {
        RCLCPP_ERROR(logger_, "Low pass filter did not find parameter divider");
        return false;
    }
    
    params_updated_ = false;
    a1 = exp(-1.0 / sampling_frequency_ * (2.0 * M_PI * damping_frequency_) / (pow(10.0, damping_intensity_ / -10.0)));
    b1 = 1.0 - a1;
    return true;
}

template <typename T>
void LowPassFilter<T>::parameterCallback()
{
    params_updated_ = true;
}

}  // namespace iirob_filters
#endif
