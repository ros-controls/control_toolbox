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

#include "test_low_pass_filter.hpp"

TEST_F(LowPassFilterTest, TestLowPassFilterParameters)
{
    std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
        std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();

    node_->declare_parameter("damping_frequency", 20.5);
    node_->declare_parameter("damping_intensity", 1.25);
    node_->declare_parameter("divider", 3);

    // should deny configuration as sampling frequency is missing
    ASSERT_FALSE(filter_->configure("", "TestLowPassFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    node_->set_parameter(rclcpp::Parameter("sampling_frequency", 0.0));
    // should deny configuration as sampling frequency is invalid
    ASSERT_FALSE(filter_->configure("", "TestLowPassFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    node_->set_parameter(rclcpp::Parameter("sampling_frequency", 1000.0));
     // should allow configuration and pass second call to unconfigured filter
    ASSERT_TRUE(filter_->configure("", "TestLowPassFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    // change a parameter
    node_->set_parameter(rclcpp::Parameter("sampling_frequency", 500.0));
    // accept second call to configure with valid parameters to already configured filter
    ASSERT_TRUE(filter_->configure("", "TestLowPassFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));
}

TEST_F(LowPassFilterTest, TestLowPassFilter)
{
    double sampling_freq = 1000.0;
    double damping_freq = 20.5;
    double damping_intensity = 1.25;
    int divider = 3;

    double a1, b1;
    a1 = exp(-1.0 / sampling_freq * (2.0 * M_PI * damping_freq) /
        (pow(10.0, damping_intensity / -10.0)));
    b1 = 1.0 - a1;

    geometry_msgs::msg::WrenchStamped in, calculated, out;
    in.header.frame_id = "world";
    in.wrench.force.x = 1.0;
    in.wrench.torque.x = 10.0;

    std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
        std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();

    // not yet configured, should deny update
    ASSERT_FALSE(filter_->update(in, out));

    // declare parameters and configure
    node_->declare_parameter("sampling_frequency", sampling_freq);
    node_->declare_parameter("damping_frequency", damping_freq);
    node_->declare_parameter("damping_intensity", damping_intensity);
    node_->declare_parameter("divider", divider);
    ASSERT_TRUE(filter_->configure("", "TestLowPassFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    // first filter pass, output should be zero as no old wrench was stored
    ASSERT_TRUE(filter_->update(in, out));
    ASSERT_EQ(out.wrench.force.x, 0.0);
    ASSERT_EQ(out.wrench.force.y, 0.0);
    ASSERT_EQ(out.wrench.force.z, 0.0);
    ASSERT_EQ(out.wrench.torque.x, 0.0);
    ASSERT_EQ(out.wrench.torque.y, 0.0);
    ASSERT_EQ(out.wrench.torque.z, 0.0);

    // second filter pass with same values:
        // calculate wrench by hand
        calculated.wrench.force.x = b1 * in.wrench.force.x;
        calculated.wrench.force.y = b1 * in.wrench.force.y;
        calculated.wrench.force.z = b1 * in.wrench.force.z;
        calculated.wrench.torque.x = b1 * in.wrench.torque.x;
        calculated.wrench.torque.y = b1 * in.wrench.torque.y;
        calculated.wrench.torque.z = b1 * in.wrench.torque.z;
        // check equality with low-pass-filter
        ASSERT_TRUE(filter_->update(in, out));
        ASSERT_EQ(out.wrench.force.x, calculated.wrench.force.x);
        ASSERT_EQ(out.wrench.force.y, calculated.wrench.force.y);
        ASSERT_EQ(out.wrench.force.z, calculated.wrench.force.z);
        ASSERT_EQ(out.wrench.torque.x, calculated.wrench.torque.x);
        ASSERT_EQ(out.wrench.torque.y, calculated.wrench.torque.y);
        ASSERT_EQ(out.wrench.torque.z, calculated.wrench.torque.z);

    // update once again and check results
        // calculate wrench by hand
        calculated.wrench.force.x = b1 * in.wrench.force.x + a1 * calculated.wrench.force.x;
        calculated.wrench.force.y = b1 * in.wrench.force.y + a1 * calculated.wrench.force.y;
        calculated.wrench.force.z = b1 * in.wrench.force.z + a1 * calculated.wrench.force.z;
        calculated.wrench.torque.x = b1 * in.wrench.torque.x + a1 * calculated.wrench.torque.x;
        calculated.wrench.torque.y = b1 * in.wrench.torque.y + a1 * calculated.wrench.torque.y;
        calculated.wrench.torque.z = b1 * in.wrench.torque.z + a1 * calculated.wrench.torque.z;
        // check equality with low-pass-filter
        ASSERT_TRUE(filter_->update(in, out));
        ASSERT_EQ(out.wrench.force.x, calculated.wrench.force.x);
        ASSERT_EQ(out.wrench.force.y, calculated.wrench.force.y);
        ASSERT_EQ(out.wrench.force.z, calculated.wrench.force.z);
        ASSERT_EQ(out.wrench.torque.x, calculated.wrench.torque.x);
        ASSERT_EQ(out.wrench.torque.y, calculated.wrench.torque.y);
        ASSERT_EQ(out.wrench.torque.z, calculated.wrench.torque.z);
}
