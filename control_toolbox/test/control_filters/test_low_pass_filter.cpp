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

#include "test_filter_util.hpp"

#include <memory>
#include "gmock/gmock.h"

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "control_filters/low_pass_filter.hpp"

TEST_F(FilterTest, TestLowPassWrenchFilterAllParameters)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();

  // should allow configuration and find parameters in sensor_filter_chain param namespace
  ASSERT_TRUE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  // change a parameter
  node_->set_parameter(rclcpp::Parameter("sampling_frequency", 500.0));
  // accept second call to configure with valid parameters to already configured filter
  // will give a warning "Filter %s already being reconfigured"
  ASSERT_TRUE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestLowPassWrenchFilterMissingParameter)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();

  // should deny configuration as sampling frequency is missing
  ASSERT_FALSE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestLowPassWrenchFilterInvalidThenFixedParameter)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();

  // should deny configuration as sampling frequency is invalid
  ASSERT_FALSE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  // fix the param
  node_->set_parameter(rclcpp::Parameter("sampling_frequency", 1000.0));
  // should allow configuration and pass second call to unconfigured filter
  ASSERT_TRUE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestLowPassFilterThrowsUnconfigured)
{
  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::LowPassFilter<double>>();
  double in = 42., out;
  ASSERT_THROW(filter_->update(in, out), std::runtime_error);
}

TEST_F(FilterTest, TestLowPassWrenchFilterThrowsUnconfigured)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();
  geometry_msgs::msg::WrenchStamped in, out;
  ASSERT_THROW(filter_->update(in, out), std::runtime_error);
}

TEST_F(FilterTest, TestLowPassWrenchFilterComputation)
{
  // parameters should match the test yaml file
  double sampling_freq = 1000.0;
  double damping_freq = 20.5;
  double damping_intensity = 1.25;

  double a1, b1;
  a1 = exp(
    -1.0 / sampling_freq * (2.0 * M_PI * damping_freq) / (pow(10.0, damping_intensity / -10.0)));
  b1 = 1.0 - a1;

  geometry_msgs::msg::WrenchStamped in, calculated, out;
  in.header.frame_id = "world";
  in.wrench.force.x = 1.0;
  in.wrench.torque.x = 10.0;

  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped>>();

  // configure
  ASSERT_TRUE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  // first filter pass, output should be equal to the input
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out.wrench.force.x, in.wrench.force.x);
  ASSERT_EQ(out.wrench.force.y, in.wrench.force.y);
  ASSERT_EQ(out.wrench.force.z, in.wrench.force.z);
  ASSERT_EQ(out.wrench.torque.x, in.wrench.torque.x);
  ASSERT_EQ(out.wrench.torque.y, in.wrench.torque.y);
  ASSERT_EQ(out.wrench.torque.z, in.wrench.torque.z);

  // second filter pass with a step in values (otherwise there is nothing to filter):
  in.wrench.force.x = 2.0;
  in.wrench.torque.x = 20.0;
  ASSERT_TRUE(filter_->update(in, out));

  // update once again and check results
  // calculate wrench by hand
  calculated.wrench.force.x = b1 * in.wrench.force.x + a1 * out.wrench.force.x;
  calculated.wrench.force.y = b1 * in.wrench.force.y + a1 * out.wrench.force.y;
  calculated.wrench.force.z = b1 * in.wrench.force.z + a1 * out.wrench.force.z;
  calculated.wrench.torque.x = b1 * in.wrench.torque.x + a1 * out.wrench.torque.x;
  calculated.wrench.torque.y = b1 * in.wrench.torque.y + a1 * out.wrench.torque.y;
  calculated.wrench.torque.z = b1 * in.wrench.torque.z + a1 * out.wrench.torque.z;
  // check equality with low-pass-filter
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out.wrench.force.x, calculated.wrench.force.x);
  ASSERT_EQ(out.wrench.force.y, calculated.wrench.force.y);
  ASSERT_EQ(out.wrench.force.z, calculated.wrench.force.z);
  ASSERT_EQ(out.wrench.torque.x, calculated.wrench.torque.x);
  ASSERT_EQ(out.wrench.torque.y, calculated.wrench.torque.y);
  ASSERT_EQ(out.wrench.torque.z, calculated.wrench.torque.z);
}

TEST_F(FilterTest, TestLowPassVectorDoubleFilterComputation)
{
  // parameters should match the test yaml file
  double sampling_freq = 1000.0;
  double damping_freq = 20.5;
  double damping_intensity = 1.25;

  double a1, b1;
  a1 = exp(
    -1.0 / sampling_freq * (2.0 * M_PI * damping_freq) / (pow(10.0, damping_intensity / -10.0)));
  b1 = 1.0 - a1;

  std::vector<double> in{1.0, 2.0, 3.0};
  std::vector<double> calculated, out;
  calculated.resize(in.size());
  out.resize(in.size());

  std::shared_ptr<filters::FilterBase<std::vector<double>>> filter_ =
    std::make_shared<control_filters::LowPassFilter<std::vector<double>>>();

  node_->declare_parameter("sampling_frequency", rclcpp::ParameterValue(sampling_freq));
  node_->declare_parameter("damping_frequency", rclcpp::ParameterValue(damping_freq));
  node_->declare_parameter("damping_intensity", rclcpp::ParameterValue(damping_intensity));
  ASSERT_TRUE(filter_->configure(
    "", "TestLowPassFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_TRUE(std::equal(in.begin(), in.end(), out.begin()));

  // second filter pass with a step in values (otherwise there is nothing to filter):
  in = {2.0, 4.0, 6.0};
  ASSERT_TRUE(filter_->update(in, out));

  // update once again and check results
  // calculate wrench by hand
  calculated[0] = b1 * in[0] + a1 * out[0];
  calculated[1] = b1 * in[1] + a1 * out[1];
  calculated[2] = b1 * in[2] + a1 * out[2];
  // check equality with low-pass-filter
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_TRUE(std::equal(out.begin(), out.end(), calculated.begin()));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
