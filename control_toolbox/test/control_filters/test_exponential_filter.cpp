// Copyright (c) 2024, AIT Austrian Institute of Technology GmbH
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
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "gmock/gmock.h"

#include "control_filters/exponential_filter.hpp"
#include "control_toolbox/exponential_filter.hpp"

TEST_F(FilterTest, TestExponentialFilterThrowsUnconfigured)
{
  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<double>>();
  double in = 42., out;
  ASSERT_THROW(filter_->update(in, out), std::runtime_error);
}

TEST_F(FilterTest, TestExponentialFilterInvalidParameterValue)
{
  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<double>>();
  ASSERT_FALSE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestExponentialFilterComputation)
{
  // parameters should match the test yaml file
  double alpha = 0.7;

  double in = 1.0, calculated, out;

  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<double>>();

  // configure
  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  // first filter pass, output should be input value as no old value was stored
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out, 1.0);

  // second filter pass with same values: no change
  // check equality with low-pass-filter
  ASSERT_TRUE(filter_->update(in, out));
  calculated = in;
  ASSERT_EQ(calculated, out);

  // input change
  in = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    ASSERT_TRUE(filter_->update(in, out));
    calculated = alpha * in + (1 - alpha) * calculated;
    ASSERT_EQ(calculated, out);
  }
}

TEST_F(FilterTest, TestExponentialWrenchFilterComputation)
{
  double alpha = 0.6;
  node_->declare_parameter("alpha", rclcpp::ParameterValue(alpha));
  geometry_msgs::msg::WrenchStamped in, calculated, out;
  in.header.frame_id = "world";
  in.wrench.force.x = 1.0;
  in.wrench.force.y = 2.0;
  in.wrench.force.z = 3.0;
  in.wrench.torque.x = 10.0;
  in.wrench.torque.y = 20.0;
  in.wrench.torque.z = 30.0;

  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<geometry_msgs::msg::WrenchStamped>>();

  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  // First filter pass, output should be equal to the input
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out.wrench.force.x, in.wrench.force.x);
  ASSERT_EQ(out.wrench.force.y, in.wrench.force.y);
  ASSERT_EQ(out.wrench.force.z, in.wrench.force.z);
  ASSERT_EQ(out.wrench.torque.x, in.wrench.torque.x);
  ASSERT_EQ(out.wrench.torque.y, in.wrench.torque.y);
  ASSERT_EQ(out.wrench.torque.z, in.wrench.torque.z);

  // Initialize calculated with first output
  calculated = out;

  // Second filter pass with a step in values
  in.wrench.force.x = 2.0;
  in.wrench.force.y = 4.0;
  in.wrench.force.z = 6.0;
  in.wrench.torque.x = 20.0;
  in.wrench.torque.y = 40.0;
  in.wrench.torque.z = 60.0;
  ASSERT_TRUE(filter_->update(in, out));

  // Manual exponential smoothing calculation for all fields
  calculated.wrench.force.x = alpha * in.wrench.force.x + (1.0 - alpha) * calculated.wrench.force.x;
  calculated.wrench.force.y = alpha * in.wrench.force.y + (1.0 - alpha) * calculated.wrench.force.y;
  calculated.wrench.force.z = alpha * in.wrench.force.z + (1.0 - alpha) * calculated.wrench.force.z;
  calculated.wrench.torque.x =
    alpha * in.wrench.torque.x + (1.0 - alpha) * calculated.wrench.torque.x;
  calculated.wrench.torque.y =
    alpha * in.wrench.torque.y + (1.0 - alpha) * calculated.wrench.torque.y;
  calculated.wrench.torque.z =
    alpha * in.wrench.torque.z + (1.0 - alpha) * calculated.wrench.torque.z;

  // Third filter pass, check against manual calculation
  ASSERT_TRUE(filter_->update(in, out));
  calculated.wrench.force.x = alpha * in.wrench.force.x + (1.0 - alpha) * calculated.wrench.force.x;
  calculated.wrench.force.y = alpha * in.wrench.force.y + (1.0 - alpha) * calculated.wrench.force.y;
  calculated.wrench.force.z = alpha * in.wrench.force.z + (1.0 - alpha) * calculated.wrench.force.z;
  calculated.wrench.torque.x =
    alpha * in.wrench.torque.x + (1.0 - alpha) * calculated.wrench.torque.x;
  calculated.wrench.torque.y =
    alpha * in.wrench.torque.y + (1.0 - alpha) * calculated.wrench.torque.y;
  calculated.wrench.torque.z =
    alpha * in.wrench.torque.z + (1.0 - alpha) * calculated.wrench.torque.z;

  ASSERT_NEAR(out.wrench.force.x, calculated.wrench.force.x, 1e-9);
  ASSERT_NEAR(out.wrench.force.y, calculated.wrench.force.y, 1e-9);
  ASSERT_NEAR(out.wrench.force.z, calculated.wrench.force.z, 1e-9);
  ASSERT_NEAR(out.wrench.torque.x, calculated.wrench.torque.x, 1e-9);
  ASSERT_NEAR(out.wrench.torque.y, calculated.wrench.torque.y, 1e-9);
  ASSERT_NEAR(out.wrench.torque.z, calculated.wrench.torque.z, 1e-9);
}

TEST_F(FilterTest, TestExponentialWrenchFilterMissingParameter)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<geometry_msgs::msg::WrenchStamped>>();

  // should deny configuration as alpha is missing
  ASSERT_FALSE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestExponentialWrenchFilterInvalidThenFixedParameter)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<geometry_msgs::msg::WrenchStamped>>();

  // should deny configuration as alpha is invalid
  ASSERT_FALSE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  // fix the param
  node_->set_parameter(rclcpp::Parameter("alpha", 0.5));
  // should allow configuration and pass second call to unconfigured filter
  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestExponentialWrenchFilterThrowsUnconfigured)
{
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<geometry_msgs::msg::WrenchStamped>>();
  geometry_msgs::msg::WrenchStamped in, out;
  ASSERT_THROW(filter_->update(in, out), std::runtime_error);
}

TEST_F(FilterTest, TestExponentialVectorDoubleFilterComputation)
{
  // parameters should match the test yaml file
  double alpha = 0.6;

  std::vector<double> in{1.0, 2.0, 3.0};
  std::vector<double> calculated, out;
  calculated.resize(in.size());
  out.resize(in.size());

  std::shared_ptr<filters::FilterBase<std::vector<double>>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<std::vector<double>>>();

  node_->declare_parameter("alpha", rclcpp::ParameterValue(alpha));
  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_TRUE(std::equal(in.begin(), in.end(), out.begin()));

  // second filter pass with a step in values (otherwise there is nothing to filter):
  in = {2.0, 4.0, 6.0};
  ASSERT_TRUE(filter_->update(in, out));

  // update once again and check results
  // calculate vector by hand using exponential smoothing: y[n] = α * x[n] + (1-α) * y[n-1]
  calculated[0] = alpha * in[0] + (1.0 - alpha) * out[0];
  calculated[1] = alpha * in[1] + (1.0 - alpha) * out[1];
  calculated[2] = alpha * in[2] + (1.0 - alpha) * out[2];
  // check equality with exponential filter
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_TRUE(std::equal(out.begin(), out.end(), calculated.begin()));
}

TEST_F(FilterTest, TestExponentialFilterAlphaZero)
{
  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<double>>();

  node_->declare_parameter("alpha", rclcpp::ParameterValue(0.0));
  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  double in = 10.0, out;
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out, 10.0);  // First call initializes with input

  in = 20.0;
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out, 10.0);  // Should remain at initial value with alpha=0
}

TEST_F(FilterTest, TestExponentialFilterAlphaOne)
{
  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<double>>();

  node_->declare_parameter("alpha", rclcpp::ParameterValue(1.0));
  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  double in = 10.0, out;
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out, 10.0);

  in = 20.0;
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out, 20.0);  // Should track input exactly with alpha=1
}

TEST_F(FilterTest, TestExponentialFilterParameterUpdate)
{
  std::shared_ptr<filters::FilterBase<double>> filter_ =
    std::make_shared<control_filters::ExponentialFilter<double>>();

  node_->declare_parameter("alpha", rclcpp::ParameterValue(0.5));
  ASSERT_TRUE(filter_->configure(
    "", "TestExponentialFilter", node_->get_node_logging_interface(),
    node_->get_node_parameters_interface()));

  double in = 10.0, out;
  ASSERT_TRUE(filter_->update(in, out));
  ASSERT_EQ(out, 10.0);

  // Change parameter
  node_->set_parameter(rclcpp::Parameter("alpha", 0.8));
  in = 20.0;
  ASSERT_TRUE(filter_->update(in, out));
  // Should use new alpha value: 0.8 * 20 + 0.2 * 10 = 18
  ASSERT_EQ(out, 18.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
