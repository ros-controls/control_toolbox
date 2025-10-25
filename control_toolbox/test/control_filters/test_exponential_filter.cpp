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
