// Copyright 2024 AIT - Austrian Institute of Technology GmbH
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

#include "test_rate_limiter.hpp"

TEST_F(RateLimiterTest, TestRateLimiterAllParameters)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::RateLimiter<double>>();

    // should allow configuration and find parameters in sensor_filter_chain param namespace
    ASSERT_TRUE(filter_->configure("", "TestRateLimiter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    // change a parameter
    node_->set_parameter(rclcpp::Parameter("sampling_interval", 0.5));
    // accept second call to configure with valid parameters to already configured filter
    ASSERT_TRUE(filter_->configure("", "TestRateLimiter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));
}


TEST_F(RateLimiterTest, TestRateLimiterMissingParameter)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::RateLimiter<double>>();

    // should deny configuration as sampling_interval is missing
    ASSERT_FALSE(filter_->configure("", "TestRateLimiter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));
}

TEST_F(RateLimiterTest, TestRateLimiterInvalidThenFixedParameter)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::RateLimiter<double>>();

    // should deny configuration as sampling_interval is invalid
    ASSERT_FALSE(filter_->configure("", "TestRateLimiter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    // fix the param
    node_->set_parameter(rclcpp::Parameter("sampling_interval", 1.0));
     // should allow configuration and pass second call to unconfigured filter
    ASSERT_TRUE(filter_->configure("", "TestRateLimiter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));
}

TEST_F(RateLimiterTest, TestRateLimiterThrowsUnconfigured)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::RateLimiter<double>>();
    double in, out;
    ASSERT_THROW(filter_->update(in, out), std::runtime_error);
}

TEST_F(RateLimiterTest, TestRateLimiterCompute)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::RateLimiter<double>>();

    ASSERT_TRUE(filter_->configure("", "TestRateLimiter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

    double in, out;
    ASSERT_NO_THROW(filter_->update(in, out));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
