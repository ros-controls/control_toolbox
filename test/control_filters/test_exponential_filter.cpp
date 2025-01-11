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
#include "gmock/gmock.h"

#include "control_filters/exponential_filter.hpp"

TEST_F(FilterTest, TestExponentialFilterThrowsUnconfigured)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::ExponentialFilter<double>>();
    double in, out;
    ASSERT_THROW(filter_->update(in, out), std::runtime_error);
}

TEST_F(FilterTest, TestExponentialFilterInvalidParameterValue)
{
    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::ExponentialFilter<double>>();
    ASSERT_FALSE(filter_->configure("", "TestExponentialFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));
}

TEST_F(FilterTest, TestExponentialFilterComputation)
{
    // parameters should match the test yaml file
    double alpha = 0.7;

    double in = 1.0, calculated, out;

    std::shared_ptr<filters::FilterBase<double>> filter_ =
        std::make_shared<control_filters::ExponentialFilter<double>>();

    // configure
    ASSERT_TRUE(filter_->configure("", "TestExponentialFilter",
        node_->get_node_logging_interface(), node_->get_node_parameters_interface()));

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
    for (int i = 0; i < 100; ++i){
      ASSERT_TRUE(filter_->update(in, out));
      calculated = alpha * in + (1 - alpha) * calculated;
      ASSERT_EQ(calculated, out);}
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
