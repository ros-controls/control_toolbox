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

#ifndef CONTROL_FILTERS__TEST_LOW_PASS_FILTER_HPP_
#define CONTROL_FILTERS__TEST_LOW_PASS_FILTER_HPP_

#include <memory>
#include <thread>
#include "gmock/gmock.h"

#include "control_filters/low_pass_filter.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_low_pass_filter");
}  // namespace

class LowPassFilterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  LowPassFilterTest()
  : node_(std::make_shared<rclcpp::Node>("test_low_pass_filter")),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  control_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped> low_pass_filter_;
  std::thread executor_thread_;
};

#endif  // CONTROL_FILTERS__TEST_LOW_PASS_FILTER_HPP_
