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

#ifndef CONTROL_FILTERS__TEST_RATE_LIMITER_HPP_
#define CONTROL_FILTERS__TEST_RATE_LIMITER_HPP_

#include <memory>
#include <thread>
#include "gmock/gmock.h"

#include "control_filters/rate_limiter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_rate_limiter");
}  // namespace

class RateLimiterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    auto testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    node_ = std::make_shared<rclcpp::Node>(testname);
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  RateLimiterTest()
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
    node_.reset();
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
};

#endif  // CONTROL_FILTERS__TEST_RATE_LIMITER_HPP_
