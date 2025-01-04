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

#ifndef CONTROL_FILTERS__TEST_FILTER_UTIL_HPP_
#define CONTROL_FILTERS__TEST_FILTER_UTIL_HPP_

#include <memory>
#include "gmock/gmock.h"

#include "rclcpp/node.hpp"
#include "rclcpp/logger.hpp"

class FilterTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    auto testname = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    node_ = std::make_shared<rclcpp::Node>(testname);
  }

  FilterTest()
  {
  }

  void TearDown() override
  {
    node_.reset();
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

#endif  // CONTROL_FILTERS__TEST_FILTER_UTIL_HPP_
