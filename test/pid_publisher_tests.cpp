// Copyright 2020 PAL Robotics SL
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


#include <chrono>
#include <cstddef>
#include <memory>
#include <thread>

#include "control_toolbox/pid.hpp"

#include "gtest/gtest.h"

#include "rclcpp/duration.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

using control_toolbox::Pid;
using control_toolbox::PidStateMsg;
using rclcpp::executors::MultiThreadedExecutor;

TEST(PidPublihserTest, PublishTest)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pid_publisher_test");

  Pid pid(1.0, 1.0, 1.0, 5.0, -5.0);

  ASSERT_NO_THROW(pid.initPublisher(node));

  bool callback_called = false;
  PidStateMsg::SharedPtr last_state_msg;
  auto state_callback = [&](const PidStateMsg::SharedPtr)
    {
      callback_called = true;
    };

  auto state_sub = node->create_subscription<PidStateMsg>("state", 10, state_callback);

  double command = pid.computeCommand(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.5, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callback_called);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  return RUN_ALL_TESTS();
}
