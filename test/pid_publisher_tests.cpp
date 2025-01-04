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

#include "control_toolbox/pid_ros.hpp"

#include "gtest/gtest.h"

#include "rclcpp/duration.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using PidStateMsg = control_msgs::msg::PidState;
using rclcpp::executors::MultiThreadedExecutor;

TEST(PidPublisherTest, PublishTest)
{
  const size_t ATTEMPTS = 100;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pid_publisher_test");

  control_toolbox::PidROS pid_ros = control_toolbox::PidROS(node);

  pid_ros.initPid(1.0, 1.0, 1.0, 5.0, -5.0, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;
  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr) {
    callback_called = true;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.computeCommand(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.5, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i) {
    pid_ros.computeCommand(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callback_called);
}

TEST(PidPublisherTest, PublishTestLifecycle)
{
  const size_t ATTEMPTS = 100;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("pid_publisher_test");

  control_toolbox::PidROS pid_ros(node);

  auto state_pub_lifecycle_ =
    std::dynamic_pointer_cast<rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::PidState>>(
      pid_ros.getPidStatePublisher());

  pid_ros.initPid(1.0, 1.0, 1.0, 5.0, -5.0, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;
  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr) {
    callback_called = true;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.computeCommand(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.5, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i) {
    pid_ros.computeCommand(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(callback_called);

  node->shutdown();  // won't be called in destructor
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
