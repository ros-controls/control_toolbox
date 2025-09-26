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

#include "gmock/gmock.h"

#include "rclcpp/duration.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using control_toolbox::AntiWindupStrategy;
using PidStateMsg = control_msgs::msg::PidState;
using rclcpp::executors::MultiThreadedExecutor;

TEST(PidPublisherTest, PublishTest)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pid_publisher_test");

  control_toolbox::PidROS pid_ros = control_toolbox::PidROS(node, "", "", true);

  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::NONE;
  antiwindup_strat.i_max = 5.0;
  antiwindup_strat.i_min = -5.0;
  antiwindup_strat.tracking_time_constant = 1.0;
  pid_ros.initialize_from_args(1.0, 1.0, 1.0, 5.0, -5.0, antiwindup_strat, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;

  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr msg)
  {
    callback_called = true;
    last_state_msg = msg;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.0, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callback_called);
  ASSERT_NE(nullptr, last_state_msg);
  EXPECT_DOUBLE_EQ(last_state_msg->output, command);
}

TEST(PidPublisherTest, PublishTest_start_deactivated)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pid_publisher_test");

  control_toolbox::PidROS pid_ros = control_toolbox::PidROS(node, "", "", false);

  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::NONE;
  antiwindup_strat.i_max = 5.0;
  antiwindup_strat.i_min = -5.0;
  antiwindup_strat.tracking_time_constant = 1.0;
  pid_ros.initialize_from_args(1.0, 1.0, 1.0, 5.0, -5.0, antiwindup_strat, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;

  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr msg)
  {
    callback_called = true;
    last_state_msg = msg;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.0, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_FALSE(callback_called);
  ASSERT_EQ(nullptr, last_state_msg);

  // activate publisher
  rcl_interfaces::msg::SetParametersResult set_result;
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("activate_state_publisher", true)));
  ASSERT_TRUE(set_result.successful);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(callback_called);
  ASSERT_NE(nullptr, last_state_msg);
  // deactivate publisher again
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("activate_state_publisher", false)));
  ASSERT_TRUE(set_result.successful);
  rclcpp::spin_some(node);  // process callbacks to ensure that no further messages are received
  callback_called = false;
  last_state_msg.reset();

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_FALSE(callback_called);
  ASSERT_EQ(nullptr, last_state_msg);
}

TEST(PidPublisherTest, PublishTest_prefix)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pid_publisher_test");

  // test with a prefix for the topic without trailing / (should be auto-added)
  control_toolbox::PidROS pid_ros = control_toolbox::PidROS(node, "", "global", true);

  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::NONE;
  antiwindup_strat.i_max = 5.0;
  antiwindup_strat.i_min = -5.0;
  antiwindup_strat.tracking_time_constant = 1.0;
  pid_ros.initialize_from_args(1.0, 1.0, 1.0, 5.0, -5.0, antiwindup_strat, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;

  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr msg)
  {
    callback_called = true;
    last_state_msg = msg;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/global/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.0, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callback_called);
  ASSERT_NE(nullptr, last_state_msg);
  EXPECT_DOUBLE_EQ(last_state_msg->output, command);
}

TEST(PidPublisherTest, PublishTest_local_prefix)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pid_publisher_test");

  control_toolbox::PidROS pid_ros = control_toolbox::PidROS(node, "", "~/local/", true);

  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::NONE;
  antiwindup_strat.i_max = 5.0;
  antiwindup_strat.i_min = -5.0;
  antiwindup_strat.tracking_time_constant = 1.0;
  pid_ros.initialize_from_args(1.0, 1.0, 1.0, 5.0, -5.0, antiwindup_strat, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;

  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr msg)
  {
    callback_called = true;
    last_state_msg = msg;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "~/local/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.0, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callback_called);
  ASSERT_NE(nullptr, last_state_msg);
  EXPECT_DOUBLE_EQ(last_state_msg->output, command);
}

TEST(PidPublisherTest, ComputeCommandWithErrorDotPublishesAndMatchesOutput)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp::Node>("pidros_compute_cmd_pub_test");

  // Enable publisher so pid_state messages are emitted
  control_toolbox::PidROS pid_ros(node, "", "", /*activate_state_publisher=*/true);

  // Gains: P=1, I=0, D=1  → expected cmd = e + e_dot
  AntiWindupStrategy anti;
  anti.type = AntiWindupStrategy::NONE;
  const double U_MAX = 1e9, U_MIN = -1e9;
  ASSERT_TRUE(
    pid_ros.initialize_from_args(1.0, 0.0, 1.0, U_MAX, U_MIN, anti, /*save_i_term=*/false));

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/pid_state", rclcpp::SensorDataQoS(),
    [&](const control_msgs::msg::PidState::SharedPtr msg)
    {
      callback_called = true;
      last_state_msg = msg;
    });

  const double error = 2.0;
  const double error_dot = 3.0;
  const rclcpp::Duration dt(0, 100000000);  // 0.1 s

  // Expected: 1*e + 0*i + 1*e_dot = 5
  const double cmd = pid_ros.compute_command(error, error_dot, dt);
  EXPECT_EQ(5.0, cmd);

  // Create an executor to service callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Wait for the message to be delivered
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    executor.spin_some();
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callback_called);
  ASSERT_NE(nullptr, last_state_msg);

  // Basic content checks (PidState fields)
  EXPECT_DOUBLE_EQ(last_state_msg->error, error);
  EXPECT_NEAR(rclcpp::Duration(last_state_msg->timestep).seconds(), dt.seconds(), 1e-9);
  EXPECT_DOUBLE_EQ(last_state_msg->output, cmd);
}

TEST(PidPublisherTest, PublishTestLifecycle)
{
  const size_t ATTEMPTS = 100;
  const std::chrono::milliseconds DELAY(250);

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("pid_publisher_test");

  control_toolbox::PidROS pid_ros(node, "", "", true);

  auto state_pub_lifecycle_ =
    std::dynamic_pointer_cast<rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::PidState>>(
      pid_ros.get_pid_state_publisher());

  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::NONE;
  antiwindup_strat.i_max = 5.0;
  antiwindup_strat.i_min = -5.0;
  antiwindup_strat.tracking_time_constant = 1.0;
  pid_ros.initialize_from_args(1.0, 1.0, 1.0, 5.0, -5.0, antiwindup_strat, false);

  bool callback_called = false;
  control_msgs::msg::PidState::SharedPtr last_state_msg;

  auto state_callback = [&](const control_msgs::msg::PidState::SharedPtr msg)
  {
    callback_called = true;
    last_state_msg = msg;
  };

  auto state_sub = node->create_subscription<control_msgs::msg::PidState>(
    "/pid_state", rclcpp::SensorDataQoS(), state_callback);

  double command = pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
  EXPECT_EQ(-1.0, command);

  // wait for callback
  for (size_t i = 0; i < ATTEMPTS && !callback_called; ++i)
  {
    pid_ros.compute_command(-0.5, rclcpp::Duration(1, 0));
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(callback_called);
  ASSERT_NE(nullptr, last_state_msg);
  EXPECT_DOUBLE_EQ(last_state_msg->output, command);

  node->shutdown();  // won't be called in destructor
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
