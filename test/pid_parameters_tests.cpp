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
#include <gtest/gtest.h>

#include <memory>

#include "control_toolbox/pid_ros.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::executors::MultiThreadedExecutor;

TEST(PidParametersTest, InitPidTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  control_toolbox::PidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;

  ASSERT_NO_THROW(pid.initPid(P, I, D, I_MAX, I_MIN, false));

  rclcpp::Parameter param;

  // check paramters were set
  ASSERT_TRUE(node->get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node->get_parameter("i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node->get_parameter("d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node->get_parameter("i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node->get_parameter("i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node->get_parameter("antiwindup", param));
  ASSERT_FALSE(param.get_value<bool>());

  // check gains were set
  control_toolbox::Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_FALSE(gains.antiwindup_);
}

TEST(PidParametersTest, InitPidWithAntiwindupTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  control_toolbox::PidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  pid.initPid(P, I, D, I_MAX, I_MIN, ANTIWINDUP);

  rclcpp::Parameter param;

  ASSERT_TRUE(node->get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node->get_parameter("i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node->get_parameter("d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node->get_parameter("i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node->get_parameter("i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node->get_parameter("antiwindup", param));
  ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);

  // check gains were set
  control_toolbox::Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
}

TEST(PidParametersTest, SetParametersTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  control_toolbox::PidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  pid.initPid(P, I, D, I_MAX, I_MIN, ANTIWINDUP);

  rcl_interfaces::msg::SetParametersResult set_result;

  // unknown parameter name
  ASSERT_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("unknown", 0.0)),
    rclcpp::exceptions::ParameterNotDeclaredException);

  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("p", P)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("i", I)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("d", D)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("i_clamp_max", I_MAX)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("i_clamp_min", I_MIN)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("antiwindup", ANTIWINDUP)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // check gains were set using the parameters
  control_toolbox::Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
}

TEST(PidParametersTest, GetParametersTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  control_toolbox::PidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  pid.initPid(0.0, 0.0, 0.0, 0.0, 0.0, false);
  pid.setGains(P, I, D, I_MAX, I_MIN, ANTIWINDUP);

  rclcpp::Parameter param;

  ASSERT_TRUE(node->get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node->get_parameter("i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node->get_parameter("d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node->get_parameter("i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node->get_parameter("i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node->get_parameter("antiwindup", param));
  ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);
}

TEST(PidParametersTest, GetParametersFromParams)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  control_toolbox::PidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;

  ASSERT_FALSE(pid.initPid());

  node->declare_parameter("p", P);
  node->declare_parameter("i", I);
  node->declare_parameter("d", D);
  node->declare_parameter("i_clamp_max", I_MAX);
  node->declare_parameter("i_clamp_min", I_MIN);

  ASSERT_TRUE(pid.initPid());

  rclcpp::Parameter param;

  ASSERT_TRUE(node->get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  return RUN_ALL_TESTS();
}
