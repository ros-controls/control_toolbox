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

class TestablePidROS : public control_toolbox::PidROS
{
  FRIEND_TEST(PidParametersTest, InitPidTest);
  FRIEND_TEST(PidParametersTest, InitPid_when_not_prefix_for_params_then_replace_slash_with_dot);
  FRIEND_TEST(PidParametersTest, InitPid_when_prefix_for_params_then_dont_replace_slash_with_dot);
  FRIEND_TEST(
    PidParametersTest,
    InitPid_when_not_prefix_for_params_then_replace_slash_with_dot_leading_slash);
  FRIEND_TEST(
    PidParametersTest,
    InitPid_when_prefix_for_params_then_dont_replace_slash_with_dot_leading_slash);

public:
  template<class NodeT>
  TestablePidROS(std::shared_ptr<NodeT> node_ptr,
                 std::string prefix = std::string(""),
                 bool prefix_is_for_params = false)
  : control_toolbox::PidROS(node_ptr, prefix, prefix_is_for_params)
  {}
};

void check_set_parameters(
  const rclcpp::Node::SharedPtr & node,
  control_toolbox::PidROS & pid,
  const std::string & prefix = ""
)
{
  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  ASSERT_NO_THROW(pid.initPid(P, I, D, I_MAX, I_MIN, ANTIWINDUP));

  rclcpp::Parameter param;

  // check parameters were set
  ASSERT_TRUE(node->get_parameter(prefix + "p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node->get_parameter(prefix + "i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node->get_parameter(prefix + "d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node->get_parameter(prefix + "i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node->get_parameter(prefix + "i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node->get_parameter(prefix + "antiwindup", param));
  ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);

  // check gains were set
  control_toolbox::Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_TRUE(gains.antiwindup_);
}

TEST(PidParametersTest, InitPidTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node);

  ASSERT_EQ(pid.topic_prefix_, "");
  ASSERT_EQ(pid.param_prefix_, "");

  check_set_parameters(node, pid);
}

TEST(PidParametersTest, InitPidTestBadParameter)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX_BAD = -10.0;
  const double I_MIN_BAD = 10.0;

  ASSERT_NO_THROW(pid.initPid(P, I, D, I_MAX_BAD, I_MIN_BAD, false));

  rclcpp::Parameter param;

  // check parameters were NOT set
  ASSERT_FALSE(node->get_parameter("p", param));
  ASSERT_FALSE(node->get_parameter("i", param));
  ASSERT_FALSE(node->get_parameter("d", param));
  ASSERT_FALSE(node->get_parameter("i_clamp_max", param));
  ASSERT_FALSE(node->get_parameter("i_clamp_min", param));
  ASSERT_FALSE(node->get_parameter("antiwindup", param));

  // check gains were NOT set
  control_toolbox::Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, 0.0);
  ASSERT_EQ(gains.i_gain_, 0.0);
  ASSERT_EQ(gains.d_gain_, 0.0);
  ASSERT_EQ(gains.i_max_, 0.0);
  ASSERT_EQ(gains.i_min_, 0.0);
  ASSERT_FALSE(gains.antiwindup_);
}

TEST(PidParametersTest, InitPid_when_not_prefix_for_params_then_replace_slash_with_dot)
{
  const std::string INPUT_PREFIX = "slash/to/dots";
  const std::string RESULTING_TOPIC_PREFIX = INPUT_PREFIX + "/";
  const std::string RESULTING_PARAM_PREFIX = "slash.to.dots.";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, INPUT_PREFIX);  // default is false

  ASSERT_EQ(pid.topic_prefix_, RESULTING_TOPIC_PREFIX);
  ASSERT_EQ(pid.param_prefix_, RESULTING_PARAM_PREFIX);

  check_set_parameters(node, pid, RESULTING_PARAM_PREFIX);
}

TEST(PidParametersTest, InitPid_when_prefix_for_params_then_dont_replace_slash_with_dot)
{
  const std::string INPUT_PREFIX = "slash/to/dots";
  const std::string RESULTING_TOPIC_PREFIX = "/" + INPUT_PREFIX + "/";
  const std::string RESULTING_PARAM_PREFIX = INPUT_PREFIX + ".";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, INPUT_PREFIX, true);  // prefix is for parameters

  ASSERT_EQ(pid.topic_prefix_, RESULTING_TOPIC_PREFIX);
  ASSERT_EQ(pid.param_prefix_, RESULTING_PARAM_PREFIX);

  check_set_parameters(node, pid, RESULTING_PARAM_PREFIX);
}

TEST(
  PidParametersTest, InitPid_when_not_prefix_for_params_then_replace_slash_with_dot_leading_slash)
{
  const std::string INPUT_PREFIX = "/slash/to/dots";
  const std::string RESULTING_TOPIC_PREFIX = INPUT_PREFIX + "/";
  const std::string RESULTING_PARAM_PREFIX = "slash.to.dots.";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, INPUT_PREFIX);  // default is false

  ASSERT_EQ(pid.topic_prefix_, RESULTING_TOPIC_PREFIX);
  ASSERT_EQ(pid.param_prefix_, RESULTING_PARAM_PREFIX);

  check_set_parameters(node, pid, RESULTING_PARAM_PREFIX);
}

TEST(
  PidParametersTest, InitPid_when_prefix_for_params_then_dont_replace_slash_with_dot_leading_slash)
{
  const std::string INPUT_PREFIX = "/slash/to/dots";
  const std::string RESULTING_TOPIC_PREFIX = INPUT_PREFIX + "/";
  const std::string RESULTING_PARAM_PREFIX = INPUT_PREFIX.substr(1) + ".";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, INPUT_PREFIX, true);  // prefix is for parameters

  ASSERT_EQ(pid.topic_prefix_, RESULTING_TOPIC_PREFIX);
  ASSERT_EQ(pid.param_prefix_, RESULTING_PARAM_PREFIX);

  check_set_parameters(node, pid, RESULTING_PARAM_PREFIX);
}

TEST(PidParametersTest, SetParametersTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node);

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

TEST(PidParametersTest, SetBadParametersTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node);

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const double I_MAX_BAD = -20.0;
  const double I_MIN_BAD = 20.0;
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
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("i_clamp_max", I_MAX_BAD)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("i_clamp_min", I_MIN_BAD)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("antiwindup", ANTIWINDUP)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // check gains were NOT set using the parameters
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

  TestablePidROS pid(node);

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

  TestablePidROS pid(node);

  ASSERT_TRUE(pid.initPid());

  rclcpp::Parameter param_p;
  ASSERT_TRUE(node->get_parameter("p", param_p));
  ASSERT_TRUE(std::isnan(param_p.get_value<double>()));

  rclcpp::Parameter param_i;
  ASSERT_TRUE(node->get_parameter("i", param_i));
  ASSERT_TRUE(std::isnan(param_i.get_value<double>()));

  rclcpp::Parameter param_d;
  ASSERT_TRUE(node->get_parameter("d", param_d));
  ASSERT_TRUE(std::isnan(param_d.get_value<double>()));

  rclcpp::Parameter param_i_clamp_max;
  ASSERT_TRUE(node->get_parameter("i_clamp_max", param_i_clamp_max));
  ASSERT_TRUE(std::isnan(param_i_clamp_max.get_value<double>()));

  rclcpp::Parameter param_i_clamp_min;
  ASSERT_TRUE(node->get_parameter("i_clamp_min", param_i_clamp_min));
  ASSERT_TRUE(std::isnan(param_i_clamp_min.get_value<double>()));
}

TEST(PidParametersTest, MultiplePidInstances)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("multiple_pid_instances");

  TestablePidROS pid_1(node, "PID_1");
  TestablePidROS pid_2(node, "PID_2");

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;

  ASSERT_NO_THROW(pid_1.initPid(P, I, D, I_MAX, I_MIN, false));
  ASSERT_NO_THROW(pid_2.initPid(P, I, D, I_MAX, I_MIN, true));

  rclcpp::Parameter param_1, param_2;
  ASSERT_TRUE(node->get_parameter("PID_1.p", param_1));
  ASSERT_EQ(param_1.get_value<double>(), P);
  ASSERT_TRUE(node->get_parameter("PID_2.p", param_2));
  ASSERT_EQ(param_2.get_value<double>(), P);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
