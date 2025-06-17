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
#include <memory>

#include "control_toolbox/pid_ros.hpp"

#include "gmock/gmock.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

using control_toolbox::AntiWindupStrategy;
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
  template <class NodeT>
  TestablePidROS(
    std::shared_ptr<NodeT> node_ptr, std::string prefix = std::string(""),
    bool prefix_is_for_params = false)
  : control_toolbox::PidROS(node_ptr, prefix, prefix_is_for_params)
  {
  }
};

void check_set_parameters(
  const rclcpp::Node::SharedPtr & node, control_toolbox::PidROS & pid,
  const std::string & prefix = "")
{
  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const double U_MAX = 10.0;
  const double U_MIN = -10.0;
  const double TRK_TC = 4.0;
  const bool SATURATION = true;
  const bool ANTIWINDUP = true;
  AntiWindupStrategy ANTIWINDUP_STRAT;
  ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
  ANTIWINDUP_STRAT.i_max = I_MAX;
  ANTIWINDUP_STRAT.i_min = I_MIN;
  ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
  ANTIWINDUP_STRAT.legacy_antiwindup = ANTIWINDUP;
  const bool SAVE_I_TERM = true;

  ASSERT_NO_THROW(pid.initialize_from_args(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, SAVE_I_TERM));

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

  ASSERT_TRUE(node->get_parameter(prefix + "u_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), U_MAX);

  ASSERT_TRUE(node->get_parameter(prefix + "u_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), U_MIN);

  ASSERT_TRUE(node->get_parameter(prefix + "tracking_time_constant", param));
  ASSERT_EQ(param.get_value<double>(), TRK_TC);

  ASSERT_TRUE(node->get_parameter(prefix + "saturation", param));
  ASSERT_EQ(param.get_value<bool>(), SATURATION);

  ASSERT_TRUE(node->get_parameter(prefix + "antiwindup", param));
  ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);

  ASSERT_TRUE(node->get_parameter(prefix + "antiwindup_strategy", param));
  ASSERT_EQ(param.get_value<std::string>(), ANTIWINDUP_STRAT.to_string());

  ASSERT_TRUE(node->get_parameter(prefix + "save_i_term", param));
  ASSERT_EQ(param.get_value<bool>(), SAVE_I_TERM);

  // check gains were set
  control_toolbox::Pid::Gains gains = pid.get_gains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.u_max_, U_MAX);
  ASSERT_EQ(gains.u_min_, U_MIN);
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_TRUE(gains.antiwindup_);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);
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
  const double U_MAX_BAD = -10.0;
  const double U_MIN_BAD = 10.0;
  const double TRK_TC = 4.0;

  AntiWindupStrategy ANTIWINDUP_STRAT;
  ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
  ANTIWINDUP_STRAT.i_max = I_MAX_BAD;
  ANTIWINDUP_STRAT.i_min = I_MIN_BAD;
  ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
  ANTIWINDUP_STRAT.legacy_antiwindup = false;

  ASSERT_NO_THROW(pid.initialize_from_args(P, I, D, U_MAX_BAD, U_MIN_BAD, ANTIWINDUP_STRAT, false));

  rclcpp::Parameter param;

  // check parameters were NOT set
  ASSERT_FALSE(node->get_parameter("p", param));
  ASSERT_FALSE(node->get_parameter("i", param));
  ASSERT_FALSE(node->get_parameter("d", param));
  ASSERT_FALSE(node->get_parameter("i_clamp_max", param));
  ASSERT_FALSE(node->get_parameter("i_clamp_min", param));
  ASSERT_FALSE(node->get_parameter("u_clamp_max", param));
  ASSERT_FALSE(node->get_parameter("u_clamp_min", param));
  ASSERT_FALSE(node->get_parameter("tracking_time_constant", param));
  ASSERT_FALSE(node->get_parameter("saturation", param));
  ASSERT_FALSE(node->get_parameter("antiwindup", param));
  ASSERT_FALSE(node->get_parameter("antiwindup_strategy", param));

  // check gains were NOT set
  control_toolbox::Pid::Gains gains = pid.get_gains();
  ASSERT_EQ(gains.p_gain_, 0.0);
  ASSERT_EQ(gains.i_gain_, 0.0);
  ASSERT_EQ(gains.d_gain_, 0.0);
  ASSERT_EQ(gains.i_max_, 0.0);
  ASSERT_EQ(gains.i_min_, 0.0);
  ASSERT_EQ(gains.u_max_, std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.u_min_, -std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, 0.0);
  ASSERT_FALSE(gains.antiwindup_);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);
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
  const double U_MAX = 10.0;
  const double U_MIN = -10.0;
  const double TRK_TC = 4.0;
  const bool SATURATION = true;
  const bool ANTIWINDUP = true;
  AntiWindupStrategy ANTIWINDUP_STRAT;
  ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
  ANTIWINDUP_STRAT.i_max = I_MAX;
  ANTIWINDUP_STRAT.i_min = I_MIN;
  ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
  ANTIWINDUP_STRAT.legacy_antiwindup = ANTIWINDUP;
  const bool SAVE_I_TERM = false;

  pid.initialize_from_args(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, SAVE_I_TERM);

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
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("u_clamp_max", U_MAX)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("u_clamp_min", U_MIN)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("tracking_time_constant", TRK_TC)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("saturation", SATURATION)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("antiwindup", ANTIWINDUP)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("antiwindup_strategy", ANTIWINDUP_STRAT)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("save_i_term", SAVE_I_TERM)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // check gains were set using the parameters
  control_toolbox::Pid::Gains gains = pid.get_gains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.u_max_, U_MAX);
  ASSERT_EQ(gains.u_min_, U_MIN);
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);
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
  const double U_MAX = 10.0;
  const double U_MIN = -10.0;
  const double U_MAX_BAD = -20.0;
  const double U_MIN_BAD = 20.0;
  const double TRK_TC = 4.0;
  const bool SATURATION = false;
  const bool ANTIWINDUP = true;

  AntiWindupStrategy ANTIWINDUP_STRAT;
  ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
  ANTIWINDUP_STRAT.i_max = I_MAX;
  ANTIWINDUP_STRAT.i_min = I_MIN;
  ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
  ANTIWINDUP_STRAT.legacy_antiwindup = ANTIWINDUP;

  pid.initialize_from_args(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, false);

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
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("u_clamp_max", U_MAX_BAD)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("u_clamp_min", U_MIN_BAD)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("tracking_time_constant", TRK_TC)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("saturation", SATURATION)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("antiwindup", ANTIWINDUP)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("antiwindup_strategy", ANTIWINDUP_STRAT)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // check gains were NOT set using the parameters but the u_max and u_min
  // were set to infinity as saturation is false
  control_toolbox::Pid::Gains gains = pid.get_gains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.u_max_, std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.u_min_, -std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);

  // Set the good gains

  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("u_clamp_max", U_MAX)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("u_clamp_min", U_MIN)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // Setting good gains doesn't help, as the saturation is still false
  gains = pid.get_gains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.u_max_, std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.u_min_, -std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);

  // Now re-enabling it should have the old gains back
  ASSERT_NO_THROW(set_result = node->set_parameter(rclcpp::Parameter("saturation", true)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // check gains were NOT set using the parameters
  control_toolbox::Pid::Gains updated_gains = pid.get_gains();
  ASSERT_EQ(updated_gains.p_gain_, P);
  ASSERT_EQ(updated_gains.i_gain_, I);
  ASSERT_EQ(updated_gains.d_gain_, D);
  ASSERT_EQ(updated_gains.i_max_, I_MAX);
  ASSERT_EQ(updated_gains.i_min_, I_MIN);
  ASSERT_EQ(updated_gains.u_max_, U_MAX);
  ASSERT_EQ(updated_gains.u_min_, U_MIN);
  ASSERT_EQ(updated_gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_EQ(updated_gains.antiwindup_, ANTIWINDUP);
  ASSERT_EQ(updated_gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);
}

TEST(PidParametersTest, GetParametersTest)
{
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

    TestablePidROS pid(node);

    const double P = 1.0;
    const double I = 2.0;
    const double D = 3.0;
    const double I_MAX = 10.0;
    const double I_MIN = -10.0;
    const double U_MAX = 10.0;
    const double U_MIN = -10.0;
    const double TRK_TC = 4.0;
    const bool SATURATION = true;
    const bool ANTIWINDUP = true;

    AntiWindupStrategy ANTIWINDUP_STRAT;
    ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
    ANTIWINDUP_STRAT.i_max = I_MAX;
    ANTIWINDUP_STRAT.i_min = I_MIN;
    ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
    ANTIWINDUP_STRAT.legacy_antiwindup = ANTIWINDUP;

    ASSERT_FALSE(pid.initialize_from_args(0.0, 0.0, 0.0, 0.0, 0.0, ANTIWINDUP_STRAT, false))
      << "Zero u_min and u_max are not valid so initialization should fail";
    ASSERT_TRUE(pid.initialize_from_args(0, 0, 0, U_MAX, U_MIN, ANTIWINDUP_STRAT, false));
    std::cout << "Setting gains with set_gains()" << std::endl;
    pid.set_gains(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT);

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

    ASSERT_TRUE(node->get_parameter("u_clamp_max", param));
    ASSERT_EQ(param.get_value<double>(), U_MAX);

    ASSERT_TRUE(node->get_parameter("u_clamp_min", param));
    ASSERT_EQ(param.get_value<double>(), U_MIN);

    ASSERT_TRUE(node->get_parameter("tracking_time_constant", param));
    ASSERT_EQ(param.get_value<double>(), TRK_TC);

    ASSERT_TRUE(node->get_parameter("saturation", param));
    ASSERT_EQ(param.get_value<bool>(), SATURATION);

    ASSERT_TRUE(node->get_parameter("antiwindup", param));
    ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);

    ASSERT_TRUE(node->get_parameter("antiwindup_strategy", param));
    ASSERT_EQ(param.get_value<std::string>(), ANTIWINDUP_STRAT.to_string());

    ASSERT_TRUE(node->get_parameter("save_i_term", param));
    ASSERT_EQ(param.get_value<bool>(), false);
  }
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

    TestablePidROS pid(node);

    const double P = 1.0;
    const double I = 2.0;
    const double D = 3.0;
    const double I_MAX = 10.0;
    const double I_MIN = -10.0;
    const double U_MAX = std::numeric_limits<double>::infinity();
    const double U_MIN = -std::numeric_limits<double>::infinity();
    const double TRK_TC = 4.0;
    const bool ANTIWINDUP = true;

    AntiWindupStrategy ANTIWINDUP_STRAT;
    ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
    ANTIWINDUP_STRAT.i_max = I_MAX;
    ANTIWINDUP_STRAT.i_min = I_MIN;
    ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
    ANTIWINDUP_STRAT.legacy_antiwindup = ANTIWINDUP;

    ASSERT_FALSE(pid.initialize_from_args(0.0, 0.0, 0.0, 0.0, 0.0, ANTIWINDUP_STRAT, false))
      << "Zero u_min and u_max are not valid so initialization should fail";
    ASSERT_TRUE(pid.initialize_from_args(0, 0, 0, U_MAX, U_MIN, ANTIWINDUP_STRAT, false));
    pid.set_gains(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT);

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

    ASSERT_TRUE(node->get_parameter("u_clamp_max", param));
    ASSERT_EQ(param.get_value<double>(), U_MAX);

    ASSERT_TRUE(node->get_parameter("u_clamp_min", param));
    ASSERT_EQ(param.get_value<double>(), U_MIN);

    ASSERT_TRUE(node->get_parameter("tracking_time_constant", param));
    ASSERT_EQ(param.get_value<double>(), TRK_TC);

    ASSERT_TRUE(node->get_parameter("saturation", param));
    ASSERT_TRUE(param.get_value<bool>()) << "Should be enabled by default!";

    ASSERT_TRUE(node->get_parameter("antiwindup", param));
    ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);

    ASSERT_TRUE(node->get_parameter("antiwindup_strategy", param));
    ASSERT_EQ(param.get_value<std::string>(), ANTIWINDUP_STRAT.to_string());

    ASSERT_TRUE(node->get_parameter("save_i_term", param));
    ASSERT_EQ(param.get_value<bool>(), false);
  }
}

TEST(PidParametersTest, GetParametersFromParams)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node);

  ASSERT_FALSE(pid.initialize_from_ros_parameters());

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

  rclcpp::Parameter param_u_clamp_max;
  ASSERT_TRUE(node->get_parameter("u_clamp_max", param_u_clamp_max));
  ASSERT_TRUE(std::isinf(param_u_clamp_max.get_value<double>()));

  rclcpp::Parameter param_u_clamp_min;
  ASSERT_TRUE(node->get_parameter("u_clamp_min", param_u_clamp_min));
  ASSERT_TRUE(std::isinf(param_u_clamp_min.get_value<double>()));

  rclcpp::Parameter param_tracking_time_constant;
  ASSERT_TRUE(node->get_parameter("tracking_time_constant", param_tracking_time_constant));
  ASSERT_TRUE(std::isnan(param_tracking_time_constant.get_value<double>()));
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
  const double U_MAX = 10.0;
  const double U_MIN = -10.0;
  const double TRK_TC = 4.0;
  AntiWindupStrategy ANTIWINDUP_STRAT;
  ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
  ANTIWINDUP_STRAT.i_max = I_MAX;
  ANTIWINDUP_STRAT.i_min = I_MIN;
  ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
  ANTIWINDUP_STRAT.legacy_antiwindup = false;

  ASSERT_NO_THROW(pid_1.initialize_from_args(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, false));
  ANTIWINDUP_STRAT.legacy_antiwindup = true;
  ASSERT_NO_THROW(pid_2.initialize_from_args(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, false));

  rclcpp::Parameter param_1, param_2;
  ASSERT_TRUE(node->get_parameter("PID_1.p", param_1));
  ASSERT_EQ(param_1.get_value<double>(), P);
  ASSERT_TRUE(node->get_parameter("PID_2.p", param_2));
  ASSERT_EQ(param_2.get_value<double>(), P);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

#pragma GCC diagnostic pop
