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
  FRIEND_TEST(PidParametersTest, InitPid_no_prefix);
  FRIEND_TEST(PidParametersTest, InitPid_prefix);
  FRIEND_TEST(PidParametersTest, InitPid_param_prefix_only);
  FRIEND_TEST(PidParametersTest, InitPid_topic_prefix_only);

public:
  template <class NodeT>
  TestablePidROS(
    std::shared_ptr<NodeT> node_ptr, std::string param_prefix = std::string(""),
    std::string topic_prefix = std::string(""), bool activate_state_publisher = false)
  : control_toolbox::PidROS(node_ptr, param_prefix, topic_prefix, activate_state_publisher)
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
  ASSERT_EQ(gains.antiwindup_strat_.i_max, I_MAX);
  ASSERT_EQ(gains.antiwindup_strat_.i_min, I_MIN);
  ASSERT_EQ(gains.u_max_, U_MAX);
  ASSERT_EQ(gains.u_min_, U_MIN);
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_TRUE(gains.antiwindup_);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);
}

TEST(PidParametersTest, InitPid_no_prefix)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, "", "", false);

  ASSERT_EQ(pid.topic_prefix_, "");
  ASSERT_EQ(pid.param_prefix_, "");

  check_set_parameters(node, pid);
}

TEST(PidParametersTest, InitPidTestBadParameter)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, "", "", false);

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

  bool ret;
  ASSERT_NO_THROW(
    ret = pid.initialize_from_args(P, I, D, U_MAX_BAD, U_MIN_BAD, ANTIWINDUP_STRAT, false));
  ASSERT_FALSE(ret);

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
  ASSERT_EQ(gains.antiwindup_strat_.i_max, std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.antiwindup_strat_.i_min, -std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.u_max_, std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.u_min_, -std::numeric_limits<double>::infinity());
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, 0.0);
  ASSERT_FALSE(gains.antiwindup_);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);

  // Try other invalid combinations
  ANTIWINDUP_STRAT.i_max = 10.;
  ANTIWINDUP_STRAT.i_min = 5.;
  ASSERT_NO_THROW(
    ret = pid.initialize_from_args(P, I, D, U_MAX_BAD, U_MIN_BAD, ANTIWINDUP_STRAT, false));
  ASSERT_FALSE(ret);

  ANTIWINDUP_STRAT.i_max = -5.;
  ANTIWINDUP_STRAT.i_min = 10.;
  ASSERT_NO_THROW(
    ret = pid.initialize_from_args(P, I, D, U_MAX_BAD, U_MIN_BAD, ANTIWINDUP_STRAT, false));
  ASSERT_FALSE(ret);
}

TEST(PidParametersTest, InitPid_param_prefix_only)
{
  const std::string PARAM_PREFIX = "some_param_prefix";
  const std::string TOPIC_PREFIX = "";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, PARAM_PREFIX, TOPIC_PREFIX, false);

  ASSERT_EQ(pid.topic_prefix_, TOPIC_PREFIX);
  ASSERT_EQ(pid.param_prefix_, PARAM_PREFIX + ".");

  check_set_parameters(node, pid, PARAM_PREFIX + ".");
}

TEST(PidParametersTest, InitPid_topic_prefix_only)
{
  const std::string PARAM_PREFIX = "";
  const std::string TOPIC_PREFIX = "some_topic_prefix";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, PARAM_PREFIX, TOPIC_PREFIX, false);

  ASSERT_EQ(pid.topic_prefix_, TOPIC_PREFIX + "/");
  ASSERT_EQ(pid.param_prefix_, PARAM_PREFIX);

  check_set_parameters(node, pid, PARAM_PREFIX);
}

TEST(PidParametersTest, InitPid_prefix)
{
  const std::string PARAM_PREFIX = "some_param_prefix";
  const std::string TOPIC_PREFIX = "some_topic_prefix";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, PARAM_PREFIX, TOPIC_PREFIX, false);

  ASSERT_EQ(pid.topic_prefix_, TOPIC_PREFIX + "/");
  ASSERT_EQ(pid.param_prefix_, PARAM_PREFIX + ".");

  check_set_parameters(node, pid, PARAM_PREFIX + ".");
}

TEST(PidParametersTest, SetParametersTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, "", "", false);

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
  ASSERT_NO_THROW(
    set_result = node->set_parameter(rclcpp::Parameter("activate_state_publisher", true)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node->get_node_base_interface());

  // check gains were set using the parameters
  control_toolbox::Pid::Gains gains = pid.get_gains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.antiwindup_strat_.i_max, I_MAX);
  ASSERT_EQ(gains.antiwindup_strat_.i_min, I_MIN);
  ASSERT_EQ(gains.u_max_, U_MAX);
  ASSERT_EQ(gains.u_min_, U_MIN);
  ASSERT_EQ(gains.antiwindup_strat_.tracking_time_constant, TRK_TC);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
  ASSERT_EQ(gains.antiwindup_strat_, AntiWindupStrategy::LEGACY);
}

TEST(PidParametersTest, SetBadParametersTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

  TestablePidROS pid(node, "", "", false);

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
  ASSERT_EQ(gains.antiwindup_strat_.i_max, I_MAX);
  ASSERT_EQ(gains.antiwindup_strat_.i_min, I_MIN);
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
  ASSERT_EQ(gains.antiwindup_strat_.i_max, I_MAX);
  ASSERT_EQ(gains.antiwindup_strat_.i_min, I_MIN);
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
  ASSERT_EQ(updated_gains.antiwindup_strat_.i_max, I_MAX);
  ASSERT_EQ(updated_gains.antiwindup_strat_.i_min, I_MIN);
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

    TestablePidROS pid(node, "", "", false);

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

    ASSERT_TRUE(node->get_parameter("activate_state_publisher", param));
    ASSERT_EQ(param.get_value<bool>(), false);
  }
  {
    // test activate_state_publisher
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

    TestablePidROS pid(node, "", "", true);
    const double P = 1.0;
    const double I = 2.0;
    const double D = 3.0;
    const double I_MAX = 10.0;
    const double I_MIN = -10.0;
    const double U_MAX = 10.0;
    const double U_MIN = -10.0;
    const double TRK_TC = 4.0;
    const bool ANTIWINDUP = true;

    AntiWindupStrategy ANTIWINDUP_STRAT;
    ANTIWINDUP_STRAT.type = AntiWindupStrategy::LEGACY;
    ANTIWINDUP_STRAT.i_max = I_MAX;
    ANTIWINDUP_STRAT.i_min = I_MIN;
    ANTIWINDUP_STRAT.tracking_time_constant = TRK_TC;
    ANTIWINDUP_STRAT.legacy_antiwindup = ANTIWINDUP;

    ASSERT_TRUE(pid.initialize_from_args(P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, false));

    rclcpp::Parameter param;
    ASSERT_TRUE(node->get_parameter("activate_state_publisher", param));
    ASSERT_EQ(param.get_value<bool>(), true);
  }
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pid_parameters_test");

    TestablePidROS pid(node, "", "", false);

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
  const bool ACTIVATE_STATE_PUBLISHER = false;
  TestablePidROS pid(node, "", "", ACTIVATE_STATE_PUBLISHER);

  ASSERT_TRUE(pid.initialize_from_ros_parameters());

  rclcpp::Parameter param_p;
  ASSERT_TRUE(node->get_parameter("p", param_p));
  EXPECT_TRUE(std::isnan(param_p.get_value<double>()));

  rclcpp::Parameter param_i;
  ASSERT_TRUE(node->get_parameter("i", param_i));
  EXPECT_TRUE(std::isnan(param_i.get_value<double>()));

  rclcpp::Parameter param_d;
  ASSERT_TRUE(node->get_parameter("d", param_d));
  EXPECT_TRUE(std::isnan(param_d.get_value<double>()));

  rclcpp::Parameter param_i_clamp_max;
  ASSERT_TRUE(node->get_parameter("i_clamp_max", param_i_clamp_max));
  EXPECT_FALSE(std::isfinite(param_i_clamp_max.get_value<double>()));

  rclcpp::Parameter param_i_clamp_min;
  ASSERT_TRUE(node->get_parameter("i_clamp_min", param_i_clamp_min));
  EXPECT_FALSE(std::isfinite(param_i_clamp_min.get_value<double>()));

  rclcpp::Parameter param_u_clamp_max;
  ASSERT_TRUE(node->get_parameter("u_clamp_max", param_u_clamp_max));
  EXPECT_TRUE(std::isinf(param_u_clamp_max.get_value<double>()));

  rclcpp::Parameter param_u_clamp_min;
  ASSERT_TRUE(node->get_parameter("u_clamp_min", param_u_clamp_min));
  EXPECT_TRUE(std::isinf(param_u_clamp_min.get_value<double>()));

  rclcpp::Parameter param_saturation;
  ASSERT_TRUE(node->get_parameter("saturation", param_saturation));
  EXPECT_FALSE(param_saturation.get_value<bool>());

  rclcpp::Parameter param_tracking_time_constant;
  ASSERT_TRUE(node->get_parameter("tracking_time_constant", param_tracking_time_constant));
  EXPECT_TRUE(std::isnan(param_tracking_time_constant.get_value<double>()));

  rclcpp::Parameter param_activate_state_publisher;
  ASSERT_TRUE(node->get_parameter("activate_state_publisher", param_activate_state_publisher));
  EXPECT_EQ(param_activate_state_publisher.get_value<bool>(), ACTIVATE_STATE_PUBLISHER);
}

TEST(PidParametersTest, ResetReadsSaveITermParam)
{
  auto node = std::make_shared<rclcpp::Node>("pidros_reset_param_test");
  TestablePidROS pid(node, "", "", false);

  // I-only controller; declare params via initialize_from_args
  AntiWindupStrategy anti;
  anti.type = AntiWindupStrategy::NONE;
  const double U_MAX = 1e9, U_MIN = -1e9;
  ASSERT_TRUE(pid.initialize_from_args(0.0, 1.0, 0.0, U_MAX, U_MIN, anti, /*save_i_term=*/false));

  const auto dt = rclcpp::Duration::from_seconds(1.0);

  // Prime integral to a known value (internal I = 2 after two steps)
  (void)pid.compute_command(1.0, dt);
  (void)pid.compute_command(1.0, dt);
  // Snapshot current integral without changing it
  const double I_after_prime = pid.compute_command(0.0, dt);
  EXPECT_GT(I_after_prime, 0.0);

  // ---- Case 1: save_i_term=false → clear on reset() ----
  if (!node->has_parameter("save_i_term"))
  {
    node->declare_parameter<bool>("save_i_term", false);
  }
  else
  {
    node->set_parameter(rclcpp::Parameter("save_i_term", false));
  }
  pid.reset();

  const double cmd_after_clear = pid.compute_command(0.0, dt);
  EXPECT_DOUBLE_EQ(0.0, cmd_after_clear);

  // Re-prime and snapshot again
  (void)pid.compute_command(1.0, dt);
  (void)pid.compute_command(1.0, dt);
  const double I_before_retain = pid.compute_command(0.0, dt);
  EXPECT_GT(I_before_retain, 0.0);

  // ---- Case 2: save_i_term=true → retain on reset() ----
  node->set_parameter(rclcpp::Parameter("save_i_term", true));
  pid.reset();

  const double cmd_after_retain = pid.compute_command(0.0, dt);
  EXPECT_DOUBLE_EQ(I_before_retain, cmd_after_retain);
}

TEST(PidParametersTest, ResetBoolClearsOrRetainsITerm)
{
  auto node = std::make_shared<rclcpp::Node>("pidros_reset_bool_test");
  TestablePidROS pid(node, "", "", false);

  AntiWindupStrategy anti;
  anti.type = AntiWindupStrategy::NONE;
  const double U_MAX = 1e9, U_MIN = -1e9;
  ASSERT_TRUE(pid.initialize_from_args(0.0, 1.0, 0.0, U_MAX, U_MIN, anti, /*save_i_term=*/false));

  const auto dt = rclcpp::Duration::from_seconds(1.0);

  // Prime and snapshot
  (void)pid.compute_command(1.0, dt);
  (void)pid.compute_command(1.0, dt);
  const double I_snapshot = pid.compute_command(0.0, dt);
  EXPECT_GT(I_snapshot, 0.0);

  // reset(false) clears integral
  pid.reset(false);
  const double after_clear = pid.compute_command(0.0, dt);
  EXPECT_DOUBLE_EQ(0.0, after_clear);

  // Re-prime and snapshot again
  (void)pid.compute_command(1.0, dt);
  (void)pid.compute_command(1.0, dt);
  const double I_snapshot2 = pid.compute_command(0.0, dt);
  EXPECT_GT(I_snapshot2, 0.0);

  // reset(true) retains integral
  pid.reset(true);
  const double after_retain = pid.compute_command(0.0, dt);
  EXPECT_DOUBLE_EQ(I_snapshot2, after_retain);
}

TEST(PidParametersTest, PrintValuesLogsExpectedContent)
{
  // --- Set up a logger capture (rcutils) ---
  static std::mutex g_mutex;
  static std::string g_last_log;  // store the last message from our logger
  static rcutils_logging_output_handler_t prev_handler = nullptr;

  const char * kLoggerName = "pid_print_test";

  auto capture_handler = [](
                           const rcutils_log_location_t * /*location*/, int /*severity*/,
                           const char * name, rcutils_time_point_value_t /*stamp*/,
                           const char * format, va_list * args)
  {
    // Only capture our logger's output
    if (!name || std::string(name) != "pid_print_test")
    {
      return;
    }
    char buf[8192];
    vsnprintf(buf, sizeof(buf), format, *args);
    std::lock_guard<std::mutex> lk(g_mutex);
    g_last_log = buf;
  };

  // Ensure our logger emits INFO
  ASSERT_EQ(
    rcutils_logging_set_logger_level(kLoggerName, RCUTILS_LOG_SEVERITY_INFO), RCUTILS_RET_OK);

  // Swap in our capture handler
  prev_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(capture_handler);

  // --- Arrange a deterministic PID state ---
  auto node = std::make_shared<rclcpp::Node>(kLoggerName);
  TestablePidROS pid(
    node, /*param_prefix=*/"", /*topic_prefix=*/"", /*activate_state_publisher=*/false);

  control_toolbox::AntiWindupStrategy anti;
  anti.type = control_toolbox::AntiWindupStrategy::NONE;  // simpler: avoid saturation dynamics
  anti.i_max = 7.0;
  anti.i_min = -7.0;
  anti.tracking_time_constant = 0.3;

  const double U_MAX = 1e9, U_MIN = -1e9;  // avoid clamping
  ASSERT_TRUE(pid.initialize_from_args(/*p=*/1.0, /*i=*/1.0, /*d=*/0.0, U_MAX, U_MIN, anti,
                                       /*save_i_term=*/false));

  // Make the internal errors non-trivial
  const auto dt = rclcpp::Duration::from_seconds(0.2);
  (void)pid.compute_command(/*error=*/2.0, dt);
  (void)pid.compute_command(/*error=*/2.0, dt);

  // --- Act: call the function under test ---
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    g_last_log.clear();
  }
  pid.print_values();

  // --- Assert: captured log has expected content ---
  std::string captured;
  {
    std::lock_guard<std::mutex> lk(g_mutex);
    captured = g_last_log;
  }

  // Basic header
  EXPECT_NE(captured.find("Current Values of PID template:"), std::string::npos);

  // Gains (format tolerant)
  EXPECT_NE(captured.find("P Gain:"), std::string::npos);
  EXPECT_NE(captured.find("I Gain:"), std::string::npos);
  EXPECT_NE(captured.find("D Gain:"), std::string::npos);

  // I bounds and output limits labels
  EXPECT_NE(captured.find("I Max:"), std::string::npos);
  EXPECT_NE(captured.find("I Min:"), std::string::npos);
  EXPECT_NE(captured.find("U_Max:"), std::string::npos);
  EXPECT_NE(captured.find("U_Min:"), std::string::npos);

  // Anti-windup summary
  EXPECT_NE(captured.find("Tracking_Time_Constant:"), std::string::npos);
  EXPECT_NE(captured.find("Antiwindup_Strategy:"), std::string::npos);

  // Runtime state
  EXPECT_NE(captured.find("P Error:"), std::string::npos);
  EXPECT_NE(captured.find("I Term:"), std::string::npos);
  EXPECT_NE(captured.find("D Error:"), std::string::npos);
  EXPECT_NE(captured.find("Command:"), std::string::npos);

  // Spot-check a couple of actual numeric values that should be stable
  // (avoid strict layout assumptions)
  EXPECT_NE(captured.find("P Gain:       1"), std::string::npos);
  EXPECT_NE(captured.find("I Gain:       1"), std::string::npos);
  EXPECT_NE(captured.find("D Gain:       0"), std::string::npos);

  // Restore previous handler so other tests aren't affected
  rcutils_logging_set_output_handler(prev_handler);
}

TEST(PidParametersTest, GetCurrentCmdTracksSetAndCompute)
{
  auto node = std::make_shared<rclcpp::Node>("pidros_get_current_cmd_test");

  // Simple, deterministic P-only controller with tight output limits to exercise saturation.
  control_toolbox::AntiWindupStrategy anti;
  anti.type = control_toolbox::AntiWindupStrategy::NONE;
  const double U_MAX = 5.0, U_MIN = -5.0;

  TestablePidROS pid(node, "", "", false);
  ASSERT_TRUE(pid.initialize_from_args(2.0, 0.0, 0.0, U_MAX, U_MIN, anti, false));

  // 1) After initialization, current command should be zero.
  EXPECT_DOUBLE_EQ(0.0, pid.get_current_cmd());

  // 2) set_current_cmd should be reflected by get_current_cmd (simple delegation/round-trip).
  pid.set_current_cmd(3.14159);
  EXPECT_DOUBLE_EQ(3.14159, pid.get_current_cmd());

  // 3) compute_command should update the "current command" to the last applied command.
  // With P=2.0, error=10 -> raw = 20, but saturated to +5.0 by [U_MIN, U_MAX].
  const auto dt = rclcpp::Duration::from_seconds(0.1);
  const double cmd1 = pid.compute_command(/*error=*/10.0, dt);
  EXPECT_DOUBLE_EQ(5.0, cmd1);
  EXPECT_DOUBLE_EQ(cmd1, pid.get_current_cmd());

  // Negative side saturation: error=-10 -> raw = -20, saturated to -5.0.
  const double cmd2 = pid.compute_command(/*error=*/-10.0, dt);
  EXPECT_DOUBLE_EQ(-5.0, cmd2);
  EXPECT_DOUBLE_EQ(cmd2, pid.get_current_cmd());

  // 4) Large but finite set_current_cmd round-trip (sanity).
  pid.set_current_cmd(-1e12);
  EXPECT_DOUBLE_EQ(-1e12, pid.get_current_cmd());
}

TEST(PidParametersTest, SetCurrentCmdStoresValueAndKeepsComputeStable)
{
  auto node = std::make_shared<rclcpp::Node>("pidros_set_current_cmd_basic_test");

  // Simple P-only controller with tight limits so compute() is always finite.
  control_toolbox::AntiWindupStrategy anti;
  anti.type = control_toolbox::AntiWindupStrategy::NONE;
  const double U_MAX = 5.0, U_MIN = -5.0;

  TestablePidROS pid(node, "", "", false);
  ASSERT_TRUE(pid.initialize_from_args(2.0, 0.0, 0.0, U_MAX, U_MIN, anti, false));

  // 1) Basic round-trip: set -> get
  pid.set_current_cmd(1.23);
  EXPECT_DOUBLE_EQ(1.23, pid.get_current_cmd());

  pid.set_current_cmd(-4.56);
  EXPECT_DOUBLE_EQ(-4.56, pid.get_current_cmd());

  // 2) Extreme-but-finite value should round-trip
  const double huge = 1e12;
  pid.set_current_cmd(huge);
  EXPECT_DOUBLE_EQ(huge, pid.get_current_cmd());

  // 3) After setting an arbitrary current command, compute_command() should still be stable/finite.
  const auto dt = rclcpp::Duration::from_seconds(0.1);
  const double cmd = pid.compute_command(/*error=*/3.0, dt);  // raw = 6.0 -> clamped to +5.0
  EXPECT_DOUBLE_EQ(5.0, cmd);
  EXPECT_DOUBLE_EQ(cmd, pid.get_current_cmd());
}

TEST(PidParametersTest, MultiplePidInstances)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("multiple_pid_instances");

  TestablePidROS pid_1(node, "PID_1");   // missing trailing dot should be auto-added
  TestablePidROS pid_2(node, "PID_2.");  // Note the trailing dot in the prefix

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
  ASSERT_NO_THROW(pid_2.initialize_from_args(2 * P, I, D, U_MAX, U_MIN, ANTIWINDUP_STRAT, false));

  rclcpp::Parameter param_1, param_2;
  ASSERT_TRUE(node->get_parameter("PID_1.p", param_1));
  EXPECT_EQ(param_1.get_value<double>(), P);
  ASSERT_TRUE(node->get_parameter("PID_2.p", param_2));
  EXPECT_EQ(param_2.get_value<double>(), 2 * P);
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
