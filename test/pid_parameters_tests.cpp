#include <gtest/gtest.h>

#include <control_toolbox/pid.hpp>

#include <rclcpp/executor.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/utilities.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace control_toolbox;
using rclcpp::executors::MultiThreadedExecutor;

TEST(PidParametersTest, InitPidTest)
{
  rclcpp::Node node("pid_parameters_test");

  Pid pid;

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;

  ASSERT_NO_THROW(pid.initPid(P, I, D, I_MAX, I_MIN, node.get_node_parameters_interface()));

  rclcpp::Parameter param;

  // check paramters were set
  ASSERT_TRUE(node.get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node.get_parameter("i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node.get_parameter("d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node.get_parameter("i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node.get_parameter("i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node.get_parameter("antiwindup", param));
  ASSERT_FALSE(param.get_value<bool>());

  // check gains were set
  Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_FALSE(gains.antiwindup_);
}

TEST(PidParametersTest, InitPidWithAntiwindupTest)
{
  rclcpp::Node node("pid_parameters_test");

  Pid pid;

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  ASSERT_NO_THROW(pid.initPid(P, I, D, I_MAX, I_MIN, ANTIWINDUP,
    node.get_node_parameters_interface()));

  rclcpp::Parameter param;

  ASSERT_TRUE(node.get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node.get_parameter("i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node.get_parameter("d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node.get_parameter("i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node.get_parameter("i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node.get_parameter("antiwindup", param));
  ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);

  // check gains were set
  Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
}

TEST(PidParametersTest, SetParametersTest)
{
  rclcpp::Node node("pid_parameters_test");

  Pid pid;

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  ASSERT_NO_THROW(pid.initPid(P, I, D, I_MAX, I_MIN, ANTIWINDUP,
    node.get_node_parameters_interface()));

  rcl_interfaces::msg::SetParametersResult set_result;

  // unknown parameter name
  ASSERT_THROW(set_result = node.set_parameter(
      rclcpp::Parameter("unknown", 0.0)), rclcpp::exceptions::ParameterNotDeclaredException);

  ASSERT_NO_THROW(set_result = node.set_parameter(rclcpp::Parameter("p", P)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node.set_parameter(rclcpp::Parameter("i", I)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node.set_parameter(rclcpp::Parameter("d", D)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node.set_parameter(rclcpp::Parameter("i_clamp_max", I_MAX)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node.set_parameter(rclcpp::Parameter("i_clamp_min", I_MIN)));
  ASSERT_TRUE(set_result.successful);
  ASSERT_NO_THROW(set_result = node.set_parameter(rclcpp::Parameter("antiwindup", ANTIWINDUP)));
  ASSERT_TRUE(set_result.successful);

  // process callbacks
  rclcpp::spin_some(node.get_node_base_interface());

  // check gains were set using the parameters
  Pid::Gains gains = pid.getGains();
  ASSERT_EQ(gains.p_gain_, P);
  ASSERT_EQ(gains.i_gain_, I);
  ASSERT_EQ(gains.d_gain_, D);
  ASSERT_EQ(gains.i_max_, I_MAX);
  ASSERT_EQ(gains.i_min_, I_MIN);
  ASSERT_EQ(gains.antiwindup_, ANTIWINDUP);
}

TEST(PidParametersTest, GetParametersTest)
{
  rclcpp::Node node("pid_parameters_test");

  Pid pid;

  const double P = 1.0;
  const double I = 2.0;
  const double D = 3.0;
  const double I_MAX = 10.0;
  const double I_MIN = -10.0;
  const bool ANTIWINDUP = true;

  ASSERT_NO_THROW(pid.initPid(0.0, 0.0, 0.0, 0.0, 0.0, false,
    node.get_node_parameters_interface()));
  ASSERT_NO_THROW(pid.setGains(P, I, D, I_MAX, I_MIN, ANTIWINDUP));

  rclcpp::Parameter param;

  ASSERT_TRUE(node.get_parameter("p", param));
  ASSERT_EQ(param.get_value<double>(), P);

  ASSERT_TRUE(node.get_parameter("i", param));
  ASSERT_EQ(param.get_value<double>(), I);

  ASSERT_TRUE(node.get_parameter("d", param));
  ASSERT_EQ(param.get_value<double>(), D);

  ASSERT_TRUE(node.get_parameter("i_clamp_max", param));
  ASSERT_EQ(param.get_value<double>(), I_MAX);

  ASSERT_TRUE(node.get_parameter("i_clamp_min", param));
  ASSERT_EQ(param.get_value<double>(), I_MIN);

  ASSERT_TRUE(node.get_parameter("antiwindup", param));
  ASSERT_EQ(param.get_value<bool>(), ANTIWINDUP);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  return RUN_ALL_TESTS();
}
