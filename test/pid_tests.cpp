
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <control_toolbox/pid.h>

#include <boost/math/special_functions/fpclassify.hpp>

using namespace control_toolbox;

TEST(ParameterTest, zeroITermBadIBoundsTest)
{
  RecordProperty("description","This test checks robustness against divide-by-zero errors when given integral term bounds which do not include 0.0.");

  Pid pid(1.0, 0.0, 0.0, -1.0, 0.0);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.computeCommand(-1.0, ros::Duration(1.0));

  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_FALSE(boost::math::isinf(ie));
  EXPECT_FALSE(boost::math::isnan(cmd));

  cmd = pid.computeCommand(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_FALSE(boost::math::isinf(ie));
  EXPECT_FALSE(boost::math::isnan(cmd));
}

TEST(ParameterTest, integrationWindupTest)
{
  RecordProperty("description","This test succeeds if the integral error is prevented from winding up when the integral gain is non-zero.");

  Pid pid(0.0, 1.0, 0.0, 1.0, -1.0);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.computeCommand(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_EQ(-1.0, ie);
  EXPECT_EQ(-1.0, cmd);

  cmd = pid.computeCommand(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_EQ(-1.0, ie);
  EXPECT_EQ(-1.0, cmd);
}

TEST(ParameterTest, integrationWindupZeroGainTest)
{
  RecordProperty("description","This test succeeds if the integral error is prevented from winding up when the integral gain is zero. If the integral error is allowed to wind up while it is disabled, it can cause sudden jumps to the minimum or maximum bound in control command when re-enabled.");

  double i_gain = 0.0;
  double i_min = -1.0;
  double i_max = 1.0;
  Pid pid(0.0, i_gain, 0.0, i_max, i_min);

  double cmd = 0.0;
  double pe,ie,de;

  cmd = pid.computeCommand(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_LE(i_min, ie);
  EXPECT_LE(ie, i_max);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.computeCommand(-1.0, ros::Duration(1.0));
  pid.getCurrentPIDErrors(&pe,&ie,&de);
  EXPECT_LE(i_min, ie);
  EXPECT_LE(ie, i_max);
  EXPECT_EQ(0.0, cmd);
}

TEST(ParameterTest, gainSettingCopyPIDTest)
{
  RecordProperty("description","This test succeeds if a PID object has its gain set at different points in time then the values are get-ed and still remain the same, as well as when PID is copied.");

  // Test values
  double p_gain = rand() % 100;
  double i_gain = rand() % 100;
  double d_gain = rand() % 100;
  double i_max = rand() % 100;
  double i_min = -1 * rand() % 100;

  // Initialize the default way
  Pid pid1(p_gain, i_gain, d_gain, i_max, i_min);
  
  // Test return values  -------------------------------------------------
  double p_gain_return, i_gain_return, d_gain_return, i_max_return, i_min_return;
  pid1.getGains(p_gain_return, i_gain_return, d_gain_return, i_max_return, i_min_return);

  EXPECT_EQ(p_gain, p_gain_return);
  EXPECT_EQ(i_gain, i_gain_return);
  EXPECT_EQ(d_gain, d_gain_return);
  EXPECT_EQ(i_max, i_max_return);
  EXPECT_EQ(i_min, i_min_return);

  // Test return values using struct -------------------------------------------------

  // New values
  p_gain = rand() % 100;
  i_gain = rand() % 100;
  d_gain = rand() % 100;
  i_max = rand() % 100;
  i_min = -1 * rand() % 100;
  pid1.setGains(p_gain, i_gain, d_gain, i_max, i_min);

  Pid::Gains g1 = pid1.getGains();
  EXPECT_EQ(p_gain, g1.p_gain_);
  EXPECT_EQ(i_gain, g1.i_gain_);
  EXPECT_EQ(d_gain, g1.d_gain_);
  EXPECT_EQ(i_max, g1.i_max_);
  EXPECT_EQ(i_min, g1.i_min_);

  // \todo test initParam() -------------------------------------------------
  

  // \todo test bool init(const ros::NodeHandle &n); -----------------------------------


  // Send update command to populate errors -------------------------------------------------
  pid1.setCurrentCmd(10);
  pid1.computeCommand(20, ros::Duration(1.0));

  // Test copy constructor -------------------------------------------------
  Pid pid2(pid1);

  pid2.getGains(p_gain_return, i_gain_return, d_gain_return, i_max_return, i_min_return);

  EXPECT_EQ(p_gain, p_gain_return);
  EXPECT_EQ(i_gain, i_gain_return);
  EXPECT_EQ(d_gain, d_gain_return);
  EXPECT_EQ(i_max, i_max_return);
  EXPECT_EQ(i_min, i_min_return);

  // Test that errors are zero
  double pe, ie, de;
  pid2.getCurrentPIDErrors(&pe, &ie, &de);
  EXPECT_EQ(0.0, pe);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, de);
    
  // Test assignment constructor -------------------------------------------------
  Pid pid3;
  pid3 = pid1;

  pid3.getGains(p_gain_return, i_gain_return, d_gain_return, i_max_return, i_min_return);

  EXPECT_EQ(p_gain, p_gain_return);
  EXPECT_EQ(i_gain, i_gain_return);
  EXPECT_EQ(d_gain, d_gain_return);
  EXPECT_EQ(i_max, i_max_return);
  EXPECT_EQ(i_min, i_min_return);

  // Test that errors are zero
  double pe2, ie2, de2;
  pid3.getCurrentPIDErrors(&pe2, &ie2, &de2);
  EXPECT_EQ(0.0, pe2);
  EXPECT_EQ(0.0, ie2);
  EXPECT_EQ(0.0, de2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
