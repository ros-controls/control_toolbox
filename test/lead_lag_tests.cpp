#include <control_toolbox/lead_lag.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/math/special_functions/fpclassify.hpp>

using namespace control_toolbox;

TEST(ParameterTest, gainSettingCopyLeadLagTest) {
  RecordProperty("description",
                 "This test succeeds if a LeadLag object has its gain set at "
                 "different points in time then the values are get-ed and "
                 "still remain the same, as well as when LeadLag is copied.");

  // Test values
  double k = rand() % 100;
  double a = rand() % 100;
  double b = rand() % 100;

  // Initialize the default way
  LeadLag controller1(k, a, b);

  // Test return values  -------------------------------------------------
  double k_return, a_return, b_return;
  controller1.getGains(k_return, a_return, b_return);

  EXPECT_EQ(k, k_return);
  EXPECT_EQ(a, a_return);
  EXPECT_EQ(b, b_return);

  // Test return values using struct
  // -------------------------------------------------

  // New values
  k = rand() % 100;
  a = rand() % 100;
  b = rand() % 100;
  controller1.setGains(k, a, b);

  LeadLag::Gains g1 = controller1.getGains();
  EXPECT_EQ(k, g1.k_);
  EXPECT_EQ(a, g1.a_);
  EXPECT_EQ(b, g1.b_);

  // \todo test initParam() -------------------------------------------------

  // \todo test bool init(const ros::NodeHandle &n);
  // -----------------------------------

  // Send update command to populate errors
  // -------------------------------------------------
  controller1.setCurrentCmd(10);
  controller1.computeCommand(20, ros::Duration(1.0));

  // Test copy constructor -------------------------------------------------
  LeadLag controller2(controller1);

  controller2.getGains(k_return, a_return, b_return);

  EXPECT_EQ(k, k_return);
  EXPECT_EQ(a, a_return);
  EXPECT_EQ(b, b_return);

  // Test that errors are zero
  EXPECT_EQ(0.0, controller2.getCurrentError());

  // Test assignment constructor
  // -------------------------------------------------
  LeadLag controller3;
  controller3 = controller1;

  controller3.getGains(k_return, a_return, b_return);

  EXPECT_EQ(k, k_return);
  EXPECT_EQ(a, a_return);
  EXPECT_EQ(b, b_return);

  // Test that errors are zero
  EXPECT_EQ(0.0, controller3.getCurrentError());

  // Test the reset() function, it should clear errors and command
  controller1.reset();

  EXPECT_EQ(0.0, controller3.getCurrentError());
  EXPECT_EQ(0.0, controller3.getCurrentCmd());
}

TEST(CommandTest, proportionalOnlyTest) {
  RecordProperty("description",
                 "This test checks that a command is computed correctly using "
                 "the proportional contribution only.");

  // Set only proportional gain
  LeadLag controller(1.0, 0.0, 0.0);
  double cmd = 0.0;

  // If initial error = 0, p-gain = 1, dt = 1
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect the same as before
  EXPECT_EQ(-0.5, cmd);

  // If call again doubling the error
  cmd = controller.computeCommand(-1.0, ros::Duration(1.0));
  // Then expect the command doubled
  EXPECT_EQ(-1.0, cmd);

  // If call with positive error
  cmd = controller.computeCommand(0.5, ros::Duration(1.0));
  // Then expect always command = error
  EXPECT_EQ(0.5, cmd);
}

TEST(CommandTest, zeroOnlyTest) {
  RecordProperty("description",
                 "This test checks that a command is computed correctly using "
                 "the zero contribution only.");
  LeadLag controller(0.0, 1.0, 0.0);
  double cmd = 0.0;

  // If initial error = 0
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect command to be zero
  EXPECT_EQ(0.0, cmd);

  // If call again with same arguments
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect the command to be zero
  EXPECT_EQ(0.0, cmd);

  // Call again with no error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // Then expect the command to be zero
  EXPECT_EQ(0.0, cmd);
}

TEST(CommandTest, poleOnlyTest) {
  RecordProperty("description",
                 "This test checks that a command is computed correctly using "
                 "the pole contribution only.");
  LeadLag controller(0.0, 0.0, 1.0);
  double cmd = 0.0;

  // If initial error = 0
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect command to be zero
  EXPECT_EQ(0.0, cmd);

  // If call again with same arguments
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect the pole part to be zero
  EXPECT_EQ(0.0, cmd);

  // Call again with no error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // Then expect the command to be zero
  EXPECT_EQ(0.0, cmd);
}

TEST(CommandTest, proportionalAndZeroTest) {
  RecordProperty("description",
                 "This test checks that a command is computed correctly using "
                 "the proportional and the zero contribution.");
  LeadLag controller(1.0, 1.0, 0.0);
  double cmd = 0.0;

  // If initial error = 0
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again with same arguments
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect the zero part to be equal to the proportional part
  EXPECT_EQ(0.0, cmd);

  // Call again with no error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // Then expect the command to be 0.5
  EXPECT_EQ(0.5, cmd);

  // Call again with no error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // Then expect the command to be zero
  EXPECT_EQ(0.0, cmd);
}

TEST(CommandTest, proportionalAndPoleTest) {
  RecordProperty("description",
                 "This test checks that a command is computed correctly using "
                 "the pole contribution only.");
  LeadLag controller(1.0, 0.0, 1.0);
  double cmd = 0.0;

  // If initial error = 0
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again with same arguments
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect the pole part to be equal to the proportional part
  EXPECT_EQ(-1.0, cmd);

  // Call again with no error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // Then expect the command to be -1.0
  EXPECT_EQ(-1.0, cmd);

  // Call again with no error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // Then expect the command to be -1.0
  EXPECT_EQ(-1.0, cmd);
}

TEST(CommandTest, completeLeadLagTest) {
  RecordProperty("description",
                 "This test checks that a command is computed correctly using "
                 "a complete LeadLag controller.");
  LeadLag controller(1.0, 1.0, -2.0);
  double cmd = 0.0;

  // If initial error = 0
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again with same arguments
  cmd = controller.computeCommand(-0.5, ros::Duration(1.0));
  // The expected command is from the Pole only
  EXPECT_EQ(1.0, cmd);

  // If call with zero error
  cmd = controller.computeCommand(0.0, ros::Duration(1.0));
  // The zero causes 0.5 but the pole causes -2.0
  EXPECT_EQ(-1.5, cmd);

  // If call with 2.0
  cmd = controller.computeCommand(2.0, ros::Duration(1.0));
  // The proportional causes 2.0 and the pole causes 3.0
  EXPECT_EQ(5.0, cmd);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
