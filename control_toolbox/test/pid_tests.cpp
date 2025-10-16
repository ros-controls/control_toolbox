// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <cstdlib>
#include <string>

#include "control_toolbox/pid.hpp"

#include "gmock/gmock.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

using control_toolbox::AntiWindupStrategy;
using control_toolbox::Pid;
using namespace std::chrono_literals;

TEST(ParameterTest, UTermBadIBoundsTestConstructor)
{
  RecordProperty(
    "description",
    "This test checks if an error is thrown for bad u_bounds specification (i.e. u_min > u_max).");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = 1.0;
  antiwindup_strat.i_min = -1.0;
  EXPECT_THROW(Pid pid(1.0, 1.0, 1.0, -1.0, 1.0, antiwindup_strat), std::invalid_argument);
}

TEST(ParameterTest, UTermBadIBoundsTest)
{
  RecordProperty(
    "description",
    "This test checks if gains remain for bad u_bounds specification (i.e. u_min > u_max).");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = 1.0;
  antiwindup_strat.i_min = -1.0;
  Pid pid(1.0, 1.0, 1.0, 1.0, -1.0, antiwindup_strat);
  auto gains = pid.get_gains();
  EXPECT_DOUBLE_EQ(gains.u_max_, 1.0);
  EXPECT_DOUBLE_EQ(gains.u_min_, -1.0);
  // Try to set bad u-bounds, i.e. u_min > u_max
  EXPECT_NO_THROW(pid.set_gains(1.0, 1.0, 1.0, -1.0, 1.0, antiwindup_strat));
  // Check if gains were not updated because u-bounds are bad, i.e. u_min > u_max
  EXPECT_DOUBLE_EQ(gains.u_max_, 1.0);
  EXPECT_DOUBLE_EQ(gains.u_min_, -1.0);
}

TEST(ParameterTest, outputClampTest)
{
  RecordProperty(
    "description", "This test succeeds if the output is clamped when the saturation is active.");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::BACK_CALCULATION;
  antiwindup_strat.tracking_time_constant = 0.0;  // Set to 0.0 to use the default value
  // Setting u_max = 1.0 and u_min = -1.0 to test clamping
  Pid pid(1.0, 0.0, 0.0, 1.0, -1.0, antiwindup_strat);

  double cmd = 0.0;

  // ***** TEST UPPER LIMIT *****

  cmd = pid.compute_command(0.5, 1.0);
  EXPECT_EQ(0.5, cmd);

  cmd = pid.compute_command(1.0, 1.0);
  EXPECT_EQ(1.0, cmd);

  cmd = pid.compute_command(2.0, 1.0);
  EXPECT_EQ(1.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  EXPECT_EQ(1.0, cmd);

  cmd = pid.compute_command(50.0, 1.0);
  EXPECT_EQ(1.0, cmd);

  cmd = pid.compute_command(100.0, 1.0);
  EXPECT_EQ(1.0, cmd);

  // ***** TEST LOWER LIMIT *****

  cmd = pid.compute_command(-0.5, 1.0);
  EXPECT_EQ(-0.5, cmd);

  cmd = pid.compute_command(-1, 1.0);
  EXPECT_EQ(-1.0, cmd);

  cmd = pid.compute_command(-2, 1.0);
  EXPECT_EQ(-1.0, cmd);

  cmd = pid.compute_command(-10, 1.0);
  EXPECT_EQ(-1.0, cmd);

  cmd = pid.compute_command(-50, 1.0);
  EXPECT_EQ(-1.0, cmd);

  cmd = pid.compute_command(-100, 1.0);
  EXPECT_EQ(-1.0, cmd);
}

TEST(ParameterTest, noOutputClampTest)
{
  RecordProperty(
    "description", "This test succeeds if the output isn't clamped when the saturation is false.");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::BACK_CALCULATION;
  antiwindup_strat.tracking_time_constant = 0.0;  // Set to 0.0 to use the default value
  // Setting u_max = INF and u_min = -INF to disable clamping
  Pid pid(
    1.0, 0.0, 0.0, std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), antiwindup_strat);

  double cmd = 0.0;

  // ***** TEST UPPER LIMIT *****

  cmd = pid.compute_command(0.5, 1.0);
  EXPECT_EQ(0.5, cmd);

  cmd = pid.compute_command(1.0, 1.0);
  EXPECT_EQ(1.0, cmd);

  cmd = pid.compute_command(2.0, 1.0);
  EXPECT_EQ(2.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  EXPECT_EQ(10.0, cmd);

  cmd = pid.compute_command(50.0, 1.0);
  EXPECT_EQ(50.0, cmd);

  cmd = pid.compute_command(100.0, 1.0);
  EXPECT_EQ(100.0, cmd);

  // ***** TEST LOWER LIMIT *****

  cmd = pid.compute_command(-0.5, 1.0);
  EXPECT_EQ(-0.5, cmd);

  cmd = pid.compute_command(-1, 1.0);
  EXPECT_EQ(-1.0, cmd);

  cmd = pid.compute_command(-2, 1.0);
  EXPECT_EQ(-2.0, cmd);

  cmd = pid.compute_command(-10, 1.0);
  EXPECT_EQ(-10.0, cmd);

  cmd = pid.compute_command(-50, 1.0);
  EXPECT_EQ(-50.0, cmd);

  cmd = pid.compute_command(-100, 1.0);
  EXPECT_EQ(-100.0, cmd);
}

TEST(ParameterTest, integrationBackCalculationZeroGainTest)
{
  RecordProperty(
    "description",
    "This test succeeds if the integral contribution is clamped when the integral gain is zero for "
    "the back calculation technique.");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::BACK_CALCULATION;
  antiwindup_strat.tracking_time_constant = 0.0;  // Set to 0.0 to use the default value
  Pid pid(0.0, 0.0, 0.0, 20.0, -20.0, antiwindup_strat);

  double cmd = 0.0;
  double pe, ie, de;

  // with i_gain = 0, tracking_time_constant should not be calculated
  auto gains = pid.get_gains();
  EXPECT_DOUBLE_EQ(gains.antiwindup_strat_.tracking_time_constant, 0.0);

  // back_calculation

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);
}

TEST(ParameterTest, integrationConditionalIntegrationZeroGainTest)
{
  RecordProperty(
    "description",
    "This test succeeds if the integral contribution is clamped when the integral gain is zero for "
    "the conditional integration technique.");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::CONDITIONAL_INTEGRATION;
  Pid pid(0.0, 0.0, 0.0, 20.0, -20.0, antiwindup_strat);

  double cmd = 0.0;
  double pe, ie, de;

  // back_calculation

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(10.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(0.0, ie);
  EXPECT_EQ(0.0, cmd);
}

TEST(ParameterTest, ITermBadIBoundsTestConstructor)
{
  RecordProperty(
    "description",
    "This test checks if an error is thrown for bad i_bounds specification (i.e. "
    "i_min > i_max).");

  // Check that the output is not a non-sense if i-bounds are bad, i.e. i_min > i_max
  EXPECT_THROW(Pid pid(1.0, 1.0, 1.0, -1.0, 1.0), std::invalid_argument);
}

TEST(ParameterTest, ITermBadIBoundsTest)
{
  RecordProperty(
    "description",
    "This test checks if gains remain for bad i_bounds specification (i.e. "
    "i_min > i_max).");

  Pid pid(1.0, 1.0, 1.0, 1.0, -1.0);
  auto gains = pid.get_gains();
  EXPECT_DOUBLE_EQ(gains.antiwindup_strat_.i_min, -1.0);
  EXPECT_DOUBLE_EQ(gains.antiwindup_strat_.i_max, 1.0);
  // Try to set bad i-bounds, i.e. i_min > i_max
  EXPECT_NO_THROW(pid.set_gains(1.0, 1.0, 1.0, -2.0, 2.0));
  // Check if gains were not updated because i-bounds are bad, i.e. i_min > i_max
  EXPECT_DOUBLE_EQ(gains.antiwindup_strat_.i_min, -1.0);
  EXPECT_DOUBLE_EQ(gains.antiwindup_strat_.i_max, 1.0);
}

TEST(ParameterTest, integrationClampTest)
{
  RecordProperty(
    "description",
    "This test succeeds if the integral contribution is clamped when the integral gain is "
    "non-zero.");

  Pid pid(0.0, 1.0, 0.0, 1.0, -1.0);

  double cmd = 0.0;

  // Test lower limit
  cmd = pid.compute_command(-10.03, 1.0);
  EXPECT_EQ(-1.0, cmd);

  // Test upper limit
  cmd = pid.compute_command(30.0, 1.0);
  EXPECT_EQ(1.0, cmd);
}

TEST(ParameterTest, integrationClampZeroGainTest)
{
  RecordProperty(
    "description",
    "This test succeeds if the integral contribution is clamped when the integral gain is zero. If "
    "the integral contribution is not clamped while it is disabled, it can cause sudden jumps to "
    "the minimum or maximum bound in control command when re-enabled.");

  double i_gain = 0.0;
  double i_min = -1.0;
  double i_max = 1.0;
  Pid pid(0.0, i_gain, 0.0, i_max, i_min);

  double cmd = 0.0;
  double pe, ie, de;

  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_LE(i_min, cmd);
  EXPECT_LE(cmd, i_max);
  EXPECT_EQ(0.0, cmd);

  cmd = pid.compute_command(-1.0, 1.0);
  EXPECT_LE(i_min, cmd);
  EXPECT_LE(cmd, i_max);
  EXPECT_EQ(0.0, cmd);
}

constexpr double EPS = 1e-9;
TEST(ParameterTest, integrationAntiwindupTest)
{
  RecordProperty(
    "description",
    "This test succeeds if the integral error is prevented from winding up when i_gain > 0");

  double i_gain = 2.0;
  double i_min = -1.0;
  double i_max = 1.0;
  Pid pid(0.0, i_gain, 0.0, i_max, i_min, true);

  double cmd = 0.0;

  cmd = pid.compute_command(-1.0, 1.0);
  EXPECT_NEAR(-1.0, cmd, EPS);

  cmd = pid.compute_command(-1.0, 1.0);
  EXPECT_NEAR(-1.0, cmd, EPS);

  cmd = pid.compute_command(0.5, 1.0);
  EXPECT_NEAR(0.0, cmd, EPS);

  cmd = pid.compute_command(-1.0, 1.0);
  EXPECT_NEAR(-1.0, cmd, EPS);
}

TEST(ParameterTest, negativeIntegrationAntiwindupTest)
{
  RecordProperty(
    "description",
    "This test succeeds if the integral error is prevented from winding up when i_gain < 0");

  double i_gain = -2.5;
  double i_min = -0.2;
  double i_max = 0.5;
  Pid pid(0.0, i_gain, 0.0, i_max, i_min, true);

  double cmd = 0.0;

  cmd = pid.compute_command(0.1, 1.0);
  EXPECT_NEAR(-0.2, cmd, EPS);

  cmd = pid.compute_command(0.1, 1.0);
  EXPECT_NEAR(-0.2, cmd, EPS);

  cmd = pid.compute_command(-0.05, 1.0);
  EXPECT_NEAR(-0.075, cmd, EPS);

  cmd = pid.compute_command(0.1, 1.0);
  EXPECT_NEAR(-0.2, cmd, EPS);
}

TEST(ParameterTest, gainSettingCopyPIDTest)
{
  RecordProperty(
    "description",
    "This test succeeds if a PID object has its gain set at different points in time then the "
    "values are get-ed and still remain the same, as well as when PID is copied.");

  // Test values
  double p_gain = std::rand() % 100;
  double i_gain = std::rand() % 100;
  double d_gain = std::rand() % 100;
  double i_max = std::rand() % 100;
  double i_min = -1 * std::rand() % 100;
  double u_max = std::numeric_limits<double>::infinity();
  double u_min = -1 * u_max;
  double tracking_time_constant = std::rand() % 100;
  bool antiwindup = false;
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = i_max;
  antiwindup_strat.i_min = i_min;
  antiwindup_strat.tracking_time_constant = tracking_time_constant;
  antiwindup_strat.legacy_antiwindup = antiwindup;

  // Initialize the default way
  Pid pid1(p_gain, i_gain, d_gain, u_max, u_min, antiwindup_strat);

  // Test return values  -------------------------------------------------
  double p_gain_return, i_gain_return, d_gain_return, u_max_return, u_min_return;
  AntiWindupStrategy antiwindup_strat_return;

  pid1.get_gains(
    p_gain_return, i_gain_return, d_gain_return, u_max_return, u_min_return,
    antiwindup_strat_return);

  EXPECT_EQ(p_gain, p_gain_return);
  EXPECT_EQ(i_gain, i_gain_return);
  EXPECT_EQ(d_gain, d_gain_return);
  EXPECT_EQ(u_max, u_max_return);
  EXPECT_EQ(u_min, u_min_return);
  EXPECT_EQ(tracking_time_constant, antiwindup_strat_return.tracking_time_constant);
  EXPECT_EQ(i_min, antiwindup_strat_return.i_min);
  EXPECT_EQ(i_max, antiwindup_strat_return.i_max);
  EXPECT_EQ(antiwindup, antiwindup_strat_return.legacy_antiwindup);
  EXPECT_EQ(antiwindup_strat.to_string(), antiwindup_strat_return.to_string());

  // Test return values using struct -------------------------------------------------

  // New values
  p_gain = std::rand() % 100;
  i_gain = std::rand() % 100;
  d_gain = std::rand() % 100;
  i_max = std::rand() % 100;
  i_min = -1 * std::rand() % 100;
  u_max = std::numeric_limits<double>::infinity();
  u_min = -1 * u_max;
  tracking_time_constant = std::rand() % 100;
  antiwindup = false;
  antiwindup_strat.type = AntiWindupStrategy::LEGACY;
  antiwindup_strat.i_max = i_max;
  antiwindup_strat.i_min = i_min;
  antiwindup_strat.tracking_time_constant = tracking_time_constant;
  antiwindup_strat.legacy_antiwindup = antiwindup;

  pid1.set_gains(p_gain, i_gain, d_gain, u_max, u_min, antiwindup_strat);

  Pid::Gains g1 = pid1.get_gains();
  EXPECT_EQ(p_gain, g1.p_gain_);
  EXPECT_EQ(i_gain, g1.i_gain_);
  EXPECT_EQ(d_gain, g1.d_gain_);
  EXPECT_EQ(i_max, g1.antiwindup_strat_.i_max);
  EXPECT_EQ(i_min, g1.antiwindup_strat_.i_min);
  EXPECT_EQ(u_max, g1.u_max_);
  EXPECT_EQ(u_min, g1.u_min_);
  EXPECT_EQ(antiwindup, g1.antiwindup_);
  EXPECT_EQ(tracking_time_constant, g1.antiwindup_strat_.tracking_time_constant);
  EXPECT_EQ(i_max, g1.antiwindup_strat_.i_max);
  EXPECT_EQ(i_min, g1.antiwindup_strat_.i_min);
  EXPECT_EQ(antiwindup, g1.antiwindup_strat_.legacy_antiwindup);
  EXPECT_EQ(antiwindup_strat.to_string(), g1.antiwindup_strat_.to_string());

  // Send update command to populate errors -------------------------------------------------
  pid1.set_current_cmd(10);
  (void)pid1.compute_command(20, 1.0);

  // Test copy constructor -------------------------------------------------
  Pid pid2(pid1);

  pid2.get_gains(
    p_gain_return, i_gain_return, d_gain_return, u_max_return, u_min_return,
    antiwindup_strat_return);

  EXPECT_EQ(p_gain_return, g1.p_gain_);
  EXPECT_EQ(i_gain_return, g1.i_gain_);
  EXPECT_EQ(d_gain_return, g1.d_gain_);
  EXPECT_EQ(antiwindup_strat_return.i_max, g1.antiwindup_strat_.i_max);
  EXPECT_EQ(antiwindup_strat_return.i_min, g1.antiwindup_strat_.i_min);
  EXPECT_EQ(u_max, g1.u_max_);
  EXPECT_EQ(u_min, g1.u_min_);
  EXPECT_EQ(tracking_time_constant, g1.antiwindup_strat_.tracking_time_constant);
  EXPECT_EQ(antiwindup_strat_return.i_max, g1.antiwindup_strat_.i_max);
  EXPECT_EQ(antiwindup_strat_return.i_min, g1.antiwindup_strat_.i_min);
  EXPECT_EQ(antiwindup, g1.antiwindup_strat_.legacy_antiwindup);
  EXPECT_EQ(antiwindup_strat.to_string(), g1.antiwindup_strat_.to_string());

  // Test that errors are zero
  double pe2, ie2, de2;
  pid2.get_current_pid_errors(pe2, ie2, de2);
  EXPECT_EQ(0.0, pe2);
  EXPECT_EQ(0.0, ie2);
  EXPECT_EQ(0.0, de2);

  // Test assignment constructor -------------------------------------------------
  Pid pid3;
  pid3 = pid1;

  pid3.get_gains(
    p_gain_return, i_gain_return, d_gain_return, u_max_return, u_min_return,
    antiwindup_strat_return);

  EXPECT_EQ(p_gain_return, g1.p_gain_);
  EXPECT_EQ(i_gain_return, g1.i_gain_);
  EXPECT_EQ(d_gain_return, g1.d_gain_);
  EXPECT_EQ(antiwindup_strat_return.i_max, g1.antiwindup_strat_.i_max);
  EXPECT_EQ(antiwindup_strat_return.i_min, g1.antiwindup_strat_.i_min);
  EXPECT_EQ(u_max, g1.u_max_);
  EXPECT_EQ(u_min, g1.u_min_);
  EXPECT_EQ(tracking_time_constant, g1.antiwindup_strat_.tracking_time_constant);
  EXPECT_EQ(antiwindup_strat_return.i_max, g1.antiwindup_strat_.i_max);
  EXPECT_EQ(antiwindup_strat_return.i_min, g1.antiwindup_strat_.i_min);
  EXPECT_EQ(antiwindup, g1.antiwindup_strat_.legacy_antiwindup);
  EXPECT_EQ(antiwindup_strat.to_string(), g1.antiwindup_strat_.to_string());

  // Test that errors are zero
  double pe3, ie3, de3;
  pid3.get_current_pid_errors(pe3, ie3, de3);
  EXPECT_EQ(0.0, pe3);
  EXPECT_EQ(0.0, ie3);
  EXPECT_EQ(0.0, de3);

  // Test the reset() function, it should clear errors and command
  pid1.reset();

  double pe1, ie1, de1;
  pid1.get_current_pid_errors(pe1, ie1, de1);
  EXPECT_EQ(0.0, pe1);
  EXPECT_EQ(0.0, ie1);
  EXPECT_EQ(0.0, de1);

  double cmd1 = pid1.get_current_cmd();
  EXPECT_EQ(0.0, cmd1);
}

TEST(CommandTest, proportionalOnlyTest)
{
  RecordProperty(
    "description",
    "This test checks that a command is computed correctly using the proportional contribution "
    "only.");

  // Set only proportional gain
  Pid pid(
    1.0, 0.0, 0.0, std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity());
  double cmd = 0.0;

  // If initial error = 0, p-gain = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect the same as before
  EXPECT_EQ(-0.5, cmd);

  // If call again doubling the error
  cmd = pid.compute_command(-1.0, 1.0);
  // Then expect the command doubled
  EXPECT_EQ(-1.0, cmd);

  // If call with positive error
  cmd = pid.compute_command(0.5, 1.0);
  // Then expect always command = error
  EXPECT_EQ(0.5, cmd);
}

TEST(CommandTest, integralOnlyTest)
{
  RecordProperty(
    "description",
    "This test checks that a command is computed correctly using the integral contribution only "
    "(ATTENTION: this test depends on the integration scheme).");

  // Set only integral gains with enough limits to test behavior
  Pid pid(0.0, 1.0, 0.0, 5.0, -5.0);
  double cmd = 0.0;

  // If initial error = 0, i-gain = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again with same arguments
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect the integral part to double the command
  EXPECT_EQ(-1.0, cmd);

  // Call again with no error
  cmd = pid.compute_command(0.0, 1.0);
  // Expect the integral part to keep the previous command because it ensures error = 0
  EXPECT_EQ(-1.0, cmd);

  // Double check that the integral contribution keep the previous command
  cmd = pid.compute_command(0.0, 1.0);
  EXPECT_EQ(-1.0, cmd);

  // Finally call again with positive error to see if the command changes in the opposite direction
  cmd = pid.compute_command(1.0, 1.0);
  // Expect that the command is cleared since error = -1 * previous command, i-gain = 1, dt = 1
  EXPECT_EQ(0.0, cmd);

  // If initial error = 0, i-gain = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);
  // after reset without argument (save_i_term=false)
  // we expect the command to be 0 if update is called error = 0
  pid.reset();
  cmd = pid.compute_command(0.0, 1.0);
  EXPECT_EQ(0.0, cmd);

  // If initial error = 0, i-gain = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);
  // after reset with argument (save_i_term=false)
  // we expect the command to be 0 if update is called error = 0
  pid.reset(false);
  cmd = pid.compute_command(0.0, 1.0);
  EXPECT_EQ(0.0, cmd);

  // If initial error = 0, i-gain = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);
  // after reset with save_i_term=true
  // we expect still the same command if update is called error = 0
  pid.reset(true);
  cmd = pid.compute_command(0.0, 1.0);
  EXPECT_EQ(-0.5, cmd);
}

TEST(CommandTest, derivativeOnlyTest)
{
  RecordProperty(
    "description",
    "This test checks that a command is computed correctly using the derivative contribution only "
    "with own differentiation (ATTENTION: this test depends on the differentiation scheme).");

  // Set only derivative gain
  Pid pid(
    0.0, 0.0, 1.0, std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity());
  double cmd = 0.0;

  // If initial error = 0, d-gain = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = error
  EXPECT_EQ(-0.5, cmd);

  // If call again with same error
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = 0 due to no variation on error
  EXPECT_EQ(0.0, cmd);

  // If call again with same error and smaller control period
  cmd = pid.compute_command(-0.5, 0.1);
  // Then expect command = 0 again
  EXPECT_EQ(0.0, cmd);

  // If the error increases,  with dt = 1
  cmd = pid.compute_command(-1.0, 1.0);
  // Then expect the command = change in dt
  EXPECT_EQ(-0.5, cmd);

  // If error decreases, with dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect always the command = change in dt (note the sign flip)
  EXPECT_EQ(0.5, cmd);
}

TEST(CommandTest, completePIDTest)
{
  RecordProperty(
    "description",
    "This test checks that  a command is computed correctly using a complete PID controller "
    "(ATTENTION: this test depends on the integral and differentiation schemes).");

  Pid pid(1.0, 1.0, 1.0, 5.0, -5.0);
  double cmd = 0.0;

  // All contributions are tested, here few tests check that they sum up correctly
  // If initial error = 0, all gains = 1, dt = 1
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = 3x error
  EXPECT_EQ(-1.5, cmd);

  // If call again with same arguments, no error change, but integration do its part
  cmd = pid.compute_command(-0.5, 1.0);
  // Then expect command = 3x error again
  EXPECT_EQ(-1.5, cmd);

  // If call again increasing the error
  cmd = pid.compute_command(-1.0, 1.0);
  // Then expect command equals to p = -1, i = -2.0 (i.e. - 0.5 - 0.5 - 1.0), d = -0.5
  EXPECT_EQ(-3.5, cmd);
}

TEST(CommandTest, compatibilityTest)
{
  RecordProperty("description", "Tests deprecated methods.");

// Disable deprecated warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

  Pid pid;
  Pid pid_old;
  pid.initialize(1.0, 1.0, 1.0, 5.0, -5.0);
  pid_old.initPid(1.0, 1.0, 1.0, 5.0, -5.0);

  auto cmd1 = pid.compute_command(-0.5, 1.0);
  auto cmd2 = pid_old.computeCommand(-0.5, static_cast<uint64_t>(1.0 * 1e9));
  EXPECT_EQ(cmd1, cmd2);

  pid.set_gains(2.0, 1.0, 1.0, 5.0, -5.0);
  pid_old.setGains(2.0, 1.0, 1.0, 5.0, -5.0);

  cmd1 = pid.compute_command(-0.5, 1.0);
  cmd2 = pid_old.computeCommand(-0.5, static_cast<uint64_t>(1.0 * 1e9));
  EXPECT_EQ(cmd1, cmd2);

// Re-enable deprecated warnings
#pragma GCC diagnostic pop
}

TEST(CommandTest, backCalculationPIDTest)
{
  RecordProperty(
    "description",
    "This test checks that  a command is computed correctly using a complete PID controller with "
    "back calculation technique.");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  // Setting u_max = 5.0 and u_min = -5.0 to test clamping
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::BACK_CALCULATION;
  antiwindup_strat.tracking_time_constant = 1.0;  // Set to 0.0 to use the default value
  Pid pid(0.0, 1.0, 0.0, 5.0, -5.0, antiwindup_strat);

  double cmd = 0.0;
  double pe, ie, de;

  // Small error to not have saturation
  cmd = pid.compute_command(1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(1.0, ie);
  EXPECT_EQ(0.0, cmd);

  // Small error to not have saturation
  cmd = pid.compute_command(2.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(3.0, ie);
  EXPECT_EQ(1.0, cmd);

  // Error to cause saturation
  cmd = pid.compute_command(3.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(6.0, ie);
  EXPECT_EQ(3.0, cmd);

  // Saturation applied, back calculation now reduces the integral term
  cmd = pid.compute_command(1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(6.0, ie);
  EXPECT_EQ(5.0, cmd);

  // Saturation applied, back calculation now reduces the integral term
  cmd = pid.compute_command(2.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(7.0, ie);
  EXPECT_EQ(5.0, cmd);

  // Saturation applied, back calculation now reduces the integral term
  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(4.0, ie);
  EXPECT_EQ(5.0, cmd);

  // PID recover from the windup/saturation
  cmd = pid.compute_command(1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(5.0, ie);
  EXPECT_EQ(4.0, cmd);
}

TEST(CommandTest, conditionalIntegrationPIDTest)
{
  RecordProperty(
    "description",
    "This test checks that  a command is computed correctly using a complete PID controller with "
    "conditional integration technique.");

  // Pid(double p, double i, double d, double u_max, double u_min,
  // AntiWindupStrategy antiwindup_strat);
  // Setting u_max = 5.0 and u_min = -5.0 to test clamping
  AntiWindupStrategy antiwindup_strat;
  antiwindup_strat.type = AntiWindupStrategy::CONDITIONAL_INTEGRATION;
  antiwindup_strat.tracking_time_constant = 1.0;
  Pid pid(0.0, 1.0, 0.0, 5.0, -5.0, antiwindup_strat);

  double cmd = 0.0;
  double pe, ie, de;

  // Small error to not have saturation
  cmd = pid.compute_command(1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(1.0, ie);
  EXPECT_EQ(0.0, cmd);

  // Small error to not have saturation
  cmd = pid.compute_command(2.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(3.0, ie);
  EXPECT_EQ(1.0, cmd);

  // Error to cause saturation
  cmd = pid.compute_command(3.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(6.0, ie);
  EXPECT_EQ(3.0, cmd);

  // Saturation applied, conditional integration now holds the integral term
  cmd = pid.compute_command(1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(6.0, ie);
  EXPECT_EQ(5.0, cmd);

  // Saturation applied, conditional integration now holds the integral term
  cmd = pid.compute_command(2.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(6.0, ie);
  EXPECT_EQ(5.0, cmd);

  // PID recover from the windup/saturation
  cmd = pid.compute_command(-1.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(5.0, ie);
  EXPECT_EQ(5.0, cmd);

  // PID recover from the windup/saturation
  cmd = pid.compute_command(0.0, 1.0);
  pid.get_current_pid_errors(pe, ie, de);
  EXPECT_EQ(5.0, ie);
  EXPECT_EQ(5.0, cmd);
}

TEST(CommandTest, timeArgumentTest)
{
  RecordProperty("description", "Tests different dt argument type methods.");

  Pid pid1(1.0, 1.0, 1.0, 5.0, -5.0);
  Pid pid2(1.0, 1.0, 1.0, 5.0, -5.0);
  Pid pid3(1.0, 1.0, 1.0, 5.0, -5.0);
  Pid pid4(1.0, 1.0, 1.0, 5.0, -5.0);

  // call without error_dt, dt is used to calculate error_dt
  auto cmd1 = pid1.compute_command(-0.5, 1.0);
  auto dt = rclcpp::Duration::from_seconds(1.0);
  auto cmd2 = pid2.compute_command(-0.5, dt);
  auto cmd3 = pid3.compute_command(-0.5, dt.nanoseconds());
  auto cmd4 = pid4.compute_command(-0.5, std::chrono::nanoseconds(1s));

  // If initial error = 0, all gains = 1, dt = 1
  // Then expect command = 3x error
  EXPECT_EQ(-1.5, cmd1);
  EXPECT_EQ(cmd1, cmd2);
  EXPECT_EQ(cmd1, cmd3);
  EXPECT_EQ(cmd1, cmd4);

  // call with error_dt -> integral part is updated with dt
  cmd1 = pid1.compute_command(-0.5, 0.0, 1.0);
  cmd2 = pid2.compute_command(-0.5, 0.0, dt);
  cmd3 = pid3.compute_command(-0.5, 0.0, dt.nanoseconds());
  cmd4 = pid4.compute_command(-0.5, 0.0, std::chrono::nanoseconds(1s));
  EXPECT_EQ(-1.5, cmd1);
  EXPECT_EQ(cmd1, cmd2);
  EXPECT_EQ(cmd1, cmd3);
  EXPECT_EQ(cmd1, cmd4);
  cmd1 = pid1.compute_command(-0.5, 0.0, 1.0);
  cmd2 = pid2.compute_command(-0.5, 0.0, dt);
  cmd3 = pid3.compute_command(-0.5, 0.0, dt.nanoseconds());
  cmd4 = pid4.compute_command(-0.5, 0.0, std::chrono::nanoseconds(1s));
  EXPECT_EQ(-2.0, cmd1);
  EXPECT_EQ(cmd1, cmd2);
  EXPECT_EQ(cmd1, cmd3);
  EXPECT_EQ(cmd1, cmd4);

  // call with dt=0, nothing should change
  double pe, ie1, de, ie2;
  pid1.get_current_pid_errors(pe, ie1, de);
  cmd1 = pid1.compute_command(-0.5, 0.0, 0.0);
  pid1.get_current_pid_errors(pe, ie2, de);
  EXPECT_EQ(-2.0, cmd1);
  EXPECT_EQ(ie1, ie2);
  // should throw if called with negative dt
  EXPECT_THROW(cmd1 = pid1.compute_command(-0.5, 0.0, -1.0), std::invalid_argument);
  // call with nan, should return NaN but not reset internal states
  cmd1 = pid1.compute_command(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0);
  cmd3 = pid1.get_current_cmd();
  EXPECT_FALSE(std::isfinite(cmd1));
  EXPECT_FALSE(std::isfinite(cmd3));
  pid1.get_current_pid_errors(pe, ie2, de);
  EXPECT_EQ(ie1, ie2);
  cmd2 = pid2.compute_command(-0.5, std::numeric_limits<double>::quiet_NaN(), 1.0);
  cmd4 = pid2.get_current_cmd();
  EXPECT_FALSE(std::isfinite(cmd2));
  EXPECT_FALSE(std::isfinite(cmd4));
  pid1.get_current_pid_errors(pe, ie2, de);
  EXPECT_EQ(ie1, ie2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}

#pragma GCC diagnostic pop
