// Copyright (c) 2026, ros2_control development team
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

#include <gmock/gmock.h>

#include <limits>
#include <vector>

#include "control_toolbox/low_pass_filter.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"

TEST(LowPassFilterTest, FirstUpdateReturnsInitialState)
{
  control_toolbox::LowPassFilter<double> filter(1000.0, 20.5, 1.25);

  ASSERT_TRUE(filter.configure(1.5));

  double out = 0.0;
  ASSERT_TRUE(filter.update(10.0, out));

  EXPECT_NEAR(out, 1.5, 1e-12);
}

TEST(LowPassFilterTest, RejectsNonFiniteInitialState)
{
  control_toolbox::LowPassFilter<double> filter(1000.0, 20.5, 1.25);

  EXPECT_FALSE(filter.configure(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(filter.is_configured());

  EXPECT_FALSE(filter.configure(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(filter.is_configured());
}

TEST(LowPassFilterTest, WithoutInitialStateFirstUpdateReturnsInput)
{
  control_toolbox::LowPassFilter<double> filter(1000.0, 20.5, 1.25);

  ASSERT_TRUE(filter.configure());

  double out = 0.0;
  ASSERT_TRUE(filter.update(10.0, out));

  EXPECT_NEAR(out, 10.0, 1e-12);
}

TEST(LowPassFilterTest, VectorFirstUpdateReturnsInitialState)
{
  control_toolbox::LowPassFilter<std::vector<double>> filter(1000.0, 20.5, 1.25);

  const std::vector<double> initial_state = {1.0, -2.0, 3.0};
  ASSERT_TRUE(filter.configure(initial_state));

  std::vector<double> out(initial_state.size(), 0.0);
  ASSERT_TRUE(filter.update({10.0, 10.0, 10.0}, out));

  ASSERT_EQ(out.size(), initial_state.size());
  for (size_t i = 0; i < out.size(); ++i)
  {
    EXPECT_NEAR(out[i], initial_state[i], 1e-12);
  }
}

TEST(LowPassFilterTest, WrenchFirstUpdateReturnsInitialState)
{
  control_toolbox::LowPassFilter<geometry_msgs::msg::WrenchStamped> filter(1000.0, 20.5, 1.25);

  geometry_msgs::msg::WrenchStamped initial_state;
  initial_state.wrench.force.x = 1.0;
  initial_state.wrench.torque.z = 6.0;
  ASSERT_TRUE(filter.configure(initial_state));

  geometry_msgs::msg::WrenchStamped in;
  in.header.frame_id = "world";
  in.wrench.force.x = 100.0;
  in.wrench.torque.z = 100.0;

  geometry_msgs::msg::WrenchStamped out;
  ASSERT_TRUE(filter.update(in, out));

  EXPECT_NEAR(out.wrench.force.x, 1.0, 1e-12);
  EXPECT_NEAR(out.wrench.torque.z, 6.0, 1e-12);
}
