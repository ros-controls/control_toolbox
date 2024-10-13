// Copyright 2024 AIT - Austrian Institute of Technology GmbH
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

#include "control_toolbox/speed_limiter.hpp"


TEST(SpeedLimiterTest, testLinearVelocityLimits)
{
  control_toolbox::SpeedLimiter limiter(true, true, true, -0.5, 1.0, -0.5, 1.0, -0.5, 5.0);

  {
    double v = 10.0;
    double limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now 1.0 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, 1.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.1);
    v = -10.0;
    limiting_factor = limiter.limit_velocity(v);
    // check if the robot speed is now -0.5 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
  }

  {
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is now limiting, not velocity
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is now limiting, not velocity
    // check if the robot speed is now -0.25 m.s-1, which is 0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }
}

TEST(SpeedLimiterTest, testLinearAccelerationLimits)
{
  control_toolbox::SpeedLimiter limiter(true, true, true, -0.5, 1.0, -0.5, 1.0, -0.5, 5.0);

  {
    double v = 10.0;
    double limiting_factor = limiter.limit_acceleration(v, 0.0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);

    v = -10.0;
    limiting_factor = limiter.limit_acceleration(v, 0.0, 0.5);
    // check if the robot speed is now -0.25 m.s-1, which is -0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }

  {
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);

    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -0.25 m.s-1, which is -0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }
}

TEST(SpeedLimiterTest, testLinearJerkLimits)
{
  control_toolbox::SpeedLimiter limiter(true, true, true, -0.5, 1.0, -0.5, 1.0, -1.0, 1.0);

  {
    double v = 10.0;
    double limiting_factor = limiter.limit_jerk(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.5m.s-1 = 1.0m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
    v = -10.0;
    limiting_factor = limiter.limit_jerk(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -0.5m.s-1 = -1.0m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
  }
  {
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.5m.s-1 = 1.0m.s-3 * 2 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // acceleration is limiting, not jerk
    // check if the robot speed is now -0.25 m.s-1, which is -0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }
}
