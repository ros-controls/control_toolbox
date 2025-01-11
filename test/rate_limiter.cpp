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
#include <limits>

#include "control_toolbox/rate_limiter.hpp"


TEST(RateLimiterTest, testWrongParams)
{
  // different value limits
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    std::numeric_limits<double>::quiet_NaN(), 1.0,
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_THROW(control_toolbox::RateLimiter limiter(
    1.0, -1.0,
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0),
    std::invalid_argument);

  // different limits for first derivative
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    -1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    1.0, -1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0),
    std::invalid_argument);
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    -1.0, 1.0,
    -10.0, std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), 10.0,
    -1.0, 1.0));
  EXPECT_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    1.0, -1.0,
    10.0, -10.0,
    -1.0, 1.0),
    std::invalid_argument);

  // different limits for second derivative
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0, -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0, -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));
  EXPECT_NO_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0, -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), 1.0));
  EXPECT_THROW(control_toolbox::RateLimiter limiter(
    -1.0, 1.0,
    -1.0, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    1.0, -1.0),
    std::invalid_argument);
}

TEST(RateLimiterTest, testNoLimits)
{
    control_toolbox::RateLimiter<double> limiter;
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the first_derivative is not limited
    EXPECT_DOUBLE_EQ(v, 10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the value is not limited
    EXPECT_DOUBLE_EQ(v, -10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
}

TEST(RateLimiterTest, testValueLimits)
{
  control_toolbox::RateLimiter limiter(
    -0.5, 1.0,
    -0.5, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -2.0, 5.0);

  {
    double v = 10.0;
    double limiting_factor = limiter.limit_value(v);
    // check if the robot speed is now 1.0 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, 1.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.1);
    v = -10.0;
    limiting_factor = limiter.limit_value(v);
    // check if the robot speed is now -0.5 m.s-1, the limit
    EXPECT_DOUBLE_EQ(v, -0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
  }

  {
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // first_derivative is now limiting, not value
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // first_derivative is now limiting, not value
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }
}

TEST(RateLimiterTest, testValueNoLimits)
{
  {
    control_toolbox::RateLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      -0.5, 1.0,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      -0.5, 5.0
    );
    double v = 10.0;
    double limiting_factor = limiter.limit_value(v);
    // check if the value is not limited
    EXPECT_DOUBLE_EQ(v, 10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
    v = -10.0;
    limiting_factor = limiter.limit_value(v);
    // check if the value is not limited
    EXPECT_DOUBLE_EQ(v, -10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
  }

  {
    control_toolbox::RateLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      -0.5, 1.0,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      -2.0, 2.0
    );
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // first_derivative is now limiting, not value
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // first_derivative is now limiting, not value
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }

  {
    control_toolbox::RateLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      -0.5, 5.0
    );
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // second_derivative is now limiting, not value
    // check if the robot speed is now 1.25 m.s-1, which is 5.0 m.s-3 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 1.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.25/10.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // second_derivative is now limiting, not value
    // check if the robot speed is now -0.25 m.s-1, which is -0.5m.s-3 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.125);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.125/10.0);
  }

  {
    control_toolbox::RateLimiter limiter(
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the value is not limited
    EXPECT_DOUBLE_EQ(v, 10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the value is not limited
    EXPECT_DOUBLE_EQ(v, -10.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 1.0);
  }
}

TEST(RateLimiterTest, testFirstDerivativeLimits)
{
  control_toolbox::RateLimiter limiter( -0.5, 1.0,
    -0.5, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -2.0, 5.0
  );

  {
    double v = 10.0;
    double limiting_factor = limiter.limit_first_derivative(v, 0.0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.5/10.0);

    v = -10.0;
    limiting_factor = limiter.limit_first_derivative(v, 0.0, 0.5);
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

TEST(RateLimiterTest, testFirstDerivativeLimitsAsymmetric)
{
  control_toolbox::RateLimiter limiter( -0.5, 1.0,
    -0.5, 1.0,
    -5.0, 10.0,
    -0.5, 5.0
  );

  {
    double v = 10.0;
    double v0 = 5.0;
    double limiting_factor = limiter.limit_first_derivative(v, v0, 0.5);
    // check if the robot speed is now 5.5 m.s-1, which is 5.0 + 1.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 5.5);
    EXPECT_DOUBLE_EQ(limiting_factor, 5.5/10.0);

    v = -10.0;
    limiting_factor = limiter.limit_first_derivative(v, v0, 0.5);
    // check if the robot speed is now 2.5 m.s-1, which is 5.0 - 5.0m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 2.5);
    EXPECT_DOUBLE_EQ(limiting_factor, -2.5/10.0);
  }

  {
    double v = 10.0;
    double v0 = -5.0;
    double limiting_factor = limiter.limit_first_derivative(v, v0, 0.5);
    // check if the robot speed is now 0.5 m.s-1, which is -5 + 10.m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.0);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.0);  // div by 0

    v = -10.0;
    limiting_factor = limiter.limit_first_derivative(v, v0, 0.5);
    // check if the robot speed is now -0.25 m.s-1, which is -5 - 0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -5.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 5.25/10.0);
  }
}

TEST(RateLimiterTest, testSecondDerivativeLimits)
{
  control_toolbox::RateLimiter limiter(
    -0.5, 1.0,
    -0.5, 1.0,
    std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
    -1.0, 1.0
    );

  {
    double v = 10.0;
    double limiting_factor = limiter.limit_second_derivative(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.25m.s-1 = 1.0m.s-3 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
    v = -10.0;
    limiting_factor = limiter.limit_second_derivative(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now -0.25m.s-1 = -1.0m.s-3 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }
  {
    double v = 10.0;
    double limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // check if the robot speed is now 0.25m.s-1 = 1.0m.s-3 * 0.5s * 0.5s
    EXPECT_DOUBLE_EQ(v, 0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
    v = -10.0;
    limiting_factor = limiter.limit(v, 0.0, 0.0, 0.5);
    // first_derivative is limiting, not second_derivative
    // check if the robot speed is now -0.25 m.s-1, which is -0.5m.s-2 * 0.5s
    EXPECT_DOUBLE_EQ(v, -0.25);
    EXPECT_DOUBLE_EQ(limiting_factor, 0.25/10.0);
  }
}
