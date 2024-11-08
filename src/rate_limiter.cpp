// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Enrique Fernández
 */

#include <algorithm>
#include <stdexcept>

#include "control_toolbox/rate_limiter.hpp"

namespace control_toolbox
{
RateLimiter::RateLimiter(
  double min_value, double max_value,
  double min_first_derivative, double max_first_derivative,
  double min_second_derivative, double max_second_derivative)
: has_value_limits_(true),
  has_first_derivative_limits_(true),
  has_second_derivative_limits_(true),
  min_value_(min_value),
  max_value_(max_value),
  min_first_derivative_(min_first_derivative),
  max_first_derivative_(max_first_derivative),
  min_second_derivative_(min_second_derivative),
  max_second_derivative_(max_second_derivative)
{
// Check if limits are valid, max must be specified, min defaults to -max if unspecified
  if (std::isnan(max_value_))
  {
    has_value_limits_ = false;
  }
  if (std::isnan(min_value_))
  {
    min_value_ = -max_value_;
  }
  if (has_value_limits_ && min_value_ > max_value_)
  {
    throw std::invalid_argument("Invalid value limits");
  }
  if (std::isnan(max_first_derivative_))
  {
    has_first_derivative_limits_ = false;
  }
  if (std::isnan(min_first_derivative_))
  {
    min_first_derivative_ = -max_first_derivative_;
  }
  if (has_first_derivative_limits_ && min_first_derivative_ > max_first_derivative_)
  {
    throw std::invalid_argument("Invalid value limits");
  }
  if (std::isnan(max_second_derivative_))
  {
    has_second_derivative_limits_ = false;
  }
  if (std::isnan(min_second_derivative_))
  {
    min_second_derivative_ = -max_second_derivative_;
  }
  if (has_second_derivative_limits_ && min_second_derivative_ > max_second_derivative_)
  {
    throw std::invalid_argument("Invalid value limits");
  }
}

double RateLimiter::limit(double & v, double v0, double v1, double dt)
{
  const double tmp = v;

  limit_second_derivative(v, v0, v1, dt);
  limit_first_derivative(v, v0, dt);
  limit_value(v);

  return tmp != 0.0 ? v / tmp : 1.0;
}

double RateLimiter::limit_value(double & v)
{
  const double tmp = v;

  if (has_value_limits_)
  {
    v = std::clamp(v, min_value_, max_value_);
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double RateLimiter::limit_first_derivative(double & v, double v0, double dt)
{
  const double tmp = v;

  if (has_first_derivative_limits_)
  {
    const double dv_min = min_first_derivative_ * dt;
    const double dv_max = max_first_derivative_ * dt;

    const double dv = std::clamp(v - v0, dv_min, dv_max);

    v = v0 + dv;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

double RateLimiter::limit_second_derivative(double & v, double v0, double v1, double dt)
{
  const double tmp = v;

  if (has_second_derivative_limits_)
  {
    const double dv = v - v0;
    const double dv0 = v0 - v1;

    const double dt2 = 2. * dt * dt;

    const double da_min = min_second_derivative_ * dt2;
    const double da_max = max_second_derivative_ * dt2;

    const double da = std::clamp(dv - dv0, da_min, da_max);

    v = v0 + dv0 + da;
  }

  return tmp != 0.0 ? v / tmp : 1.0;
}

}  // namespace control_toolbox
