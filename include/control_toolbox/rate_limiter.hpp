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

#ifndef CONTROL_TOOLBOX__RATE_LIMITER_HPP_
#define CONTROL_TOOLBOX__RATE_LIMITER_HPP_

#include <cmath>
#include <limits>

#include <algorithm>
#include <stdexcept>

namespace control_toolbox
{
template <typename T>
class RateLimiter
{
public:
  /**
   * \brief Constructor
   *
   * \param [in] min_value Minimum value, e.g. [m/s], usually <= 0
   * \param [in] max_value Maximum value, e.g. [m/s], usually >= 0
   * \param [in] min_first_derivative_neg Minimum first_derivative, negative value, e.g. [m/s^2], usually <= 0
   * \param [in] max_first_derivative_pos Maximum first_derivative, positive value, e.g. [m/s^2], usually >= 0
   * \param [in] min_first_derivative_pos Asymmetric Minimum first_derivative, positive value, e.g. [m/s^2], usually <= 0
   * \param [in] max_first_derivative_neg Asymmetric Maximum first_derivative, negative value, e.g. [m/s^2], usually >= 0
   * \param [in] min_second_derivative Minimum second_derivative, e.g. [m/s^3], usually <= 0
   * \param [in] max_second_derivative Maximum second_derivative, e.g. [m/s^3], usually >= 0
   *
   * \note
   * If max_* values are NAN, the respective limit is deactivated
   * If min_* values are NAN (unspecified), defaults to -max
   * If min_first_derivative_pos/max_first_derivative_neg values are NAN, symmetric limits are used
   *
   * Disclaimer about the jerk limits:
   *    The jerk limit is only applied when accelerating or reverse_accelerating (i.e., "sign(jerk * accel) > 0").
   *    This condition prevents oscillating closed-loop behavior, see discussion details in
   *    https://github.com/ros-controls/control_toolbox/issues/240.
   *    if you use this feature, you should perform a test to check that the behavior is really as you expect.
   *
   */
  RateLimiter(
    T min_value = std::numeric_limits<T>::quiet_NaN(),
    T max_value = std::numeric_limits<T>::quiet_NaN(),
    T min_first_derivative_neg = std::numeric_limits<T>::quiet_NaN(),
    T max_first_derivative_pos = std::numeric_limits<T>::quiet_NaN(),
    T min_first_derivative_pos = std::numeric_limits<T>::quiet_NaN(),
    T max_first_derivative_neg = std::numeric_limits<T>::quiet_NaN(),
    T min_second_derivative = std::numeric_limits<T>::quiet_NaN(),
    T max_second_derivative = std::numeric_limits<T>::quiet_NaN());

  /**
   * \brief Limit the value, first_derivative, and second_derivative
   * \param [in, out] v  value, e.g. [m/s]
   * \param [in]      v0 Previous value to v , e.g. [m/s]
   * \param [in]      v1 Previous value to v0, e.g. [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  T limit(T & v, T v0, T v1, T dt);

  /**
   * \brief Limit the value
   * \param [in, out] v value, e.g. [m/s]
   * \return Limiting factor (1.0 if none)
   */
  T limit_value(T & v);

  /**
   * \brief Limit the first_derivative
   * \param [in, out] v  value, e.g. [m/s]
   * \param [in]      v0 Previous value, e.g. [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  T limit_first_derivative(T & v, T v0, T dt);

  /**
   * \brief Limit the second_derivative
   * \param [in, out] v  value, e.g. [m/s]
   * \param [in]      v0 Previous value to v , e.g. [m/s]
   * \param [in]      v1 Previous value to v0, e.g. [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   * \see http://en.wikipedia.org/wiki/jerk_%28physics%29#Motion_control
   * \note
   * The jerk limit is only applied when accelerating or reverse_accelerating (i.e., "sign(jerk * accel) > 0").
   */
  T limit_second_derivative(T & v, T v0, T v1, T dt);


  /**
   * \brief Set the parameters
   *
   * \param [in] min_value Minimum value, e.g. [m/s], usually <= 0
   * \param [in] max_value Maximum value, e.g. [m/s], usually >= 0
   * \param [in] min_first_derivative_neg Minimum first_derivative, negative value, e.g. [m/s^2], usually <= 0
   * \param [in] max_first_derivative_pos Maximum first_derivative, positive value, e.g. [m/s^2], usually >= 0
   * \param [in] min_first_derivative_pos Asymmetric Minimum first_derivative, positive value, e.g. [m/s^2], usually <= 0
   * \param [in] max_first_derivative_neg Asymmetric Maximum first_derivative, negative value, e.g. [m/s^2], usually >= 0
   * \param [in] min_second_derivative Minimum second_derivative, e.g. [m/s^3], usually <= 0
   * \param [in] max_second_derivative Maximum second_derivative, e.g. [m/s^3], usually >= 0
   *
   * \note
   * If max_* values are NAN, the respective limit is deactivated
   * If min_* values are NAN  (unspecified), defaults to -max
   * If min_first_derivative_pos/max_first_derivative_neg values are NAN, symmetric limits are used
   */
  void set_params(
    T min_value = std::numeric_limits<T>::quiet_NaN(),
    T max_value = std::numeric_limits<T>::quiet_NaN(),
    T min_first_derivative_neg = std::numeric_limits<T>::quiet_NaN(),
    T max_first_derivative_pos = std::numeric_limits<T>::quiet_NaN(),
    T min_first_derivative_pos = std::numeric_limits<T>::quiet_NaN(),
    T max_first_derivative_neg = std::numeric_limits<T>::quiet_NaN(),
    T min_second_derivative = std::numeric_limits<T>::quiet_NaN(),
    T max_second_derivative = std::numeric_limits<T>::quiet_NaN());

private:
  // Enable/Disable value/first_derivative/second_derivative limits:
  bool has_value_limits_ = true;
  bool has_first_derivative_limits_ = true;
  bool has_second_derivative_limits_ = true;

  // value limits:
  T min_value_ = std::numeric_limits<T>::quiet_NaN();
  T max_value_ = std::numeric_limits<T>::quiet_NaN();

  // first_derivative limits:
  T min_first_derivative_neg_ = std::numeric_limits<T>::quiet_NaN();
  T max_first_derivative_pos_ = std::numeric_limits<T>::quiet_NaN();
  T min_first_derivative_pos_ = std::numeric_limits<T>::quiet_NaN();
  T max_first_derivative_neg_ = std::numeric_limits<T>::quiet_NaN();

  // second_derivative limits:
  T min_second_derivative_ = std::numeric_limits<T>::quiet_NaN();
  T max_second_derivative_ = std::numeric_limits<T>::quiet_NaN();
};

template <typename T>
RateLimiter<T>::RateLimiter(
  T min_value, T max_value,
  T min_first_derivative_neg, T max_first_derivative_pos,
  T min_first_derivative_pos, T max_first_derivative_neg,
  T min_second_derivative, T max_second_derivative)
{
  set_params(
    min_value, max_value,
    min_first_derivative_neg, max_first_derivative_pos,
    min_first_derivative_pos, max_first_derivative_neg,
    min_second_derivative, max_second_derivative
  );
};

// Check if limits are valid, max must be specified, min defaults to -max if unspecified
template <typename T>
void RateLimiter<T>::set_params(
  T min_value, T max_value,
  T min_first_derivative_neg, T max_first_derivative_pos,
  T min_first_derivative_pos, T max_first_derivative_neg,
  T min_second_derivative, T max_second_derivative)
  {
  min_value_ = min_value;
  max_value_ = max_value;
  min_first_derivative_neg_ = min_first_derivative_neg;
  max_first_derivative_pos_ = max_first_derivative_pos;
  min_first_derivative_pos_ = min_first_derivative_pos;
  max_first_derivative_neg_ = max_first_derivative_neg;
  min_second_derivative_ = min_second_derivative;
  max_second_derivative_ = max_second_derivative;

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

  if (std::isnan(max_first_derivative_pos_))
  {
    has_first_derivative_limits_ = false;
  }
  if (std::isnan(min_first_derivative_neg_))
  {
    min_first_derivative_neg_ = -max_first_derivative_pos_;
  }
  if (has_first_derivative_limits_ && min_first_derivative_neg_ > max_first_derivative_pos_)
  {
    throw std::invalid_argument("Invalid first derivative limits");
  }
  if (has_first_derivative_limits_)
  {
    if (std::isnan(max_first_derivative_neg_))
    {
      max_first_derivative_neg_ = max_first_derivative_pos_;
    }
    if (std::isnan(min_first_derivative_pos_))
    {
      min_first_derivative_pos_ = min_first_derivative_neg_;
    }
    if (has_first_derivative_limits_ && min_first_derivative_pos_ > max_first_derivative_neg_)
    {
      throw std::invalid_argument("Invalid first derivative limits");
    }
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
    throw std::invalid_argument("Invalid second derivative limits");
  }
}

template <typename T>
T RateLimiter<T>::limit(T & v, T v0, T v1, T dt)
{
  const T tmp = v;

  limit_second_derivative(v, v0, v1, dt);
  limit_first_derivative(v, v0, dt);
  limit_value(v);

  return tmp != static_cast<T>(0.0) ? v / tmp : static_cast<T>(1.0);
}

template <typename T>
T RateLimiter<T>::limit_value(T & v)
{
  const T tmp = v;

  if (has_value_limits_)
  {
    v = std::clamp(v, min_value_, max_value_);
  }

  return tmp != static_cast<T>(0.0) ? v / tmp : static_cast<T>(1.0);
}

template <typename T>
T RateLimiter<T>::limit_first_derivative(T & v, T v0, T dt)
{
  const T tmp = v;

  if (has_first_derivative_limits_)
  {
    T dv_max, dv_min = 0;
    if (v0 > static_cast<T>(0.0))
    {
      dv_max = max_first_derivative_pos_ * dt;
      dv_min = min_first_derivative_pos_ * dt;
    }
    else if (v0 < static_cast<T>(0.0))
    {
      dv_min = min_first_derivative_neg_ * dt;
      dv_max = max_first_derivative_neg_ * dt;
    }
    else
    {
      dv_min = min_first_derivative_neg_ * dt;
      dv_max = max_first_derivative_pos_ * dt;
    }
    const T dv = std::clamp(v - v0, dv_min, dv_max);

    v = v0 + dv;
  }

  return tmp != static_cast<T>(0.0) ? v / tmp : static_cast<T>(1.0);
}

template <typename T>
T RateLimiter<T>::limit_second_derivative(T & v, T v0, T v1, T dt)
{
  const T tmp = v;

  if (has_second_derivative_limits_)
  {
    const T dv = v - v0;
    const T dv0 = v0 - v1;

    // Only limit jerk when accelerating or reverse_accelerating
    // Note: this prevents oscillating closed-loop behavior, see discussion
    // details in https://github.com/ros-controls/control_toolbox/issues/240.
    if ((dv - dv0) * (v - v0) > 0)
    {
      const T dt2 = dt * dt;

      const T da_min = min_second_derivative_ * dt2;
      const T da_max = max_second_derivative_ * dt2;

      const T da = std::clamp(dv - dv0, da_min, da_max);

      v = v0 + dv0 + da;
    }
  }

  return tmp != static_cast<T>(0.0) ? v / tmp : static_cast<T>(1.0);
}

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__RATE_LIMITER_HPP_
