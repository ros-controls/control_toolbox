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
   * If min_* values are NAN, defaults to -max if unspecified
   * If min_first_derivative_pos/max_first_derivative_neg values are NAN, symmetric limits are used
   */
  RateLimiter(
    T min_value = NAN, T max_value = NAN,
    T min_first_derivative_neg = NAN, T max_first_derivative_pos = NAN,
    T min_first_derivative_pos = NAN, T max_first_derivative_neg = NAN,
    T min_second_derivative = NAN, T max_second_derivative = NAN);

  /**
   * \brief Limit the value and first_derivative
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
   * If min_* values are NAN, defaults to -max if unspecified
   * If min_first_derivative_pos/max_first_derivative_neg values are NAN, symmetric limits are used
   */
  void set_params(
    T min_value = NAN, T max_value = NAN,
    T min_first_derivative_neg = NAN, T max_first_derivative_pos = NAN,
    T min_first_derivative_pos = NAN, T max_first_derivative_neg = NAN,
    T min_second_derivative = NAN, T max_second_derivative = NAN);

private:
  // Enable/Disable value/first_derivative/second_derivative limits:
  bool has_value_limits_ = true;
  bool has_first_derivative_limits_ = true;
  bool has_second_derivative_limits_ = true;

  // value limits:
  T min_value_ = NAN;
  T max_value_ = NAN;

  // first_derivative limits:
  T min_first_derivative_neg_ = NAN;
  T max_first_derivative_pos_ = NAN;
  T min_first_derivative_pos_ = NAN;
  T max_first_derivative_neg_ = NAN;

  // second_derivative limits:
  T min_second_derivative_ = NAN;
  T max_second_derivative_ = NAN;
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__RATE_LIMITER_HPP_
