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
    double min_value = NAN, double max_value = NAN,
    double min_first_derivative_neg = NAN, double max_first_derivative_pos = NAN,
    double min_first_derivative_pos = NAN, double max_first_derivative_neg = NAN,
    double min_second_derivative = NAN, double max_second_derivative = NAN);

  /**
   * \brief Limit the value and first_derivative
   * \param [in, out] v  value, e.g. [m/s]
   * \param [in]      v0 Previous value to v , e.g. [m/s]
   * \param [in]      v1 Previous value to v0, e.g. [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  /**
   * \brief Limit the value
   * \param [in, out] v value, e.g. [m/s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_value(double & v);

  /**
   * \brief Limit the first_derivative
   * \param [in, out] v  value, e.g. [m/s]
   * \param [in]      v0 Previous value, e.g. [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit_first_derivative(double & v, double v0, double dt);

  /**
   * \brief Limit the second_derivative
   * \param [in, out] v  value, e.g. [m/s]
   * \param [in]      v0 Previous value to v , e.g. [m/s]
   * \param [in]      v1 Previous value to v0, e.g. [m/s]
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   * \see http://en.wikipedia.org/wiki/jerk_%28physics%29#Motion_control
   */
  double limit_second_derivative(double & v, double v0, double v1, double dt);

private:
  // Enable/Disable value/first_derivative/second_derivative limits:
  bool has_value_limits_;
  bool has_first_derivative_limits_;
  bool has_second_derivative_limits_;

  // value limits:
  double min_value_;
  double max_value_;

  // first_derivative limits:
  double min_first_derivative_neg_;
  double max_first_derivative_pos_;
  double min_first_derivative_pos_;
  double max_first_derivative_neg_;

  // second_derivative limits:
  double min_second_derivative_;
  double max_second_derivative_;
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__RATE_LIMITER_HPP_
