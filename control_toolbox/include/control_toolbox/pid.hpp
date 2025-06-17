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

#ifndef CONTROL_TOOLBOX__PID_HPP_
#define CONTROL_TOOLBOX__PID_HPP_

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "fmt/format.h"
#include "rclcpp/duration.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

namespace control_toolbox
{
/**
 * \brief Antiwindup strategy for PID controllers.
 *
 * This class defines various antiwindup strategies that can be used in PID controllers.
 * It allows setting the type of antiwindup strategy and validates the parameters accordingly.
 *
 * \param i_max Upper integral clamp.
 * \param i_min Lower integral clamp.
 * \param u_max Upper output clamp.
 * \param u_min Lower output clamp.
 * \param tracking_time_constant Specifies the tracking time constant for the 'back_calculation' strategy. If set
 *    to 0.0 when this strategy is selected, a recommended default value will be applied.
 * \param legacy_antiwindup Anti-windup functionality. When set to true, limits
      the integral error to prevent windup; otherwise, constrains the
      integral contribution to the control output. i_max and
      i_min are applied in both scenarios.
  * \param error_deadband Error deadband is used to stop integration when the error is within the given range.
  * \param type Specifies the antiwindup strategy type. Valid values are:
 *   - `NONE`: No antiwindup strategy applied.
 *   - `LEGACY`: Legacy antiwindup strategy, which limits the integral term to prevent windup (deprecated: This option will be removed in a future release).
 *   - `BACK_CALCULATION`: Back calculation antiwindup strategy, which uses a tracking time constant.
 *   - `CONDITIONAL_INTEGRATION`: Conditional integration antiwindup strategy, which integrates only when certain conditions are met.
 */
struct AntiWindupStrategy
{
public:
  enum Value : int8_t
  {
    UNDEFINED = -1,
    NONE,
    LEGACY,
    BACK_CALCULATION,
    CONDITIONAL_INTEGRATION
  };

  AntiWindupStrategy()
  : type(UNDEFINED),
    i_min(std::numeric_limits<double>::quiet_NaN()),
    i_max(std::numeric_limits<double>::quiet_NaN()),
    legacy_antiwindup(false),
    tracking_time_constant(0.0),
    error_deadband(std::numeric_limits<double>::epsilon())
  {
  }

  void set_type(const std::string & s)
  {
    if (s == "back_calculation")
    {
      type = BACK_CALCULATION;
    }
    else if (s == "conditional_integration")
    {
      type = CONDITIONAL_INTEGRATION;
    }
    else if (s == "legacy")
    {
      type = LEGACY;
      std::cout << "Using the legacy anti-windup technique is deprecated. This option will be "
                   "removed by the ROS 2 Kilted Kaiju release."
                << std::endl;
    }
    else if (s == "none")
    {
      type = NONE;
    }
    else
    {
      type = UNDEFINED;
      throw std::invalid_argument(
        "AntiWindupStrategy: Unknown antiwindup strategy : '" + s +
        "'. Valid strategies are: 'back_calculation', 'conditional_integration', 'legacy', "
        "'none'.");
    }
  }

  void validate() const
  {
    if (type == UNDEFINED)
    {
      throw std::invalid_argument("AntiWindupStrategy is UNDEFINED. Please set a valid type");
    }
    if (
      type == BACK_CALCULATION &&
      (tracking_time_constant < 0.0 || !std::isfinite(tracking_time_constant)))
    {
      throw std::invalid_argument(
        "AntiWindupStrategy 'back_calculation' requires a valid positive tracking time constant "
        "(tracking_time_constant)");
    }
    if (type == LEGACY && ((i_min > i_max) || !std::isfinite(i_min) || !std::isfinite(i_max)))
    {
      throw std::invalid_argument(
        fmt::format(
          "AntiWindupStrategy 'legacy' requires i_min < i_max and to be finite (i_min: {}, i_max: "
          "{})",
          i_min, i_max));
    }
    if (std::isfinite(i_min) || std::isfinite(i_max))
    {
      std::cout << "Warning: The i_min and i_max are only valid for the deprecated LEGACY "
                   "antiwindup strategy. Please use the AntiWindupStrategy::set_type() method to "
                   "set the type of antiwindup strategy you want to use."
                << std::endl;
    }
    if (
      type != NONE && type != UNDEFINED && type != LEGACY && type != BACK_CALCULATION &&
      type != CONDITIONAL_INTEGRATION)
    {
      throw std::invalid_argument("AntiWindupStrategy has an invalid type");
    }
  }

  operator std::string() const { return to_string(); }

  constexpr bool operator==(Value other) const { return type == other; }
  constexpr bool operator!=(Value other) const { return type != other; }

  std::string to_string() const
  {
    switch (type)
    {
      case BACK_CALCULATION:
        return "back_calculation";
      case CONDITIONAL_INTEGRATION:
        return "conditional_integration";
      case LEGACY:
        return "legacy";
      case NONE:
        return "none";
      case UNDEFINED:
      default:
        return "UNDEFINED";
    }
  }

  Value type = UNDEFINED;
  double i_min = std::numeric_limits<double>::quiet_NaN(); /**< Minimum allowable integral term. */
  double i_max = std::numeric_limits<double>::quiet_NaN(); /**< Maximum allowable integral term. */

  bool legacy_antiwindup = false; /**< Use legacy anti-windup strategy. */

  // tracking_time_constant Specifies the tracking time constant for the 'back_calculation'
  // strategy. If set to 0.0 a recommended default value will be applied.
  double tracking_time_constant = 0.0; /**< Tracking time constant for back_calculation strategy. */

  double error_deadband =
    std::numeric_limits<double>::epsilon(); /**< Error deadband to avoid integration. */
};

template <typename T>
inline bool is_zero(T value, T tolerance = std::numeric_limits<T>::epsilon())
{
  return std::abs(value) <= tolerance;
}

/***************************************************/
/*! \class Pid
  \brief A basic pid class.

  This class implements a generic structure that
  can be used to create a wide range of pid
  controllers. It can function independently or
  be subclassed to provide more specific controls
  based on a particular control loop.

  This class also allows for retention of integral
  term on reset.  This is useful for control loops
  that are enabled/disabled with a constant steady-state
  external disturbance.  Once the integrator cancels
  out the external disturbance, disabling/resetting/
  re-enabling closed-loop control does not require
  the integrator to wind up again.

  In particular, this class implements the standard
  pid equation:

  \f$command  = p_{term} + i_{term} + d_{term} \f$

  where: <br>
  <UL TYPE="none">
  <LI>  \f$ p_{term}  = p_{gain} * p_{error} \f$
  <LI>  \f$ i_{term}  = i_{term} + \int{i_{gain} * p_{error} * dt} \f$
  <LI>  \f$ d_{term}  = d_{gain} * d_{error} \f$
  <LI>  \f$ d_{error} = (p_{error} - p_{error last}) / dt \f$
  </UL>

  given:<br>
  <UL TYPE="none">
  <LI>  \f$ p_{error}  = p_{desired} - p_{state} \f$.
  </UL>

  \param p Proportional gain

  \param d Derivative gain

  \param i Integral gain

  \param i_clamp Minimum and maximum bounds for the integral windup, the clamp is applied to the \f$i_{term}\f$

  \param u_clamp Minimum and maximum bounds for the controller output. The clamp is applied to the \f$command\f$.

  \section Usage

  To use the Pid class, you should first call some version of init()
  (in non-realtime) and then call updatePid() at every update step.
  For example:

  \verbatim
  control_toolbox::Pid pid;
  pid.initialize(6.0, 1.0, 2.0, 0.3, -0.3);
  double position_desired = 0.5;
  ...
  rclcpp::Time last_time = get_clock()->now();
  while (true) {
  rclcpp::Time time = get_clock()->now();
  double effort = pid.compute_command(position_desired - currentPosition(), time - last_time);
  last_time = time;
  }
  \endverbatim

*/
/***************************************************/

class Pid
{
public:
  /*!
   * \brief Store gains in a struct to allow easier realtime box usage
   */
  struct Gains
  {
    /*!
   * \brief Optional constructor for passing in values without antiwindup and saturation
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   *
   */
    [[deprecated("Use constructor with AntiWindupStrategy instead.")]]
    Gains(double p, double i, double d, double i_max, double i_min)
    : p_gain_(p),
      i_gain_(i),
      d_gain_(d),
      i_max_(i_max),
      i_min_(i_min),
      u_max_(std::numeric_limits<double>::infinity()),
      u_min_(-std::numeric_limits<double>::infinity()),
      antiwindup_(false)
    {
      antiwindup_strat_.type = AntiWindupStrategy::LEGACY;
      antiwindup_strat_.i_max = i_max;
      antiwindup_strat_.i_min = i_min;
      antiwindup_strat_.legacy_antiwindup = true;
    }

    /*!
   * \brief Optional constructor for passing in values without saturation
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   *
   */
    [[deprecated("Use constructor with AntiWindupStrategy instead.")]]
    Gains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
    : p_gain_(p),
      i_gain_(i),
      d_gain_(d),
      i_max_(i_max),
      i_min_(i_min),
      u_max_(std::numeric_limits<double>::infinity()),
      u_min_(-std::numeric_limits<double>::infinity()),
      antiwindup_(antiwindup)
    {
      antiwindup_strat_.type = AntiWindupStrategy::LEGACY;
      antiwindup_strat_.i_max = i_max;
      antiwindup_strat_.i_min = i_min;
      antiwindup_strat_.legacy_antiwindup = antiwindup;
    }

    /*!
   * \brief Constructor for passing in values.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   *
   */
    Gains(
      double p, double i, double d, double u_max, double u_min,
      const AntiWindupStrategy & antiwindup_strat)
    : p_gain_(p),
      i_gain_(i),
      d_gain_(d),
      i_max_(antiwindup_strat.i_max),
      i_min_(antiwindup_strat.i_min),
      u_max_(u_max),
      u_min_(u_min),
      antiwindup_(antiwindup_strat.legacy_antiwindup),
      antiwindup_strat_(antiwindup_strat)
    {
      if (std::isnan(u_min) || std::isnan(u_max))
      {
        throw std::invalid_argument("Gains: u_min and u_max must not be NaN");
      }
      if (u_min > u_max)
      {
        std::cout << "Received invalid u_min and u_max values: " << "u_min: " << u_min
                  << ", u_max: " << u_max << ". Setting saturation to false." << std::endl;
        u_max_ = std::numeric_limits<double>::infinity();
        u_min_ = -std::numeric_limits<double>::infinity();
      }
    }

    bool validate(std::string & error_msg) const
    {
      if (i_min_ > i_max_)
      {
        error_msg = fmt::format("Gains: i_min ({}) must be less than i_max ({})", i_min_, i_max_);
        return false;
      }
      else if (u_min_ >= u_max_)
      {
        error_msg = fmt::format("Gains: u_min ({}) must be less than u_max ({})", u_min_, u_max_);
        return false;
      }
      else if (std::isnan(u_min_) || std::isnan(u_max_))
      {
        error_msg = "Gains: u_min and u_max must not be NaN";
        return false;
      }
      try
      {
        antiwindup_strat_.validate();
      }
      catch (const std::exception & e)
      {
        error_msg = e.what();
        return false;
      }
      return true;
    }

    // Default constructor
    [[deprecated(
      "Use constructor with AntiWindupStrategy only. The default constructor might be deleted in "
      "future")]] Gains()
    {
    }

    void print() const
    {
      std::cout << "Gains: p: " << p_gain_ << ", i: " << i_gain_ << ", d: " << d_gain_
                << ", i_max: " << i_max_ << ", i_min: " << i_min_ << ", u_max: " << u_max_
                << ", u_min: " << u_min_ << ", antiwindup: " << antiwindup_
                << ", antiwindup_strat: " << antiwindup_strat_.to_string() << std::endl;
    }

    double p_gain_ = 0.0; /**< Proportional gain. */
    double i_gain_ = 0.0; /**< Integral gain. */
    double d_gain_ = 0.0; /**< Derivative gain. */
    double i_max_ = 0.0;  /**< Maximum allowable integral term. */
    double i_min_ = 0.0;  /**< Minimum allowable integral term. */
    double u_max_ = std::numeric_limits<double>::infinity();  /**< Maximum allowable output. */
    double u_min_ = -std::numeric_limits<double>::infinity(); /**< Minimum allowable output. */
    bool antiwindup_ = false;                                 /**< Anti-windup. */
    AntiWindupStrategy antiwindup_strat_;                     /**< Anti-windup strategy. */
  };

  /*!
   * \brief Constructor, zeros out Pid values when created and
   *        initialize Pid-gains and integral term limits.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   *
   * \throws An std::invalid_argument exception is thrown if i_min > i_max
   */
  [[deprecated("Use constructor with AntiWindupStrategy only.")]]
  Pid(
    double p = 0.0, double i = 0.0, double d = 0.0, double i_max = 0.0, double i_min = -0.0,
    bool antiwindup = false);

  /*!
   * \brief Constructor, initialize Pid-gains and term limits.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   *
   * \throws An std::invalid_argument exception is thrown if u_min > u_max
   */
  Pid(
    double p, double i, double d, double u_max, double u_min,
    const AntiWindupStrategy & antiwindup_strat);

  /*!
   * \brief Copy constructor required for preventing mutexes from being copied
   * \param source - Pid to copy
   */
  Pid(const Pid & source);

  /*!
   * \brief Destructor of Pid class.
   */
  ~Pid();

  /*!
   * \brief Zeros out Pid values and initialize Pid-gains and term limits
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   * \return True if all parameters are successfully set, False otherwise.
   * \note New gains are not applied if i_min_ > i_max_
   */
  [[deprecated("Use initialize with AntiWindupStrategy instead.")]]
  bool initialize(
    double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

  /*!
   * \brief Initialize Pid-gains and term limits.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if i_min_ > i_max_ or u_min > u_max
   */
  bool initialize(
    double p, double i, double d, double u_max, double u_min,
    const AntiWindupStrategy & antiwindup_strat);

  /*!
   * \brief Reset the state of this PID controller
   * @note The integral term is not retained and it is reset to zero
   */
  void reset();

  /*!
   * \brief Reset the state of this PID controller
   *
   * \param save_i_term boolean indicating if integral term is retained on reset()
   */
  void reset(bool save_i_term);

  /*!
   * \brief Clear the saved integrator output of this controller
   */
  void clear_saved_iterm();

  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   *
   * \note This method is not RT safe
   */
  void get_gains(double & p, double & i, double & d, double & i_max, double & i_min);

  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   *
   * \note This method is not RT safe
   */
  [[deprecated("Use get_gains overload with AntiWindupStrategy argument.")]]
  void get_gains(
    double & p, double & i, double & d, double & i_max, double & i_min, bool & antiwindup);

  /*!
   * \brief Get PID gains for the controller (preferred).
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   *
   * \note This method is not RT safe
   */
  void get_gains(
    double & p, double & i, double & d, double & u_max, double & u_min,
    AntiWindupStrategy & antiwindup_strat);

  /*!
   * \brief Get PID gains for the controller.
   * \return gains A struct of the PID gain values
   *
   * \note This method is not RT safe
   */
  Gains get_gains();

  /*!
   * \brief Get PID gains for the controller.
   * \return gains A struct of the PID gain values
   *
   * \note This method can be called from the RT loop
   */
  Gains get_gains_rt() { return gains_; }

  /*!
   * \brief Set PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max Upper integral clamp.
   * \param i_min Lower integral clamp.
   * \param antiwindup Anti-windup functionality. When set to true, limits
        the integral error to prevent windup; otherwise, constrains the
        integral contribution to the control output. i_max and
        i_min are applied in both scenarios.
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if i_min > i_max
   * \note This method is not RT safe
   */
  [[deprecated("Use set_gains with AntiWindupStrategy instead.")]]
  bool set_gains(double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

  /*!
   * \brief Set PID gains for the controller.
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param u_max Upper output clamp.
   * \param u_min Lower output clamp.
   * \param antiwindup_strat Specifies the anti-windup strategy. Options: 'back_calculation',
        'conditional_integration', or 'none'. Note that the 'back_calculation' strategy use the
        tracking_time_constant parameter to tune the anti-windup behavior.
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if i_min_ > i_max_ or u_min > u_max
   * \note This method is not RT safe
   */
  bool set_gains(
    double p, double i, double d, double u_max, double u_min,
    const AntiWindupStrategy & antiwindup_strat);

  /*!
   * \brief Set PID gains for the controller.
   * \param gains A struct of the PID gain values
   * \return True if all parameters are successfully set, False otherwise.
   *
   * \note New gains are not applied if gains.i_min_ > gains.i_max_
   * \note This method is not RT safe
   */
  bool set_gains(const Gains & gains);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt_s.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt_s Change in time since last call in seconds
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, const double & dt_s);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt_ns.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt_ns Change in time since last call, measured in nanoseconds.
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, const rcl_duration_value_t & dt_ns);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time since last call
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, const rclcpp::Duration & dt);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt_ns.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt_ns Change in time since last call
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, const std::chrono::nanoseconds & dt_ns);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/dt_s since last call
   * \param dt_s Change in time since last call in seconds
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, double error_dot, const double & dt_s);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/dt_ns since last call
   * \param dt_ns Change in time since last call, measured in nanoseconds.
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(
    double error, double error_dot, const rcl_duration_value_t & dt_ns);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, double error_dot, const rclcpp::Duration & dt);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/(dt_ns/1e9) since last call
   * \param dt_ns Change in time since last call, measured in nanoseconds.
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(
    double error, double error_dot, const std::chrono::nanoseconds & dt_ns);

  /*!
   * \brief Set current command for this PID controller
   */
  void set_current_cmd(double cmd);

  /*!
   * \brief Return current command for this PID controller
   */
  double get_current_cmd();

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe  The proportional error.
   * \param ie  The weighted integral error.
   * \param de  The derivative error.
   */
  void get_current_pid_errors(double & pe, double & ie, double & de);

  /*!
   * @brief Custom assignment operator
   *        Does not initialize dynamic reconfigure for PID gains
   */
  Pid & operator=(const Pid & source)
  {
    if (this == &source)
    {
      return *this;
    }

    // Copy the realtime box to then new PID class
    gains_box_ = source.gains_box_;

    // Reset the state of this PID controller
    reset();

    return *this;
  }

protected:
  // local copy of the gains for the RT loop
  Gains gains_{
    0.0,
    0.0,
    0.0,
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    AntiWindupStrategy()};
  // Store the PID gains in a realtime box to allow dynamic reconfigure to update it without
  // blocking the realtime update loop
  realtime_tools::RealtimeThreadSafeBox<Gains> gains_box_{gains_};

  double p_error_last_ = 0; /** Save state for derivative state calculation. */
  double p_error_ = 0;      /** Error. */
  double d_error_ = 0;      /** Derivative of error. */
  double i_term_ = 0;       /** Integrator state. */
  double cmd_ = 0;          /** Command to send. */
  double cmd_unsat_ = 0;    /** command without saturation. */
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__PID_HPP_
