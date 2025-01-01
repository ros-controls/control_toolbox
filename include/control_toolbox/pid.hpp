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

#include "rclcpp/duration.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "control_toolbox/visibility_control.hpp"

namespace control_toolbox
{
/***************************************************/
/*! \class Pid
  \brief A basic pid class.

  This class implements a generic structure that
  can be used to create a wide range of pid
  controllers. It can function independently or
  be subclassed to provide more specific controls
  based on a particular control loop.

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

  \param i_clamp Min/max bounds for the integral windup, the clamp is applied to the \f$i_{term}\f$

  \section Usage

  To use the Pid class, you should first call some version of init()
  (in non-realtime) and then call updatePid() at every update step.
  For example:

  \verbatim
  control_toolbox::Pid pid;
  pid.initialize(6.0, 1.0, 2.0, 0.3, -0.3);
  double position_desi = 0.5;
  ...
  rclcpp::Time last_time = get_clock()->now();
  while (true) {
  rclcpp::Time time = get_clock()->now();
  double effort = pid.compute_command(position_desi - currentPosition(), time - last_time);
  last_time = time;
  }
  \endverbatim

*/
/***************************************************/

class CONTROL_TOOLBOX_PUBLIC Pid
{
public:
  /*!
   * \brief Store gains in a struct to allow easier realtime buffer usage
   */
  struct Gains
  {
  /*!
   * \brief Optional constructor for passing in values without antiwindup
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   *
   */
    Gains(double p, double i, double d, double i_max, double i_min)
    : p_gain_(p), i_gain_(i), d_gain_(d), i_max_(i_max), i_min_(i_min), antiwindup_(true)
    {
    }

  /*!
   * \brief Optional constructor for passing in values
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   *
   */
    Gains(
      double p = 0.0, double i = 0.0, double d = 0.0, double i_max = 0.0, double i_min = 0.0,
      bool antiwindup = false)
    : p_gain_(p), i_gain_(i), d_gain_(d), i_max_(i_max), i_min_(i_min), antiwindup_(antiwindup)
    {
    }

    double p_gain_;   /**< Proportional gain. */
    double i_gain_;   /**< Integral gain. */
    double d_gain_;   /**< Derivative gain. */
    double i_max_;    /**< Maximum allowable integral term. */
    double i_min_;    /**< Minimum allowable integral term. */
    bool antiwindup_; /**< Antiwindup. */
  };

  /*!
   * \brief Constructor, zeros out Pid values when created and
   *        initialize Pid-gains and integral term limits.
   *        Does not initialize dynamic reconfigure for PID gains
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   *
   * \throws An std::invalid_argument exception is thrown if i_min > i_max
   */
  Pid(
    double p = 0.0, double i = 0.0, double d = 0.0, double i_max = 0.0, double i_min = -0.0,
    bool antiwindup = false);

  /**
   * \brief Copy constructor required for preventing mutexes from being copied
   * \param source - Pid to copy
   */
  Pid(const Pid & source);

  /*!
   * \brief Destructor of Pid class.
   */
  ~Pid();

  /*!
   * \brief Zeros out Pid values and initialize Pid-gains and integral term limits
   *        Does not initialize the node's parameter interface for PID gains
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   *
   * \note New gains are not applied if i_min_ > i_max_
   */
  void initialize(
    double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

  /*!
   * \brief Zeros out Pid values and initialize Pid-gains and integral term limits
   *        Does not initialize the node's parameter interface for PID gains
   *
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   *
   * \note New gains are not applied if i_min_ > i_max_
   */
  [[deprecated("Use initialize() instead")]] void initPid(
    double p, double i, double d, double i_max, double i_min, bool antiwindup = false) {
    initialize(p, i, d, i_max, i_min, antiwindup);
  }

  /*!
   * \brief Reset the state of this PID controller
   */
  void reset();

  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   */
  void get_gains(double & p, double & i, double & d, double & i_max, double & i_min);

  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   */
  [[deprecated("Use get_gains() instead")]] void getGains(
    double & p, double & i, double & d, double & i_max, double & i_min) {
    get_gains(p, i, d, i_max, i_min);
  }

  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   */
  void get_gains(
    double & p, double & i, double & d, double & i_max, double & i_min, bool & antiwindup);

  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   */
  [[deprecated("Use get_gains() instead")]] void getGains(
    double & p, double & i, double & d, double & i_max, double & i_min, bool & antiwindup) {
    get_gains(p, i, d, i_max, i_min, antiwindup);
    }

  /*!
   * \brief Get PID gains for the controller.
   * \return gains A struct of the PID gain values
   */
  Gains get_gains();

  /*!
   * \brief Get PID gains for the controller.
   * \return gains A struct of the PID gain values
   */
  [[deprecated("Use get_gains() instead")]] Gains getGains() {
    return get_gains();
  }

  /*!
   * \brief Set PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   *
   * \note New gains are not applied if i_min > i_max
   */
  void set_gains(double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

  /*!
   * \brief Set PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   *
   * \note New gains are not applied if i_min > i_max
   */
  [[deprecated("Use set_gains() instead")]] void setGains(
    double p, double i, double d, double i_max, double i_min, bool antiwindup = false) {
    set_gains(p, i, d, i_max, i_min, antiwindup);
  }

  /*!
   * \brief Set PID gains for the controller.
   * \param gains A struct of the PID gain values
   *
   * \note New gains are not applied if gains.i_min_ > gains.i_max_
   */
  void set_gains(const Gains & gains);

  /*!
   * \brief Set PID gains for the controller.
   * \param gains A struct of the PID gain values
   *
   * \note New gains are not applied if gains.i_min_ > gains.i_max_
   */
  [[deprecated("Use set_gains() instead")]] void setGains(const Gains & gains) {
    set_gains(gains);
  }

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time since last call in seconds
   *
   * \returns PID command
   */
  [[nodiscard]] double compute_command(double error, const double & dt_s);

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time since last call in nanoseconds
   *
   * \returns PID command
   */
  [[deprecated("Use compute_command() instead")]] [[nodiscard]] double computeCommand(
    double error, uint64_t dt_ns) {
    return compute_command(error, static_cast<double>(dt_ns) / 1.e9);
  }

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time since last call, measured in nanoseconds.
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
   * and the timestep \c dt.
   *
   * \param error  Error since last call (error = target - state)
   * \param dt Change in time since last call
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
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call in seconds
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
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call in nanoseconds
   *
   * \returns PID command
   */
  [[deprecated("Use compute_command() instead")]] [[nodiscard]] double computeCommand(
    double error, double error_dot, uint64_t dt_ns) {
    return compute_command(error, error_dot, static_cast<double>(dt_ns) / 1.e9);
  }

  /*!
   * \brief Set the PID error and compute the PID command with nonuniform
   * time step size. This also allows the user to pass in a precomputed
   * derivative error.
   *
   * \param error Error since last call (error = target - state)
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call, measured in nanoseconds.
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
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call, measured in nanoseconds.
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
   * \brief Set current command for this PID controller
   */
  [[deprecated("Use set_current_cmd() instead")]] void setCurrentCmd(double cmd) {
    set_current_cmd(cmd);
  }

  /*!
   * \brief Return current command for this PID controller
   */
  double get_current_cmd();

  /*!
   * \brief Return current command for this PID controller
   */
  [[deprecated("Use get_current_cmd() instead")]] double getCurrentCmd() {
    return get_current_cmd();
  }

  /*!
   * \brief Return derivative error
   */
  [[deprecated("Use get_current_pid_errors() instead")]]
  double getDerivativeError() {
    double pe, ie, de;
    get_current_pid_errors(pe, ie, de);
    return de;
  }

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe  The proportional error.
   * \param ie  The integral error.
   * \param de  The derivative error.
   */
  void get_current_pid_errors(double & pe, double & ie, double & de);

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe  The proportional error.
   * \param ie  The integral error.
   * \param de  The derivative error.
   */
  [[deprecated("Use get_current_pid_errors() instead")]] void getCurrentPIDErrors(
    double & pe, double & ie, double & de) {
    get_current_pid_errors(pe, ie, de);
  }

  /*!
   * @brief Custom assignment operator
   *        Does not initialize dynamic reconfigure for PID gains
   */
  Pid & operator=(const Pid & source)
  {
    if (this == &source) {
      return *this;
    }

    // Copy the realtime buffer to then new PID class
    gains_buffer_ = source.gains_buffer_;

    // Reset the state of this PID controller
    reset();

    return *this;
  }

protected:
  // Store the PID gains in a realtime buffer to allow dynamic reconfigure to update it without
  // blocking the realtime update loop
  realtime_tools::RealtimeBuffer<Gains> gains_buffer_;

  double p_error_last_; /** Save state for derivative state calculation. */
  double p_error_;      /** Error. */
  double i_error_;      /** Integral of error. */
  double d_error_;      /** Derivative of error. */
  double cmd_;          /** Command to send. */
  // TODO(christophfroehlich) remove this -> breaks ABI
  [[deprecated("Use d_error_")]] double error_dot_;    /** Derivative error */
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__PID_HPP_
