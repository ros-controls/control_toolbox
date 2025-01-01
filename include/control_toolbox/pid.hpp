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

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "realtime_tools/realtime_buffer.hpp"

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
  pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
  double position_desi_ = 0.5;
  ...
  rclcpp::Time last_time = get_clock()->now();
  while (true) {
  rclcpp::Time time = get_clock()->now();
  double effort = pid.computeCommand(position_desi_ - currentPosition(), (time - last_time).nanoseconds());
  last_time = time;
  }
  \endverbatim

*/
/***************************************************/

class Pid
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
   * \throws An std::invalid_argument exception is thrown if i_min > i_max
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
   * \throws An std::invalid_argument exception is thrown if i_min > i_max
   */
    Gains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
    : p_gain_(p), i_gain_(i), d_gain_(d), i_max_(i_max), i_min_(i_min), antiwindup_(antiwindup)
    {
    }
    // Default constructor
    Gains() : p_gain_(0.0), i_gain_(0.0), d_gain_(0.0), i_max_(0.0), i_min_(0.0), antiwindup_(false)
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
  void initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

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
  void getGains(double & p, double & i, double & d, double & i_max, double & i_min);
  /*!
   * \brief Get PID gains for the controller.
   * \param p The proportional gain.
   * \param i The integral gain.
   * \param d The derivative gain.
   * \param i_max The max integral windup.
   * \param i_min The min integral windup.
   * \param antiwindup If true, antiwindup is enabled and i_max/i_min are enforced
   */
  void getGains(
    double & p, double & i, double & d, double & i_max, double & i_min, bool & antiwindup);

  /*!
   * \brief Get PID gains for the controller.
   * \return gains A struct of the PID gain values
   */
  Gains getGains();

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
  void setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

  /*!
   * \brief Set PID gains for the controller.
   * \param gains A struct of the PID gain values
   *
   * \note New gains are not applied if gains.i_min_ > gains.i_max_
   */
  void setGains(const Gains & gains);

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
  [[nodiscard]] double computeCommand(double error, uint64_t dt);

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
  [[nodiscard]] double computeCommand(double error, double error_dot, uint64_t dt);

  /*!
   * \brief Set current command for this PID controller
   */
  void setCurrentCmd(double cmd);

  /*!
   * \brief Return current command for this PID controller
   */
  double getCurrentCmd();

  /*!
   * \brief Return derivative error
   */
  double getDerivativeError();

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe  The proportional error.
   * \param ie  The integral error.
   * \param de  The derivative error.
   */
  void getCurrentPIDErrors(double & pe, double & ie, double & de);

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

  double p_error_last_; /**< _Save position state for derivative state calculation. */
  double p_error_;      /**< Position error. */
  double i_error_;      /**< Integral of position error. */
  double d_error_;      /**< Derivative of position error. */
  double cmd_;          /**< Command to send. */
  double error_dot_;    /**< Derivative error */
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__PID_HPP_
