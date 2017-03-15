/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Aris Synodinos

#ifndef CONTROL_TOOLBOX__LEAD_LAG_H
#define CONTROL_TOOLBOX__LEAD_LAG_H

#include <ros/ros.h>
#include <string>

// Dynamic reconfigure
#include <control_toolbox/LeadLagConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/mutex.hpp>

// Realtime buffer
#include <realtime_tools/realtime_buffer.h>

class TiXmlElement;

namespace control_toolbox {
/***************************************************/
/*! \class LeadLag
 @brief A basic Lead/Lag class.

 This class implements a generic structure that
 can be used to create a wide range of lead / lag
 controllers. It can function independently or
 be subclassed to provide more specific controls
 based on a particular control loop.

 In particular, this class implements the standard
 lead-lag equation:

 \f$ G(s) = gain * \frac{1 + \frac{w}{w_z}}{1 + \frac{w}{w_p}} \f$

 where: -$w_z$ is the zero, -$w_p$ is the pole
 and gain is the proportional gain of the controller.

 Discretized using the Bilinear trasformation

 \f$ s = \frac{2}{T} \frac{z - 1}{z + 1} \f$

 Which results in the discrete form of

 \f$ G(z) = k * \frac{z - a}{z - b} \f$

 where: <br>
 <UL TYPE="none">
 <LI>  \f$ k = gain * \frac{ w_p ( w_z + \frac{2}{dt} ) }{ w_z ( w_p + \frac{2}{dt} ) } \f$
 <LI>  \f$ a = - \frac{ w_z - \frac{2}{dt} }{ w_z + \frac{2}{dt} } \f$
 <LI>  \f$ b = - \frac{ w_p - \frac{2}{dt} }{ w_p + \frac{2}{dt} } \f$
 </UL>

 and can be converted in the discrete time-domain as

 \f$ y = b * y_{-1} + k * ( x - a * x_{-1} ) \f$

 \section ROS ROS interface

 \param k Proportional gain

 \param a Zero parameter

 \param b Pole parameter

 \section Usage

 To use the LeadLag class, you should first call some version of init()
 (in non-realtime) and then call computeCommand() at every step.
 For example:

 \verbatim
 control_toolbox::LeadLag comp;
 comp.initLeadLag(2.0, 0.8, 1.2);
 double desired_position = 0.5;
 ...
 ros::Time last_time = ros::Time::now();
 while (true) {
   ros::Time time = ros::Time::now();
   double effort = comp.computeCommand(current_position - desired_position, time - last_time);
   last_time = time;
 }
 \endverbatim
 */
 /***************************************************/

class LeadLag {
 public:

  /**
   * @brief Stores the gains to allow easier realtime buffer usage
   */
  struct Gains {
    Gains(double k, double a, double b) : k_(k), a_(a), b_(b) {}
    Gains() {}
    double k_, a_, b_;
  };

  /**
   * @brief Constructor, zeros out LeadLag values when created and
   * initializes the controller state.
   * Does not initialize dynamic reconfigure.
   * @param k The proportional gain.
   * @param a The zero parameter.
   * @param b The pole parameter.
   */
  LeadLag(double k = 0.0, double a = 0.0, double b = 0.0);

  /**
   * @brief Copy constructor required for preventing mutexes from being copied
   * @param source - LeadLag to copy
   */
  LeadLag(const LeadLag &source);

  /**
   * @brief Destructor of LeadLag class.
   */
  ~LeadLag();

  /**
   * @brief Zeros out LeadLag values and initializes the gains
   * Does not initialize dynamic reconfigure.
   * @param k The proportional gain.
   * @param a The zero parameter.
   * @param b The pole parameter.
   */
  void initLeadLag(double k, double a, double b);

  /**
   * @brief Zeros out LeadLag values and initializes the gains
   * Does not initialize dynamic reconfigure.
   * @param k The proportional gain.
   * @param a The zero parameter.
   * @param b The pole parameter.
   */
  void initLeadLag(double k, double a, double b,
                   const ros::NodeHandle & /*node*/);

  /**
   * @brief Initialize LeadLag with the parameters in a namespace
   * Initializes dynamic reconfigure for LeadLag gains
   *
   * @param prefix The namespace prefix.
   * @param quiet If true, no error messages will be emitted on failure
   * @return True on success, false otherwise.
   */
  bool initParam(const std::string &prefix, const bool quiet = false);

  /**
   * @brief Initialize LeadLag with the parameters in a NodeHandle namespace
   * Initializes dynamic reconfigure for LeadLag gains
   *
   * @param n The NodeHandle which should be used to query parameters.
   * @param quiet If true, no error messages will be emitted on failure
   * @return True on success, false otherwise.
   */
  bool init(const ros::NodeHandle &n, const bool quiet = false);

  /**
   * @brief Initialize LeadLag with the parameters in an XML element
   * Initializes dynamic reconfigure for LeadLag gains
   *
   * @param config the XML element
   * @return True on success, false otherwise.
   */
  bool initXml(TiXmlElement *config);

  /**
   * @brief Start the dynamic reconfigure node and lead the default values.
   * @param node - a NodeHandle where dynamic reconfigure services will be published.
   */
  void initDynamicReconfig(ros::NodeHandle &node);

  /**
   * @brief Reset the state of this LeadLag controller.
   */
  void reset();

  /**
   * @brief Get LeadLag gains for the controller.
   * @param k The proportional gain.
   * @param a The zero parameter.
   * @param b The pole parameter.
   */
  void getGains(double &k, double &a, double &b);

  /**
   * @brief Get LeadLag gains for the controller.
   * @return gains A struct of the LeadLag gain values
   */
  Gains getGains();

  /**
   * @brief Set LeadLag gains for the controller.
   * @param k The proportional gain.
   * @param a The zero parameter.
   * @param b The pole parameter.
   */
  void setGains(double k, double a, double b);

  /**
   * @brief Set LeadLag gains for the controller.
   * @param gains A struct of the LeadLag gain values
   */
  void setGains(const Gains &gains);

  /**
   * @brief Set Dynamic Reconfigure's gains to LeadLag's values
   */
  void updateDynamicReconfig();
  void updateDynamicReconfig(LeadLag::Gains gains);
  void updateDynamicReconfig(control_toolbox::LeadLagConfig config);

  /**
   * @brief Update the LeadLag parameters from dynamic reconfigure
   */
  void dynamicReconfigCallback(control_toolbox::LeadLagConfig &config,
                               uint32_t /*level*/);

  /**
   * @brief Set the LeadLag error and compute the command.
   * @param error Error since last call (error = target - state)
   * @return LeadLag command
   */
  double computeCommand(double error, ros::Duration /*dt*/);

  /**
   * @brief Set current command for this LeadLag controller.
   */
  void setCurrentCmd(double cmd);

  /**
   * @brief Return current command for this LeadLag controller.
   */
  double getCurrentCmd();

  /**
   * @brief Return current error.
   */
  double getCurrentError();

  /**
   * @brief Print to console the current parameters
   */
  void printValues();

  /**
   * @brief Custom assignment operator
   * Does not initialize dynamic reconfigure for LeadLag gains
   */
  LeadLag &operator=(const LeadLag &source) {
    if (this == &source) return *this;

    // Copy the realtime buffer to then new LeadLag class
    gains_buffer_ = source.gains_buffer_;

    // Reset the state of this LeadLag controller
    reset();

    return *this;
  }

 private:
  // Store the LeadLag gains in a realtime buffer to allow dynamic reconfigure
  // to update it without
  // blocking the realtime update loop
  realtime_tools::RealtimeBuffer<LeadLag::Gains> gains_buffer_;

  double error_last_;
  double cmd_last_;
  double error_;
  double cmd_;

  // Dynamic reconfigure
  bool dynamic_reconfig_initialized_;
  typedef dynamic_reconfigure::Server<control_toolbox::LeadLagConfig>
      DynamicReconfigServer;
  boost::shared_ptr<DynamicReconfigServer> lead_lag_reconfig_server_;
  DynamicReconfigServer::CallbackType lead_lag_reconfig_callback_;
  boost::recursive_mutex lead_lag_reconfig_mutex_;
};

typedef boost::shared_ptr<LeadLag> LeadLagPtr;
typedef boost::shared_ptr<LeadLag const> LeadLagConstPtr;

}

#endif  // CONTROL_TOOLBOX__LEAD_LAG_H
