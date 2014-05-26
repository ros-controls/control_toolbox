/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/*
  Author: Melonee Wise
  Contributors: Dave Coleman, Jonathan Bohren, Bob Holmberg, Wim Meeussen
  Desc: Implements a standard proportional-integral-derivative controller
*/

#include <control_toolbox/pid.h>
#include <tinyxml.h>

namespace control_toolbox {

static const std::string DEFAULT_NAMESPACE = "pid"; // \todo better default prefix?

Pid::Pid(double p, double i, double d, double i_max, double i_min)
  : dynamic_reconfig_initialized_(false)
{
  setGains(p,i,d,i_max,i_min);

  reset();
}

Pid::Pid(const Pid &source)
   : dynamic_reconfig_initialized_(false)
{
  // Copy the realtime buffer to then new PID class
  gains_buffer_ = source.gains_buffer_;

  // Reset the state of this PID controller
  reset();
}

Pid::~Pid()
{
}

void Pid::initPid(double p, double i, double d, double i_max, double i_min, 
  const ros::NodeHandle &node)
{
  initPid(p, i, d, i_max, i_min);

  // Create node handle for dynamic reconfigure
  ros::NodeHandle nh(DEFAULT_NAMESPACE);
  initDynamicReconfig(nh);
}

void Pid::initPid(double p, double i, double d, double i_max, double i_min)
{
  setGains(p,i,d,i_max,i_min);

  reset();
}

bool Pid::initParam(const std::string& prefix, const bool quiet)
{
  ros::NodeHandle nh(prefix);
  return init(nh, quiet);
}

bool Pid::init(const ros::NodeHandle &node, const bool quiet)
{
  ros::NodeHandle nh(node);

  Gains gains;

  // Load PID gains from parameter server
  if (!nh.getParam("p", gains.p_gain_)) 
  {
    if (!quiet) {
      ROS_ERROR("No p gain specified for pid.  Namespace: %s", nh.getNamespace().c_str());
    }
    return false;
  }
  // Only the P gain is required, the I and D gains are optional and default to 0:
  nh.param("i", gains.i_gain_, 0.0);
  nh.param("d", gains.d_gain_, 0.0);

  // Load integral clamp from param server or default to 0
  double i_clamp;
  nh.param("i_clamp", i_clamp, 0.0);
  gains.i_max_ = std::abs(i_clamp);
  gains.i_min_ = -std::abs(i_clamp);
  setGains(gains);

  reset();
  initDynamicReconfig(nh);

  return true;
}

bool Pid::initXml(TiXmlElement *config)
{
  // Create node handle for dynamic reconfigure
  ros::NodeHandle nh(DEFAULT_NAMESPACE);

  double i_clamp;
  i_clamp = config->Attribute("iClamp") ? atof(config->Attribute("iClamp")) : 0.0;

  setGains( 
    config->Attribute("p") ? atof(config->Attribute("p")) : 0.0,
    config->Attribute("i") ? atof(config->Attribute("i")) : 0.0,
    config->Attribute("d") ? atof(config->Attribute("d")) : 0.0,
    std::abs(i_clamp),
    -std::abs(i_clamp)
  );

  reset();
  initDynamicReconfig(nh);

  return true;
}

void Pid::initDynamicReconfig(ros::NodeHandle &node)
{
  ROS_DEBUG_STREAM_NAMED("pid","Initializing dynamic reconfigure in namespace " 
    << node.getNamespace());

  // Start dynamic reconfigure server
  param_reconfig_server_.reset(new DynamicReconfigServer(param_reconfig_mutex_, node));
  dynamic_reconfig_initialized_ = true;
 
  // Set Dynamic Reconfigure's gains to Pid's values
  updateDynamicReconfig();

  // Set callback
  param_reconfig_callback_ = boost::bind(&Pid::dynamicReconfigCallback, this, _1, _2);
  param_reconfig_server_->setCallback(param_reconfig_callback_); 
}

void Pid::reset()
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_term_  = 0.0;
  cmd_ = 0.0;
}

void Pid::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  Gains gains = *gains_buffer_.readFromRT();

  p     = gains.p_gain_;
  i     = gains.i_gain_;
  d     = gains.d_gain_;
  i_max = gains.i_max_;
  i_min = gains.i_min_;
}

Pid::Gains Pid::getGains() 
{
  return *gains_buffer_.readFromRT();
}

void Pid::setGains(double p, double i, double d, double i_max, double i_min)
{
  Gains gains(p,i,d,i_max,i_min);
  
  setGains(gains);
}

void Pid::setGains(const Gains &gains)
{
  gains_buffer_.writeFromNonRT(gains);

  // Update dynamic reconfigure with the new gains
  updateDynamicReconfig(gains);
}

void Pid::updateDynamicReconfig()
{
  // Make sure dynamic reconfigure is initialized
  if(!dynamic_reconfig_initialized_)
    return;

  // Get starting values 
  control_toolbox::ParametersConfig config;

  // Get starting values   
  getGains(config.p_gain, config.i_gain, config.d_gain, config.i_clamp_max, config.i_clamp_min);

  updateDynamicReconfig(config);
}

void Pid::updateDynamicReconfig(Gains gains_config)
{
  // Make sure dynamic reconfigure is initialized
  if(!dynamic_reconfig_initialized_)
    return;

  control_toolbox::ParametersConfig config;

  // Convert to dynamic reconfigure format
  config.p_gain = gains_config.p_gain_;
  config.i_gain = gains_config.i_gain_;
  config.d_gain = gains_config.d_gain_;
  config.i_clamp_max = gains_config.i_max_;
  config.i_clamp_min = gains_config.i_min_;

  updateDynamicReconfig(config);
}

void Pid::updateDynamicReconfig(control_toolbox::ParametersConfig config)
{
  // Make sure dynamic reconfigure is initialized
  if(!dynamic_reconfig_initialized_)
    return;

  // Set starting values, using a shared mutex with dynamic reconfig
  param_reconfig_mutex_.lock(); 
  param_reconfig_server_->updateConfig(config);
  param_reconfig_mutex_.unlock(); 
}

void Pid::dynamicReconfigCallback(control_toolbox::ParametersConfig &config, uint32_t level)
{
  ROS_DEBUG_STREAM_NAMED("pid","Dynamics reconfigure callback recieved.");

  // Set the gains
  setGains(config.p_gain, config.i_gain, config.d_gain, config.i_clamp_max, config.i_clamp_min);
}

double Pid::computeCommand(double error, ros::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term;
  p_error_ = error; // this is error = target - state

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + gains.i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( gains.i_min_, std::min( i_term_, gains.i_max_) );

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    d_error_ = (p_error_ - p_error_last_) / dt.toSec();
    p_error_last_ = p_error_;
  }
  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;
  cmd_ = p_term + i_term_ + d_term;

  return cmd_;
}

double Pid::updatePid(double error, ros::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term;
  p_error_ = error; //this is pError = pState-pTarget

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + gains.i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( gains.i_min_, std::min( i_term_, gains.i_max_) );

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    d_error_ = (p_error_ - p_error_last_) / dt.toSec();
    p_error_last_ = p_error_;
  }
  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;
  cmd_ = - p_term - i_term_ - d_term;

  return cmd_;
}

double Pid::computeCommand(double error, double error_dot, ros::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term;
  p_error_ = error; // this is error = target - state
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + gains.i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( gains.i_min_, std::min( i_term_, gains.i_max_) );

  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;
  cmd_ = p_term + i_term_ + d_term;

  return cmd_;
}

double Pid::updatePid(double error, double error_dot, ros::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term;
  p_error_ = error; //this is pError = pState-pTarget
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  //Calculate integral contribution to command
  i_term_ = i_term_ + gains.i_gain_ * dt.toSec() * p_error_;

  // Limit i_term_ so that the limit is meaningful in the output
  i_term_ = std::max( gains.i_min_, std::min( i_term_, gains.i_max_) );

  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;
  cmd_ = - p_term - i_term_ - d_term;

  return cmd_;
}

void Pid::setCurrentCmd(double cmd)
{
  cmd_ = cmd;
}

double Pid::getCurrentCmd()
{
  return cmd_;
}

void Pid::getCurrentPIDErrors(double *pe, double *ie, double *de)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  *pe = p_error_;
  *ie = gains.i_gain_ ? i_term_/gains.i_gain_ : 0.0;
  *de = d_error_;
}

void Pid::printValues()
{
  Gains gains = getGains();

  ROS_INFO_STREAM_NAMED("pid","Current Values of PID Class:\n"
    << "  P Gain: " << gains.p_gain_ << "\n"
    << "  I Gain: " << gains.i_gain_ << "\n"
    << "  D Gain: " << gains.d_gain_ << "\n"
    << "  I_Max:  " << gains.i_max_  << "\n"
    << "  I_Min:  " << gains.i_min_  << "\n"
    << "  P_Error_Last: " << p_error_last_  << "\n"
    << "  P_Error:      " << p_error_  << "\n"
    << "  D_Error:      " << d_error_  << "\n"
    << "  I_Term:       " << i_term_  << "\n"
    << "  Command:      " << cmd_
  );

}

} // namespace
