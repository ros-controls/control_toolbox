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

#include <boost/algorithm/clamp.hpp>
#include <boost/algorithm/minmax.hpp>

namespace control_toolbox {

static const std::string DEFAULT_NAMESPACE = "pid"; // \todo better default prefix?

Pid::Pid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
  : dynamic_reconfig_initialized_(false)
{
  setGains(p,i,d,i_max,i_min,antiwindup);

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
  const ros::NodeHandle& /*node*/)
{
  initPid(p, i, d, i_max, i_min);

  // Create node handle for dynamic reconfigure
  ros::NodeHandle nh(DEFAULT_NAMESPACE);
  initDynamicReconfig(nh);
}

void Pid::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup,
  const ros::NodeHandle& /*node*/)
{
  initPid(p, i, d, i_max, i_min, antiwindup);

  // Create node handle for dynamic reconfigure
  ros::NodeHandle nh(DEFAULT_NAMESPACE);
  initDynamicReconfig(nh);
}

void Pid::initPid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  setGains(p,i,d,i_max,i_min, antiwindup);

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
  if(nh.hasParam("i_clamp_min"))
  {
    nh.param("i_clamp_min", gains.i_min_, gains.i_min_); // use i_clamp_min parameter, otherwise keep -i_clamp
    gains.i_min_ = -std::abs(gains.i_min_); // make sure the value is <= 0
  }
  if(nh.hasParam("i_clamp_max"))
  {
    nh.param("i_clamp_max", gains.i_max_, gains.i_max_); // use i_clamp_max parameter, otherwise keep i_clamp
    gains.i_max_ = std::abs(gains.i_max_); // make sure the value is >= 0
  }
  nh.param("antiwindup", gains.antiwindup_, false);

  nh.param("publish_state", publish_state_, false);

  if(publish_state_)
  {
    state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::PidState>());
    state_publisher_->init(nh, "state", 1);
  }

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
    -std::abs(i_clamp),
    config->Attribute("antiwindup") ? atof(config->Attribute("antiwindup")) : false
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
  i_error_ = 0.0;
  d_error_ = 0.0;
  cmd_ = 0.0;
}

void Pid::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool antiwindup;
  getGains(p, i, d, i_max, i_min, antiwindup);
}

void Pid::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  Gains gains = *gains_buffer_.readFromRT();

  p     = gains.p_gain_;
  i     = gains.i_gain_;
  d     = gains.d_gain_;
  i_max = gains.i_max_;
  i_min = gains.i_min_;
  antiwindup = gains.antiwindup_;
}

Pid::Gains Pid::getGains()
{
  return *gains_buffer_.readFromRT();
}

void Pid::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  Gains gains(p,i,d,i_max,i_min, antiwindup);

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
  getGains(config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);

  updateDynamicReconfig(config);
}

void Pid::updateDynamicReconfig(Gains gains_config)
{
  // Make sure dynamic reconfigure is initialized
  if(!dynamic_reconfig_initialized_)
    return;

  control_toolbox::ParametersConfig config;

  // Convert to dynamic reconfigure format
  config.p = gains_config.p_gain_;
  config.i = gains_config.i_gain_;
  config.d = gains_config.d_gain_;
  config.i_clamp_max = gains_config.i_max_;
  config.i_clamp_min = gains_config.i_min_;
  config.antiwindup = gains_config.antiwindup_;

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

void Pid::dynamicReconfigCallback(control_toolbox::ParametersConfig &config, uint32_t /*level*/)
{
  ROS_DEBUG_STREAM_NAMED("pid","Dynamics reconfigure callback recieved.");

  // Set the gains
  setGains(config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min, config.antiwindup);
}

double Pid::computeCommand(double error, ros::Duration dt)
{

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error))
    return 0.0;

  double error_dot = d_error_;

  // Calculate the derivative error
  if (dt.toSec() > 0.0)
  {
    error_dot = (error - p_error_last_) / dt.toSec();
    p_error_last_ = error;
  }

  return computeCommand(error, error_dot, dt);
}

double Pid::updatePid(double error, ros::Duration dt)
{
  return -computeCommand(error, dt);
}

double Pid::computeCommand(double error, double error_dot, ros::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();

  double p_term, d_term, i_term;
  p_error_ = error; // this is error = target - state
  d_error_ = error_dot;

  if (dt == ros::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  // Calculate the integral of the position error
  i_error_ += dt.toSec() * p_error_;

  if(gains.antiwindup_ && gains.i_gain_!=0)
  {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    boost::tuple<double, double> bounds = boost::minmax<double>(gains.i_min_ / gains.i_gain_, gains.i_max_ / gains.i_gain_);
    i_error_ = boost::algorithm::clamp(i_error_, bounds.get<0>(), bounds.get<1>());
  }

  // Calculate integral contribution to command
  i_term = gains.i_gain_ * i_error_;

  if(!gains.antiwindup_)
  {
    // Limit i_term so that the limit is meaningful in the output
    i_term = boost::algorithm::clamp(i_term, gains.i_min_, gains.i_max_);
  }

  // Calculate derivative contribution to command
  d_term = gains.d_gain_ * d_error_;

  // Compute the command
  cmd_ = p_term + i_term + d_term;

  // Publish controller state if configured
  if (publish_state_ && state_publisher_)
  {
    if (state_publisher_->trylock())
    {
      state_publisher_->msg_.header.stamp = ros::Time::now();
      state_publisher_->msg_.timestep = dt;
      state_publisher_->msg_.error = error;
      state_publisher_->msg_.error_dot = error_dot;
      state_publisher_->msg_.p_error = p_error_;
      state_publisher_->msg_.i_error = i_error_;
      state_publisher_->msg_.d_error = d_error_;
      state_publisher_->msg_.p_term = p_term;
      state_publisher_->msg_.i_term = i_term;
      state_publisher_->msg_.d_term = d_term;
      state_publisher_->msg_.i_max = gains.i_max_;
      state_publisher_->msg_.i_min = gains.i_min_;
      state_publisher_->msg_.output = cmd_;
      state_publisher_->unlockAndPublish();
    }
  }

  return cmd_;
}

double Pid::updatePid(double error, double error_dot, ros::Duration dt)
{
  return -computeCommand(error, error_dot, dt);
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
  *ie = i_error_;
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
    << "  Antiwindup:  " << gains.antiwindup_  << "\n"
    << "  P_Error_Last: " << p_error_last_  << "\n"
    << "  P_Error:      " << p_error_  << "\n"
    << "  I_Error:       " << i_error_  << "\n"
    << "  D_Error:      " << d_error_  << "\n"
    << "  Command:      " << cmd_
  );

}

} // namespace
