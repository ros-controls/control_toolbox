/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,  Open Source Robotics Foundation
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
  Author: Aris Synodinos
  Desc: Implements a standard lead-lag controller
*/

#include <control_toolbox/lead_lag.h>
#include <tinyxml.h>

using namespace control_toolbox;

static const std::string DEFAULT_NAMESPACE = "lead_lag";

LeadLag::LeadLag(double k, double a, double b)
    : dynamic_reconfig_initialized_(false) {
  setGains(k, a, b);
  reset();
}

LeadLag::LeadLag(const LeadLag& source) : dynamic_reconfig_initialized_(false) {
  // Copy the realtime buffer to then new LeadLag class
  gains_buffer_ = source.gains_buffer_;

  // Reset the state of this LeadLag controller
  reset();
}

LeadLag::~LeadLag() {}

void LeadLag::initLeadLag(double k, double a, double b) {
  setGains(k, a, b);
  reset();
}

void LeadLag::initLeadLag(double k, double a, double b,
                          const ros::NodeHandle& /*node*/) {
  initLeadLag(k, a, b);
  ros::NodeHandle nh(DEFAULT_NAMESPACE);
  initDynamicReconfig(nh);
}

bool LeadLag::initParam(const std::string& prefix, const bool quiet) {
  ros::NodeHandle nh(prefix);
  return init(nh, quiet);
}

bool LeadLag::init(const ros::NodeHandle& n, const bool quiet) {
  ros::NodeHandle nh(n);

  Gains gains;

  // Load LeadLag gains from parameter server
  if (!nh.getParam("k", gains.k_)) {
    if (!quiet) {
      ROS_ERROR_NAMED("lead-lag",
                      "No k gain specified for lead-lag. Namespace: %s",
                      nh.getNamespace().c_str());
    }
    return false;
  }

  // Only the K gain is required, the A and B gains are optional and default to
  // 0:
  nh.param("a", gains.a_, 0.0);
  nh.param("b", gains.b_, 0.0);

  setGains(gains);

  reset();
  initDynamicReconfig(nh);

  return true;
}

bool LeadLag::initXml(TiXmlElement* config) {
  ros::NodeHandle nh(DEFAULT_NAMESPACE);

  setGains(config->Attribute("k") ? atof(config->Attribute("k")) : 0.0,
           config->Attribute("a") ? atof(config->Attribute("a")) : 0.0,
           config->Attribute("b") ? atof(config->Attribute("b")) : 0.0);
  reset();
  initDynamicReconfig(nh);

  return true;
}

void LeadLag::initDynamicReconfig(ros::NodeHandle& node) {
  ROS_DEBUG_STREAM_NAMED("lead-lag",
                         "Initializing dynamic reconfigure in namespace "
                             << node.getNamespace());

  // Start Dynamic reconfigure server
  lead_lag_reconfig_server_.reset(
      new DynamicReconfigServer(lead_lag_reconfig_mutex_, node));
  dynamic_reconfig_initialized_ = true;

  // Set Dynamic Reconfigure's gains to LeadLag's values
  updateDynamicReconfig();

  // Set callback
  lead_lag_reconfig_callback_ =
      boost::bind(&LeadLag::dynamicReconfigCallback, this, _1, _2);
  lead_lag_reconfig_server_->setCallback(lead_lag_reconfig_callback_);
}

void LeadLag::reset() {
  error_last_ = 0.0;
  error_ = 0.0;
  cmd_last_ = 0.0;
  cmd_ = 0.0;
}

void LeadLag::getGains(double& k, double& a, double& b) {
  Gains gains = *gains_buffer_.readFromRT();

  k = gains.k_;
  a = gains.a_;
  b = gains.b_;
}

LeadLag::Gains LeadLag::getGains() { return *gains_buffer_.readFromRT(); }

void LeadLag::setGains(double k, double a, double b) {
  Gains gains(k, a, b);

  setGains(gains);
}

void LeadLag::setGains(const LeadLag::Gains& gains) {
  gains_buffer_.writeFromNonRT(gains);

  // Update dynamic reconfigure with the new gains
  updateDynamicReconfig(gains);
}

void LeadLag::updateDynamicReconfig() {
  // Make sure dynamic reconfigure is initialized
  if (!dynamic_reconfig_initialized_) return;

  // Get starting values
  control_toolbox::LeadLagConfig config;

  // Get starting values
  getGains(config.k, config.a, config.b);

  updateDynamicReconfig(config);
}

void LeadLag::updateDynamicReconfig(LeadLag::Gains gains) {
  // Make sure dynamic reconfigure is initialized
  if (!dynamic_reconfig_initialized_) return;

  control_toolbox::LeadLagConfig config;

  config.k = gains.k_;
  config.a = gains.a_;
  config.b = gains.b_;

  updateDynamicReconfig(config);
}

void LeadLag::updateDynamicReconfig(control_toolbox::LeadLagConfig config) {
  // Make sure dynamic reconfigure is initialized
  if (!dynamic_reconfig_initialized_) return;

  // Set starting values, using a shared mutex with dynamic reconfig
  {
    boost::recursive_mutex::scoped_lock lock(lead_lag_reconfig_mutex_);
    lead_lag_reconfig_server_->updateConfig(config);
  }
}

void LeadLag::dynamicReconfigCallback(control_toolbox::LeadLagConfig& config,
                                      uint32_t /*level*/) {
  ROS_DEBUG_STREAM_NAMED("lead-lag", "Dynamics reconfigure callback received.");

  // Set the gains
  setGains(config.k, config.a, config.b);
}

double LeadLag::computeCommand(double error, ros::Duration /*dt*/) {
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_.readFromRT();
  error_ = error;

  if (std::isnan(error) || std::isinf(error)) return 0.0;

  cmd_ = gains.k_ * (error_ - gains.a_ * error_last_) + gains.b_ * cmd_last_;
  error_last_ = error_;
  cmd_last_ = cmd_;
  return cmd_;
}

void LeadLag::setCurrentCmd(double cmd) { cmd_ = cmd; }

double LeadLag::getCurrentCmd() { return cmd_; }

double LeadLag::getCurrentError() { return  error_; }

void LeadLag::printValues() {
  Gains gains = getGains();

  ROS_INFO_STREAM_NAMED("lead-lag",
                        "Current Values of LeadLag Class:\n"
                            << "  K Gain: " << gains.k_ << "\n"
                            << "  A Gain: " << gains.a_ << "\n"
                            << "  B Gain: " << gains.b_ << "\n"
                            << "  Error_Last: " << error_last_ << "\n"
                            << "  Error: " << error_ << "\n"
                            << "  Command_Last: " << cmd_last_ << "\n"
                            << "  Command: " << cmd_);
}
