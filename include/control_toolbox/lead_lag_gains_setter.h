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

// Exposes a ROS interface for tuning a set of lead lag controllers.
//
// Author: Aris Synodinos

#ifndef CONTROL_TOOLBOX__LEAD_LAG_GAINS_SETTER_H
#define CONTROL_TOOLBOX__LEAD_LAG_GAINS_SETTER_H

#include <control_toolbox/SetLeadLagGains.h>
#include <control_toolbox/lead_lag.h>
#include <ros/node_handle.h>

namespace control_toolbox {

class LeadLagGainsSetter {
 public:
  LeadLagGainsSetter() {}

  ~LeadLagGainsSetter();

  LeadLagGainsSetter &add(LeadLag *controller);

  void advertise(const ros::NodeHandle &n);

  void advertise(const std::string &ns) { advertise(ros::NodeHandle(ns)); }

  bool setGains(control_toolbox::SetLeadLagGains::Request& req,
                control_toolbox::SetLeadLagGains::Response& /*res*/);

 private:
  ros::NodeHandle node_;
  ros::ServiceServer serve_set_gains_;
  std::vector<LeadLag*> controllers_;
};
}

#endif  // CONTROL_TOOLBOX__LEAD_LAG_GAINS_SETTER_H
