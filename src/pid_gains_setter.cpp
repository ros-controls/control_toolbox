/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "control_toolbox/pid_gains_setter.h"

namespace control_toolbox {

PidGainsSetter::~PidGainsSetter()
{
  serve_set_gains_.shutdown();
}

PidGainsSetter& PidGainsSetter::add(Pid *pid)
{
  assert(pid);

  if(pids_.size() > 0){
    //use existing pid config for new added controller
    pid->setGains(pids_.begin().operator *()->getGains());
  }

  pids_.push_back(pid);


  return *this;
}

void PidGainsSetter::advertise(const std::string name_postfix, const ros::NodeHandle &n)
{
  node_ = n;
  serve_set_gains_ = node_.advertiseService(std::string("set_gains_") + name_postfix, &PidGainsSetter::setGains, this);
  serve_get_gains_ = node_.advertiseService(std::string("get_gains_")+ name_postfix, &PidGainsSetter::getGains, this);
}

bool PidGainsSetter::setGains(control_toolbox::SetPidGains::Request &req,
                              control_toolbox::SetPidGains::Response &resp)
{
  for (size_t i = 0; i < pids_.size(); ++i)
    pids_[i]->setGains(req.p, req.i, req.d, req.i_clamp_max, -req.i_clamp_min);
  return true;
}

bool PidGainsSetter::getGains(control_toolbox::GetPidGains::Request &req,
                              control_toolbox::GetPidGains::Response &resp)
{
  if(pids_.size()>0){
    pids_[0]->getGains(resp.p, resp.i, resp.d ,resp.i_clamp_max,resp.i_clamp_min);
  }

  return true;
}

}
