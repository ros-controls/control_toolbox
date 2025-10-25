// Copyright (c) 2025, ros2_control developers
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

#ifndef CONTROL_TOOLBOX__TF_UTILS_HPP_
#define CONTROL_TOOLBOX__TF_UTILS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace control_toolbox
{
/**
* @brief Apply a TF prefix to a given frame.
* @param tf_prefix_enabled Whether to apply the TF prefix
* @param prefix TF prefix
* @param frame Frame name
* @param node_ns Node namespace to use as prefix if prefix is empty
* @return The prefixed frame name if prefix is not empty, otherwise the original frame name
*/
inline std::string apply_tf_prefix(
  bool tf_prefix_enabled, std::string prefix, const std::string & node_ns,
  const std::string & frame)
{
  if (!tf_prefix_enabled)
  {
    return frame;
  }

  // fallback to node namespace if explicit prefix not set
  if (prefix.empty())
  {
    prefix = node_ns;
  }

  // normalize: remove leading '/' and ensure trailing '/'
  if (!prefix.empty())
  {
    if (prefix.front() == '/')
    {
      prefix.erase(0, 1);
    }
    if (prefix.back() != '/')
    {
      prefix.push_back('/');
    }
  }

  return prefix + frame;
}

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__TF_UTILS_HPP_
