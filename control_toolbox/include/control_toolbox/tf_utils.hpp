// Copyright (c) 2025, ros2_control developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROL_TOOLBOX__TF_UTILS_HPP_
#define CONTROL_TOOLBOX__TF_UTILS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace control_toolbox
{
/**
* @brief Apply TF prefix to given frame
* @param prefix TF prefix
* @param node_ns Node namespace to use as prefix if prefix is empty
* @param frame Frame name
* @return The prefixed frame name if prefix is not empty, otherwise the original frame name
*/
inline std::string apply_tf_prefix(
  const std::string & prefix, const std::string & node_ns, const std::string & frame)
{
  std::string nprefix = prefix.empty() ? node_ns : prefix;

  // Normalize the prefix
  if (!nprefix.empty())
  {
    // ensure trailing '/'
    if (nprefix.back() != '/')
    {
      nprefix.push_back('/');
    }
    // remove leading '/'
    if (nprefix.front() == '/')
    {
      nprefix.erase(0, 1);
    }
  }
  return nprefix + frame;
}

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__TF_UTILS_HPP_
