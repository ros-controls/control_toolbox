// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef CONTROL_TOOLBOX__FILTER_TRAITS_HPP_
#define CONTROL_TOOLBOX__FILTER_TRAITS_HPP_

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

#include <Eigen/Dense>
#include <limits>

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_toolbox
{

// Default: storage type is just T itself
template <typename T>
struct FilterTraits
{
  using StorageType = T;

  static void initialize(StorageType & storage)
  {
    storage = T{std::numeric_limits<T>::quiet_NaN()};
  }
};

// Specialization: for WrenchStamped, use struct Vec6, wrapper for Eigen::Matrix
template <>
struct FilterTraits<geometry_msgs::msg::WrenchStamped>
{
  using StorageType = Eigen::Matrix<double, 6, 1>;

  static void initialize(StorageType & storage)
  {
    // Evocation of the default constructor through EIGEN_INITIALIZE_MATRICES_BY_NAN
    storage = StorageType();
  }
};
}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__FILTER_TRAITS_HPP_
