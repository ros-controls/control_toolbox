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
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_toolbox
{

// Forward declaration of FilterTraits
template <typename T>
struct FilterTraits;

// Wrapper around std::vector<double> to be used as the std::vector<double> StorageType specialization
// This is a workaround for the fact that std::vector<double>'s operator* and operator+ cannot be overloaded.
struct FilterVector
{
  std::vector<double> data;

  FilterVector() = default;

  explicit FilterVector(const std::vector<double> & vec) : data(vec) {}

  explicit FilterVector(size_t size, double initial_value = 0.0) : data(size, initial_value) {}

  FilterVector operator*(double scalar) const
  {
    FilterVector result = *this;
    for (auto & val : result.data)
    {
      val *= scalar;
    }
    return result;
  }

  FilterVector operator+(const FilterVector & other) const
  {
    assert(data.size() == other.data.size() && "Vectors must be of the same size for addition");
    FilterVector result = *this;
    for (size_t i = 0; i < data.size(); ++i)
    {
      result.data[i] += other.data[i];
    }
    return result;
  }

  size_t size() const { return data.size(); }
};

// Enable scalar * FilterVector
inline FilterVector operator*(double scalar, const FilterVector & vec) { return vec * scalar; }

// Default: storage type is just T itself
template <typename T>
struct FilterTraits
{
  using StorageType = T;

  static void initialize(StorageType & storage)
  {
    storage = T{std::numeric_limits<T>::quiet_NaN()};
  }

  static bool is_nan(const StorageType & storage) { return std::isnan(storage); }

  static bool is_infinite(const StorageType & storage) { return !std::isfinite(storage); }

  static bool is_empty(const StorageType & storage)
  {
    (void)storage;
    return false;
  }

  static void assign(StorageType & storage, const StorageType & data_in) { storage = data_in; }
};

// Specialization: for WrenchStamped, use struct Vec6, wrapper for Eigen::Matrix
template <>
struct FilterTraits<geometry_msgs::msg::WrenchStamped>
{
  using StorageType = Eigen::Matrix<double, 6, 1>;
  using DataType = geometry_msgs::msg::WrenchStamped;

  static void initialize(StorageType & storage)
  {
    // Evocation of the default constructor through EIGEN_INITIALIZE_MATRICES_BY_NAN
    storage = StorageType();
  }

  static bool is_nan(const StorageType & storage) { return storage.hasNaN(); }

  static bool is_infinite(const DataType & data)
  {
    return !std::isfinite(data.wrench.force.x) || !std::isfinite(data.wrench.force.y) ||
           !std::isfinite(data.wrench.force.z) || !std::isfinite(data.wrench.torque.x) ||
           !std::isfinite(data.wrench.torque.y) || !std::isfinite(data.wrench.torque.z);
  }

  static bool is_valid(const StorageType & storage)
  {
    return std::all_of(
      storage.begin(), storage.end(), [](double val) { return std::isfinite(val); });
  }

  static bool is_empty(const StorageType & storage)
  {
    (void)storage;
    return false;
  }

  static void assign(DataType & data_in, const StorageType & storage)
  {
    data_in.wrench.force.x = storage[0];
    data_in.wrench.force.y = storage[1];
    data_in.wrench.force.z = storage[2];
    data_in.wrench.torque.x = storage[3];
    data_in.wrench.torque.y = storage[4];
    data_in.wrench.torque.z = storage[5];
  }

  static void assign(StorageType & storage, const DataType & data_in)
  {
    storage[0] = data_in.wrench.force.x;
    storage[1] = data_in.wrench.force.y;
    storage[2] = data_in.wrench.force.z;
    storage[3] = data_in.wrench.torque.x;
    storage[4] = data_in.wrench.torque.y;
    storage[5] = data_in.wrench.torque.z;
  }

  static void assign(StorageType & storage, const StorageType & data_in) { storage = data_in; }
};

// Specialization: for std::vector<double>
template <>
struct FilterTraits<std::vector<double>>
{
  using StorageType = FilterVector;
  using DataType = std::vector<double>;

  static void initialize(StorageType & storage)
  {
    storage.data = std::vector<double>{std::numeric_limits<double>::quiet_NaN()};
  }

  static bool is_infinite(const StorageType & storage)
  {
    return std::all_of(
      storage.data.begin(), storage.data.end(), [](double val) { return std::isfinite(val); });
  }

  static bool is_infinite(const DataType & storage)
  {
    return std::all_of(
      storage.begin(), storage.end(), [](double val) { return std::isfinite(val); });
  }

  static bool is_empty(const StorageType & storage) { return storage.data.empty(); }

  static bool is_nan(const StorageType & storage)
  {
    for (const auto & val : storage.data)
    {
      if (std::isnan(val))
      {
        return true;
      }
    }

    return false;
  }

  static void assign(StorageType & storage, const DataType & data_in) { storage.data = data_in; }

  static void assign(DataType & storage, const StorageType & data_in) { storage = data_in.data; }

  static void assign(StorageType & storage, const StorageType & data_in)
  {
    storage.data = data_in.data;
  }
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__FILTER_TRAITS_HPP_
