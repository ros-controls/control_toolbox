// Copyright (c) 2025, ros2_control development team
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

#include <fmt/core.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_toolbox
{

template <typename T>
struct FilterTraits;

// Wrapper around std::vector<U> to be used as
// the std::vector<U> StorageType specialization.
// This is a workaround for the fact that
// std::vector<U>'s operator* and operator+ cannot be overloaded.
template <typename U>
struct FilterVector
{
  std::vector<U> data;

  FilterVector() = default;

  explicit FilterVector(const std::vector<U> & vec) : data(vec) {}

  explicit FilterVector(size_t size, const U & initial_value = U{}) : data(size, initial_value) {}

  FilterVector operator*(const U & scalar) const
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
    if (data.size() != other.data.size())
    {
      throw std::runtime_error(
        fmt::format(
          "Vectors must be of the same size for addition ({} vs {}).", data.size(),
          other.data.size()));
    }
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
template <typename U>
inline FilterVector<U> operator*(const U & scalar, const FilterVector<U> & vec)
{
  return vec * scalar;
}

template <typename T>
struct FilterTraits
{
  using StorageType = T;

  static void initialize(StorageType & storage)
  {
    storage = T{std::numeric_limits<T>::quiet_NaN()};
  }

  static bool is_nan(const StorageType & storage) { return std::isnan(storage); }

  static bool is_finite(const StorageType & storage) { return std::isfinite(storage); }

  static bool is_empty(const StorageType & storage)
  {
    (void)storage;
    return false;
  }

  static void assign(StorageType & storage, const StorageType & data_in) { storage = data_in; }

  static void validate_input(const T & data_in, const StorageType & filtered_value, T & data_out)
  {
    (void)data_in;
    (void)filtered_value;
    (void)data_out;  // Suppress unused warnings
  }

  static void add_metadata(StorageType & storage, const StorageType & data_in)
  {
    (void)storage;
    (void)data_in;
  }
};

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

  static bool is_finite(const DataType & data)
  {
    return std::isfinite(data.wrench.force.x) && std::isfinite(data.wrench.force.y) &&
           std::isfinite(data.wrench.force.z) && std::isfinite(data.wrench.torque.x) &&
           std::isfinite(data.wrench.torque.y) && std::isfinite(data.wrench.torque.z);
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

  static void validate_input(
    const DataType & data_in, const StorageType & filtered_value, DataType & data_out)
  {
    (void)filtered_value;  // filtered_value has no header

    // Compare new input's frame_id with previous output's frame_id,
    // but only if it existed (filter_chains does not initialize output)
    if (!data_out.header.frame_id.empty() && data_in.header.frame_id != data_out.header.frame_id)
    {
      throw std::runtime_error(
        "Frame ID changed between filter updates! Out: " + data_out.header.frame_id +
        ", In: " + data_in.header.frame_id);
    }
  }

  static void add_metadata(DataType & data_out, const DataType & data_in)
  {
    data_out.header = data_in.header;
  }
};

template <typename U>
struct FilterTraits<std::vector<U>>
{
  using StorageType = FilterVector<U>;
  using DataType = std::vector<U>;

  static void initialize(StorageType & storage) { (void)storage; }

  static bool is_finite(const StorageType & storage)
  {
    return std::all_of(
      storage.data.begin(), storage.data.end(), [](U val) { return std::isfinite(val); });
  }

  static bool is_finite(const DataType & storage)
  {
    return std::all_of(storage.begin(), storage.end(), [](U val) { return std::isfinite(val); });
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

  static void validate_input(
    const DataType & data_in, const StorageType & filtered_value, DataType & data_out)
  {
    if (data_in.size() != filtered_value.size())
    {
      throw std::runtime_error(
        fmt::format(
          "Input vector size ({}) does not match internal state size ({}).", data_in.size(),
          filtered_value.size()));
    }
    // Compare new input's size with output's size,
    // but only if it existed (filter_chains does not initialize output)
    if (!data_out.empty() && data_out.size() != data_in.size())
    {
      throw std::runtime_error(
        fmt::format(
          "Input and output vectors must be the same size, {} vs {}.", data_out.size(),
          data_in.size()));
    }
  }

  static void add_metadata(DataType & storage, const DataType & data_in)
  {
    (void)storage;
    (void)data_in;
  }
};

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__FILTER_TRAITS_HPP_
