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

#ifndef CONTROL_TOOLBOX__EXPONENTIAL_FILTER_HPP_
#define CONTROL_TOOLBOX__EXPONENTIAL_FILTER_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "control_toolbox/filter_traits.hpp"
#include "control_toolbox/filters.hpp"

namespace control_toolbox
{
template <typename T>
class ExponentialFilter
{
public:
  // Default constructor
  ExponentialFilter();
  explicit ExponentialFilter(double alpha) { set_params(alpha); }

  ~ExponentialFilter();

  bool configure();

  bool update(const T & data_in, T & data_out);

  bool is_configured() const { return configured_; }

  void set_params(double alpha) { alpha_ = alpha; }

private:
  double alpha_;

  // Define the storage type based on T
  using Traits = FilterTraits<T>;
  using StorageType = typename Traits::StorageType;

  StorageType old_value_;

  bool configured_ = false;
};

template <typename T>
ExponentialFilter<T>::ExponentialFilter() : alpha_(0.5)
{
}

template <typename T>
ExponentialFilter<T>::~ExponentialFilter()
{
}

template <typename T>
bool ExponentialFilter<T>::configure()
{
  Traits::initialize(old_value_);

  return configured_ = true;
}

template <typename T>
bool ExponentialFilter<T>::update(const T & data_in, T & data_out)
{
  if (!configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // First call: initialize filter state
  if (Traits::is_nan(old_value_) || Traits::is_empty(old_value_))
  {
    if (!Traits::is_finite(data_in))
    {
      return false;
    }
    Traits::assign(old_value_, data_in);
  }
  else
  {
    Traits::validate_input(data_in, old_value_, data_out);
  }

  // Convert data_in to StorageType for arithmetic
  StorageType storage_in;
  Traits::assign(storage_in, data_in);

  // Exponential filter update using the templated exponentialSmoothing function
  old_value_ = filters::exponentialSmoothing(storage_in, old_value_, alpha_);

  Traits::assign(data_out, old_value_);
  Traits::add_metadata(data_out, data_in);

  return true;
}

}  // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__EXPONENTIAL_FILTER_HPP_
