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

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace control_toolbox
{
template <typename T>
class ExponentialFilter
{
public:
  //Default constructor
  ExponentialFilter();
  ExponentialFilter(double alpha)
  {
    set_alpha(alpha);
  }

  ~ExponentialFilter();

  bool configure();

  bool update (const T & data_in, T & data_out);

  bool is_configured() const { return configured_;}

  void set_alpha(double alpha);

  private:

  double alpha_;

  // Define the storage type based on T
  using Traits = FilterTraits<T>;
  using StorageType = typename Traits::StorageType;

  StorageType input_value,output_value,old_value;

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
  Traits::initialize(output_value);
  Traits::initialize(input_value);
  Traits::initialize(old_value);

  return configured_ = true;
}

template <typename T>
void ExponentialFilter<T>::set_alpha(double alpha)
{
  alpha_ = alpha;
}

template <typename T>
bool ExponentialFilter<T>::update(const T & data_in, T & data_out)
{
  if (!configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // First call: initialize filter state
  if (Traits::is_nan(output_value) || Traits::is_empty(output_value))
  {
    if (!Traits::is_finite(data_in))
    {
      return false;
    }
    
    Traits::assign(output_value, data_in);  // Initialize with first input
  }
  else
  {
    Traits::validate_input(data_in, output_value, data_out);
  }

  // Exponential filter update: y[n] = α * x[n] + (1-α) * y[n-1]
  output_value = alpha_ * data_in + (1.0 - alpha_) * output_value;
  
  Traits::assign(data_out, output_value);
  Traits::add_metadata(data_out, data_in);

  return true;

}

} // namespace control_toolbox

#endif  // CONTROL_TOOLBOX__EXPONENTIAL_FILTER_HPP_
