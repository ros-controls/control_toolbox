// Copyright (c) 2024, AIT Austrian Institute of Technology GmbH
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

#include <gmock/gmock.h>
#include <memory>
#include <string>

#include "rclcpp/utilities.hpp"
#include "pluginlib/class_loader.hpp"

#include "control_filters/exponential_filter.hpp"

TEST(TestLoadExponentialFilter, load_exponential_filter_double)
{
  rclcpp::init(0, nullptr);

  pluginlib::ClassLoader<filters::FilterBase<double>> filter_loader("filters",
                                                                    "filters::FilterBase<double>");
  std::shared_ptr<filters::FilterBase<double>> filter;
  auto available_classes = filter_loader.getDeclaredClasses();
  std::stringstream sstr;
  sstr << "available filters:" << std::endl;
  for (const auto& available_class : available_classes)
  {
    sstr << "  " << available_class << std::endl;
  }

  std::string filter_type = "control_filters/ExponentialFilterDouble";
  ASSERT_TRUE(filter_loader.isClassAvailable(filter_type)) << sstr.str();
  EXPECT_NO_THROW(filter = filter_loader.createSharedInstance(filter_type)) << sstr.str();

  rclcpp::shutdown();
}
