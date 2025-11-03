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

#include <gmock/gmock.h>
#include "control_toolbox/tf_utils.hpp"

TEST(ApplyTFPrefixTest, UsesNamespaceWhenPrefixEmpty)
{
  EXPECT_EQ(control_toolbox::apply_tf_prefix("", "/ns", "odom"), "ns/odom");
}

TEST(ApplyTFPrefixTest, UsesExplicitPrefix)
{
  EXPECT_EQ(control_toolbox::apply_tf_prefix("robot", "/ns", "base"), "robot/base");
}

TEST(ApplyTFPrefixTest, NormalizesPrefixSlashes)
{
  EXPECT_EQ(control_toolbox::apply_tf_prefix("/robot1", "/ns", "link"), "robot1/link");
  EXPECT_EQ(control_toolbox::apply_tf_prefix("robot2//", "/ns", "odom"), "robot2//odom");
  EXPECT_EQ(control_toolbox::apply_tf_prefix("/robot3/", "/ns", "base_link"), "robot3/base_link");
  EXPECT_EQ(control_toolbox::apply_tf_prefix("/", "/ns", "odom"), "odom");
}

TEST(ApplyTFPrefixTest, EmptyPrefixAndNamespace)
{
  EXPECT_EQ(control_toolbox::apply_tf_prefix("", "", "odom"), "odom");
}

TEST(ApplyTFPrefixTest, FrameHasLeadingSlash)
{
  EXPECT_EQ(control_toolbox::apply_tf_prefix("robot", "/ns", "/odom"), "robot//odom");
}

TEST(ApplyTFPrefixTest, ComplexPrefixAndNamespace)
{
  EXPECT_EQ(control_toolbox::apply_tf_prefix("/robot/", "/my_ns/", "odom"), "robot/odom");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
