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
