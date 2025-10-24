// Copyright (c) 2008, Willow Garage, Inc.
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

TEST(ApplyTFPrefixTest, DisabledPrefix)
{
  EXPECT_EQ(control_toolbox::applyTFPrefix(false, "", "/ns", "base_link"), "base_link");
}

TEST(ApplyTFPrefixTest, EmptyExplicitUsesNamespace)
{
  EXPECT_EQ(control_toolbox::applyTFPrefix(true, "", "/my_ns", "odom"), "my_ns/odom");
}

TEST(ApplyTFPrefixTest, ExplicitPrefixUsed)
{
  EXPECT_EQ(control_toolbox::applyTFPrefix(true, "robot1", "/ns", "base"), "robot1/base");
}

TEST(ApplyTFPrefixTest, LeadingSlashRemoved)
{
  EXPECT_EQ(control_toolbox::applyTFPrefix(true, "/robot2", "/ns", "link"), "robot2/link");
}

TEST(ApplyTFPrefixTest, TrailingSlashAdded)
{
  EXPECT_EQ(control_toolbox::applyTFPrefix(true, "robot3", "/ns", "odom"), "robot3/odom");
}

TEST(ApplyTFPrefixTest, BothSlashesNormalized)
{
  EXPECT_EQ(
    control_toolbox::applyTFPrefix(true, "/robot4/", "/ns", "base_link"), "robot4/base_link");
}

TEST(ApplyTFPrefixTest, NodeNamespaceWithSlash)
{
  EXPECT_EQ(control_toolbox::applyTFPrefix(true, "", "/robot_ns/", "odom"), "robot_ns/odom");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
