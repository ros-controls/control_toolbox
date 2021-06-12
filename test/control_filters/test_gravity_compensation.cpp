// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_gravity_compensation.hpp"

TEST_F(GravityCompensationTest, TestGravityCompensation)
{
  node_->declare_parameter("world_frame", "world");
  node_->declare_parameter("sensor_frame", "sensor");
  node_->declare_parameter("CoG_x", 0.0);
  node_->declare_parameter("CoG_y", 0.0);
  node_->declare_parameter("CoG_z", 0.0);
  node_->declare_parameter("force", 50.0);

  ASSERT_TRUE(gravity_compensation_.configure());

  geometry_msgs::msg::WrenchStamped in, out;
  in.header.frame_id = "world";
  in.wrench.force.x = 1.0;
  in.wrench.torque.x = 10.0;

  ASSERT_TRUE(gravity_compensation_.update(in, out));

  ASSERT_EQ(out.wrench.force.x, 1.0);
  ASSERT_EQ(out.wrench.force.y, 0.0);
  ASSERT_EQ(out.wrench.force.z, 50.0);

  ASSERT_EQ(out.wrench.torque.x, 10.0);
  ASSERT_EQ(out.wrench.torque.y, 0.0);
  ASSERT_EQ(out.wrench.torque.z, 0.0);
}
