// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include "vision_msgs/BoundingBox2D.h"
#include "vision_msgs/BoundingBox3D.h"
#include "vision_msgs/create_aabb.h"

TEST(vision_msgs, CreateAABB2D)
{
  vision_msgs::BoundingBox2D bbox = vision_msgs::createAABB2D(1,2,3,4);
  EXPECT_FLOAT_EQ(bbox.center.x, 2.5);  // 1 + 3/2
  EXPECT_FLOAT_EQ(bbox.center.y, 4);    // 2 + 4/2
  EXPECT_EQ(bbox.size_x, 3);
  EXPECT_EQ(bbox.size_y, 4);
  EXPECT_EQ(bbox.center.theta, 0);
}

TEST(vision_msgs, CreateAABB3D)
{
  vision_msgs::BoundingBox3D bbox = vision_msgs::createAABB3D(1,2,3,4,5,6);
  EXPECT_FLOAT_EQ(bbox.center.position.x, 3);   // 1 + 4/2
  EXPECT_FLOAT_EQ(bbox.center.position.y, 4.5); // 2 + 5/2
  EXPECT_FLOAT_EQ(bbox.center.position.z, 6);   // 3 + 6/2
  EXPECT_EQ(bbox.center.orientation.x, 0);
  EXPECT_EQ(bbox.center.orientation.y, 0);
  EXPECT_EQ(bbox.center.orientation.z, 0);
  EXPECT_EQ(bbox.center.orientation.w, 1);
  EXPECT_EQ(bbox.size.x, 4);
  EXPECT_EQ(bbox.size.y, 5);
  EXPECT_EQ(bbox.size.z, 6);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}