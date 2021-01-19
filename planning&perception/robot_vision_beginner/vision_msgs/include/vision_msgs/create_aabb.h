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

#ifndef VISION_MSGS_BUILD_BBOX_H_
#define VISION_MSGS_BUILD_BBOX_H_

#include "vision_msgs/BoundingBox2D.h"
#include "vision_msgs/BoundingBox3D.h"

namespace vision_msgs
{
  /**
   * Create an axis-aligned bounding box (AABB) given the upper-left corner,
   * width, and height. This allows easy conversion from the OpenCV rectangle
   * representation.
   */
  static inline BoundingBox2D createAABB2D(uint32_t left,
                                           uint32_t top,
                                           uint32_t width,
                                           uint32_t height)
  {
    BoundingBox2D bbox;

    bbox.center.x = left + width/2.0;
    bbox.center.y = top + height/2.0;
    bbox.size_x = width;
    bbox.size_y = height;

    return bbox;
  }

  /**
   * Create an axis-aligned bounding box (AABB) given the upper-left-front
   * corner, width, height, and depth. This allows easy conversion from the
   * OpenCV rectangle representation.
   */
  static inline BoundingBox3D createAABB3D(uint32_t min_x,
                                           uint32_t min_y,
                                           uint32_t min_z,
                                           uint32_t size_x,
                                           uint32_t size_y,
                                           uint32_t size_z)
  {
    BoundingBox3D bbox;

    bbox.center.position.x = min_x + size_x/2.0;
    bbox.center.position.y = min_y + size_y/2.0;
    bbox.center.position.z = min_z + size_z/2.0;
    bbox.center.orientation.w = 1;
    bbox.size.x = size_x;
    bbox.size.y = size_y;
    bbox.size.z = size_z;

    return bbox;
  }
}

#endif