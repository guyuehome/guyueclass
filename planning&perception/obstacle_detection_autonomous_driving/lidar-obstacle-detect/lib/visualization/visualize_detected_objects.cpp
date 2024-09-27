/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#include "visualize_detected_objects.h"

VisualizeDetectedObjects::VisualizeDetectedObjects() : arrow_height_(0.5), label_height_(1.0)
{
  object_speed_threshold_ = 0.1;
  arrow_speed_threshold_ = 0.25;
  marker_display_duration_ =0.2; 

  std::vector<double> label_color{255., 255., 255., 1.0};
  label_color_ = ParseColor(label_color);
  // ROS_INFO("[%s] label_color: %s", __APP_NAME__, ColorToString(label_color_).c_str());

  std::vector<double> arrow_color{0.,255.,0.,0.8};
  arrow_color_ = ParseColor(arrow_color);
  // ROS_INFO("[%s] arrow_color: %s", __APP_NAME__, ColorToString(arrow_color_).c_str());

  std::vector<double> hull_color{51.,204.,51.,0.8};// orange
  hull_color_ = ParseColor(hull_color);
  // ROS_INFO("[%s] hull_color: %s", __APP_NAME__, ColorToString(hull_color_).c_str());

  // std::vector<double> freespace_color{0.,255.,0.,0.2}; // green
  // freespace_color = ParseColor(freespace_color);
  // ROS_INFO("[%s] freespace_color_: %s", __APP_NAME__, ColorToString(freespace_color_).c_str());


  std::vector<double> cube_color{51.,128.,204.,0.8};
  cube_color_ = ParseColor(cube_color);
  // ROS_INFO("[%s] box_color: %s", __APP_NAME__, ColorToString(box_color_).c_str());

  std::vector<double> model_color{190.,190.,190.,0.5};
  model_color_ = ParseColor(model_color);
  // ROS_INFO("[%s] model_color: %s", __APP_NAME__, ColorToString(model_color_).c_str());

  std::vector<double> centroid_color{77.,121.,255.,0.8};
  centroid_color_ = ParseColor(centroid_color);
  // ROS_INFO("[%s] centroid_color: %s", __APP_NAME__, ColorToString(centroid_color_).c_str());

}

std::string VisualizeDetectedObjects::ColorToString(const std_msgs::ColorRGBA &in_color)
{
  std::stringstream stream;

  stream << "{R:" << std::fixed << std::setprecision(1) << in_color.r*255 << ", ";
  stream << "G:" << std::fixed << std::setprecision(1) << in_color.g*255 << ", ";
  stream << "B:" << std::fixed << std::setprecision(1) << in_color.b*255 << ", ";
  stream << "A:" << std::fixed << std::setprecision(1) << in_color.a << "}";
  return stream.str();
}

float VisualizeDetectedObjects::CheckColor(double value)
{
  float final_value;
  if (value > 255.)
    final_value = 1.f;
  else if (value < 0)
    final_value = 0.f;
  else
    final_value = value/255.f;
  return final_value;
}

float VisualizeDetectedObjects::CheckAlpha(double value)
{
  float final_value;
  if (value > 1.)
    final_value = 1.f;
  else if (value < 0.1)
    final_value = 0.1f;
  else
    final_value = value;
  return final_value;
}

std_msgs::ColorRGBA VisualizeDetectedObjects::ParseColor(const std::vector<double> &in_color)
{
  std_msgs::ColorRGBA color;
  float r,g,b,a;
  if (in_color.size() == 4) //r,g,b,a
  {
    color.r = CheckColor(in_color[0]);
    color.g = CheckColor(in_color[1]);
    color.b = CheckColor(in_color[2]);
    color.a = CheckAlpha(in_color[3]);
  }
  return color;
}

void VisualizeDetectedObjects::DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers, arrow_markers, centroid_markers, polygon_hulls, bounding_cubes,bounding_boxes,
                                  object_models;

  visualization_msgs::MarkerArray visualization_markers;

  marker_id_ = 0;

  label_markers = ObjectsToLabels(in_objects);
  arrow_markers = ObjectsToArrows(in_objects);
  polygon_hulls = ObjectsToHulls(in_objects);
  bounding_cubes = ObjectsToCubes(in_objects);
  bounding_boxes = ObjectsToBoxes(in_objects);
  object_models = ObjectsToModels(in_objects);
  centroid_markers = ObjectsToCentroids(in_objects);

  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       label_markers.markers.begin(), label_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       arrow_markers.markers.begin(), arrow_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       polygon_hulls.markers.begin(), polygon_hulls.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       bounding_cubes.markers.begin(), bounding_cubes.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       bounding_boxes.markers.begin(), bounding_boxes.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       object_models.markers.begin(), object_models.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       centroid_markers.markers.begin(), centroid_markers.markers.end());

  publisher_markers_.publish(visualization_markers);

}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToCentroids(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray centroid_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
    {
      visualization_msgs::Marker centroid_marker;
      centroid_marker.lifetime = ros::Duration(marker_display_duration_);

      centroid_marker.header = in_objects.header;
      centroid_marker.type = visualization_msgs::Marker::SPHERE;
      centroid_marker.action = visualization_msgs::Marker::ADD;
      centroid_marker.pose = object.pose;
      centroid_marker.ns = ros_namespace_ + "centroid_markers";

      centroid_marker.scale.x = 0.5;
      centroid_marker.scale.y = 0.5;
      centroid_marker.scale.z = 0.5;

      if (object.color.a == 0)
      {
        centroid_marker.color = centroid_color_;
      }
      else
      {
        centroid_marker.color = object.color;
      }
      centroid_marker.id = marker_id_++;
      centroid_markers.markers.push_back(centroid_marker);
    }
  }
  return centroid_markers;
}//ObjectsToCentroids

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToCubes(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray object_boxes;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
      (object.pose_reliable || object.label == "unknown") &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      visualization_msgs::Marker box;
      box.lifetime = ros::Duration(marker_display_duration_);
      box.header = in_objects.header;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.ns = ros_namespace_ + "cube_markers";
      box.id = marker_id_++;
      box.scale = object.dimensions;
      box.pose.position = object.pose.position;

      // if (object.pose_reliable)
      box.pose.orientation = object.pose.orientation;

      if (object.color.a == 0)
      {
        box.color = cube_color_;
      }
      else
      {
        box.color = object.color;
      }

      object_boxes.markers.push_back(box);
    }
  }
  return object_boxes;
}//ObjectsToCubes


visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray object_boxes;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
      (object.pose_reliable || object.label == "unknown") &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      visualization_msgs::Marker box;
      box.lifetime = ros::Duration(marker_display_duration_); // marker_display_duration_
      box.header = in_objects.header;
      box.type = visualization_msgs::Marker::LINE_STRIP;
      // box.type =  visualization_msgs::Marker::LINE_LIST;
      box.action = visualization_msgs::Marker::ADD;
      box.ns = ros_namespace_ + "box_markers";
      box.id = marker_id_++;
      box.scale.x = 0.15;   
      
      box.color.r = 128.0f;
      box.color.g = 255.0f;
      box.color.b = 255.0f;
      box.color.a = 1.0;     


      // 包围盒自身坐标系坐标， dimensions是关于包围盒的长宽高
      geometry_msgs::Point p[8];
      p[0].x = object.dimensions.x / 2; 
      p[0].y = object.dimensions.y / 2; 
      p[0].z = object.dimensions.z / 2; 

      p[1].x = object.dimensions.x / 2; 
      p[1].y = - object.dimensions.y / 2; 
      p[1].z = object.dimensions.z / 2;

      p[2].x = object.dimensions.x / 2;
      p[2].y = - object.dimensions.y / 2;
      p[2].z = - object.dimensions.z / 2;

      p[3].x = object.dimensions.x / 2;
      p[3].y = object.dimensions.y / 2;
      p[3].z = - object.dimensions.z / 2;

      p[4].x = - object.dimensions.x / 2;
      p[4].y = object.dimensions.y / 2;
      p[4].z = - object.dimensions.z / 2;

      p[5].x = - object.dimensions.x / 2;
      p[5].y = - object.dimensions.y / 2;
      p[5].z = - object.dimensions.z / 2;

      p[6].x = - object.dimensions.x / 2;
      p[6].y = - object.dimensions.y / 2;
      p[6].z = object.dimensions.z / 2;

      p[7].x = - object.dimensions.x / 2; 
      p[7].y =  object.dimensions.y / 2; 
      p[7].z = object.dimensions.z / 2;  

       
      // if (object.pose_reliable){
      if(true){
      // // 方法1: 欧拉角求旋转平移矩阵
      // float angle = object.pose.orientation;
      // float cos_ = cos(angle);
      // float sin_ = sin(angle);
      // float RT[12] = {
      //   cos_, -sin_, 0., object.pose.position.x,
      //   sin_, cos_,  0., object.pose.position.y,
      //   0.,   0.,    1.,  object.pose.position.z 
      // };
      // // 方法2: 四元数求旋转平移矩阵
          float x = object.pose.orientation.x; // pose.orientation表示方向，四元数的格式
          float y = object.pose.orientation.y;
          float z = object.pose.orientation.z;
          float w = object.pose.orientation.w;
          float RT[12] = {
             1-2*y*y-2*z*z,  2*x*y-2*z*w,    2*x*z+2*y*w,    float(object.pose.position.x), // pose.position表示包围盒中心点坐标
             2*x*y+2*z*w,    1-2*x*x-2*z*z,  2*y*z-2*x*w,    float(object.pose.position.y),
             2*x*z-2*y*w,    2*y*z+2*x*w,    1-2*x*x-2*y*y,  float(object.pose.position.z) };        
          for(int i=0;i<8;i++)
          {
              float x = RT[0] * p[i].x + RT[1] * p[i].y +  RT[2] * p[i].z + RT[3];
              float y = RT[4] * p[i].x + RT[5] * p[i].y +  RT[6] * p[i].z + RT[7];
              float z = RT[8] * p[i].x + RT[9] * p[i].y +  RT[10] * p[i].z + RT[11];
              p[i].x = x;
              p[i].y = y;
              p[i].z = z;
          }    
      }
      else
      {
         for (size_t j = 0; j < 8; j++)
        {
          p[j].x +=  object.pose.position.x;
          p[j].y +=  object.pose.position.y;
          p[j].z +=  object.pose.position.z;
        }
      }          
      for (size_t i = 0; i < 8; i++)
      {
        box.points.push_back(p[i]);
      }
      box.points.push_back(p[0]);
      box.points.push_back(p[3]);
      box.points.push_back(p[2]);
      box.points.push_back(p[5]);
      box.points.push_back(p[6]);
      box.points.push_back(p[1]);
      box.points.push_back(p[0]);
      box.points.push_back(p[7]);
      box.points.push_back(p[4]);
      object_boxes.markers.push_back(box);
    }
  }
  return object_boxes;
}//ObjectsToBoxes


visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToModels(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray object_models;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) &&
      object.label != "unknown" &&
        (object.dimensions.x + object.dimensions.y + object.dimensions.z) < object_max_linear_size_)
    {
      visualization_msgs::Marker model;

      model.lifetime = ros::Duration(marker_display_duration_);
      model.header = in_objects.header;
      model.type = visualization_msgs::Marker::MESH_RESOURCE;
      model.action = visualization_msgs::Marker::ADD;
      model.ns = ros_namespace_ + "model_markers";
      model.mesh_use_embedded_materials = false;
      model.color = model_color_;
      if(object.label == "car")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/car.dae";
      }
      else if (object.label == "person")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/person.dae";
      }
      else if (object.label == "bicycle" || object.label == "bike")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/bike.dae";
      }
      else if (object.label == "bus")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/bus.dae";
      }
      else if(object.label == "truck")
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/truck.dae";
      }
      else
      {
        model.mesh_resource = "package://detected_objects_visualizer/models/box.dae";
      }
      model.scale.x = 1;
      model.scale.y = 1;
      model.scale.z = 1;
      model.id = marker_id_++;
      model.pose.position = object.pose.position;
      model.pose.position.z-= object.dimensions.z/2;

      if (object.pose_reliable)
        model.pose.orientation = object.pose.orientation;

      object_models.markers.push_back(model);
    }
  }
  return object_models;
}//ObjectsToModels

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToHulls(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray polygon_hulls;

  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty() && object.label == "unknown")
    {
      visualization_msgs::Marker hull;
      hull.lifetime = ros::Duration(marker_display_duration_);
      hull.header = in_objects.header;
      hull.type = visualization_msgs::Marker::LINE_STRIP;
      hull.action = visualization_msgs::Marker::ADD;
      hull.ns = ros_namespace_ + "hull_markers";
      hull.id = marker_id_++;
      hull.scale.x = 0.15;

      for(auto const &point: object.convex_hull.polygon.points)
      {
        geometry_msgs::Point tmp_point;
        tmp_point.x = point.x;
        tmp_point.y = point.y;
        tmp_point.z = point.z;
        hull.points.push_back(tmp_point);
      }

      if (object.color.a == 0)
      {
        hull.color = hull_color_;
      }
      else
      {
        hull.color = object.color;
      }

      polygon_hulls.markers.push_back(hull);
    }
  }
  return polygon_hulls;
}

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToArrows(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray arrow_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object) && object.pose_reliable)
    {
      // std::cout << " 333333" << std::endl;
      double velocity = object.velocity.linear.x;

      if (abs(velocity) >= arrow_speed_threshold_)
      {
        visualization_msgs::Marker arrow_marker;
        arrow_marker.lifetime = ros::Duration(marker_display_duration_);

        tf::Quaternion q(object.pose.orientation.x,
                         object.pose.orientation.y,
                         object.pose.orientation.z,
                         object.pose.orientation.w);
        double roll, pitch, yaw;

        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // in the case motion model fit opposite direction
        if (velocity < -0.1)
        {
          yaw += M_PI;
          // normalize angle
          while (yaw > M_PI)
            yaw -= 2. * M_PI;
          while (yaw < -M_PI)
            yaw += 2. * M_PI;
        }

        tf::Matrix3x3 obs_mat;
        tf::Quaternion q_tf;

        obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
        obs_mat.getRotation(q_tf);

        arrow_marker.header = in_objects.header;
        arrow_marker.ns = ros_namespace_ + "arrow_markers";
        arrow_marker.action = visualization_msgs::Marker::ADD;
        arrow_marker.type = visualization_msgs::Marker::ARROW;

        // green
        if (object.color.a == 0)
        {
          arrow_marker.color = arrow_color_;
        }
        else
        {
          arrow_marker.color = object.color;
        }
        arrow_marker.id = marker_id_++;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        arrow_marker.pose.position.x = object.pose.position.x;
        arrow_marker.pose.position.y = object.pose.position.y;
        arrow_marker.pose.position.z = arrow_height_;

        arrow_marker.pose.orientation.x = q_tf.getX();
        arrow_marker.pose.orientation.y = q_tf.getY();
        arrow_marker.pose.orientation.z = q_tf.getZ();
        arrow_marker.pose.orientation.w = q_tf.getW();

        // Set the scale of the arrow -- 1x1x1 here means 1m on a side
        arrow_marker.scale.x = 3;
        arrow_marker.scale.y = 0.1;
        arrow_marker.scale.z = 0.1;

        arrow_markers.markers.push_back(arrow_marker);
      }//velocity threshold
    }//valid object
  }//end for
  return arrow_markers;
}//ObjectsToArrows

visualization_msgs::MarkerArray
VisualizeDetectedObjects::ObjectsToLabels(const autoware_msgs::DetectedObjectArray &in_objects)
{
  visualization_msgs::MarkerArray label_markers;
  for (auto const &object: in_objects.objects)
  {
    if (IsObjectValid(object))
    {
      visualization_msgs::Marker label_marker;

      label_marker.lifetime = ros::Duration(marker_display_duration_);
      label_marker.header = in_objects.header;
      label_marker.ns = ros_namespace_ + "label_markers";
      label_marker.action = visualization_msgs::Marker::ADD;
      label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label_marker.scale.x = 1.5;
      label_marker.scale.y = 1.5;
      label_marker.scale.z = 1.5;

      label_marker.color = label_color_;

      label_marker.id = marker_id_++;

      if(!object.label.empty() && object.label != "unknown")
        label_marker.text = object.label + " "; //Object Class if available

      std::stringstream distance_stream;
      distance_stream << std::fixed << std::setprecision(1)
                      << sqrt((object.pose.position.x * object.pose.position.x) +
                                (object.pose.position.y * object.pose.position.y));
      std::string distance_str = distance_stream.str() + " m";
      label_marker.text += distance_str;

      if (object.velocity_reliable)
      {
        double velocity = object.velocity.linear.x;
        if (velocity < -0.1)
        {
          velocity *= -1;
        }

        if (abs(velocity) < object_speed_threshold_)
        {
          velocity = 0.0;
        }

        tf::Quaternion q(object.pose.orientation.x, object.pose.orientation.y,
                         object.pose.orientation.z, object.pose.orientation.w);

        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // convert m/s to km/h
        std::stringstream kmh_velocity_stream;
        kmh_velocity_stream << std::fixed << std::setprecision(1) << (velocity * 3.6);
        std::string text = "\n<" + std::to_string(object.id) + "> " + kmh_velocity_stream.str() + " km/h";
        label_marker.text += text;
      }

      label_marker.pose.position.x = object.pose.position.x;
      label_marker.pose.position.y = object.pose.position.y;
      label_marker.pose.position.z = label_height_;
      label_marker.scale.z = 1.0;
      if (!label_marker.text.empty())
        label_markers.markers.push_back(label_marker);
    }
  }  // end in_objects.objects loop

  return label_markers;
}//ObjectsToLabels

bool VisualizeDetectedObjects::IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      std::isnan(in_object.pose.orientation.x) ||
      std::isnan(in_object.pose.orientation.y) ||
      std::isnan(in_object.pose.orientation.z) ||
      std::isnan(in_object.pose.orientation.w) ||
      std::isnan(in_object.pose.position.x) ||
      std::isnan(in_object.pose.position.y) ||
      std::isnan(in_object.pose.position.z) ||
      (in_object.pose.position.x == 0.) ||
      (in_object.pose.position.y == 0.) ||
      (in_object.dimensions.x <= 0.) ||
      (in_object.dimensions.y <= 0.) ||
      (in_object.dimensions.z <= 0.)
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid


void VisualizeDetectedObjects::visualizeDetectedObjs(const autoware_msgs::DetectedObjectArray &in_objects, 
                          visualization_msgs::MarkerArray &visualization_markers)
{
  visualization_msgs::MarkerArray label_markers, arrow_markers, centroid_markers, polygon_hulls, bounding_cubes, bounding_boxes, object_models, polygon_freespace;
  marker_id_ = 0;

  label_markers = ObjectsToLabels(in_objects);
  arrow_markers = ObjectsToArrows(in_objects);
  polygon_hulls = ObjectsToHulls(in_objects);
  bounding_cubes = ObjectsToCubes(in_objects);
  bounding_boxes = ObjectsToBoxes(in_objects);
  centroid_markers = ObjectsToCentroids(in_objects);

  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       label_markers.markers.begin(), label_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       arrow_markers.markers.begin(), arrow_markers.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       polygon_hulls.markers.begin(), polygon_hulls.markers.end());
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       bounding_cubes.markers.begin(), bounding_cubes.markers.end());  
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       bounding_boxes.markers.begin(), bounding_boxes.markers.end());                                        
  visualization_markers.markers.insert(visualization_markers.markers.end(),
                                       centroid_markers.markers.begin(), centroid_markers.markers.end());

  // publisher_markers_.publish(visualization_markers);
}