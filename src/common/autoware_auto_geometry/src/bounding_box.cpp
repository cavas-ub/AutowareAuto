// Copyright 2017-2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <geometry/bounding_box/bounding_box_common.hpp>
#include <geometry/bounding_box/rotating_calipers.hpp>
#include <geometry/bounding_box/eigenbox_2d.hpp>
#include <geometry/bounding_box/lfit.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <algorithm>
#include <limits>
#include <list>
#include <utility>
#include <vector>

namespace autoware
{
namespace common
{
namespace geometry
{
namespace bounding_box
{
namespace details
{
////////////////////////////////////////////////////////////////////////////////
geometry_msgs::msg::Point32 size_2d(
  const decltype(Polygon::points) & corners)
{
  geometry_msgs::msg::Point32 ret;
  ret.x = std::max(
    norm_2d(
      minus_2d(
        corners[1U],
        corners[0U])), std::numeric_limits<float32_t>::epsilon());
  ret.y = std::max(
    norm_2d(
      minus_2d(
        corners[2U],
        corners[1U])), std::numeric_limits<float32_t>::epsilon());
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
void finalize_box(const decltype(Polygon::points) & corners, DetectedObject & box)
{
  (void)std::copy(&corners[0U], &corners[corners.size()], &box.shape.polygon.points[0U]);
  // orientation: this is robust to lines
  const float32_t yaw_rad_2 =
    atan2f(box.shape.polygon.points[2U].y - box.shape.polygon.points[1U].y, box.shape.polygon.points[2U].x - box.shape.polygon.points[1U].x) * 0.5F;
  box.kinematics.orientation.w = cosf(yaw_rad_2);
  box.kinematics.orientation.z = sinf(yaw_rad_2);
  // centroid
  auto tmp_point = times_2d(plus_2d(corners[0U], corners[2U]), 0.5F);
  box.kinematics.centroid_position.x = tmp_point.x;
  box.kinematics.centroid_position.y = tmp_point.y;
  box.kinematics.centroid_position.z = tmp_point.z;
}


autoware_auto_msgs::msg::Shape make_shape(const DetectedObject & box)
{
  autoware_auto_msgs::msg::Shape ret;
  for (auto corner_pt : box.shape.polygon.points) {
    // NOTE(esteve): commented out because DetectedObject does not have a size field
    corner_pt.z -= box.height;
    ret.polygon.points.push_back(corner_pt);
  }

  // NOTE(esteve): commented out because DetectedObject does not have a size field
  ret.height = 2.0F * box.height;
  return ret;
}

autoware_auto_msgs::msg::DetectedObject make_detected_object(const DetectedObject & box)
{
  autoware_auto_msgs::msg::DetectedObject ret;

  ret.kinematics.centroid_position.x = static_cast<double>(box.kinematics.centroid_position.x);
  ret.kinematics.centroid_position.y = static_cast<double>(box.kinematics.centroid_position.y);
  ret.kinematics.centroid_position.z = static_cast<double>(box.kinematics.centroid_position.z);

  ret.shape = make_shape(box);

  ret.existence_probability = 1.0F;

  autoware_auto_msgs::msg::ObjectClassification label;
  label.classification = autoware_auto_msgs::msg::ObjectClassification::UNKNOWN;
  label.probability = 1.0F;
  ret.classification.emplace_back(std::move(label));

  return ret;
}

}  // namespace details
///////////////////////////////////////////////////////////////////////////////
// precompilation
using autoware::common::types::PointXYZIF;
template DetectedObject minimum_area_bounding_box<PointXYZIF>(std::list<PointXYZIF> & list);
template DetectedObject minimum_perimeter_bounding_box<PointXYZIF>(std::list<PointXYZIF> & list);
using PointXYZIFVIT = std::vector<PointXYZIF>::iterator;
template DetectedObject eigenbox_2d<PointXYZIFVIT>(const PointXYZIFVIT begin, const PointXYZIFVIT end);
template DetectedObject lfit_bounding_box_2d<PointXYZIFVIT>(
  const PointXYZIFVIT begin, const PointXYZIFVIT end);
using geometry_msgs::msg::Point32;
template DetectedObject minimum_area_bounding_box<Point32>(std::list<Point32> & list);
template DetectedObject minimum_perimeter_bounding_box<Point32>(std::list<Point32> & list);
using Point32VIT = std::vector<Point32>::iterator;
template DetectedObject eigenbox_2d<Point32VIT>(const Point32VIT begin, const Point32VIT end);
template DetectedObject lfit_bounding_box_2d<Point32VIT>(const Point32VIT begin, const Point32VIT end);
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware
