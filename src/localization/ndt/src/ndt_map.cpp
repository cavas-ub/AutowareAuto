// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <ndt/ndt_map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
uint32_t validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  // lambda to check the fields of a PointField
  auto field_valid = [](const auto & field, auto data_type, auto offset, auto count) {
      return (field.datatype == data_type) && (field.offset == offset) && (field.count == count);
    };

  auto ret = 0U;
  const auto double_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64));
  const auto uint_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::UINT32));

  // TODO(cvasfi): Possibly allow additional fields that don't change the order of the fields
  constexpr auto expected_num_fields = 10U;

  // Check general pc metadata
  if ((msg.fields.size()) != expected_num_fields ||
    (msg.point_step != (((expected_num_fields - 1U) * double_field_size) + 2 * uint_field_size)) ||
    (msg.height != 1U))
  {
    return 0U;
  }

  // check PointField fields
  // Get ID of the last field before cell_ID for reverse iterating. (used to calculate offset)
  auto double_field_idx = expected_num_fields - 2U;
  if (!std::all_of(msg.fields.rbegin() + 1U, msg.fields.rend(),  // check all float fields
    [&double_field_idx, &field_valid, double_field_size](auto & field) {
      return field_valid(field, sensor_msgs::msg::PointField::FLOAT64,
      ((double_field_idx--) * double_field_size), 1U);
    }) ||
    !field_valid(msg.fields[9U], sensor_msgs::msg::PointField::UINT32,  // check the cell id field
    (9U * double_field_size), 2U) )
  {
    return 0U;
  }

  // Check field names
  if ((msg.fields[0U].name != "x") ||
    (msg.fields[1U].name != "y") ||
    (msg.fields[2U].name != "z") ||
    (msg.fields[3U].name != "cov_xx") ||
    (msg.fields[4U].name != "cov_xy") ||
    (msg.fields[5U].name != "cov_xz") ||
    (msg.fields[6U].name != "cov_yy") ||
    (msg.fields[7U].name != "cov_yz") ||
    (msg.fields[8U].name != "cov_zz") ||
    (msg.fields[9U].name != "cell_id"))
  {
    return 0U;
  }


  // If the actual size and the meta data is in conflict, use the minimum length to be safe.
  const auto min_data_length = std::min(static_cast<decltype(msg.row_step)>(msg.data.size()),
      std::min(msg.row_step, msg.width * msg.point_step));
  // Trim the length to make it divisible to point_step, excess data cannot be read.
  const auto safe_data_length = min_data_length - (min_data_length % msg.point_step);
  // Return number of points that can safely be read from the point cloud
  ret = safe_data_length / msg.point_step;

  return ret;
}

GridLookupPattern::GridLookupPattern(const GridConfig & grid_config, float32_t radius)
: m_config{grid_config}
{
  const auto min_point =
    grid_config.centroid<Point>(grid_config.index(Point{-radius, -radius, -radius}));
  const auto max_point =
    grid_config.centroid<Point>(grid_config.index(Point{radius, radius, radius}));

  const auto diff_pt = max_point - min_point;
  const auto volume = diff_pt(0) * diff_pt(1) * diff_pt(2);

  m_base_pattern.reserve(volume);
  m_output_pattern.reserve(volume);

  constexpr auto tol = std::numeric_limits<Real>::epsilon();
  for (auto curr_x = min_point(0);
    (max_point(0) - curr_x) >= tol;
    curr_x += grid_config.get_voxel_size().x)
  {
    for (auto curr_y = min_point(1);
      (max_point(1) - curr_y) >= tol;
      curr_y += grid_config.get_voxel_size().y)
    {
      for (auto curr_z = min_point(2);
        (max_point(2) - curr_z) >= tol;
        curr_z += grid_config.get_voxel_size().z)
      {
        if ((std::sqrt(curr_x * curr_x + curr_y * curr_y + curr_z * curr_z) - radius) < tol) {
          m_base_pattern.insert(grid_config.index(Point{curr_x, curr_y, curr_z}));
        }
      }
    }
  }
}

const GridLookupPattern::Indexes & GridLookupPattern::lookup_indices(const Point & pt)
{
  m_output_pattern.clear();
  for (const auto idx : m_base_pattern) {
    m_output_pattern.insert(m_config.index(pt + m_config.centroid<Point>(idx)));
  }
  return m_output_pattern;
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
