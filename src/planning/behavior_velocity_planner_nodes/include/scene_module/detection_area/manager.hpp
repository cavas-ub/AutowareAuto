// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENE_MODULE__DETECTION_AREA__MANAGER_HPP_
#define SCENE_MODULE__DETECTION_AREA__MANAGER_HPP_

#include <functional>
#include <memory>

#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "rclcpp/rclcpp.hpp"

#include "scene_module/detection_area/scene.hpp"
#include "scene_module/scene_module_interface.hpp"

namespace autoware
{
namespace planning
{
namespace behavior_velocity_planner_nodes
{
class DetectionAreaModuleManager : public SceneModuleManagerInterface
{
public:
  explicit DetectionAreaModuleManager(rclcpp::Node & node);

  const char * getModuleName() override {return "detection_area";}

private:
  DetectionAreaModule::PlannerParam planner_param_;
  void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;
};
}  // namespace behavior_velocity_planner_nodes
}  // namespace planning
}  // namespace autoware

#endif  // SCENE_MODULE__DETECTION_AREA__MANAGER_HPP_
