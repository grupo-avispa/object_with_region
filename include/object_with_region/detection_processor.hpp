// Copyright (c) 2024 Óscar Pons Fernández
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef OBJECT_WITH_REGION__DETECTION_PROCESSOR_HPP_
#define OBJECT_WITH_REGION__DETECTION_PROCESSOR_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vision_msgs/msg/detection3_d.hpp"

#include "object_with_region/msg/object_region3_d.hpp"

namespace object_with_region
{
namespace detection_processor
{

/**
 * @brief Validate a 3D detection and resolve its class name from the known labels.
 *
 * Pure function with no ROS I/O, so it can be unit tested directly. Leaves
 * `region` set to "unknown" and does not set the output header; callers are
 * expected to fill in the region (e.g. from a service call) and the header.
 *
 * @param detection Input 3D detection.
 * @param labels Known class labels, indexed by the detection's numeric class_id.
 * @return The resolved ObjectRegion3D, or std::nullopt if `detection` has no
 * results, or its class_id is not a valid numeric index into `labels`.
 */
std::optional<object_with_region::msg::ObjectRegion3D> build_object_region(
  const vision_msgs::msg::Detection3D & detection,
  const std::vector<std::string> & labels);

}  // namespace detection_processor
}  // namespace object_with_region

#endif  // OBJECT_WITH_REGION__DETECTION_PROCESSOR_HPP_
