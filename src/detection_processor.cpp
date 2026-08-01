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

#include "object_with_region/detection_processor.hpp"

#include <exception>

namespace object_with_region
{
namespace detection_processor
{

std::optional<object_with_region::msg::ObjectRegion3D> build_object_region(
  const vision_msgs::msg::Detection3D & detection,
  const std::vector<std::string> & labels)
{
  if (detection.results.empty()) {
    return std::nullopt;
  }

  int class_index = 0;
  try {
    class_index = std::stoi(detection.results[0].hypothesis.class_id);
  } catch (const std::exception &) {
    return std::nullopt;
  }

  if (class_index < 0 || static_cast<std::size_t>(class_index) >= labels.size()) {
    return std::nullopt;
  }

  object_with_region::msg::ObjectRegion3D object_region;
  object_region.object = detection;
  object_region.object.results[0].hypothesis.class_id =
    labels[static_cast<std::size_t>(class_index)];
  object_region.region = "unknown";
  return object_region;
}

}  // namespace detection_processor
}  // namespace object_with_region
