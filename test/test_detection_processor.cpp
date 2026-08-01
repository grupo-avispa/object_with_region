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

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "object_with_region/detection_processor.hpp"

namespace
{

const std::vector<std::string> kLabels = {"person", "chair", "table"};  // NOLINT

vision_msgs::msg::Detection3D make_detection(const std::string & class_id)
{
  vision_msgs::msg::Detection3D detection;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis_with_pose;
  hypothesis_with_pose.hypothesis.class_id = class_id;
  hypothesis_with_pose.hypothesis.score = 0.9;
  detection.results.push_back(hypothesis_with_pose);
  detection.bbox.center.position.x = 1.0;
  detection.bbox.center.position.y = 2.0;
  detection.bbox.center.position.z = 3.0;
  return detection;
}

}  // namespace

TEST(BuildObjectRegion, ReturnsNulloptWhenNoResults)
{
  vision_msgs::msg::Detection3D detection;
  EXPECT_FALSE(object_with_region::detection_processor::build_object_region(detection, kLabels));
}

TEST(BuildObjectRegion, ReturnsNulloptOnNonNumericClassId)
{
  auto detection = make_detection("not_a_number");
  EXPECT_FALSE(object_with_region::detection_processor::build_object_region(detection, kLabels));
}

TEST(BuildObjectRegion, ReturnsNulloptOnOutOfRangeClassId)
{
  auto detection = make_detection("10");
  EXPECT_FALSE(object_with_region::detection_processor::build_object_region(detection, kLabels));
}

TEST(BuildObjectRegion, ReturnsNulloptOnNegativeClassId)
{
  auto detection = make_detection("-1");
  EXPECT_FALSE(object_with_region::detection_processor::build_object_region(detection, kLabels));
}

TEST(BuildObjectRegion, ResolvesClassNameAndPreservesClassIdOnHappyPath)
{
  auto detection = make_detection("1");

  auto object_region =
    object_with_region::detection_processor::build_object_region(detection, kLabels);

  ASSERT_TRUE(object_region.has_value());
  EXPECT_EQ(object_region->class_name, "chair");
  EXPECT_EQ(object_region->object.results[0].hypothesis.class_id, "1");
  EXPECT_EQ(object_region->region, "unknown");
  EXPECT_DOUBLE_EQ(object_region->object.bbox.center.position.x, 1.0);
}
