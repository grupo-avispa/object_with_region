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

#ifndef OBJECT_WITH_REGION__OBJECT_WITH_REGION_HPP_
#define OBJECT_WITH_REGION__OBJECT_WITH_REGION_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/label_info.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "object_with_region/msg/object_region3_d_array.hpp"
#include "semantic_navigation_msgs/srv/get_region_name.hpp"

namespace object_with_region
{

class ObjectWithRegionNode : public rclcpp::Node
{
public:
  ObjectWithRegionNode();
  ~ObjectWithRegionNode();

private:
  /**
   * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param default_value Parameter value to initialize with
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }

  // Callback for detections 3D subscriber
  void detection_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  // Get parameters from parameter server
  void get_params();

  // Call the service to get the region name from a position
  std::string client_call(const geometry_msgs::msg::PointStamped & position);

  // Callback for label info subscriber
  void label_info_callback(const vision_msgs::msg::LabelInfo::SharedPtr info);

  // Detections 3D subscriber
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;

  /// Subscriber of the meta-information about the label info.
  rclcpp::Subscription<vision_msgs::msg::LabelInfo>::SharedPtr label_info_sub_;

  // Objects with region publisher
  rclcpp::Publisher<object_with_region::msg::ObjectRegion3DArray>::SharedPtr object_with_region_pub_;

  // Client to get region name from position
  rclcpp::Client<semantic_navigation_msgs::srv::GetRegionName>::SharedPtr get_region_name_client_;

  // The callback group for info subscribers
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  // Topic of the detected objects in 3 dimensions.
  std::string detections_3d_topic_;

  // Topic of the label info.
  std::string label_info_topic_;

  // Topic to publish the detected objects with their associated region.
  std::string objects_with_region_topic_;

  // Service name to get the region name from a position.
  std::string get_region_name_service_;

  // Flag to enable/disable getting region info
  bool get_region_enabled_;

  /// Class labels of the neural network.
  std::vector<std::string> labels_;

};

} // namespace object_with_region

#endif  // OBJECT_WITH_REGION__OBJECT_WITH_REGION_HPP_
