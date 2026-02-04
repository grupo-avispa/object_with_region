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

#include "rclcpp/version.h"
#include "nav2_util/string_utils.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "object_with_region/msg/object_region3_d.hpp"
#include "object_with_region/object_with_region.hpp"


// For Kilted compatibility in Message Filters API change
#if RCLCPP_VERSION_GTE(29, 6, 0)
#include "nav2_ros_common/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
using nav2::declare_parameter_if_not_declared;
// For Humble and Older compatibility in Message Filters API change
#else
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
using nav2_util::declare_parameter_if_not_declared;
#endif

using std::placeholders::_1;

namespace object_with_region
{

ObjectWithRegionNode::ObjectWithRegionNode()
: Node("object_with_region_node")
{
    // Get parameters from parameter server
  get_params();

    // Initialize transform buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create callback group for subscribers
  sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_cb_group_;

    // Create detections 3D subscriber
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        detections_3d_topic_, 10,
        std::bind(&ObjectWithRegionNode::detection_callback, this, _1),
        sub_options);

    // Create label info subscriber
  label_info_sub_ = this->create_subscription<vision_msgs::msg::LabelInfo>(
                    label_info_topic_,
                    10,
                    std::bind(&ObjectWithRegionNode::label_info_callback, this,
      std::placeholders::_1),
        sub_options);


    // Create objects with region publisher
  object_with_region_pub_ = this->create_publisher<object_with_region::msg::ObjectRegion3DArray>(
        objects_with_region_topic_, 10);

  if(get_region_enabled_) {
      // Create client to get region name from position
    get_region_name_client_ = this->create_client<semantic_navigation_msgs::srv::GetRegionName>(
          get_region_name_service_);
  }
}

ObjectWithRegionNode::~ObjectWithRegionNode()
{
}

void ObjectWithRegionNode::label_info_callback(const vision_msgs::msg::LabelInfo::SharedPtr info)
{
  for (const auto & l_class: info->class_map) {
    labels_.push_back(l_class.class_name);
  }
  RCLCPP_INFO(this->get_logger(), "Received label info with %ld classes", labels_.size());
  label_info_sub_.reset();
}


void ObjectWithRegionNode::detection_callback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  // This callbacks needs labels_ vector initialized
  if(labels_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Labels vector not initialized yet ...");
    return;
  }

  object_with_region::msg::ObjectRegion3DArray object_region_array_msg;
  object_region_array_msg.header = msg->header;

  for (const auto & detection : msg->detections) {
    // Transform the detection to the map frame
    geometry_msgs::msg::PointStamped detection_position;
    detection_position.header = detection.header;

    // Create ObjectRegion3D message
    object_with_region::msg::ObjectRegion3D object_region_msg;
    object_region_msg.object = detection;
    try {
      object_region_msg.object.results[0].hypothesis.class_id =
        labels_[std::stoi(detection.results[0].hypothesis.class_id)];
      object_region_msg.region = "unknown";
      object_region_msg.header = msg->header;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Could not process detection: %s",
        detection.results[0].hypothesis.class_id.c_str());
      continue;
    }

    // Call the service to get the region name
    if (get_region_enabled_) {
      auto region_name = client_call(detection_position);
      if (region_name.empty()) {continue;}
      object_region_msg.region = region_name;
    }

    // Add to array
    object_region_array_msg.objects.push_back(object_region_msg);
    RCLCPP_INFO(this->get_logger(),
      "Object %s assigned to region: %s",
      object_region_msg.object.results[0].hypothesis.class_id.c_str(),
      object_region_msg.region.c_str());
  }
  // Publish the objects with region
  object_with_region_pub_->publish(object_region_array_msg);
}


std::string ObjectWithRegionNode::client_call(const geometry_msgs::msg::PointStamped & position)
{
  std::string result;
    // Create the request
  auto request = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Request>();
  request->position = position;
    // Wait for service
  while (!get_region_name_client_->wait_for_service(std::chrono::milliseconds(500))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return result;       // empty result
    }
    RCLCPP_DEBUG(this->get_logger(), "service not available, waiting again...");
  }

  using namespace std::chrono_literals;
  auto future = get_region_name_client_->async_send_request(request);
  std::future_status status = future.wait_for(std::chrono::seconds(5));    // timeout to guarantee a graceful finish
  if (status == std::future_status::ready) {
    result = future.get()->region_name;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", get_region_name_service_.c_str());
  }
  return result;
}


void ObjectWithRegionNode::get_params()
{
  declare_parameter_if_not_declared(
    this, "detections_3d_topic",
    rclcpp::ParameterValue("detections_3d"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Topic of the 3D detections"));
  this->get_parameter("detections_3d_topic", detections_3d_topic_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter detections_3d_topic is set to: [%s]", detections_3d_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "label_info_topic",
    rclcpp::ParameterValue("label_info"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Topic of the label info"));
  this->get_parameter("label_info_topic", label_info_topic_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter label_info_topic is set to: [%s]", label_info_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "objects_with_region_topic",
    rclcpp::ParameterValue("objects_with_region_3d"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Topic to publish the detected objects with their associated region"));
  this->get_parameter("objects_with_region_topic", objects_with_region_topic_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter objects_with_region_topic is set to: [%s]", objects_with_region_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "get_region_name_service",
    rclcpp::ParameterValue("get_region_name"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Service name to get the region name from a position"));
  this->get_parameter("get_region_name_service", get_region_name_service_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter get_region_name_service is set to: [%s]", get_region_name_service_.c_str());


  declare_parameter_if_not_declared(
    this, "get_region_enabled",
    rclcpp::ParameterValue(true),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Boolean to enable/disable getting region names"));
  this->get_parameter("get_region_enabled", get_region_enabled_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter get_region_enabled is set to: [%s]", get_region_enabled_ ? "true" : "false");
}

} // namespace object_with_region
