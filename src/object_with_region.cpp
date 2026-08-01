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

#include <chrono>
#include <iterator>
#include <memory>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "object_with_region/detection_processor.hpp"
#include "object_with_region/msg/object_region3_d.hpp"
#include "object_with_region/object_with_region.hpp"

using std::placeholders::_1;

namespace object_with_region
{

namespace
{
// Depth of the subscription/publisher queues.
constexpr int kDefaultQueueDepth = 10;
// How often to check pending_requests_ for entries that timed out.
constexpr std::chrono::seconds kRequestReaperPeriod{1};
}  // namespace

ObjectWithRegionNode::ObjectWithRegionNode()
: rclcpp_lifecycle::LifecycleNode("object_with_region_node")
{
  // Get parameters from parameter server. ROS entities (subscriptions,
  // publisher, client, TF) are created in on_configure() instead, so this
  // node can be configured/activated independently of construction, per the
  // managed node lifecycle.
  get_params();
}

ObjectWithRegionNode::~ObjectWithRegionNode()
{
}


ObjectWithRegionNode::CallbackReturn ObjectWithRegionNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  // Initialize transform buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create callback group for subscribers
  sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_cb_group_;

  // Create detections 3D subscriber
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    detections_3d_topic_, kDefaultQueueDepth,
    std::bind(&ObjectWithRegionNode::detection_callback, this, _1),
    sub_options);

  // Create label info subscriber.
  // Label maps are typically published once with a transient local QoS, so this
  // subscription must match it or it may never receive the message.
  auto label_info_qos = rclcpp::QoS(1).transient_local().reliable();
  label_info_sub_ = this->create_subscription<vision_msgs::msg::LabelInfo>(
    label_info_topic_,
    label_info_qos,
    std::bind(&ObjectWithRegionNode::label_info_callback, this, std::placeholders::_1),
    sub_options);

  // Create objects with region publisher
  object_with_region_pub_ = this->create_publisher<object_with_region::msg::ObjectRegion3DArray>(
    objects_with_region_topic_, kDefaultQueueDepth);

  if (get_region_enabled_) {
    // Create client to get region name from position. It is assigned to
    // sub_cb_group_ so its response callbacks are serialized with the
    // subscription callbacks above and pending_requests_ needs no locking.
    get_region_name_client_ = this->create_client<semantic_navigation_msgs::srv::GetRegionName>(
      get_region_name_service_, rmw_qos_profile_services_default, sub_cb_group_);

    // Periodically resolve region requests that have been pending for too long.
    request_reaper_timer_ = this->create_wall_timer(
      kRequestReaperPeriod,
      std::bind(&ObjectWithRegionNode::reap_timed_out_requests, this),
      sub_cb_group_);
  }

  return CallbackReturn::SUCCESS;
}


ObjectWithRegionNode::CallbackReturn ObjectWithRegionNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  object_with_region_pub_->on_activate();
  return CallbackReturn::SUCCESS;
}


ObjectWithRegionNode::CallbackReturn ObjectWithRegionNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  object_with_region_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}


ObjectWithRegionNode::CallbackReturn ObjectWithRegionNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  request_reaper_timer_.reset();
  get_region_name_client_.reset();
  object_with_region_pub_.reset();
  label_info_sub_.reset();
  detection_sub_.reset();
  sub_cb_group_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  pending_requests_.clear();
  labels_.clear();
  return CallbackReturn::SUCCESS;
}


ObjectWithRegionNode::CallbackReturn ObjectWithRegionNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
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
  // Subscriptions stay up across the inactive state (they are only torn down
  // on cleanup/shutdown); avoid doing any work, including TF/service calls,
  // while not active.
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // This callbacks needs labels_ vector initialized
  if(labels_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Labels vector not initialized yet ...");
    return;
  }

  auto frame = std::make_shared<PendingFrame>();
  frame->array_msg.header = msg->header;
  frame->on_complete =
    [this](const object_with_region::msg::ObjectRegion3DArray & array_msg) {
      object_with_region_pub_->publish(array_msg);
    };
  // Sentinel: keeps the frame open while this loop is still issuing requests,
  // so a detection that resolves synchronously below cannot trigger a
  // premature publish before later detections in this same message have been
  // processed. Released once the loop below has finished.
  frame->pending_count = 1;

  for (const auto & detection : msg->detections) {
    auto object_region = detection_processor::build_object_region(detection, labels_);
    if (!object_region) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not process detection: missing results or invalid class_id, skipping");
      continue;
    }

    ++frame->pending_count;

    if (!get_region_enabled_) {
      RCLCPP_INFO(
        this->get_logger(), "Object %s assigned to region: %s",
        object_region->class_name.c_str(), object_region->region.c_str());
      frame->resolve(std::move(object_region));
      continue;
    }

    // Detection3DArray commonly carries the frame in the array header rather than
    // in each Detection3D, so fall back to it when the per-detection one is empty.
    geometry_msgs::msg::PointStamped detection_position;
    detection_position.header =
      !detection.header.frame_id.empty() ? detection.header : msg->header;
    detection_position.point = detection.bbox.center.position;

    auto target_position = transform_to_target_frame(detection_position);
    if (!target_position) {
      frame->resolve(std::nullopt);
      continue;
    }

    request_region(frame, std::move(*object_region), *target_position);
  }

  frame->resolve(std::nullopt);  // release the sentinel
}


std::optional<geometry_msgs::msg::PointStamped> ObjectWithRegionNode::transform_to_target_frame(
  const geometry_msgs::msg::PointStamped & input)
{
  try {
    return tf_buffer_->transform(input, target_frame_, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform detection from '%s' to '%s': %s",
      input.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    return std::nullopt;
  }
}


void ObjectWithRegionNode::request_region(
  const std::shared_ptr<PendingFrame> & frame,
  object_with_region::msg::ObjectRegion3D object_region,
  const geometry_msgs::msg::PointStamped & position)
{
  // Rather than blocking on wait_for_service(), skip immediately if the
  // service is not available: this callback must never block the executor.
  if (!get_region_name_client_->service_is_ready()) {
    RCLCPP_ERROR(
      this->get_logger(), "Service %s not available, skipping",
      get_region_name_service_.c_str());
    frame->resolve(std::nullopt);
    return;
  }

  auto pending = std::make_shared<PendingRegionRequest>();
  pending->frame = frame;
  pending->object_region = std::move(object_region);
  pending->deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(service_call_timeout_));

  auto request = std::make_shared<semantic_navigation_msgs::srv::GetRegionName::Request>();
  request->position = position;

  get_region_name_client_->async_send_request(
    request,
    [this, pending](
      rclcpp::Client<semantic_navigation_msgs::srv::GetRegionName>::SharedFuture future) {
      // The reaper may already have resolved this request as timed out.
      if (pending->resolved) {return;}
      pending->resolved = true;

      const auto & region_name = future.get()->region_name;
      if (region_name.empty()) {
        pending->frame->resolve(std::nullopt);
        return;
      }
      pending->object_region.region = region_name;
      RCLCPP_INFO(
        this->get_logger(), "Object %s assigned to region: %s",
        pending->object_region.class_name.c_str(), pending->object_region.region.c_str());
      pending->frame->resolve(std::move(pending->object_region));
    });

  pending_requests_.push_back(pending);
}


void ObjectWithRegionNode::reap_timed_out_requests()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto it = pending_requests_.begin(); it != pending_requests_.end(); ) {
    auto & pending = *it;
    if (!pending->resolved && now >= pending->deadline) {
      pending->resolved = true;
      RCLCPP_ERROR(
        this->get_logger(), "Timed out waiting for service %s", get_region_name_service_.c_str());
      pending->frame->resolve(std::nullopt);
    }
    it = pending->resolved ? pending_requests_.erase(it) : std::next(it);
  }
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

  declare_parameter_if_not_declared(
    this, "service_call_timeout",
    rclcpp::ParameterValue(5.0),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description("Timeout in seconds to wait for the region service response"));
  this->get_parameter("service_call_timeout", service_call_timeout_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter service_call_timeout is set to: [%.2f]", service_call_timeout_);

  declare_parameter_if_not_declared(
    this, "target_frame",
    rclcpp::ParameterValue("map"),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__description(
      "Frame in which detection positions are expressed before querying the region service"));
  this->get_parameter("target_frame", target_frame_);
  RCLCPP_INFO(
    this->get_logger(),
    "The parameter target_frame is set to: [%s]", target_frame_.c_str());
}

} // namespace object_with_region
