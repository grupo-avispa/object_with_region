# object_with_region

![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/object_with_region)
[![Build](https://github.com/grupo-avispa/object_with_region/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/grupo-avispa/object_with_region/actions/workflows/build.yml)
## Description

`object_with_region` is a ROS 2 package that enriches 3D object detections with semantic region information. This node takes three-dimensional object detections and assigns them to the spatial region where they are located (e.g., "kitchen", "living room", "bedroom"), facilitating semantic navigation and spatial reasoning in mobile robotics.

## Features

- **3D Detection Integration**: Processes `vision_msgs/Detection3DArray` messages to add region context.
- **Semantic Region Querying**: Uses a semantic navigation service to determine which region each detected object is in.
- **Label Resolution**: Converts numeric class IDs to human-readable class names.
- **Configurable Mode**: Allows enabling/disabling region querying via parameters.
- **TF2 Transformation Support**: Transforms each detection's position into `target_frame` before querying the region service.
- **Managed Lifecycle**: Implemented as a lifecycle node, so it only starts processing once explicitly configured and activated.

## Architecture

The node acts as an intermediary between a 3D object detector and high-level reasoning systems:

```
┌────────────────────┐         ┌──────────────────────┐         ┌─────────────────────┐
│  3D Detector       │────────▶│  object_with_region  │────────▶│  High-Level         │
│  (Detection3DArray)│         │      Node            │         │  System             │
└────────────────────┘         └──────────────────────┘         └─────────────────────┘
                                         │
                                         │ (service call)
                                         ▼
                               ┌──────────────────────┐
                               │  Region Service      │
                               │  (GetRegionName)     │
                               └──────────────────────┘
```

## Dependencies

### ROS 2 Dependencies

- `rclcpp`: ROS 2 C++ client library
- `vision_msgs`: Standard vision messages for ROS
- `std_msgs`: Standard ROS messages
- `geometry_msgs`: Geometry messages
- `tf2_ros`: Transformation library
- `tf2_geometry_msgs`: Geometry conversions for TF2
- `semantic_navigation_msgs`: Custom messages for semantic navigation 

## Installation

1. Navigate to your ROS 2 workspace:
```bash
cd ~/colcon_ws/src
```

2. Clone the repository (if you haven't already):
```bash
git clone https://github.com/grupo-avispa/object_with_region.git
```

3. Install dependencies:
```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:
```bash
colcon build --packages-select object_with_region --symlink-install
```

5. Source the environment:
```bash
source install/setup.bash
```

## Usage

### Basic Launch

To launch the node with default configuration:

```bash
ros2 launch object_with_region default.launch.py
```

## Lifecycle

`object_with_region_node` is a managed lifecycle node
([design.ros2.org: Managed Nodes](https://design.ros2.org/articles/node_lifecycle.html)): it does
not create any subscription, publisher or service client until it is configured, and it does not
publish until it is activated. This keeps it from processing or publishing anything before
`label_info_topic` has actually been consumed and the node is ready.

The provided launch file drives the `configure` and `activate` transitions automatically (see the
`autostart` parameter below); outside of it, transition the node manually, e.g.:

```bash
ros2 lifecycle set /object_with_region configure
ros2 lifecycle set /object_with_region activate
```

## Parameters

The node accepts the following parameters (defined in [params/params.yaml](params/params.yaml)):

| Parameter                   | Type   | Default Value                            | Description                                                |
| --------------------------- | ------ | ---------------------------------------- | ---------------------------------------------------------- |
| `detections_3d_topic`       | string | `/object_detection/detections_3d`        | Input topic with 3D detections                             |
| `label_info_topic`          | string | `/object_detection/label_info`           | Topic with class label information                         |
| `objects_with_region_topic` | string | `/object_detection/objects_with_region`  | Output topic with objects enriched with region information |
| `get_region_name_service`   | string | `/get_region_name`                       | Service name to get the region from a position             |
| `get_region_enabled`        | bool   | `true`                                   | Enable/disable region querying (useful for debugging)      |
| `service_call_timeout`      | double | `5.0`                                    | Timeout in seconds to wait for the region service response |
| `target_frame`              | string | `map`                                    | Frame detection positions are transformed into before querying the region service |

## Frames

`get_region_name_service` is assumed to interpret positions in `target_frame`. Each
detection's position is transformed from its own frame (or the `detections_3d_topic`
message frame, if the detection does not carry one) into `target_frame` via TF2 before
the service is called; the detection itself is still published in its original frame.
If no transform is available, that detection is skipped and logged as a warning.

## Topics

### Subscriptions

- **`detections_3d_topic`** ([vision_msgs/Detection3DArray](https://github.com/ros-perception/vision_msgs/blob/ros2/msg/Detection3DArray.msg))
  - 3D object detections from a detector
  
- **`label_info_topic`** ([vision_msgs/LabelInfo](https://github.com/ros-perception/vision_msgs/blob/ros2/msg/LabelInfo.msg))
  - Meta-information about object classes (ID to name mapping)

### Publishers

- **`objects_with_region_topic`** ([object_with_region/ObjectRegion3DArray](msg/ObjectRegion3DArray.msg))
  - Object detections enriched with semantic region information

## Services

### Client

- **`get_region_name_service`** ([semantic_navigation_msgs/GetRegionName](semantic_navigation_msgs/srv/GetRegionName.srv))
  - Service used to query the region name given a 3D position

## Custom Messages

### ObjectRegion3D

Represents a detected object with its associated region:

```
vision_msgs/Detection3D object
string class_name
string region
```

### ObjectRegion3DArray

Array of objects with regions:

```
std_msgs/Header header
ObjectRegion3D[] objects
```

## License

This package is licensed under the Apache 2.0 license. See the [LICENSE](LICENSE) file for more details.

## Authors

- **Óscar Pons Fernández** - [@opfernandez](https://github.com/opfernandez)
- **Grupo Avispa, DTE, Universidad de Málaga**

## References

- [vision_msgs](https://github.com/ros-perception/vision_msgs): Standard vision messages for ROS 2
- [Nav2](https://navigation.ros.org/): Navigation framework for ROS 2
- [TF2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html): ROS 2 transformation system
