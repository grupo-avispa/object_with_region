=========
CHANGELOG
=========

All notable changes to the ``object_with_region`` package will be documented in this file.

[Unreleased]
============

Added
-----
- Added ``target_frame`` parameter and reintroduced the TF2 buffer/listener in
  ``ObjectWithRegionNode`` to actually transform each detection's position into
  ``target_frame`` before calling ``get_region_name_service``, and documented the
  frame contract in the README.

Fixed
-----
- Fixed the region service receiving detection positions in the original sensor
  frame instead of the frame it expects.
- Detection position now falls back to the ``Detection3DArray`` header when a
  single ``Detection3D`` does not carry its own ``frame_id``.

Changed
-------
- Extracted the label-resolution and validation logic out of
  ``ObjectWithRegionNode::detection_callback()`` into a pure, ROS-free function
  ``object_with_region::detection_processor::build_object_region()``
  (``include/object_with_region/detection_processor.hpp``,
  ``src/detection_processor.cpp``), enabling direct unit testing.

[0.1.0] - 25-07-2026
=====================

Added
-----
- Added ``service_call_timeout`` parameter to ``ObjectWithRegionNode`` to configure how
  long to wait for a response from the region service.

Changed
-------
- Subscribed to ``label_info_topic`` with a transient local QoS profile in
  ``ObjectWithRegionNode`` so a latched label map is received regardless of
  publisher/subscriber startup order.
- Replaced the unbounded ``wait_for_service`` retry loop in
  ``ObjectWithRegionNode::client_call()`` with a single bounded wait.
- Extracted the hardcoded subscription/publisher queue depth and service wait
  timeout in ``src/object_with_region.cpp`` into named constants.

Fixed
-----
- Fixed undefined behavior in ``ObjectWithRegionNode::detection_callback()`` from
  accessing ``detection.results[0]`` and indexing ``labels_`` without bounds checks.
- Corrected the ``label_info_topic`` default documented in ``README.md`` to match
  ``params/params.yaml``.
- Corrected the Codecov ``slug`` in ``.github/workflows/build.yml``, which pointed
  to an unrelated repository.

Removed
-------
- Removed the unused ``tf2_ros``, ``tf2_geometry_msgs`` and ``tf2_sensor_msgs``
  includes and dependencies: the TF buffer/listener were created but never used
  to transform anything.
- Removed the stale ``nav2_util`` / ``nav2_ros_common`` dependency mention from
  ``README.md``.

Documentation
-------------
- Filled in the package description and bumped the version in ``package.xml``.
- Added ``find_package(std_msgs REQUIRED)`` to ``CMakeLists.txt`` for the
  ``std_msgs`` dependency already declared in ``rosidl_generate_interfaces``.
