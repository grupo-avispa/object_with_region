=========
CHANGELOG
=========

All notable changes to the ``object_with_region`` package will be documented in this file.

[Unreleased]
============

Changed
-------
- **Breaking:** ``ObjectWithRegionNode`` is now a managed lifecycle node
  (``rclcpp_lifecycle::LifecycleNode``) instead of a plain ``rclcpp::Node``.
  Subscriptions, the publisher, the TF buffer/listener, the region client and
  its timeout reaper are created in ``on_configure()`` rather than the
  constructor; the publisher is a ``LifecyclePublisher`` activated in
  ``on_activate()`` / deactivated in ``on_deactivate()``; ``on_cleanup()`` and
  ``on_shutdown()`` tear everything back down. ``detection_callback()`` is now
  a no-op unless the node is in the active state. ``main.cpp`` adds the node
  to the executor via ``get_node_base_interface()``. New ``rclcpp_lifecycle``
  and ``lifecycle_msgs`` dependencies.

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
- **Breaking:** ``ObjectRegion3D.msg`` no longer overwrites
  ``object.results[0].hypothesis.class_id`` with the resolved label; the
  numeric id is preserved and the human-readable label is now published in a
  new ``string class_name`` field.
- **Breaking:** Removed the redundant ``std_msgs/Header header`` field from
  ``ObjectRegion3D.msg``. ``object`` already carries its own header (or the
  ``ObjectRegion3DArray`` header is authoritative if it does not).

Changed
-------
- **Performance:** Replaced the per-detection blocking region-service call in
  ``ObjectWithRegionNode`` with an asynchronous pipeline: ``detection_callback()``
  now issues all region requests for a frame via ``async_send_request()`` and
  returns immediately, instead of blocking the callback (and its callback
  group) for up to ``service_call_timeout`` seconds per detection. A new
  ``PendingFrame``/``PendingRegionRequest`` pair (declared in
  ``object_with_region.hpp``) accumulates the resolved objects for a frame and
  publishes it once every detection has been resolved, either from the
  service response or from a periodic timeout reaper
  (``reap_timed_out_requests()``). This also supersedes the bounded
  ``wait_for_service`` wait added earlier: an unavailable service is now
  detected via ``service_is_ready()`` without blocking at all.

Added
-----
- Added ``test/test_detection_processor.cpp``, a ``gtest`` suite covering
  ``build_object_region()``: missing results, non-numeric class_id,
  out-of-range and negative class_id, and the happy path. Wired up via
  ``ament_add_gtest`` and a new ``BUILD_TESTING`` block in ``CMakeLists.txt``.
  Replaced the ``ament_cmake_test`` test_depend, which does not provide gtest
  support, with ``ament_cmake_gtest`` in ``package.xml``.

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
