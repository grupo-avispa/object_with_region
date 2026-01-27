cmake_minimum_required(VERSION 3.8)
project(object_with_region)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CMAKE_CXX_STANDARD 17)
  else()
    message(FATAL_ERROR "cxx_std_17 could not be found.")
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

option(COVERAGE_ENABLED "Enable code coverage" FALSE)

if(COVERAGE_ENABLED)
  add_compile_options(--coverage)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

# Defaults for Microsoft C++ compiler
if(MSVC)
  # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(semantic_navigation_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

##########################
## Declare ROS messages ##
##########################
set(msg_files
  "msg/ObjectRegion3D.msg"
  "msg/ObjectRegion3DArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs vision_msgs
)

# ##########
# # Build ##
# ##########
# Executable node
set(EXECUTABLE_NAME ${PROJECT_NAME}_node)

add_executable(${EXECUTABLE_NAME}
  src/object_with_region.cpp
  src/main.cpp
)

target_include_directories(${EXECUTABLE_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${EXECUTABLE_NAME}
  PUBLIC
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  ${vision_msgs_TARGETS}
  ${tf2_sensor_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${semantic_navigation_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Add dependency on the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${EXECUTABLE_NAME} PUBLIC "${cpp_typesupport_target}")

if("$ENV{ROS_DISTRO}" STREQUAL "jazzy" OR
   "$ENV{ROS_DISTRO}" STREQUAL "humble")
  target_link_libraries(${EXECUTABLE_NAME} PUBLIC nav2_util::nav2_util_core)
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  target_link_libraries(${EXECUTABLE_NAME}
    PRIVATE
    nav2_util::nav2_util_core
  )
endif()

# ############
# # Install ##
# ############
install(TARGETS ${EXECUTABLE_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  nav2_util
  rclcpp
  std_srvs
  tf2_ros
  vision_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  semantic_navigation_msgs
  geometry_msgs
)
ament_package()
