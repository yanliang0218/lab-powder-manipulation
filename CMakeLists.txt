cmake_minimum_required(VERSION 3.8)
project(arm_v2_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ==============================================================================
# 1. Find ROS 2 and external dependencies
# ==============================================================================

find_package(ament_cmake REQUIRED)

# Kinematics and robot-model dependencies.
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

# ROS 2 C++ and TF dependencies retained for the package's other utilities.
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Message dependencies used by the C++ nodes and custom interfaces.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Custom message generation.
find_package(rosidl_default_generators REQUIRED)


# ==============================================================================
# 2. Generate custom ROS 2 messages
# ==============================================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SyncedMsg.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)


# ==============================================================================
# 3. KDL inspection and standalone FK utilities
# ==============================================================================

add_executable(
  print_kdl_tree
  src/FK/print_kdl_tree.cpp
)

ament_target_dependencies(
  print_kdl_tree
  kdl_parser
  orocos_kdl
)

add_executable(
  print_kdl_chain
  src/FK/print_kdl_chain.cpp
)

ament_target_dependencies(
  print_kdl_chain
  kdl_parser
  orocos_kdl
)

add_executable(
  compute_fk
  src/FK/compute_fk.cpp
)

ament_target_dependencies(
  compute_fk
  kdl_parser
  orocos_kdl
)


# ==============================================================================
# 4. Online robot-state, event-driven FK and synchronization nodes
# ==============================================================================

# Subscribe to /joint_states. Every received JointState directly triggers one
# KDL FK calculation and one /FK_pose publication with the same timestamp.
# The executable name is retained as record_tf_path for launch compatibility.
add_executable(
  record_tf_path
  src/FK/record_tf_path.cpp
)

ament_target_dependencies(
  record_tf_path
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  orocos_kdl
)

# Read the physical encoders and publish /joint_angles plus /joint_states.
add_executable(
  encoder_joint_state_bridge
  src/FK/encoder_joint_state_bridge.cpp
)

ament_target_dependencies(
  encoder_joint_state_bridge
  rclcpp
  sensor_msgs
)

# Package each unique compressed camera image with the latest FK pose and
# optional joint angles for offline hand-eye calibration.
add_executable(
  sensor_sync
  src/sync/sensor_sync_offline_tag.cpp
)

ament_target_dependencies(
  sensor_sync
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  sensor_sync
  "${cpp_typesupport_target}"
)


# ==============================================================================
# 5. Install compiled C++ executables
# ==============================================================================

install(
  TARGETS
    print_kdl_tree
    print_kdl_chain
    compute_fk
    record_tf_path
    encoder_joint_state_bridge
    sensor_sync
  DESTINATION lib/${PROJECT_NAME}
)


# ==============================================================================
# 6. Install package resources
# ==============================================================================

install(
  DIRECTORY
    urdf
    meshes
    launch
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

# The camera acquisition launch file currently lives under src/vision.
install(
  FILES
    src/vision/camera_pose/webcam_handeye_720p_150hz.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)


# ==============================================================================
# 7. Install Python calibration tools
# ==============================================================================

install(
  PROGRAMS
    src/vision/camera_pose/capture_calibration_images.py
    src/vision/camera_pose/calibrate_camera.py
    src/vision/camera_pose/offline_handeye_from_rosbag.py
  DESTINATION lib/${PROJECT_NAME}
)


# ==============================================================================
# 8. Finalize the ament package
# ==============================================================================

ament_package()

