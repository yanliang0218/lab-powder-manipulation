cmake_minimum_required(VERSION 3.8)
project(arm_v2_description)

find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(print_kdl_tree src/print_kdl_tree.cpp)

ament_target_dependencies(
  print_kdl_tree
  kdl_parser
  orocos_kdl
)

add_executable(print_kdl_chain src/print_kdl_chain.cpp)

ament_target_dependencies(
  print_kdl_chain
  kdl_parser
  orocos_kdl
)

add_executable(compute_fk src/compute_fk.cpp)

ament_target_dependencies(
  compute_fk
  kdl_parser
  orocos_kdl
)

add_executable(record_tf_path src/record_tf_path.cpp)

ament_target_dependencies(
  record_tf_path
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

install(
  TARGETS 
    print_kdl_tree 
    print_kdl_chain
    compute_fk
    record_tf_path
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
