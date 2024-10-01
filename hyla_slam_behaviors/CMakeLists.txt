cmake_minimum_required(VERSION 3.8)
project(hyla_slam_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hyla_slam_interfaces REQUIRED)

include_directories(
  include
)

set(BEHAVIOR_TREE_DEPS 
  rclcpp
  behaviortree_cpp
  std_srvs
  geometry_msgs
  sensor_msgs
  hyla_slam_interfaces
)

add_library(hyla_slam_behaviors STATIC
  src/disable_localization.cpp
  src/enable_localization.cpp
  src/get_localization_displacement.cpp
  src/get_map.cpp
  src/get_mapping_displacement.cpp
  src/index_data.cpp
  src/set_localization_estimate.cpp
  src/unload_data.cpp
  src/update_localization_map.cpp
)
ament_target_dependencies(hyla_slam_behaviors ${BEHAVIOR_TREE_DEPS})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${BEHAVIOR_TREE_DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
