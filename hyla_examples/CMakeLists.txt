cmake_minimum_required(VERSION 3.16...3.26)
project(hyla_examples VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hyla_slam_behaviors REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(watertender_behaviors REQUIRED)

add_executable(run_behavior_tree src/run_behavior_tree.cpp)
ament_target_dependencies(run_behavior_tree
    hyla_slam_behaviors
    watertender_behaviors
    behaviortree_cpp
)

install(TARGETS
    run_behavior_tree
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY behavior_trees launch config
    DESTINATION share/${PROJECT_NAME}    
)

ament_package()
