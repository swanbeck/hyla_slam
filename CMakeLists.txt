cmake_minimum_required(VERSION 3.16...3.26)
project(hyla_slam VERSION 1.0.0 LANGUAGES CXX)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../kiss-icp/cpp/kiss_icp/ AND NOT IS_SYMLINK ${CMAKE_CURRENT_SOURCE_DIR})
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../kiss-icp/cpp/kiss_icp ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp)
else()
    cmake_minimum_required(VERSION 3.18)
    message(STATUS "Performing out-of-tree build, fetching KISS-ICP v${CMAKE_PROJECT_VERSION} Release from Github")
    include(FetchContent)
    FetchContent_Declare(
        ext_kiss_icp_core PREFIX kiss_icp_core
        URL https://github.com/PRBonn/kiss-icp/archive/refs/tags/v${CMAKE_PROJECT_VERSION}.tar.gz SOURCE_SUBDIR
            cpp/kiss_icp)
    FetchContent_MakeAvailable(ext_kiss_icp_core)
endif()

if(NOT TARGET kiss_icp_core)
    message(FATAL_ERROR "kiss_icp_core target not found")
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

# hyla_kiss
add_library(hyla_kiss STATIC
    include/hyla_slam/hyla_kiss.hpp
    src/hyla_kiss.cpp
)
add_dependencies(hyla_kiss kiss_icp_core)
target_include_directories(hyla_kiss PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hyla_kiss PUBLIC kiss_icp_core)

# utils
add_library(utils STATIC
    include/hyla_slam/utils.hpp
    src/utils.cpp
)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# hylacomylus
add_library(hylacomylus STATIC
    include/hyla_slam/hylacomylus.hpp
    src/hylacomylus.cpp
)
target_include_directories(hylacomylus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(hylacomylus PUBLIC
    utils
    ${PCL_LIBRARIES}
)

add_executable(hyla_slam_ros_node src/hyla_slam_ros_node.cpp)
ament_target_dependencies(hyla_slam_ros_node
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    pcl_ros
    pcl_conversions
)
target_link_libraries(hyla_slam_ros_node ${PCL_LIBRARIES} hylacomylus hyla_kiss)

install(TARGETS
    hyla_slam_ros_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
