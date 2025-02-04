cmake_minimum_required(VERSION 3.16...3.26)
project(hyla_slam VERSION 1.0.0 LANGUAGES C CXX)

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
find_package(std_srvs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hyla_slam_interfaces REQUIRED)
find_package(Boost REQUIRED)

include(FetchContent)
set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "Don't use fmt for Sophus library")
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "Don't build Sophus Examples")
FetchContent_Declare(sophus SYSTEM URL https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz
                     PATCH_COMMAND patch -p1 < ${CMAKE_SOURCE_DIR}/../kiss-icp/cpp/kiss_icp/3rdparty/sophus/sophus.patch UPDATE_DISCONNECTED 1)
FetchContent_MakeAvailable(sophus)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# hyla_kiss
add_library(hyla_kiss STATIC
    hyla_kiss/hyla_kiss.hpp
    hyla_kiss/hyla_kiss.cpp
    hyla_kiss/kiss_icp_utils.hpp
)
add_dependencies(hyla_kiss kiss_icp_core)
target_include_directories(hyla_kiss PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hyla_kiss PUBLIC kiss_icp_core)

# hylacomylus
add_library(hylacomylus STATIC
    hylacomylus/hylacomylus.hpp
    hylacomylus/hylacomylus.cpp
    hylacomylus/chunk.hpp
    hylacomylus/hasher.hpp
    hylacomylus/types.hpp
    hylacomylus/types.cpp
)
target_include_directories(hylacomylus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hylacomylus PUBLIC ${PCL_LIBRARIES} sophus ${Boost_LIBRARIES})

generate_parameter_library(
    hyla_slam_parameters
    ros/hyla_slam_parameters.yaml
)

add_executable(behaviors_node ros/behaviors_node.cpp)
ament_target_dependencies(behaviors_node
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    pcl_ros
    pcl_conversions
    std_srvs
    hyla_slam_interfaces
)
target_link_libraries(behaviors_node
    ${PCL_LIBRARIES}
    hylacomylus
    hyla_kiss
    hyla_slam_parameters
)

install(TARGETS
    behaviors_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
